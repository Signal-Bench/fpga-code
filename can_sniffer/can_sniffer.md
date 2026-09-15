# can_sniffer — CAN 2.0B bus sniffer design

Passive CAN sniffer for the iCE40 UP5K on an UPduino v3, draining decoded frames to a
host over bit-banged SPI. Structurally the successor to `i2c_sniffer/`, but the front
end is entirely different: CAN has no clock line, so the bit clock must be recovered.

**Status as of 2026-09-10:** bit timing, CRC-15 and the frame decoder are implemented
and passing simulation (16 tests). Record packing, the EBR ring, the SPI drain and the
top module are designed below but **not written yet**. Nothing has been on hardware.

The decoder synthesizes for the UP5K at **316 LUT4 + ~271 FF**, roughly 6% of the
device's 5280 LUTs — `make synth-check`.

## Target bus

The bench bus is one STM32 and one RoboMaster GM6020 motor. From the GM6020 manual
(`docs/RM GM6020 使用说明（英）20231103.pdf`):

- **1 Mbit/s**, fixed
- **Standard frames only** — 11-bit ID, DATA frames, DLC = 8 throughout
- Commands from the STM32: `0x1FF` / `0x2FF` (voltage), `0x1FE` / `0x2FE` (current)
- Feedback from the motor: `0x204 + motor_id`, so `0x205` with DIP ID 1, **at 1 kHz**
- Feedback payload: `[0:1]` rotor angle 0–8191, `[2:3]` rpm (signed), `[4:5]` torque
  current, `[6]` temperature, `[7]` null

Extended (29-bit) frames are supported in the decoder because the project targets CAN
2.0B, but **the bench rig will never exercise them** — they need a synthetic testbench.

Bus load with one motor: ~2000 frames/s × ~119 bits ≈ **24%**. Comfortable.

## Clocking

**Clocked from the UPduino's 12 MHz on-board oscillator via jumper R16 ("OSC") on
`gpio_20`, not `SB_HFOSC`.** `SB_HFOSC` is ±10% (Lattice FPGA-DS-02008 Table 4.11)
against an ISO 11898-1 budget of 0.98%.

Full derivation, the ISO equations, what SJW is, and the simulation results are in
[clock_choice.md](clock_choice.md). Read that before changing any timing parameter.

Configuration: 12 tq per bit at BRP = 0, split `SYNC=1 / PROP_SEG=4 / PHASE_SEG1=4 /
PHASE_SEG2=3`, sample point at 75%, SJW = 3 (the largest ISO 11898-1 clause 11.3.1.2 allows with PHASE_SEG2 = 3). Other bit rates come from the prescaler as
`1 MHz / (BRP+1)`.

## Module breakdown

| File | Role | Status |
|---|---|---|
| `can_bit_timing.v` | tq generator, segment FSM, hard sync + resync, sample point | **done** |
| `can_crc15.v` | CRC-15 shift register, poly `0x4599` | **done** |
| `can_frame_fsm.v` | field FSM, destuffing, error + overload detection | **done** |
| `can_sniffer.v` | top: record packing, EBR ring, SPI drain, LEDs | planned |

`can_frame_fsm.v` emits a record on `frame_strobe` with the identifier, IDE/RTR, DLC,
data (left-justified so `data[0]` is always in `[63:56]`, whatever the DLC), `crc_ok`,
`ack_ok`, an overload flag and a 3-bit error code. It also drives `bus_idle` back into
`can_bit_timing.v` to gate hard synchronization.

Split into separate modules rather than one file like `i2c_sniffer.v` specifically so
the bit timing can have its own sweep testbench — the frequency-offset sweep is the
analogue of `i2c_sniffer_start_sweep_tb.v`'s start-gap sweep, and it is the only way to
prove the resync logic without hardware.

## Frame decoding

Field order, per ISO 11898-1 and CAN 2.0B Part B:

```
SOF → 11-bit base ID → RTR/SRR → IDE ─┬─ (IDE dominant)  → r0 → DLC → …
                                       └─ (IDE recessive) → 18-bit ext ID → RTR → r1 → r0 → DLC → …
… → DATA (0–8 bytes) → CRC-15 → CRC delim → ACK slot → ACK delim → EOF (7) → IFS (3)
```

The standard/extended decision is one bit: after the 11 base ID bits, read the next two.
If the second (IDE) is dominant it is a standard frame and the first was RTR; if
recessive it is extended, the first was SRR, and 18 more ID bits follow.

Two scopes that are easy to conflate:

- **Bit stuffing** covers SOF through the **CRC sequence** (inclusive). CRC delimiter,
  ACK field and EOF are fixed-form and *not* stuffed — destuffing must be switched off
  at the CRC delimiter.
- **CRC-15** is computed over the **destuffed** stream from SOF through the end of the
  **data field** — it does not include the CRC sequence itself.

Reserved bits `r0` / `r1` must **not** be flagged as form errors: the spec explicitly
says receivers accept dominant and recessive in all combinations.

Arbitration needs no handling. The bus level is the wired-AND, so when a node loses
arbitration and drops out the winning frame continues seamlessly — a passive observer
sees exactly one coherent frame.

## Errors a passive sniffer can and cannot see

| ISO error type | Detectable passively? | How |
|---|---|---|
| **Stuff** | yes, directly | 6 consecutive equal bits in a stuffed field |
| **CRC** | yes, directly | computed CRC ≠ received CRC sequence |
| **Form** | yes, directly | CRC delim / ACK delim / EOF not recessive |
| **ACK** | yes, directly | ACK slot recessive → nobody accepted the frame |
| **Bit** | **no** | requires knowing what a node *intended* to transmit |

Bit errors can only be **inferred by elimination**: an error frame (6+ dominant bits)
appearing where we detected no stuff/CRC/form/ACK error means someone saw something we
could not. Report this as **"unattributed error"**, not "bit error" — a node-local
disturbance at someone else's receiver produces the same signature, and we sit at one
point on the bus.

**Known blind spot:** a *passive* error flag is 6 **recessive** bits, so when the
error-passive node is a receiver and someone else is transmitting, the flag is
overwritten and completely invisible. It is only visible when the error-passive node was
itself the transmitter, where it shows up as a frame truncating mid-field with no error
frame and no EOF.

### Overload frames

An overload frame is 6 dominant + 8 recessive — **structurally identical to an active
error frame**, but it is flow control, not an error: a receiver asking for more time.
They are distinguished by *position*: an overload flag begins at the first bit of
intermission, an error flag begins mid-frame.

The FSM **must** handle overload frames — mis-parsing one desynchronizes the decoder and
loses frame alignment. But it gets **no LED colour and no error counter**, only a flag
bit in the record, because it is not a fault. They are effectively extinct in modern
devices (they existed for 1990s controllers that could not drain a mailbox in time); an
STM32 and a GM6020 will not generate one. Handling costs two states; not handling it
costs frame sync.

### Not an ISO error type, but worth detecting

**Bus stuck dominant** for more than 11 bit times — a shorted bus or jammed node. This
is the failure that actually happens on a bench. Note the transceiver will lie to you
about a *permanently* stuck bus: both the VP230 and the TCAN330 implement a receiver
dominant timeout (TCAN330: 1.6–3 ms) that forces RXD recessive after the timeout. At
1 Mbit/s that is >1600 bit times, so an 11-bit detector is unaffected, but you cannot
watch RXD indefinitely and expect it to stay dominant.

## Record format

Unlike I2C, a CAN frame is **bounded and self-describing** — at most 8 data bytes,
always. So there is no need for `i2c_sniffer.v`'s variable-length ping-pong buffers and
`{buf_sel, len}` descriptor queue. Fixed-size records into a flat ring buffer:

```
byte  0      0xAA record marker
byte  1      flags: IDE, RTR, overload_seen, ack_ok, crc_ok, err_code[2:0]
bytes 2–5    identifier, 29-bit right-justified, big-endian
byte  6      DLC
bytes 7–14   data[0..7]
bytes 15–18  timestamp, 32-bit, in bit times
byte  19     reserved / error bit position
```

**20 bytes, fixed stride** — addressing is `record_idx × 20`, no length field, no
descriptor queue, no partial-byte flush.

At ~2000 frames/s that is 40 KB/s against 125 KB/s available at the existing 1 MHz SPI
drain, so ~3× headroom and no need to raise the SPI clock for v1.

### Timestamps

Latched once per frame, at the SOF hard synchronization — **not** per edge. This is not
a logic analyzer and does not try to be.

The counter increments on `bit_tick` from `can_bit_timing.v`, so it counts **bit times**,
which at 1 Mbit/s is exactly microseconds. That means no second timebase to reconcile,
no extra clock domain, and the unit scales automatically with the configured bit rate.

Resolution is one bit time. That is enough for inter-frame gap, bus load, whether the
motor's 1 kHz feedback period is stable, and command→feedback latency. It is *not*
enough for physical-layer questions — bit width variation, ringing, slew — which need a
scope on the transceiver's RXD pin.

## LEDs

Coarse local indication only; exact per-type error counters go to the host over SPI.
Sticky latch of ~200 ms so a single event is visible.

| Colour | Meaning |
|---|---|
| Green | frames decoding cleanly |
| Blue | bus idle (11+ recessive) |
| Yellow | stuff or form error |
| Magenta | CRC error |
| Red | ACK error — nobody accepted the frame |
| White | unattributed error (error frame, no local cause) |
| Blinking red | bus stuck dominant |

Use **`SB_RGBA_DRV`**, not raw pin driving. The UP5K has three 24 mA constant-current
RGB outputs (FPGA-DS-02008; see FPGA-TN-02021). The existing designs in this repo drive
pins 39/40/41 directly, which gives uneven brightness across the three channels — mixed
colours land wrong, and "yellow" reads as orange.

## Transceiver

CAN is differential; the UP5K cannot read CAN_H/CAN_L directly. One wire from the
transceiver's RXD is the entire sniffer input.

### In hand: TI VP230 (SN65HVD230), 3.3 V

`docs/sn65hvd230.pdf` §10.4.3 and Table 2: RS high puts it in **"Standby Mode (Listen
Only Mode)"** — driver **disabled**, receiver **enabled**, RXD mirrors bus state,
*"completely passive to the bus."* The receiver switching characteristics (§8.8,
tPLH/tPHL 35 ns typ, 50 ns max) carry **no RS condition** — only the driver table is
RS-dependent, because that is slope control. So standby costs no receive bandwidth.

### For CAN FD later: TI TCAN330GD

`docs/tcan330.pdf`. This is the better part for where the project is going:

- **5 Mbit/s**, with the loop-delay-symmetry parameters CAN FD needs. The non-G
  TCAN330 is only specified to 1 Mbit/s — **the G suffix is what makes it FD-capable**,
  so `TCAN330GD` (G variant, D = SOIC-8) is the right order code.
- §6.4.3 **Silent Mode**: *"the silent or receive only mode... The CAN driver is
  disabled but the receiver is fully operational."* Purpose-built for this application
  rather than repurposed from a low-power mode.
- Table 6-3: S pin **HIGH** → silent, driver off, receiver on. S has an **integrated
  pull-down**, so it defaults to *normal* mode — S must be actively driven or tied high.
- Table 6-5: SHDN **LOW or NC** → normal, so pin 5 can be left open.

### Wiring — identical for both parts

Pins 1, 2, 3, 4, 6, 7 are the same on both, and on both **pin 8 high means listen-only**
(VP230 RS → standby, TCAN330G S → silent). So one wiring works for both and the parts
drop in for each other:

| Pin | VP230 | TCAN330GD | Connect to |
|---|---|---|---|
| 1 | D (TXD) | TXD | **3.3 V** — never to the FPGA |
| 2 | GND | GND | GND |
| 3 | VCC | VCC | **3.3 V** (not 5 V) |
| 4 | R (RXD) | RXD | **FPGA input** |
| 5 | Vref | SHDN | leave open |
| 6 | CANL | CANL | CAN_L — GM6020 cable B (black) |
| 7 | CANH | CANH | CAN_H — GM6020 cable A (red) |
| 8 | RS | S | **3.3 V** — listen-only |

- **Check the breakout board's termination resistor.** Many VP230 modules carry a 120 Ω,
  sometimes hard-soldered. Remove or disable it — this taps mid-bus, and the GM6020 has
  its own on DIP switch 4.
- Also check RS on the module: if it is hard-wired through a 10 kΩ–100 kΩ resistor to
  GND it is in *slope-control* mode, not standby. The fallback is RS to GND with TXD
  tied high, which is still passive, just relying on TXD rather than a disabled driver.
- Common ground between UPduino, transceiver and the motor's CAN ground.

## Pin plan

```
clk_12        20    # 12 MHz oscillator, requires R16 / "OSC" jumper shorted
can_rx        38    # transceiver RXD  (reuses i2c_sniffer's scl_pin)
spi_mosi      21
spi_cs        19
spi_sck       11
spi_cs_flash  16    # driven high to disable the on-board flash
led_r         41
led_g         39
led_b         40
```

SPI pins carried over unchanged from `i2c_sniffer.pcf`.

## SPI drain

Bit-banged **master**, lifted from `i2c_sniffer.v`'s drain FSM. The SignalBench target
architecture has the FPGA as SPI *slave* to an ESP32-H2 (see `../CLAUDE.md`), but the
link here exists to **validate the decoder**, so master-first is deliberate: it is
copy-paste from a working design and verifiable with a logic analyzer before the ESP32
exists. Swapping to the slave interface is a later, separate step.

## Open question

**Raw-bit capture mode.** When the decoder hits an error mid-frame it can report the
error type and bit position, but not what the bits actually *were*. A raw mode would
also store the as-sampled, pre-destuffing bitstream (~150 bits ≈ 19 bytes), roughly
doubling bytes per frame while enabled.

Deferred for v1. Timestamp + error type + bit position, plus the option of putting a
scope on RXD when bit-level truth is needed, should be enough. Easy to add later.

## Testing

```
make sim              # both testbenches
make sim-timing       # bit recovery only, nominal rate
make sim-frame        # full decode chain, 16 tests
make synth-check      # yosys resource estimate (no top module yet)
make wave / wave-frame        # as above but dump VCD and open gtkwave
make sweep-timing             # frequency-offset sweep, both stimulus patterns
make sweep-segments           # compare candidate segment configurations
```

`can_frame_fsm_tb.v` builds real frames from scratch — it computes the CRC-15 and
applies bit stuffing itself, independently of the DUT — then drives the bit stream onto
`rx` at 1 Mbit/s. Coverage: standard and extended data frames, standard and extended
remote frames, DLC 0 and DLC 8, a stuff-heavy payload, and every error path
(CRC, ACK, stuff, form, unattributed, stuck-dominant) plus an overload frame between two
data frames.

Two decoder behaviours worth knowing, both verified in simulation:

- A bus that goes dominant from idle produces **two** records — a stuff error first
  (from idle it is indistinguishable from a SOF followed by a stuffing violation), then
  the stuck-dominant record once 13 dominant bits have gone by. 13 is deliberately above
  the 12-bit maximum of a legal error-flag superposition.
- The overload flag rides on the **next** record, not the one before it: a frame's
  record is emitted at the last-but-one EOF bit, which is before the overload condition
  can occur.

The best hardware bring-up test costs nothing: **power the GM6020 with the STM32
disconnected.** The motor transmits feedback at 1 kHz, nobody ACKs, so it takes an ACK
error every frame and its transmit error count climbs by 8 until it goes error-passive
at 128. It never goes bus-off — Exception 1 of the fault confinement rules freezes the
count for exactly this case (CAN 2.0B Part A p26). The sniffer should show ID `0x205`,
DLC 8, **recessive ACK slot**, then long recessive gaps between retransmissions of the
same ID. That exercises ACK-error detection and retransmission detection with no fault
injection at all.
