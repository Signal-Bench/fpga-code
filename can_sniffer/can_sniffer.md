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
| `can_sniffer.v` | top: record packing, EBR ring, SPI drain, LEDs | **done** |
| `can_bringup.v` | reduced top: decoder + LEDs only, no drain | **done** |

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
byte  0      0xAA   start marker
byte  1      flags  {ide, rtr, overload, ack_ok, crc_ok, err[2:0]}
bytes 2–5    identifier, 29 bits right-justified in 32, big-endian
byte  6      {dropped, 3'b000, dlc[3:0]}
bytes 7–14   data[0..7]  (left-justified: data[0] is always byte 7)
bytes 15–18  timestamp, 32-bit, big-endian, in bit times, latched at SOF
byte  19     0x55   end marker
```

**20 bytes, fixed stride.** `0xAA` start and `0x55` end give a strong framing signature,
so a logic analyzer or host can resynchronize on any record boundary.

`err`: 0 none, 1 stuff, 2 crc, 3 form, 4 ack, 5 unattributed, 6 stuck-dominant.
`dropped` means at least one record was lost to a full buffer before this one.
**`crc_ok` reports the CRC comparison alone**, independent of any other error — so a
no-ACK frame comes back with `err=4`, `ack_ok=0` and `crc_ok=1`, which correctly says
the frame was intact and merely unacknowledged.

Buffering is one EBR as **16 records × 32 bytes** (only 20 used; the power-of-two stride
turns addressing into a concatenation instead of a multiply) — burst tolerance on top of
a drain that already keeps up.

SCK is not continuous: each byte costs 19 ticks rather than 16, because the EBR read
takes a cycle to issue, a cycle to land, and a cycle to latch. SCK sits low ~250 ns
between bytes with CS still asserted. That is ordinary SPI and analyzers decode it
normally — they count clock edges, not time. The settle cycle was a latent bug: at
1 MHz the six-clock tick hid a missing wait state, and at 6 MHz every byte came out
lagged by one.

The drain runs at **6 MHz SCK** — the ceiling from a 12 MHz clock, since the divider
can only halve. A record takes ~32 µs, and the shortest possible 1 Mbit/s CAN frame
(DLC 0, worst-case stuffing) is ~57 µs, so **the drain keeps up with a fully saturated
bus**. At the original 1 MHz a record took 160 µs and could not. 10 MHz is not
reachable without a PLL, and the core clocks that would allow it (20/40/60 MHz) are
at or beyond this design's timing closure (~21 MHz).

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

## Bench wiring

There is now a bitstream to flash: `can_bringup.v` (`make build && make flash`). It is
clock + pin + decoder + LEDs only — no SPI drain, no capture RAM, no record packing.
Its job is to answer on hardware how far the chain gets. 533 LCs (10% of the UP5K),
timing closes at 20.6 MHz against the 12 MHz constraint.

LED map — priority-encoded and mutually exclusive, so each state is one colour rather
than the three channels summing to white:

| What you see | Means |
|---|---|
| dark | no power, or not configured |
| **blue blinking** (~1.4 Hz) | clocked — R16/OSC jumper is good — but no edges on `can_rx` |
| **red blinking** | edges arriving at the FPGA pin, but nothing decodes |
| **red solid** | frames decoding, but every one carries an error |
| **green** | frames decoding cleanly |

The blinking-red state is driven from *raw pin edges*, independent of the decoder, so a
wiring fault reads differently from a decode fault. Solid red versus blinking red is
the difference between "the decoder is working and the bus has a problem" and "the
decoder is getting nothing usable". `dbg_frame` (FPGA pin 42, header
22 — deliberately next to `can_rx` on header 23, so one scope ground reaches both)
pulses ~1 µs on every decoded record, for a scope trigger.

The UPduino header has two numberings that are easy to confuse: the **FPGA pin** (what
goes in the `.pcf`) and the **header position** (where the wire physically goes). The
table below gives both; the map is `UPduino-v3.0/docs/source/features/specs.rst`.

### 1. UPduino — one jumper first

**Short R16**, marked **"OSC"** on the silkscreen, next to the oscillator. This routes
the on-board 12 MHz into FPGA pin 20. Without it the design has no clock and does
nothing at all — this is the single most likely "it doesn't work" cause on first
power-up. Once shorted, **do not connect anything to header position 44** (`gpio_20`);
it is now driven by the oscillator.

Do not take the clock from header position 41 instead: it works, but the v3.0
silkscreen has positions 41 (12 MHz) and 42 (GND) **swapped**, and a signal wire to
the wrong one shorts the oscillator output to ground.

### 2. UPduino ↔ transceiver

| Signal | FPGA pin (`.pcf`) | Header position | Transceiver pin |
|---|---|---|---|
| `can_rx` | 38 | **23** (left, 13th from top) | 4 — RXD / R |
| 3.3 V | — | **9** (left) | 3 — VCC, and pins 1 and 8 |
| GND | — | **10** (left) or **42** (right) | 2 — GND |

> **Take 3.3 V from header position 9, not position 2.** Position 2 is labelled `VIO`
> and is *not* a supply — it is the bank I/O voltage **input**. Per
> `UPduino-v3.0/docs/source/tutorials/bank_voltages.rst` and the board schematic, each
> bank's VCCIO reaches either `+3.3V` (through R31 / R20, shorted on the board) or the
> `VIO` net (through R19 / R26, which are **open**). None of those four appear in the
> BoM — they are trace jumpers and bare pads. So `VIO` is floating by default and
> meters as an arbitrary value (~2.3 V is typical leakage). Bank 1 is hardwired to
> 3.3 V and cannot be changed. Position 8 is +5 V; positions 1, 10 and 42 are GND.
>
> If the UPduino's 3.3 V is unavailable or you would rather not load its LDO, the
> Development Board Type C can supply it instead: its **Customizable I/O Port** (8-pin
> ejector header, 2.54 mm pitch, manual p10) has **pin 4 = 3.3 V** and **pin 2 = GND**.
> That port defaults to 3.3 V — the 5 V option needs R210 fitted and R209 removed, a
> manual rework — but meter it first. Powering the transceiver from there does **not**
> remove the need to tie UPduino ground to that same ground: `can_rx` is a logic signal
> and must share the FPGA's reference. Note the manual describes that rail as "mainly
> used for the onboard device power supply" and gives it no external current budget,
> unlike 5 V @ 1 A — fine for a transceiver, not for anything larger.
>
> Quick health check on the UPduino rail: 3.3 V powers the FT232H, so if the board
> enumerates over USB (`lsusb` shows `0403:6014`, and `iceprog -t` reads a flash ID)
> the rail is good and a low reading is a measurement problem, not a board problem.

Transceiver, identical for the VP230 in hand and the TCAN330GD later:

| Pin | Name | Connect to | Why |
|---|---|---|---|
| 1 | TXD / D | **3.3 V** | recessive; never let the FPGA drive this |
| 2 | GND | GND | |
| 3 | VCC | **3.3 V**, not 5 V | both are 3.3 V parts; 5 V damages the VP230 |
| 4 | RXD / R | UPduino header **23** | the entire sniffer input |
| 5 | Vref / SHDN | **leave open** | VP230: reference output, unused. TCAN330: SHDN, NC = normal |
| 6 | CANL | CAN_L | GM6020 cable **B, black** |
| 7 | CANH | CAN_H | GM6020 cable **A, red** |
| 8 | RS / S | **VP230: leave on its 10 kΩ to GND.** TCAN330: 3.3 V | see the hardware result below — VP230 standby does *not* receive at 1 Mbit/s |

One wire from header 23 to transceiver pin 4 is the whole data path. Everything else
is power and strapping.

**The VP230 breakout in hand carries a 10 kΩ and a 120 Ω.** What each one does, and
what hardware actually showed:

- **10 kΩ, RS (pin 8) to GND** — *slope-control* mode
  (`sn65hvd230.pdf` pin table: "10kΩ to 100kΩ pull down to GND = slope control mode").
  **Leave it exactly as it is.** See the finding below.
- **120 Ω, across CANH/CANL** — bus termination. **Keep it.** The Development Board
  Type C appears to carry no terminator of its own, so the sniffer terminates that end
  of the bus. See the topology below.

### Hardware finding, 2026-09-16: do NOT put the VP230 in standby mode

**Tying RS to 3.3 V leaves RXD stuck recessive — no data at 1 Mbit/s.** Verified on
hardware: CANH/CANL carried clean traffic on a scope while RXD sat permanently high.
Removing the RS wire, so the module's 10 kΩ pulls it back to slope-control mode, made
the receiver work immediately. The part was confirmed marked **VP230**, so this is not
a mis-populated SN65HVD231.

This contradicts what the datasheet implies. §10.4.3 calls RS-high "Standby Mode
(Listen Only Mode)" and says "the driver is switched off and the receiver remains
active"; Table 2 lists RXD as "Mirrors Bus State"; and the receiver switching
characteristics in §8.8 carry no RS condition at all. **An absent degraded-timing spec
is not a guarantee of full-rate operation in that mode.** Read §10.4.3's framing
again and the intent shows: it talks about letting the controller "monitor the bus for
activity" and waking on a dominant edge — that is wake-up detection, not 1 Mbit/s data
reception.

**So the correct VP230 strapping is:**

| Pin | Connect to | Role |
|---|---|---|
| 8 — RS | **nothing** — leave the module's 10 kΩ to GND | slope-control mode, receiver fully active |
| 1 — TXD / CTX | **3.3 V**, hard-wired | **this is now the entire passivity guarantee** |

Slope control costs nothing here: it only limits the *driver's* slew rate, and the
driver is never used. Tying RS to GND (high-speed mode) works equally well; the 10 kΩ
is already fitted, so there is no reason to touch it.

**The TXD strap is safety-critical in this configuration.** In slope-control mode the
driver is **enabled**. The only thing preventing the sniffer from asserting dominant on
a live motor bus is TXD being held high. Solder it; do not rely on a probe touching a
pad, and never leave it floating. The datasheet calls the D pin's internal pull-up
*weak* and recommends an external 1–10 kΩ pull-up for a dependable recessive state — a
hard wire is stronger still.

Module header, final: `3V3` → 3.3 V, `GND` → GND, `CTX` → 3.3 V, `CRX` → UPduino
header 23. Nothing on RS. The 120 Ω stays.

**For the TCAN330GD later, this finding may not transfer.** Its Silent Mode (§6.4.3) is
a purpose-built receive-only mode — "the CAN driver is disabled but the receiver is
fully operational" — not a repurposed low-power standby, so S → 3.3 V is probably
genuinely listen-only. But verify it on hardware the same way (scope on CANH/CANL and
on RXD simultaneously) before trusting it, and keep the TXD strap either way.

### Debugging a silent RXD

The sequence that found this, worth repeating if RXD ever goes quiet:

1. **Scope CANH/CANL first** — on *analog* channels, not digital. That separates "the
   bus is dead" from "the transceiver is not receiving." Idle sits at ~2.5 V on both;
   dominant pulls CANH to ~3.5 V and CANL to ~1.5 V.
2. **Check absolute levels, not just toggling.** Digital channels only give high/low.
   `CAN1` on the Dev Board is a 2-pin port with **no ground**, and the GM6020 runs from
   its own 24 V supply, so the bus common-mode can drift if grounds are not tied. The
   SN65HVD230 resolves the differential only within **−2 V to +7 V** (§8.3) of its own
   ground; outside that the receiver stops and RXD parks recessive.
3. **Logic threshold for RXD: 1.65 V** (half of 3.3 V), user-defined. Do **not** use a
   2.5 V "CMOS" preset — that value targets 5 V logic, and the datasheet's receiver
   VOH minimum is **2.4 V** (§8.6), below such a threshold.
4. **Measure VCC at the chip**, not at the supply. Recommended range is **3.0–3.6 V**
   (§8.3).
5. **Read the chip marking.** VP230 = standby (receiver on), VP231 = *sleep*, receiver
   **off**, RXD parked high — an identical symptom from a different cause.

### 3. The bus itself — who terminates what

A CAN bus wants **120 Ω at each of its two ends** and nothing in between. On this
bench the three nodes are the RoboMaster **Development Board Type C**, the sniffer,
and the **GM6020**. Where their terminators are decides the layout.

- **GM6020 — has a switchable terminator.** `RM GM6020 使用说明（英）20231103.pdf` p5:
  the DIP block is silkscreened **"CAN RESISTOR"**, switches 1–3 set the motor ID and
  **switch 4 controls CAN terminal resistance — ON enables it**. Note ID `000` is
  listed as *Invalid*, so at least one of 1–3 must be ON; `001` = ID 1 = feedback on
  `0x205`, control on `0x1FF`.
- **Development Board Type C — appears to have none.** Its manual (p11–12) shows all
  four CAN connectors — J23/J22 for CAN1, J21/J20 for CAN2 — and **no termination
  resistor or jumper anywhere on them**. The only `120.0R` in the whole document is
  R13, the IMU heater, on an unrelated page. The transceiver is a TJA1044 and the bus
  is rated to 1 Mbit/s. The giveaway is that **each CAN bus is wired to two connectors
  in parallel**: that is a pass-through design, meant to sit mid-chain with the end
  devices terminating.
- **Sniffer — has a 120 Ω** on the breakout, as shipped.

So the board's dual connectors give the ideal layout for free, with **no resistor
removal at all**:

```
   GM6020                Dev Board Type C              sniffer
 [DIP 4 = ON]          (no terminator, mid-bus)     [VP230 120 Ω kept]
   120 Ω  ──── CAN_H ──── J23 ╪ J22 ──── CAN_H ────  120 Ω
          ──── CAN_L ────     ╪     ──── CAN_L ────
   end of bus          two connectors, same net        end of bus
```

Motor into one CAN1 connector, sniffer into the other, Dev Board in the middle. Both
ends terminated, nothing stubbed off, ~60 Ω across the pair.

**Confirm with a meter before trusting the manual.** A user manual not showing a
resistor is not proof the PCB lacks one. Unpowered and unplugged, measure CANH to
CANL: ~120 Ω means it is terminated, open/high means it is not. Do the same on the
VP230 module (expect ~120 Ω) and on the GM6020 with DIP 4 ON and OFF. If the Dev Board
*does* turn out to be terminated, then remove the VP230's 120 Ω after all and hang the
sniffer off J22 as a short stub instead.

**Grounds.** `CAN1` is a 2-pin port — **CANL, CANH, and no ground**. If the sniffer
goes on CAN1, run a separate ground wire from the Dev Board to the UPduino/VP230
ground; a CAN transceiver tolerates a few volts of common-mode offset but not a
floating reference. `CAN2` is 4-pin (1: 5 V, 2: GND, 3: CANH, 4: CANL) and carries
ground in the connector, which avoids the extra wire — but the GM6020's own cable is
2-pin, so CAN1 is the natural bus for the motor. Keep the motor and the sniffer on the
*same* bus either way.

Power the GM6020 from 24 V on its XT30.

### 4. Validation outputs (planned pins, from the pin plan above)

Once the top module exists, the SPI drain goes to a logic analyzer:

| Signal | FPGA pin | Header position |
|---|---|---|
| `spi_sck` | 11 | **35** (right) |
| `spi_cs` | 19 | **37** (right) |
| `spi_mosi` | 21 | **39** (right) |
| GND | — | **42** (right) |

These three are already reserved in `can_sniffer.pcf` even though `can_bringup.v` has
no drain to bind them to — `-nowarn` makes an unbound `set_io` harmless, and the
reservation stops anything else being assigned there by accident. `dbg_frame` sits on
FPGA pin **42** (header 22) specifically to stay clear of them; an earlier revision had
it on pin 21, which is `spi_mosi`.

Same pins as `i2c_sniffer`, so an existing analyzer setup carries over. A scope
channel on transceiver pin 4 (RXD) alongside is the way to settle any "is it the bus
or the decoder" question — that pin is the ground truth the FPGA sees.

### 5. First power-up order

1. Jumper R16. Verify with a scope on header 41 that 12 MHz is present.
2. Wire the transceiver, power it at 3.3 V, **nothing on the bus yet**. RXD should sit
   high (recessive) — if it is low, the transceiver is in the wrong mode or CANH/CANL
   are shorted.
3. Connect the GM6020 alone, powered, STM32 disconnected. RXD should now show 1 kHz
   bursts. This is the no-ACK scenario from the Testing section — the sniffer should
   report `0x205` with ACK error once it exists.
4. Add the STM32. ACK errors stop; both `0x1FF` commands and `0x205` feedback appear.

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
