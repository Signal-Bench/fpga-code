# spi_sniffer — passive SPI bus sniffer design

Passive SPI sniffer for the iCE40 UP5K on an UPduino v3: four probe lines in, nothing
driven, decoded transactions drained to a host over bit-banged SPI. Third in the
sniffer family after `i2c_sniffer/` and `can_sniffer/`.

**Status: design only. No RTL exists yet.** Nothing in this document has been
simulated or seen on hardware. Numbers below are derived, not measured — they are
budgets to design against, not results.

The front end is a smaller problem than CAN (there is a clock line, so no bit recovery)
but the back end is a bigger one: SPI is **full duplex**, so every 8 clock edges
produce *two* bytes rather than one, and transactions are unbounded in length. That
combination breaks both existing buffering schemes.


## What is different about SPI

| | I2C | CAN | SPI |
|---|---|---|---|
| Clock line | yes | no (recovered) | yes |
| Data lines | 1 | 1 | **2, simultaneous** |
| Transaction length | short, bounded in practice | ≤ 8 data bytes, hard bound | **unbounded** |
| Protocol errors detectable passively | ACK/NACK | stuff, CRC, form, ACK | **none** |
| Bytes captured per 8 clocks | 1 | ~1 | **2** |

Three consequences drive the whole design:

1. **Two bytes per byte-time.** MOSI and MISO are sampled on the *same* edge — the
   master drives MOSI and the slave drives MISO on one edge, both are sampled on the
   other — so it is one bit counter feeding two shift registers. Trivial logic; the
   cost lands entirely downstream in bandwidth and record format.
2. **No errors to report.** No ACK, no CRC, no parity, no stuffing, no framing rules.
   Everything `can_frame_fsm.v` exists to do has no analogue here. A passive SPI
   sniffer can flag exactly two anomalies: a transaction that was not a whole number of
   bytes, and data lost to a full buffer.
3. **Unbounded transactions.** A flash page read is 256+ bytes on the wire, which is
   512 captured bytes. `i2c_sniffer.v`'s 4 × 128-byte ping-pong buffers cannot hold one
   such transaction, let alone ping-pong between them.


## Scope for v1

- **One chip-select line.** Multi-slave buses would need a capture channel per CS or a
  CS-index field in the record; deferred, and it changes the pin plan.
- **All four SPI modes**, configured statically (see "Sampling" — this is not free).
- **MSB and LSB bit order**, configured statically.
- No trigger subsystem yet — that is the shared SignalBench block described in
  `../CLAUDE.md`, not something this decoder owns.

Target for bring-up: any bench SPI device with a known-good traffic pattern. The
`spi_counter_stream/` and `spi_hw_stream/` designs already produce a predictable stream
from a second UPduino, which makes them a convenient first target — a known payload
means a wrong mode or bit-order setting is obvious rather than merely plausible.


## Sampling — clocked on the probed SCK, not oversampled

**The shift registers are clocked by the target's SCK.** Not sampled in the core clock
domain the way `i2c_sniffer.v` treats SCL.

That is a deliberate break from every other design in this repo, and the reason is that
you own neither end of this bus. Oversampling needs roughly 4 core cycles per SCK half
period for a 2-stage synchronizer plus edge detect, so a 12 MHz core caps the probed
bus at **~1.5 MHz**. Real SPI targets run 1–50 MHz. Oversampling would make most of
them unsniffable, and unlike the ESP32 link there is nothing to negotiate — the target
clocks at whatever rate it clocks at.

Shape of it:

```verilog
always @(posedge sck_sample or posedge spi_p_cs) begin
    if (spi_p_cs) begin
        bit_cnt <= 3'd0;                 // CS idle-high = async reset; no SCK edges anyway
    end else begin
        mosi_sr <= {mosi_sr[6:0], spi_p_mosi};
        miso_sr <= {miso_sr[6:0], spi_p_miso};
        bit_cnt <= bit_cnt + 1'b1;
        if (bit_cnt == 3'd7) begin
            pair_hold <= {{mosi_sr[6:0], spi_p_mosi}, {miso_sr[6:0], spi_p_miso}};
            pair_tog  <= ~pair_tog;      // flag the core domain
        end
    end
end
```

Only the toggle crosses into the core domain; `pair_hold` is stable for the next 7 SCK
periods, so the standard 2-FF-plus-edge-detect handoff is safe:

```verilog
reg [2:0] tog_sync;
always @(posedge clk_core) tog_sync <= {tog_sync[1:0], pair_tog};
wire pair_valid = tog_sync[2] ^ tog_sync[1];
```

Note that the bit counter counts *sample* edges, so CPHA falls out for free: with
CPHA=0 the first edge is a sample edge, with CPHA=1 it is a shift edge and the second
is the sample edge. Clock the right edge and the count is identical either way.

### The one real gotcha: a configurable sample edge means a clock mux

The sample edge is rising when `CPOL == CPHA` and falling otherwise:

| Mode | CPOL | CPHA | Sample edge |
|---|---|---|---|
| 0 | 0 | 0 | rising |
| 1 | 0 | 1 | falling |
| 2 | 1 | 0 | falling |
| 3 | 1 | 1 | rising |

So `sck_sample` is either `spi_p_sck` or its inverse — a **combinational mux on a clock
net**, which glitches if switched while the clock is running. Two ways out:

- **Mux before the global buffer, and latch the mode only while capture is disarmed**
  (recommended). The mode is static configuration, so the restriction costs nothing in
  practice; it just has to be enforced, not assumed.
- **Capture on both edges** into two shift-register pairs and let the core pick. ~32
  extra FFs plus duplicate bit counters — cheap in LCs, and no clock mux at all. Falls
  back to this if the glitch-free switch turns out to be awkward in nextpnr.

Do not skip this and support mode 0 only. Modes 1 and 2 are common enough on real
devices that the decoder would be half-useful.

### Probed SCK must land on a global-input pin

Verified against `docs/iCE40UP-5k-Pinout.xlsx` and the UPduino v3 pinout diagram
(`UPduino-v3.0/docs/source/features/upduino_pinout.png`): on the **SG48 package the
GBIN-capable pins are 20, 35, 37 and 44**. Pin 20 is the 12 MHz oscillator that
`can_sniffer` requires, so **35, 37 and 44 are free**.

Probed SCK goes on one of those. The alternative is `SB_GB` promoting an ordinary pin
from general routing, which works but adds pad→fabric→global delay on the one signal
where delay directly costs you maximum sniffable clock rate. Use a real GBIN pin.


## Record format

**Length-prefixed, not sentinel-delimited.** This is the one place where copying
`i2c_sniffer.v` would be actively wrong: it frames with a leading `0xFF`, and `0xFF` is
the single most common byte on a real SPI bus — undriven MISO with a pullup, flash
command padding, dummy cycles. A `0xFF` sentinel would false-trigger constantly.
`can_sniffer.v`'s fixed 20-byte record does not fit either, because transactions are
variable and unbounded.

```
byte  0          0xAA           start marker
byte  1          flags          {cpol, cpha, lsbf, partial, dropped, resid[2:0]}
bytes 2–3        n_pairs        16-bit big-endian, count of COMPLETE byte pairs
bytes 4–7        timestamp      32-bit big-endian, core-clock ticks, latched at CS assert
bytes 8 …        mosi0, miso0, mosi1, miso1, …   (2 × n_pairs bytes)
last byte        0x55           end marker
```

**Header 8 bytes + 2N payload + 1 trailer.** The length field is authoritative; `0xAA`
and `0x55` are a resync aid and an integrity check — if the byte following the declared
length is not `0x55`, the host has lost sync and should hunt for the next `0xAA`.

Flags:

- `cpol`, `cpha`, `lsbf` — the configuration this capture actually ran under, so a
  mis-set mode is visible in the data rather than inferred from memory.
- `partial` — the transaction was not a whole number of bytes, with `resid[2:0]`
  carrying the 1–7 leftover bits. Genuinely useful: it catches 12-bit ADCs and similar
  non-multiple-of-8 devices, and it catches a probe that is missing clock edges.
- `dropped` — at least one record was lost to a full buffer before this one. Same
  convention as `can_sniffer.v`.

Interleaving the pairs rather than keeping two separate streams makes simultaneity
structural: `mosi[k]` and `miso[k]` were on the wire during the same eight clock edges,
by construction, with no re-pairing needed at the host.


## Buffering — SPRAM, not EBR

The interleaved pair format lines up with the UP5K's SPRAM exactly. Per
FPGA-TN-02022, the device has **four SPRAM blocks, each 16K × 16** — so one 16-bit word
holds one `{mosi, miso}` pair, one block holds 16,384 pairs, and all four hold **65,536
pairs = 128 KB**, which is also the range a 16-bit `n_pairs` field can describe (max
65,535 — one pair short of the full ring, and not worth a 24-bit field to recover).

(Note the user guide's units are wrong in places — it calls each block "256 KB" and the
total "1024 KB". The arithmetic is 16384 × 16 bits = 256 Kbit = 32 KB per block, 1 Mbit
= 128 KB total. Use the bit counts, not the guide's byte figures.)

Going to SPRAM also leaves the 120 Kbit of EBR free for the decoder, the drain queue and
whatever the trigger subsystem eventually needs — EBR is the contended resource in this
project, SPRAM is currently entirely unused by any design in the repo.

Structure: **one large circular pair buffer plus a small record-descriptor queue**,
draining continuously. Not `i2c_sniffer.v`'s per-transaction ping-pong — a single
transaction can exceed any fixed buffer, so the descriptor records `{start_addr,
n_pairs, flags, timestamp}` and the payload lives in the ring. Drop-on-full at the
descriptor level, so a record is either complete or absent, never truncated mid-payload.

Working references for SPRAM instantiation: `up5k/spram/` and the SPRAM examples in
`ice40_ultraplus_examples/`.


## Throughput budget

Capture produces `SCK / 4` bytes/s (two bytes per eight clocks).

The drain, carried over from `can_sniffer.v`, runs at 6 MHz SCK but costs **19 core
ticks per byte, not 16** — the EBR/SPRAM read takes a cycle to issue, a cycle to land
and a cycle to latch (this is the `S_SETTLE` lesson; read that comment before touching
the drain FSM). So the real drain rate is `12 MHz / 19 ≈ 631 kB/s` — one byte every 19 core ticks,
1.58 µs. (Cross-check: `can_sniffer`'s 20-byte record takes ~32 µs, which is the same
number.)

Break-even: **SCK ≈ 2.5 Mbit/s** on a continuously-clocked target.

Header overhead makes short transactions worse. A 4-byte SPI transaction is 4 pairs =
8 payload bytes plus 9 bytes of header and trailer — **2.1× overhead**, pulling the
sustainable rate for short back-to-back transactions down to roughly **1.2 Mbit/s**.

This is not a flaw to engineer away, it is the case the trigger subsystem exists for.
`../CLAUDE.md` already scopes the trigger as being "for buses above 1 MHz, where
continuous BLE streaming can't keep up," and a fast SPI target is squarely
trigger-and-record territory. Streaming mode should be honest about its ceiling: wire
`dropped` to an LED the way `can_sniffer` does, and expect it to light on a busy flash
bus.

The 128 KB SPRAM ring is what buys burst tolerance above the sustained rate — at
10 MHz SCK the ring fills at 1.25 M pairs/s against a drain of ~316 K pairs/s, so it
absorbs **~70 ms** of continuous traffic before overflowing.


## Timestamps

There is no bit clock to count, so unlike `can_sniffer.v` the timestamp counts **core
clock ticks**, latched once at CS assert. At 12 MHz that is 83 ns resolution and 357 s
of range in 32 bits.

Resolution is one core clock, and the counter is in the core domain while CS is in the
probe domain, so the honest accuracy is ±1 core tick. That is enough for
inter-transaction gaps, duty cycle and polling-rate questions. It is *not* enough for
intra-transaction bit timing — that needs a scope.


## What this cannot tell you

Worth stating plainly, because SPI gives a passive observer less than either other
protocol:

- **Nothing about correctness.** No ACK, no CRC, no parity. Every byte is equally
  "valid" as far as the sniffer is concerned.
- **Whether MISO was driven at all.** An undriven MISO line reads as whatever the
  pullup or leakage produces — typically `0xFF` or `0x00`. There is no way to
  distinguish "the slave sent `0xFF`" from "nobody was driving." With a single CS this
  is mostly moot (MISO should be driven whenever CS is low), but it bites on any bus
  where another master or slave is present.
- **Word size and intent.** A 24-bit register write is three bytes to this decoder.
  Meaning is device-specific and belongs in the host-side tooling.
- **A wrong mode setting, directly.** Sampling on the wrong edge usually yields
  plausible-but-shifted data rather than obvious garbage, and `partial` will not catch
  it. This is the main reason the mode bits are echoed into every record's flags.


## Pin plan

FPGA pin numbers, as the `.pcf` requires — these are **not** UPduino header positions.
Confirm header positions against the pinout diagram before wiring, the way
`can_sniffer.pcf` documents its own mapping.

```
spi_p_sck      35    # probed SCK — MUST be GBIN (20/35/37/44; 20 is the CAN oscillator)
spi_p_cs       31    # probed CS, active low
spi_p_mosi     34    # probed MOSI
spi_p_miso     43    # probed MISO
spi_mosi       21    # drain, carried over from i2c_sniffer.pcf
spi_cs         19    # drain
spi_sck        11    # drain
spi_cs_flash   16    # driven high to disable the on-board flash
led_r          41
led_g          39
led_b          40
```

Probe pins are chosen from the left header block so a single ground lead reaches all
four; `can_sniffer.pcf` notes GND at header position 42. Pins 37 and 44 are left free
deliberately — they are the remaining global inputs, and a future decoder may need one.

Clocking: `SB_HFOSC` divided to 24 MHz is fine for this design in isolation, since the
core clock no longer gates the sniffable rate. But if this is ever integrated with
`can_sniffer`, the core must be the 12 MHz oscillator — see `../can_sniffer/clock_choice.md`.
The throughput figures above assume 12 MHz, which is the conservative case.


## Module breakdown

| File | Role |
|---|---|
| `spi_bit_capture.v` | SCK-domain shift registers, bit counter, CDC into the core domain |
| `spi_frame_fsm.v` | transaction assembly, partial-byte detection, descriptor emit |
| `spi_sniffer.v` | top: SPRAM ring, descriptor queue, drain FSM, LEDs |

Split rather than a single file like `i2c_sniffer.v`, for the same reason `can_sniffer`
is split: `spi_bit_capture.v` is the piece with a clock domain crossing in it, and it
deserves its own testbench driving real SCK at rates the core clock cannot follow.


## LEDs

Use `SB_RGBA_DRV`, not raw pin driving — see the note in `../can_sniffer/can_sniffer.md`
about uneven brightness across the three channels. Sticky ~200 ms latch so a single
event is visible.

| Colour | Meaning |
|---|---|
| Blue | CS asserted — bus active |
| Green | transactions completing and being queued |
| Yellow | partial byte seen (bit count not a multiple of 8) |
| Red | record dropped — buffer full, target outrunning the drain |


## Testing plan

Mirror `can_frame_fsm_tb.v`'s approach: the testbench should be a **behavioral SPI
master built from scratch**, driving real SCK/CS/MOSI/MISO edges, so it is a genuine
cross-check rather than a round-trip against the decoder's own assumptions.

Coverage it needs to reach:

- All four modes, both bit orders
- Transactions of 1, 2, 8, 256 and 65535 pairs
- A partial transaction (CS rising mid-byte) at each of the 7 residual bit counts
- Back-to-back transactions with minimal CS-high gap
- SCK faster than the core clock — the whole point of the SCK-domain design, and
  unreachable by any existing testbench in this repo
- Descriptor-queue overflow and the `dropped` flag
- A transaction that wraps the SPRAM ring

**Logic analyzer settings for the drain:** MSB first, 8-bit words, SPI mode 0, CS active
low for framing, ~6 MHz. Every record starts `0xAA`; the byte after the declared payload
length must be `0x55`. Because `0xAA` bit-reversed is `0x55`, a wrong bit-order setting
on the *analyzer* shows the markers swapped rather than garbled — the same unambiguous
signal `can_sniffer` relies on.

For the probed side, the useful bring-up trick is a **known payload**: point the probe
at `spi_counter_stream/` running on a second UPduino. An incrementing byte stream makes
a wrong sample edge visible immediately as a shifted or duplicated sequence, where
arbitrary traffic would look plausible.


## Open questions

- **Mode auto-detection.** Plausible in principle — watch the idle level of SCK for
  CPOL, and whether MOSI changes before or after the first edge for CPHA — but
  unreliable on a bus that idles between transactions. Static configuration for v1;
  revisit if setting it by hand proves annoying in practice.
- **Transactions larger than the ring.** Split into multiple records with a
  continuation flag, or truncate and set `dropped`? Truncation is simpler and probably
  right, since a 128 KB single transaction is pathological, but it should be a decision
  rather than an accident.
- **Whether to store MISO at all when it is known to be unused.** A MOSI-only mode
  would halve the bandwidth for write-only devices (displays, DACs). One config bit,
  and it doubles the effective sniffable clock rate for that case.
- **Multi-CS.** Deferred for v1 by decision, not oversight. Revisit when a bench bus
  with more than one slave actually exists.
