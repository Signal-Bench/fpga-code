# Why the CAN sniffer uses the UPduino's 12 MHz oscillator instead of SB_HFOSC

*Written 2026-09-09. Self-contained explainer — hand this to a fresh session and it
should have everything needed to explain or challenge the decision.*

## Short version

Every other design in this repo clocks off `SB_HFOSC`, the iCE40's internal 48 MHz
oscillator. The CAN sniffer cannot, because `SB_HFOSC` is only accurate to **±10%**
and CAN needs better than **±1%**. The fix is on the board already: short jumper
**R16** (silkscreen "OSC") to route the UPduino's on-board 12 MHz oscillator to
`gpio_20`, and clock the design from that.

## Why this problem is new

It never came up before because none of the earlier designs needed an accurate clock.

**I2C carries its own clock.** `i2c_sniffer.v` watches SCL and samples SDA on
`scl_rising`. The FPGA's clock only has to be fast enough to *observe* SCL edges — if
it runs 10% fast or slow, nothing breaks, it just oversamples slightly differently.
There is no frequency accuracy requirement at all.

**CAN has no clock line.** It is NRZ (non-return-to-zero) on a single wire: a bit is
just "the bus was dominant for one bit time." Nothing on the wire tells you when a bit
starts or ends except the data transitions themselves. The receiver has to *reconstruct*
the bit clock from those transitions and then hold that reconstruction accurately enough
to sample every bit in the right place.

That reconstruction is only as good as the local oscillator between transitions. This
is why oscillator accuracy suddenly matters.

## The bit time, and what SJW is

CAN divides each bit into **time quanta** (tq), a tq being some integer number of core
clocks. ISO 11898-1:2015 clause 11.3 splits the bit into four segments:

```
|  SYNC_SEG  |   PROP_SEG   |  PHASE_SEG1  |  PHASE_SEG2  |
|    1 tq    |              |              ^              |
|<---------------- one bit time ---------->|              |
                                    sample point
```

- **SYNC_SEG** — always 1 tq. This is where an edge is *expected* to fall if the
  receiver is perfectly synchronized.
- **PROP_SEG** — compensates for physical signal propagation delay around the bus.
- **PHASE_SEG1 / PHASE_SEG2** — the two "phase buffer" segments. These are the
  adjustable ones.
- **Sample point** — at the end of PHASE_SEG1. The bus level read here *is* the bit's
  value. Everything else is machinery to put this point in the right place.

The receiver keeps a free-running counter through these segments. Because its clock is
never exactly the transmitter's, that counter drifts. CAN corrects the drift two ways:

- **Hard synchronization** — on the falling edge that starts a frame (SOF), the bit
  time is simply restarted at SYNC_SEG. A full reset of phase, used once per frame.
- **Resynchronization** — on every *other* recessive→dominant edge during the frame,
  the receiver measures how far the edge missed SYNC_SEG (the **phase error**) and
  nudges its bit time to compensate:
  - Edge arrived **late** (during PROP_SEG or PHASE_SEG1) → **lengthen PHASE_SEG1**,
    delaying the sample point to catch up.
  - Edge arrived **early** (during PHASE_SEG2) → **shorten PHASE_SEG2**, ending the
    bit sooner.

### SJW — Synchronization Jump Width

**SJW is the cap on how much a single resynchronization is allowed to move the bit
time**, measured in time quanta. If the phase error is 6 tq but SJW is 4, the receiver
only corrects 4 tq this bit and the rest carries over.

Why cap it at all? Because a phase error might not be real drift — it could be a glitch,
ringing on the bus, or noise. Without a cap, one spurious edge could yank the sample
point far enough to corrupt the next several bits. SJW is the trade-off knob:

- **Larger SJW** → tolerates worse oscillator mismatch, but a single bad edge can throw
  the receiver further off.
- **Smaller SJW** → more robust to glitches, but demands a more accurate clock.

SJW can never exceed PHASE_SEG1 (you cannot shorten a segment below zero), and in
classical CAN it is conventionally capped at 4. We use **SJW = 4**.

Two more rules bound how often this can happen (ISO 11898-1 clause 11.3.2.1): **at most
one synchronization per bit time**, and **only on an edge where the previous sample
point read recessive**. Both are implemented in `can_bit_timing.v`.

## The tolerance budget

ISO 11898-1:2015 clause 11.3.2.5 defines the allowed oscillator deviation `df` — the
fractional error of a node's clock from nominal — with two conditions for classical CAN
(equations 3 and 4; equations 5 and 6 apply only to CAN FD's data phase):

```
(3)   df  <  SJW / (2 × 10 × bit_time)

(4)   df  <  min(PHASE_SEG1, PHASE_SEG2) / (2 × [13 × bit_time − PHASE_SEG2])
```

with everything in time quanta. The standard also states that **the maximum difference
between any two nodes' oscillators is 2 × df × f_nom** — so if both ends are at the
limit in opposite directions, the relative error can be twice `df`.

Where does the 13 in equation (4) come from? It is the worst-case span, in bit times,
that a frame can go without offering a recessive→dominant edge to resynchronize on. The
tail of a frame — ACK slot, ACK delimiter, 7-bit end-of-frame, 3-bit intermission — is a
long recessive stretch with no usable edge. Drift accumulates unchecked across it.

### Our numbers

The configuration is **12 MHz core, 12 tq per bit** (so tq = 1 core clock = 83.3 ns,
bit time = 1 µs = 1 Mbit/s), split `SYNC=1, PROP_SEG=4, PHASE_SEG1=4, PHASE_SEG2=3`,
sample point at 9/12 = **75%**, SJW = 4:

```
(3)   df < 4 / (2 × 10 × 12)           = 4/240 = 1.67%
(4)   df < min(4,3) / (2 × [13×12 − 3]) = 3/306 = 0.98%   <-- binding
```

**So df < 0.98%.**

## Why SB_HFOSC fails

Lattice **FPGA-DS-02008** (iCE40 UltraPlus Family Data Sheet), Table 4.11:

| Parameter | Min | Typ | Max |
|---|---|---|---|
| HFOSC clock frequency, commercial (tJ 0–85 °C) | **−10%** | 48 MHz | **+10%** |
| HFOSC clock frequency, industrial (tJ −40–100 °C) | **−20%** | 48 MHz | **+20%** |

±10% against a budget of 0.98% is **about 10× over**. Lattice says so directly in
**FPGA-TN-02008** (iCE40 Oscillator User Guide): *"Note that Oscillator cannot provide
accurate frequency"*, and it instructs you to overconstrain a 1:1 PLL to 52.8 MHz —
48 × 1.1 — *"to cover the 10% tolerance of the OSC."*

The failure mode is the nasty kind. It would not refuse to work; it would decode *some*
frames and not others, depending on bit patterns, because how far the receiver drifts
depends on how long it goes between resynchronization edges. Frames with dense
transitions would work and frames with long runs would not. That is an expensive bug to
chase on hardware, which is the main argument for fixing it before writing the decoder.

## The fix: the on-board 12 MHz oscillator

From `UPduino-v3.0/docs/source/tutorials/oscillator.rst` and
`docs/source/features/specs.rst`:

> The UPduino has an on-board oscillator that generates 12MHz... distributed to the
> FTDI, an external pin and also to a global buffer on the FPGA via an optional jumper.
> The 12MHz can be routed to the FPGA directly on the board... by shorting R16. Note
> that this is marked as OSC on the silkscreen.

- Short **R16** ("OSC" on the silkscreen).
- The clock arrives on **`gpio_20`** — pin IOB_25B_G3, a **global clock input**, chosen
  by the board designers because that bank is already fixed at 3.3 V.
- Constraint line: `set_io -nowarn clk_12 20`
- Board errata worth knowing if you instead take the clock off the header: on UPduino
  v3.0 the silkscreen for pins 41 and 42 (GND and 12 MHz) is **swapped**.

A packaged oscillator of this type is typically ±50 ppm = **0.005%**, roughly **200×
inside** the 0.98% budget. The STM32 on the other end of your bench bus is also
crystal-driven, so the relative error between the two nodes will be on the order of
±100 ppm total.

## Why 12 tq per bit, and why not PLL up to 24 MHz

12 MHz ÷ 1 Mbit/s = exactly 12 core clocks per bit, so **1 tq = 1 core clock** with no
prescaling. ISO permits 8–25 tq per bit for classical CAN, so 12 sits comfortably in
range, and 12 tq is an ordinary configuration for real CAN controllers.

Other bit rates fall out of the baud rate prescaler as `rate = 1 MHz / (BRP+1)`, which
hits 1M, 500k, 250k, 200k, 125k, 100k, 50k and 20k exactly.

The alternative was a PLL from 12 MHz to 24 MHz for 24 tq per bit, giving finer
sample-point placement and matching `i2c_sniffer.v`'s existing 24 MHz constants. It was
rejected for v1 because:

- 12 tq already meets the tolerance budget by ~200× once on a crystal.
- It removes an `SB_PLL40_CORE` instantiation and its jitter from the first bring-up.
- Timing closure at 12 MHz is trivial, whereas `i2c_sniffer.v` had to divide 48 MHz down
  to 24 MHz specifically to close timing.

It remains an easy upgrade: the segment lengths are module parameters, so moving to
24 tq is a parameter change plus the PLL, with no change to the frame decoder. **For
the eventual CAN FD upgrade this is likely necessary**, since FD's data phase runs at
2–8 Mbit/s and 12 MHz does not divide usefully into those rates.

## What the simulation actually shows

`can_bit_timing_tb.v` drives the decoder at a deliberately *wrong* bit rate and checks
that every bit is still sampled correctly. `make sweep-timing` runs it across a range of
offsets. Two stimulus patterns:

- **pattern 0** — worst case inside the stuffed region: 5 dominant, 5 recessive, so a
  resync edge every 10 bit times.
- **pattern 1** — worst case across the frame tail: 11 recessive bits with no
  recessive→dominant edge at all, the span equation (4) is derived against.

Measured breaking points on pattern 1:

| Configuration | Sample point | ISO eq. (4) bound | Measured pass range |
|---|---|---|---|
| `1/4/4/3` (chosen) | 75% | 0.98% | −1.4% … beyond +6% |
| `1/3/4/4` | 67% | 1.32% | −2.0% … beyond +6% |

Two things to read from this:

1. **The ISO bound is conservative by roughly 1.4×**, as expected — it has to cover two
   nodes drifting in opposite directions plus propagation delay, while the testbench
   models relative error between one ideal transmitter and us.
2. **The margin is strongly asymmetric.** A transmitter running *fast* (negative offset,
   shorter bits) breaks us first, because with the sample point at 75% there are only
   3 tq of PHASE_SEG2 after it but 8 tq before it.

Rebalancing to `1/3/4/4` buys ~40% more tolerance, and it was **deliberately not
taken**. With a crystal the real relative error is around ±100 ppm — 140× inside even
the tighter configuration's measured limit — so tolerance is simply not the binding
constraint any more. The freedom is better spent on a **later sample point**, which
buys immunity to bus ringing and reflections, and 75% is standard practice.

## One thing that is different because we are passive

The usual rule is `PROP_SEG ≥ 2 × (bus delay + transceiver loop delay + controller
input delay)`. The factor of 2 is a *round trip*: a transmitting node must send a bit,
have it propagate to the far end of the bus, and see the result back at its own sample
point, so that arbitration and bit-error monitoring work.

**A sniffer never transmits.** There is no round trip. Our receive path delay
(transceiver loop delay ~135 ns, plus 2 core clocks through the input synchronizer) is a
*constant* offset, and hard synchronization aligns us to the edge as we actually observe
it — the delay is absorbed, not accumulated. So PROP_SEG here only needs to be large
enough to place the sample point sensibly, not to cover a round trip.

This is worth remembering if this block is ever reused for a transmitting node: the
PROP_SEG value chosen here would be too small.

## Sources

All local, all official:

- `docs/FPGA-DS-02008-2-4-iCE40-UltraPlus-Family-Data-Sheet.pdf` — Table 4.11, HFOSC
  tolerance
- `docs/FPGA-TN-02008-1-8-iCE40-Oscillator-User-Guide.pdf` — "cannot provide accurate
  frequency", the 52.8 MHz overconstraint note
- `docs/ISO 11898-1_2015.pdf` — clause 11.3 (bit timing, segments, synchronization),
  clause 11.3.2.5 (oscillator tolerance equations)
- `UPduino-v3.0/docs/source/tutorials/oscillator.rst`, `docs/source/features/specs.rst`
  — the 12 MHz oscillator, R16/OSC jumper, gpio_20, pin 41/42 errata

Note the ISO copies in `docs/` are AENOR-licensed to Texas A&M and marked no further
reproduction — cite clause numbers in the report rather than pasting text.
