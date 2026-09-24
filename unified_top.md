# unified_top — merging the four sniffers under one central state machine

How `i2c_sniffer`, `uart_to_spi`, `spi_sniffer` and `can_sniffer` become one bitstream
with a central state machine driven by the ESP32-H2 over SPI: clock architecture,
decoder selection, resource budget and pin plan.

**Status: design only. No unified RTL exists yet.** The numbers below are *measured* —
they come from synthesizing and placing a probe design that instantiates a realistic
superset of the logic — but the probe is not the real top module, and its I2C and SPI
front ends are compact re-implementations rather than the final decoders. Treat the
resource and timing figures as well-founded budgets, not as results from shipping code.

This document complements two others and does not repeat them:

- [`state_management.md`](state_management.md) — the SPI command protocol between MCU
  and FPGA (opcodes, transaction traces, trigger loading).
- [`can_sniffer/clock_choice.md`](can_sniffer/clock_choice.md) — why CAN needs the
  12 MHz crystal. Everything here builds on that conclusion rather than revisiting it.


## The problem

The two complete sniffers cannot currently coexist, for a reason that has nothing to do
with logic capacity:

| Design | Core clock | Source |
|---|---|---|
| `i2c_sniffer.v` | 24 MHz | `SB_HFOSC` (48 MHz) ÷ 2 |
| `can_sniffer.v` | 12 MHz | board crystal, gpio_20, jumper R16 |
| `uart_to_spi.v` | 48 MHz | `SB_HFOSC` |

`clock_choice.md` establishes that CAN *cannot* run from `SB_HFOSC` at all — ISO
11898-1 clause 11.3.2.5 imposes `df < 0.98%` and `SB_HFOSC` is spec'd at ±10%
commercial. So the crystal is non-negotiable, and every other decoder has to move onto
it.

Capacity turned out never to be the constraint. The real constraints are **timing
closure** and **global-buffer-input pins**.


## Clock architecture

### One core domain at 24 MHz, PLL-multiplied from the crystal

```
   12 MHz crystal (pin 20, jumper R16)
          |
          v
   SB_PLL40_CORE  (DIVR=0, DIVF=63, DIVQ=5, FILTER_RANGE=1, VCO 768 MHz)
          |
          v
      24 MHz  clk  -- everything below runs here
```

**A PLL multiplies frequency exactly.** The output is `f_in × (DIVF+1) / ((DIVR+1) ×
2^DIVQ)`, an exact rational ratio, so the *fractional* error of the output equals the
fractional error of the input. The crystal's ±50 ppm is preserved intact, and CAN's
`df < 0.98%` budget still clears by roughly 200×. The PLL adds jitter, but jitter is
random per-cycle and does not accumulate into frequency error across the 13-bit-time
span that equation (4) in `clock_choice.md` is derived against.

Verified against **FPGA-DS-02008 Table 4.22** (sysCLOCK PLL Timing):

| Parameter | Spec | This design |
|---|---|---|
| `fIN` | 10 – 133 MHz | 12 MHz ✓ |
| `fOUT` | 16 – 275 MHz | 24 MHz ✓ |
| `fVCO` | 533 – 1066 MHz | 768 MHz ✓ |
| `fPFD` | 10 – 133 MHz | 12 MHz ✓ |
| Output period jitter (`fOUT` < 100 MHz) | 0.05 UI p-p | ≈ 2.1 ns |

The divider values are what `icepll -i 12 -o 24` produces.

### Why 24 MHz rather than staying at 12

Nothing about CAN needs 24 MHz — `clock_choice.md` deliberately chose 12 tq/bit at
12 MHz and that decision stands. The reason to double is everything *else*:

- **The MCU link.** The FPGA is SPI *slave*, so the MCU supplies SCK. A soft slave that
  oversamples SCK needs roughly four core clocks per SCK half-period, so the core clock
  sets the ceiling on link speed. (This becomes moot if the hard IP is used — see
  "MCU link" — but it sets the floor for the alternative.)
- **Headroom for the shared back end.** Trigger comparison, record packing and the
  drain all run once per decoded event, and events from four decoders share one
  datapath.
- **I2C and UART are already there.** `i2c_sniffer.v` runs at 24 MHz today; moving it
  to 12 would halve its sampling resolution for no benefit.

### CAN runs at BRP=1, keeping its verified bit timing exactly

At 24 MHz with `brp = 1`, one time quantum is 2 core clocks = 83.3 ns, and 12 tq/bit
gives 1 Mbit/s — **numerically identical to the 12 MHz, BRP=0 configuration that was
verified on hardware against the GM6020.** The segment split stays
`SYNC=1, PROP_SEG=4, PHASE_SEG1=4, PHASE_SEG2=3, SJW=3`, the tolerance budget stays
`df < 0.98%`, and no part of the hardware qualification is invalidated.

This is preferable to the obvious alternative of 24 tq/bit at BRP=0. That also works —
simulation showed it slightly *better* than the baseline, breaking at −18000 ppm
instead of −16000 ppm on the worst-case pattern — but it changes every timing parameter
and would need re-qualifying on the bench for no practical gain.

### Three clock domains, not one

"One core domain" is the design goal, but two interfaces are irreducibly asynchronous
and must not be forced into it:

| Domain | What runs in it | Crossing |
|---|---|---|
| **Core, 24 MHz** | all four decoders' back ends, packer, trigger, capture RAM, central FSM, drain | — |
| **Probed SCK** (target's clock, GBIN pin) | SPI sniffer shift registers only — 2 × 8-bit SR, 3-bit counter, pair latch, toggle flag. ~40 flops. | toggle handshake into core |
| **`SB_SPI` internal SCK** (MCU's clock) | the hard IP's own shift register | handled in silicon by the IP |

Everything else — including the SPI sniffer's *record packing, trigger compare, RAM
writes and drain* — is in the core domain. Only the shift registers leave it.


## The timing blocker, and the fix

### Symptom

Placing the merged design at 24 MHz fails. Across six nextpnr seeds:

```
seed 1: 24.63 MHz  PASS      seed 4: 22.98 MHz  FAIL
seed 2: 23.36 MHz  FAIL      seed 5: 23.32 MHz  FAIL
seed 3: 23.28 MHz  FAIL      seed 6: 23.62 MHz  FAIL
```

One seed in six is not a design; it is luck.

### Cause

The critical path is entirely inside [`can_sniffer/can_bit_timing.v`](can_sniffer/can_bit_timing.v),
lines 130–138 — the resynchronization arithmetic:

```verilog
wire [7:0] tseg1_new = tseg1_eff + lengthen;                       // add
wire [7:0] tseg1_use = (do_resync & in_tseg1) ? tseg1_new : tseg1_eff;  // mux
wire [7:0] tseg2_use = (do_resync & in_tseg2) ? tseg2_new : tseg2_eff;  // mux
wire at_bitend = (tq_pos == (tseg1_use + tseg2_use));              // add + compare
```

Compare → add → mux → add → compare, all 8-bit, evaluated combinationally on every
`tq_en`. This is **not new** and not caused by the merge: it already limits the
standalone `can_sniffer` build to 24.25 MHz against its 12 MHz constraint, where the
2× margin hides it. Merging adds routing pressure and pushes it under the line.

### Fix

Precompute the resync candidates into registers. Every input (`tq_pos`, `tseg1_eff`,
`tseg2_eff`) only changes at a tq boundary, so with `brp ≥ 1` — tq of 2 or more core
clocks — the registered values are settled before the next boundary needs them:

```verilog
always @(posedge clk) begin
    tseg1_new_r  <= tseg1_new;
    tseg2_new_r  <= tseg2_new;
    bitend_nom_r <= tseg1_eff + tseg2_eff;
    bitend_rs1_r <= tseg1_new + tseg2_eff;
    bitend_rs2_r <= tseg1_eff + tseg2_new;
    in_tseg1_r   <= in_tseg1;
    in_tseg2_r   <= in_tseg2;
end

wire rs1 = do_resync & in_tseg1_r;
wire rs2 = do_resync & in_tseg2_r;
wire [7:0] bitend_use = rs1 ? bitend_rs1_r : rs2 ? bitend_rs2_r : bitend_nom_r;
wire at_sample = (tq_pos == (rs1 ? tseg1_new_r : tseg1_eff));
wire at_bitend = (tq_pos == bitend_use);
```

The adders leave the critical path; only a mux and a compare remain.

**`brp ≥ 1` is a hard precondition.** At `brp = 0` the registered values are one clock
stale and the module is wrong. This is compatible with the BRP=1 configuration above,
but it must be asserted, not assumed — a simulation guard in the same style as the
existing `SJW > PHASE_SEG1` check belongs here.

### Result

| | LC | fmax, 6 seeds | seeds passing 24 MHz |
|---|---|---|---|
| before | 1340 | 22.98 – 24.63 MHz | 1 / 6 |
| after | 1392 | **29.70 – 32.28 MHz** | **6 / 6** |

52 LCs for ~30% more frequency.

**Behaviour is unchanged.** Running the pipelined module at 24 MHz / BRP=1 / 12 tq
through `can_bit_timing_tb.v`, the worst-case pattern-1 stimulus breaks at the same
offset (−16000 ppm) as the 12 MHz / BRP=0 baseline, and passes to +16000 ppm on both.


## Central state machine

### Decoders are selected by reset, never by clock gating

All four decoders are instantiated permanently and in parallel. A `proto` register
selects which one drives the shared back end; the others are held in **synchronous
reset**:

```verilog
can_bit_timing u_bt (.clk(clk), .rst(rst | (proto != P_CAN)), ...);
```

Clock gating would be the wrong instinct here. On iCE40 a gated clock needs its own
global buffer, and the device has eight — a scarce resource already under pressure
(see "Pin plan"). It also manufactures clock-domain crossings where none need exist,
and glitches on gate transitions. Holding a decoder in reset costs one LUT input per
register bank and removes the problem entirely.

### One event stream

Each decoder is normalized to a common event — a strobe, a payload byte, and a
metadata word — by a combinational mux on `proto`:

```verilog
always @* begin
    case (proto)
        P_CAN : begin ev_strobe = can_strobe; ev_byte = ...; ev_meta = {id, ide, rtr, dlc, ...}; end
        P_I2C : begin ev_strobe = i2c_byte;   ev_byte = i2c_data; ev_meta = {ack, start, stop, ts}; end
        P_UART: begin ev_strobe = u_strobe;   ev_byte = u_data;   ev_meta = {dir, ts}; end
        default:begin ev_strobe = pair_valid; ev_byte = pair[15:8]; ev_meta = {pair[7:0], mode, ts}; end
    endcase
end
```

Downstream of this mux there is exactly one trigger comparator, one record packer, one
capture RAM and one drain path — shared by all four protocols. This is why merging is
nearly free: the expensive shared machinery is built once instead of four times.

### Switching protocol is not instantaneous

A decoder coming out of reset is not immediately trustworthy, and the status register
must reflect that rather than reporting "streaming" the moment `proto` changes:

- **CAN** needs 11 consecutive recessive bits before `bus_idle` asserts and hard
  synchronization can occur. At 1 Mbit/s that is 11 µs *if the bus cooperates*; on a
  busy bus it can be considerably longer. Do not report ready until `bus_idle` has
  been observed.
- **I2C** must wait for a START condition; it cannot join mid-transaction.
- **UART** resynchronizes on the next start bit, but a switch landing mid-character
  will produce one framing error. Discarding the first byte after a switch is cheaper
  than explaining it.
- **SPI** must wait for a CS assertion; joining mid-transaction misaligns every byte
  boundary.

The common thread: **every decoder must join at a protocol boundary, not at an
arbitrary clock cycle.** The central FSM should expose an "armed, waiting for
boundary" state distinct from "streaming".


## The SPI sniffer's separate clock domain

This is the one decoder that does not sample in the core domain, and the reasoning is
in [`spi_sniffer/spi_sniffer.md`](spi_sniffer/spi_sniffer.md); it is summarized here
because it shapes the top-level clock plan.

Oversampling the target's SCK in the core domain needs roughly four core cycles per SCK
half-period, capping the sniffable bus at a few MHz. Real SPI devices run 1–50 MHz, and
unlike the MCU link there is nothing to negotiate — the target clocks at whatever rate
it clocks at. So the shift registers are clocked from the probed SCK pin directly.

**Measured: the probed-SCK domain places at 110–133 MHz**, comfortably past any
realistic target.

What crosses into the core domain is a single toggle bit. The captured pair is stable
for the next seven SCK periods, so a 2-FF-plus-edge-detect read is safe:

```verilog
reg [2:0] tog_sync;
always @(posedge clk) tog_sync <= {tog_sync[1:0], pair_tog};
wire pair_valid = tog_sync[2] ^ tog_sync[1];
```

### The probed SCK does not reach the capture RAM

Worth stating plainly, because it is easy to assume otherwise: **the probed SCK clocks
the shift registers and nothing else.** Trigger comparison, record packing, capture-RAM
writes and the drain are all in the 24 MHz core domain, for every protocol including
SPI.

```
probed-SCK domain              |  core domain, 24 MHz
-------------------------------|------------------------------------
shift MOSI/MISO bits           |
at bit 7: latch pair, flip tog --> 2FF sync -> pair_valid
                               |  -> trigger compare
                               |  -> record packer
                               |  -> capture RAM write
                               |  -> drain
```

Two reasons it has to be this way. The capture RAM is **shared by all four decoders**,
so it cannot be clocked by a signal that only one of them has. And the probed SCK is a
**gated** clock — it stops between transactions and does not exist at power-on — so
nothing that must make progress on its own can live in that domain.

Three distinct SPI clocks exist in this design and conflating them causes real
confusion:

| Clock | Direction | Drives |
|---|---|---|
| **probed SCK** | in, from target bus | SPI sniffer shift registers only |
| **MCU SCK** | in, from ESP32-H2 | `SB_SPI` internal shift register only |
| **drain SCK** | out, FPGA as master | generated by division from the core clock (`can_sniffer`, `uart_to_spi`) |

### What actually limits sniffable SPI rate

The 110–133 MHz figure is the **shift registers'** placed fmax. It is not the
end-to-end sustained capture ceiling, which is set by the core domain: one byte pair
arrives every 8 SCK periods and needs a few core clocks to be handshaked across and
written to RAM.

```
8 / f_sck  >=  N / f_core        N = core clocks consumed per pair
```

With N ≈ 3 (toggle edge detect, then two RAM writes, partly overlapped), a 24 MHz core
sustains roughly **60 MHz** of probed SCK. Both numbers sit above any realistic target,
so the core clock is not the binding constraint — but it is the one that binds *first*,
and it is the number to recompute if the record format grows or the core clock drops.

Note this applies only while a transaction is in flight; the per-transaction header and
trailer cost core clocks once, not per pair.

### Capture both edges rather than muxing the clock

`spi_sniffer.md` presents two ways to handle the configurable sample edge (rising when
`CPOL == CPHA`, falling otherwise): a glitch-free mux on the clock net, or duplicate
shift registers on both edges with the core selecting. It recommends the mux and treats
both-edges as a fallback.

**The recommendation should be inverted.** Both-edges capture was built into the probe
and the whole design still places at 24% with the SCK domain at 110+ MHz — the extra
flops are lost in the noise at this scale. In exchange it removes a combinational mux
from a clock net entirely, and removes the operational rule that mode may only be
changed while capture is disarmed. A rule that must be enforced in both FPGA logic and
MCU firmware is worth more than ~32 flops.


## MCU link: use the hard IP

The FPGA is SPI slave to the ESP32-H2. Three implementations were compared:

| Approach | LC (merged design) | fmax | SCK ceiling |
|---|---|---|---|
| soft slave, oversampled ([`spi_slave/spi_slave.v`](spi_slave/spi_slave.v)) | 1392 | 29.7 – 32.3 MHz | ~core/4 |
| **`SB_SPI` hard IP** | **1278** | **33.1 – 35.3 MHz** | set by service loop, ~10 MHz |
| soft slave clocked by SCK | not measured | — | very high, but costs a global buffer and a gated-clock domain |

**`SB_SPI` wins on every axis**: 114 LCs cheaper *and* ~2 MHz faster than the soft
version, because the oversampling logic and its edge detectors disappear into silicon.
It shifts on `SCKI` internally and presents a system-bus interface on the core clock,
so the crossing is the vendor's problem rather than ours. The device has **two**; only
one is needed.

Two constraints follow from **FPGA-TN-02011**:

- **The dummy byte is mandatory, not a design choice.** Table 12.8: *"When operating as
  a Slave, SPITXDR must be written when SPISR[TRDY] is '1' and at ... TXEDGE = LSBF = 0,
  SPITXDR must be written prior to the CCLK rising edge used to sample the LSB (bit 0)
  of the previous byte. **This timing requires at least one protocol dummy byte be
  included for all slave SPI read operations.**"* The command-byte-then-response protocol
  in `state_management.md` must allow for this.
- **`SPICR2[SDBRE]` answers the streaming-backpressure question.** It emits `0xFF` until
  the first `SPITXDR` write, then a single `0x00` marker, then real data — giving the
  MCU a deterministic sync point without either a dedicated data-ready GPIO or a
  leading valid byte on every poll. Both options listed as undecided in
  `state_management.md` step 7 are superseded by this.

The refill deadline (~7 bit times after TRDY) is serviced from the core clock. At
24 MHz a poll + RX drain + TX load is roughly 9 core cycles ≈ 375 ns, which fits under
a 7-bit-time window down to about 10 MHz SCK.


## Resource budget

All figures from `nextpnr-ice40 --up5k --package sg48`.

### Reference points

| Build | LC | % | EBR | Notes |
|---|---|---|---|---|
| `i2c_sniffer` standalone | 408 | 7% | 1 | incl. its own packer + drain |
| `can_sniffer` standalone | 1292 | 24% | 1 | incl. its own packer + ring + drain |
| `spi_counter_stream` standalone | 1195 | 22% | **0** | see "Elsewhere in the repo" |

### The merged design

Probe containing: CAN chain, I2C front end, **two** `uart_rx` instances (target TX and
RX, sharing one baud generator, direction-tagged as `uart_to_spi.v` does with its
`0x01`/`0x02` flag), SPI sniffer with both-edge SCK-domain capture, `SB_SPI` MCU link,
config registers, 4-byte rolling trigger comparator, record packer and capture RAM.

```
ICESTORM_LC   : 1274 / 5280   24%
ICESTORM_RAM  :    1 /   30    3%
ICESTORM_PLL  :    1 /    1  100%
SB_SPI        :    1 /    2   50%
SB_GB         :    8 /    8  100%   <- see note
SB_IO         :   19 /   96   19%

Max frequency 'clk'   (core)      : 31.96 – 33.83 MHz   PASS at 24 MHz  (4 seeds)
Max frequency 'sck_g' (probed SPI): 110.31 – 132.93 MHz PASS            (4 seeds)
```

**The merged design is smaller than `can_sniffer` alone was**, because the packer, RAM
and drain are built once rather than per-protocol. Roughly 4000 LCs and 29 EBRs remain
free — ample for the trigger subsystem, deeper buffering, and the decoders growing into
their real implementations.

**`SB_GB 8/8` is not a shortage.** Only one is a real clock net; nextpnr opportunistically
promotes high-fanout reset and clock-enable nets into otherwise-idle buffers. It will
promote fewer when a genuine clock needs one. The probe's second real clock (`sck_g`)
placed without difficulty.


## Pin plan

### Global buffer inputs are the scarce resource

Testing `SB_GB_IO` on every UPduino v3 GPIO, the **GBIN-capable pins on SG48 are
20, 35, 37, 44 and 23.** (`spi_sniffer.md` lists the first four; **pin 23 is missing
from it**.)

**Instantiating the PLL consumes pin 35.** The PLL BEL sits in that I/O site, and per
FPGA-TN-02052 §5.1.1 the GPLL pin can then only drive the PLL reference. Verified:

```
PLL + SB_GB_IO on pin 35 : ERROR: PLL 'u_pll' couldn't be placed anywhere
PLL + SB_GB_IO on pin 37 : OK
PLL + SB_GB_IO on pin 44 : OK
PLL + SB_GB_IO on pin 23 : OK
```

So `spi_sniffer.md`'s "35, 37 and 44 are free" holds only in a design without a PLL.
The actual budget:

| Pin | Use |
|---|---|
| 20 | 12 MHz crystal → PLL reference (required by CAN) |
| 35 | **reserved — consumed by the PLL, must stay unassigned** |
| 37 | probed SPI SCK |
| 44, 23 | free GBIN — keep them free; do not spend on ordinary outputs |

### Working assignment

This placed and met timing; it is a starting point, not a fixed pinout.

```
clk_12       20    GBIN, crystal via R16      mcu_sck      11
p_sck        37    GBIN, probed SPI SCK       mcu_mosi     21
can_rx       38                               mcu_cs       19
scl_pin      42                               mcu_miso     13
sda_pin      36                               mcu_irq      -- (not 44)
uart_tx_pin  43    target TX                  spi_cs_flash 16
uart_rx_pin  34    target RX                  led_r/g/b    41/39/40
p_mosi/p_miso/p_cs  31/32/28
```

19 of 96 I/O. Note `mcu_irq` should *not* sit on pin 44 — that wastes a GBIN pin on an
output.


## Elsewhere in the repo

Findings outside the scope of this document, recorded because they affect the budget.

- **[`common/util.v`](common/util.v) `fifo` cannot infer block RAM.** It does
  `assign read_data = buffer[read_ptr]` — an *asynchronous* read, which iCE40 EBR
  cannot perform (EBR reads are registered). Yosys therefore builds the array from
  flops. This is why `spi_counter_stream` places at 1195 LC with **`ICESTORM_RAM: 0/30`**:
  its 64-entry FIFO is ~500 LCs of registers where one EBR would hold 512 bytes.
  Registering the read output would fix it, at the cost of one cycle of read latency
  that every caller would need to absorb.

- **[`spi_slave/spi_slave.v`](spi_slave/spi_slave.v) is one flop short on its
  synchronizers.** `spi_clk_reg` and `mosi_reg` are 2 bits, and the edge detect reads
  bit `[0]` — the *first* flop off an asynchronous pin — so a metastable sample can
  reach the state machine. `i2c_sniffer.v` (`scl_sync[2:0]`) and `can_bit_timing.v`
  (`rx_ff[2:0]`) both use the correct 3-flop pattern. Moot if `SB_SPI` is adopted.

- **SPRAM is unused and large.** FPGA-DS-02008: the UP5K has **four 256 kb SPRAM
  blocks — 1024 kb / 128 KB** — *in addition* to up to 120 kb of EBR. `state_management.md`
  and the capstone report size the capture buffer against the 120 kb EBR alone.
  Recording depth could be roughly 9× deeper. `spi_sniffer.md` already targets SPRAM;
  the shared capture buffer should too.


## Open questions

- **Baud/bit-rate configuration is per-protocol but the register is shared.** UART baud
  divisor, CAN BRP and SPI mode are different quantities; whether they get one register
  interpreted per protocol or separate registers is undecided.
- **Trigger comparison against what.** The probe compares a rolling window against the
  `ev_byte` stream, which is right for UART and I2C but questionable for CAN, where the
  interesting match is an identifier field rather than a payload byte position.
- **Whether the shared capture RAM should be EBR or SPRAM**, and whether streaming and
  recording share it or get separate buffers.
- **What the status register reports during the join-at-boundary window** for each
  protocol, and whether the MCU polls or is interrupted.
- **Real decoder sizes.** The probe's I2C and SPI front ends are compact
  re-implementations; the final modules will be larger. The 4000 LCs of headroom makes
  this low-risk, but the 1274 figure will grow.
- **`SB_SPI` on hardware.** Every hard-IP design in this repo is simulation-only, and
  the testbenches stub `SB_SPI` behaviorally — that model is not silicon-accurate. The
  achievable SCK on fabric-routed pads (rather than the dedicated config pads) is an
  open hardware question these numbers do not answer.


## Reproducing the measurements

The probe designs are not checked in. To regenerate the key numbers:

```sh
# Standalone reference points
cd i2c_sniffer && make build      # 408 LC
cd can_sniffer && make build      # 1292 LC, fmax 24.25 MHz at 12 MHz constraint

# Confirm the CAN critical path is can_bit_timing.v:130-138
nextpnr-ice40 --up5k --package sg48 --json can_sniffer.json \
  --pcf can_sniffer.pcf --asc /dev/null --freq 12 2>&1 | grep 'can_bit_timing.v'

# PLL divider values
icepll -i 12 -o 24                # DIVR=0 DIVF=63 DIVQ=5 FILTER_RANGE=1

# Pipelined bit timing against the baseline (expect the same breakpoints)
iverilog -g2012 -P can_bit_timing_tb.CLK_MHZ=24.0 -o bt.out <pipelined>.v <tb with brp=1>.v
for p in 0 16000 -16000 -18000; do vvp bt.out +pattern=1 +offset_ppm=$p; done
```

Seed sweeps matter: a single nextpnr run is not evidence of closure. Use
`--seed 1..6` and require every seed to pass.


## Sources

- **FPGA-DS-02008** (iCE40 UltraPlus Family Data Sheet) — Table 4.22 sysCLOCK PLL
  timing; Table 4.11 HFOSC tolerance; SPRAM block count.
- **FPGA-TN-02052** (iCE40 sysCLOCK PLL Design and User Guide) — §5.1 PLL placement
  rules, GPLL pin restriction.
- **FPGA-TN-02011** (Advanced iCE40 I2C and SPI Hardened IP User Guide) — Table 12.8
  SPITXDR slave write deadline and dummy-byte requirement; SPICR2[SDBRE].
- **ISO 11898-1:2015** clause 11.3.2.5 — oscillator tolerance, via `clock_choice.md`.
- `docs/` is gitignored and local-only; cite by filename and clause, not by quoting at
  length.
