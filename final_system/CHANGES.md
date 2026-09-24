# What changed in each sniffer to build the unified system

Companion to [`../unified_top.md`](../unified_top.md), which explains *why*. This file
is the audit trail: exactly what was touched in existing, working designs, and what was
written fresh.

**The short version: almost nothing was changed.** One existing module gained an opt-in
parameter that defaults to off. Everything else is either new code or unmodified reuse.

---

## can_sniffer — one opt-in parameter, default off

### `can_sniffer/can_bit_timing.v` — MODIFIED

Added `parameter integer PIPELINE_RESYNC = 0`. When 0 (the default) the module is
**bit-for-bit what it was**; the original combinational path is selected by a
`generate` branch.

Why: the resync arithmetic at lines 130-138 is a
compare → add → mux → add → compare chain evaluated every `tq_en`. It caps placed fmax
at ~24 MHz, which was fine at a 12 MHz constraint but is what stops a 24 MHz unified
core clock closing. With `PIPELINE_RESYNC=1` the adder candidates are precomputed into
registers and only a mux and a compare remain.

**`PIPELINE_RESYNC=1` requires `brp >= 1`.** The precomputed values lag `tq_pos` by one
core clock, so they are only settled in time when a time quantum is 2+ core clocks. At
`brp = 0` the module is wrong. A simulation-only guard checks this on `brp` itself
rather than as a parameter assertion, because `brp` is a runtime input and an MCU
writing 0 would otherwise break it silently. `sb_mcu_link.v` additionally clamps
`can_brp` to a minimum of 1 in hardware.

`can_sniffer.v` does not pass the parameter, so it keeps the default and keeps the
exact configuration verified on the GM6020.

**Regression evidence** — after the change, from `can_sniffer/`:

```
make sim   ->  23 PASS, 0 FAIL
               ALL TESTS PASSED (16 records)        # can_frame_fsm_tb
               ALL TESTS PASSED (60 bytes drained)  # can_sniffer_tb
make build ->  1292 LC, PASS at 12.00 MHz           # unchanged
```

The pipelined path was then checked against the original at the configuration the
unified top uses. Identical results, including the same failure detail:

```
12 MHz, brp=0, combinational   pattern 1  -16000 ppm  FAIL (12 bad, first at bit 47)
24 MHz, brp=1, pipelined       pattern 1  -16000 ppm  FAIL (12 bad, first at bit 47)
```

### `can_sniffer/can_bit_timing_tb.v` — MODIFIED

Added `BRP` and `PIPELINE_RESYNC` parameters so the pipelined path is testable. `brp`
was previously hardcoded to `8'd0`; it now comes from the `BRP` parameter, which
defaults to 0. No existing invocation changes behaviour.

### `can_sniffer/can_frame_fsm.v`, `can_crc15.v` — UNMODIFIED

Instantiated directly by `signalbench.v`. Not copied, not forked.

---

## i2c_sniffer — not modified; front end re-implemented

### `i2c_sniffer/i2c_sniffer.v` — UNMODIFIED

Still builds and runs exactly as before.

### `final_system/sb_decoders.v` :: `sb_i2c` — NEW

The capture front end was re-implemented rather than extracted, because the logic worth
reusing is not separable from the parts being replaced. `i2c_sniffer.v` is a complete
top module: oscillator, 3-stage synchronizers, byte assembly, a 4×128-byte ping-pong
RAM, a drain queue and an SPI master. Only the first three are wanted; the shared back
end supplies the rest.

Carried over unchanged in substance: the 3-stage synchronizers, the START/STOP
derivation (`sda_falling & scl_high`, `sda_rising & scl_high`), sampling SDA on SCL
rising, and ACK/NACK on the 9th clock.

Deliberately different:

- **No `0xFF` repeated-START sentinel.** `i2c_sniffer.v` inserts a marker byte into RAM
  because its record is a raw byte stream with nowhere else to put the information.
  Here START/STOP/ACK are flag bits in the record header, so a sentinel that collides
  with real payload data is not needed.
- **STOP emits a zero-length marker event** so a host can bound a transaction without
  scanning for a sentinel.

---

## uart_to_spi — not modified; receivers reused

### `uart_to_spi/uart_to_spi.v` — UNMODIFIED

### `final_system/sb_decoders.v` :: `sb_uart` — NEW wrapper around existing `uart_rx`

Same arrangement `uart_to_spi.v` uses — two `uart_rx` instances from `common/uart.v`
sniffing the target's TX and RX lines, sharing one baud generator. `uart_rx` itself is
reused unmodified.

Differences:

- **Baud rate is runtime-configurable** via the MCU link. `uart_to_spi.v` hardcodes
  9600 through `divide_by_n #(.N(1250))`.
- **Direction moves from a payload byte to a flag bit.** `uart_to_spi.v` sends
  `[0x48 header][0x01 or 0x02][data]`; here the direction is `ev_flags[5]`, so it costs
  no payload space.
- RS-232 and RS-485 reuse this unchanged — they differ only at the physical layer,
  ahead of the transceiver.

**Known limitation:** if both lines complete a byte in the same core clock, TX wins and
the RX byte is dropped. Genuine full-duplex traffic can do this. `uart_to_spi.v` has
the same structure and the same issue.

---

## spi_sniffer — design existed, RTL did not

### `spi_sniffer/spi_sniffer.md` — UNMODIFIED (but two corrections are owed)

The design document was followed. Two things in it need updating; they are described in
`unified_top.md` and have **not** been edited into that file:

1. **GBIN pin list is incomplete.** It gives 20, 35, 37, 44. Testing `SB_GB_IO` on every
   UPduino GPIO shows **pin 23 is also GBIN-capable**. More importantly, "35, 37 and 44
   are free" holds only without a PLL — instantiating `SB_PLL40_CORE` consumes pin 35
   outright, verified by place-and-route failing with
   `PLL 'u_pll' couldn't be placed anywhere`.
2. **"Capture both edges" should be the primary choice, not the fallback.** The document
   recommends a glitch-free clock mux. Both-edges capture was built instead and costs
   almost nothing at this scale, while removing a glitch source from a clock net and
   removing the "only change mode while disarmed" rule that would have to be enforced
   in both FPGA logic and MCU firmware.

### `final_system/sb_decoders.v` :: `sb_spi` — NEW

First RTL for this decoder. Follows the document's core decision: **shift registers
clocked by the probed SCK**, not oversampled in the core domain, with only a toggle
crossing into the core domain and `pair_hold` stable for 7 SCK periods.

Also deliberately different: the record is the shared 32-byte-stride format, not the
length-prefixed variable-length format the document proposes. The document is right
that a `0xFF` sentinel is unusable on an SPI bus, but `0xAA`/`0x55` at a fixed stride
does not have that problem. Revisit if one-record-per-transaction becomes important.

Not yet implemented from the document: LSB-first bit order (the flag is carried in the
record but not acted on), the `partial`/`resid` non-multiple-of-8 detection, and SPRAM
buffering.

---

## spi_slave — not used

### `spi_slave/spi_slave.v` — UNMODIFIED, and not instantiated

`SB_SPI` hard IP is used instead. Measured on the merged design, the hard IP is
**114 LCs cheaper and ~2 MHz faster** than the soft slave, because the oversampling
logic and its edge detectors disappear into silicon.

Separately, `spi_slave.v` has a real defect that should be fixed if it is ever used:
`spi_clk_reg` and `mosi_reg` are 2 bits and the edge detect reads bit `[0]` — the
*first* flop off an asynchronous pin — so a metastable sample can reach the state
machine. `i2c_sniffer.v` (`scl_sync[2:0]`) and `can_bit_timing.v` (`rx_ff[2:0]`) both
use the correct 3-flop pattern.

---

## New files

| File | What it is |
|---|---|
| `signalbench.v` | Top: PLL, decoder instantiation, reset-based selection, event mux, LEDs |
| `sb_decoders.v` | `sb_i2c`, `sb_uart`, `sb_spi` |
| `sb_capture.v` | Shared trigger comparator, record packer, capture RAM, drain pointer |
| `sb_mcu_link.v` | `SB_SPI` slave + **placeholder** command protocol |
| `signalbench.pcf` | Pinout, with pin 35 deliberately unassigned |
| `signalbench_tb.v` | End-to-end test; models `SB_PLL40_CORE`, `SB_GB_IO`, `SB_IO`, `SB_SPI` |
| `Makefile` | `build`, `sim`, `seeds`, `time`, `synth-check`, `flash` |

---

## Bugs found by the testbench

Three real defects, all in code written this session, all found by `make sim` rather
than by inspection. Recorded because each is a trap that would recur.

1. **The triggering event was dropped.** `recording` is set by a non-blocking
   assignment, so it was still 0 during the cycle the trigger matched, and the matching
   event — the single most interesting record in a triggered capture — fell outside the
   capture window. Fixed by including `trigger_now` in `capturing`.

2. **The trigger fired one event late, and byte order was reversed.** The rolling window
   was compared *before* the current byte was shifted in, so a 1-byte pattern matched
   the byte *after* the one configured. Separately, `trig0` was matching the newest byte
   where `state_management.md` specifies it as the first byte on the wire. Both fixed.

3. **The drain pointer ran away.** The service FSM reloads `SPITXDR` whenever `TRDY` is
   set — which is what meets the refill deadline, and happens continuously whether or
   not a transaction is in progress. Advancing the read pointer on the TX load therefore
   walked it through the whole buffer while the bus sat idle, so the first record was
   gone before the MCU asked for it. Fixed by advancing per byte *received* and framing
   on CS.

---

## Results

```
ICESTORM_LC   : 1902 / 5280   36%
ICESTORM_RAM  :    4 /   30   13%
ICESTORM_PLL  :    1 /    1
SB_SPI        :    1 /    2
SB_GB         :    8 /    8        (1 real clock; the rest are opportunistic promotions)
SB_IO         :   19 /   96

yosys:  1189 SB_LUT4, 232 SB_CARRY

Core clock, 6 seeds : 27.74 - 29.98 MHz, all PASS at 24 MHz
Probed SCK domain   : >110 MHz
icetime              : 25.97 MHz, PASSED against the 24 MHz constraint
make sim             : ALL TESTS PASSED
```

## What is NOT verified

- **Nothing here has been on hardware.** Not flashed, not scoped.
- **The MCU protocol is invented.** See the header of `sb_mcu_link.v`. The two *silicon*
  constraints it respects (mandatory dummy byte, ~7-bit-time refill deadline) are from
  FPGA-TN-02011 and are real; the opcodes are not.
- **`SB_SPI` is modelled, not simulated faithfully.** The model is the one from
  `spi_counter_stream_tb.v` and has no leading dummy byte, unlike real silicon.
- **CAN is not exercised end to end in `signalbench_tb.v`.** Building conformant frames
  needs the CRC-15 and stuffing generators that live in `can_frame_fsm_tb.v`; the CAN
  decoder is covered there instead, and that suite still passes.
- **SPI and the `dropped`/buffer-full paths are not exercised** by the testbench either.
