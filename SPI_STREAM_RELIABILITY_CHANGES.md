# SPI Stream Reliability Changes

## Summary

This branch sorts out what a full transmit FIFO should mean, which turned out
to be two different answers for two different jobs. It also expands the
testbenches and documentation to match, and adds a third image for the case
where no byte may ever be lost.

These changes apply to:

- `spi_counter_stream`: selectable ascending ASCII / `Hello World!` stream
- `spi_hw_stream`: repeating `Hello World!` message
- `spi_replay_stream`: **new** — preloaded 4 KB EBR buffer, master-paced

## Problem

A full FIFO can be handled two ways, and the right answer depends on what the
image is for:

- **Drop the new byte.** A forward skip appears on the master side. This is
  what a *live bus* does — real traffic cannot be told to wait — and the size
  of the skip measures how far behind the master is. The cost is that a skip
  alone no longer distinguishes "master too slow" from "link lost a byte";
  the red LED is what separates them.
- **Stall the source.** The sequence stays consecutive, so any skip is
  unambiguously a transport fault. But throughput becomes unmeasurable, and it
  models nothing real — no bus waits for its observer.

An earlier revision of this branch switched both generators to stalling. That
made the skip unambiguous but removed the rate measurement, which is what
these two images exist for. Both are now back to **dropping**, and the
lossless case got its own design instead of being retrofitted onto them.

The iCE40 hard-SPI design also has limited timing margin near 10 MHz when its
general-fabric pins are used. The companion ESP32 firmware defaults to a 2 MHz
SPI clock. Hardware captures showed that explicitly selecting the phase-1
sample point increased one-to-zero MISO errors, so the firmware now retains
the ESP32-H2 default sample point and restores a one-clock CS setup interval.
Its 64-byte polling interval is reduced from 20 ms to 10 ms, increasing nominal
read capacity from 3.2 kB/s to 6.4 kB/s for the 3 kB/s test source.

## Changes

**Live-bus images (`spi_hw_stream`, `spi_counter_stream`) — drop-on-full:**

- The pattern source advances every tick regardless of FIFO space; a full FIFO
  drops the new byte and never overwrites or reorders what is queued.
- Red LED pulses on a dropped byte, which is what distinguishes a generator
  drop from transport loss.
- Simulation asserts the source *does* keep advancing while full, and that
  queued data survives the overflow intact and in order.
- Both READMEs state the ambiguity explicitly and point at
  `spi_replay_stream` for the lossless case.

**Command protocol (`spi_counter_stream`):**

- Ready/mode command decoding with follow-up transaction ACKs.
- SPI mode selects ascending printable ASCII, DIO mode selects `Hello World!`.
- Drain hard-SPI RX before TX refills. Prioritizing TRDY could leave `SPIRXDR`
  occupied until the next byte arrived, setting ROE and losing MOSI bytes on
  real silicon.
- The magic-byte matcher re-anchors, so `53 53 42 ...` is still recognised.

**New lossless image (`spi_replay_stream`):**

- 4096 bytes preloaded into inferred EBR from bitstream INIT values; no
  producer, no tick divider, no FIFO, no drop path.
- The replay pointer advances only when the hard IP has taken a byte, so the
  master is the sole flow-control authority and the byte sequence is identical
  at any read rate.
- Models the recording-drain mode in `state_management.md`, where holding is
  correct and dropping is not.

**MCU side:**

- Restore the ESP32-H2 default sample point after phase-1 regressed hardware
  captures, and restore a one-clock CS setup interval.
- Add command turnaround time and bounded mode-command retries.

**Testbench compile fixes** — both existing testbenches were committed in a
state that could not compile, so the PASS results below had not in fact been
produced by the committed code:

- `spi_counter_stream_tb.v` declared `integer matches`. `matches` is a
  SystemVerilog reserved keyword, a hard syntax error under the `-g2012` the
  Makefile passes. Renamed to `match_ok`.
- `spi_hw_stream_tb.v` declared `reg [dut.MSG_IW-1:0]`. Icarus rejects a
  hierarchical reference in a constant width expression. Fixed to `reg [7:0]`.

## Verification

All three behavioral simulations pass with Icarus Verilog:

```text
PASS: all spi_counter_stream tests completed successfully
PASS: all spi_hw_stream tests completed successfully
PASS: all spi_replay_stream tests completed successfully
```

The live-bus simulations cover normal sequencing, FIFO-full overflow,
drop-on-full behavior, buffered-data integrity, and continuity across
chip-select toggles. `spi_counter_stream` also covers the ready/mode ACKs and
both selectable payloads.

The replay simulation covers rate independence (200 kHz / 2 MHz / 4 MHz), a
5 ms idle gap with the pointer held, a full 4096-byte drain checked byte-exact
against an independently computed copy of the record layout, the wrap back to
record 0, and that bytes loaded into the IP exceed bytes read by at most the
pre-load pipeline depth.

Post-change synthesis:

| Design | LC | EBR | Fmax |
|---|---|---|---|
| `spi_counter_stream` | 1326 (25%) | 0 | 30.2 MHz |
| `spi_replay_stream` | 112 (2%) | 8 of 30 (26%) | 48.6 MHz |

`spi_counter_stream` was 1180 LC / 35.6 MHz before the command decoder. It
still passes the 30 MHz `pnr_freq_mhz` constraint, and `clk_core` is 24 MHz, but
the margin is now thin enough to watch.

## Remaining Limitation

`spi_counter_stream` now decodes the ready command and the temporary SPI/DIO
test-pattern mode commands. `spi_hw_stream` and `spi_replay_stream` are
stream-only and ignore MOSI. A mode is considered selected only after the
command-capable image returns a valid ACK in the follow-up transaction.

The behavioral `SB_SPI` model is not silicon-accurate. Final validation still
requires synthesizing the bitstream and checking the stream on the target
board at 2 MHz. Two specific gaps:

- The model has **no leading dummy byte**, while FPGA-TN-02011 Table 12.8
  requires `SPITXDR` to be written ≥ 0.5 SCK periods before the first bit
  reaches SO. On hardware, expect the payload to begin on byte 2.
- The model oversamples SCKI through a 2-flop synchronizer in the 24 MHz
  `SBCLKI` domain, so it cannot represent SCK above ~4 MHz — past that the
  *model* aliases. The ROE fix above is exactly the kind of thing it cannot
  confirm either. Nothing on this branch has been run on the board.
