# SPI Stream Reliability Changes

## Summary

This branch makes the synthetic SPI stream generators preserve a consecutive
test sequence when their transmit FIFO becomes full. It also expands the
testbenches and documentation to describe FIFO backpressure accurately.

These changes apply to:

- `spi_counter_stream`: selectable ascending ASCII / `Hello World!` stream
- `spi_hw_stream`: repeating `Hello World!` message

## Problem

Both generators previously advanced their source on every sample tick, even
when the FIFO was full. The full FIFO rejected the new byte, but the source
continued advancing. Once the buffered data drained, the master observed a
forward jump that looked exactly like an SPI byte-loss problem.

This was especially confusing for the counter test, whose purpose is to make
real missing, repeated, or reordered SPI bytes easy to detect.

The iCE40 hard-SPI design also has limited timing margin near 10 MHz when its
general-fabric pins are used. The companion ESP32 firmware defaults to a 2 MHz
SPI clock. Hardware captures showed that explicitly selecting the phase-1
sample point increased one-to-zero MISO errors, so the firmware now retains
the ESP32-H2 default sample point and restores a one-clock CS setup interval.
Its 64-byte polling interval is reduced from 20 ms to 10 ms, increasing nominal
read capacity from 3.2 kB/s to 6.4 kB/s for the 3 kB/s test source.

## Changes

- Advance `data_counter` only when a counter byte is accepted into the FIFO.
- Advance `msg_idx` only when a message byte is accepted into the FIFO.
- Treat a full FIFO as backpressure that pauses the synthetic source.
- Preserve the red LED indication while the source is stalled.
- Update both READMEs to distinguish source backpressure from transport loss.
- Add simulation assertions that the source does not advance while full.
- Restore the ESP32-H2 default sample point after phase-1 regressed hardware captures.
- Drain hard-SPI RX before TX refills to prevent command receive overruns.
- Add command turnaround time and bounded mode-command retries on the MCU.
- Add ready/mode command decoding and follow-up transaction ACKs.
- Map SPI mode to ascending printable ASCII and DIO mode to `Hello World!`.

With this behavior, a counter jump or skipped message character is evidence
of a transport or hard-SPI issue rather than an intentional generator drop.

## Verification

Both behavioral simulations pass with Icarus Verilog:

```text
PASS: all spi_counter_stream tests completed successfully
PASS: all spi_hw_stream tests completed successfully
```

The simulations cover normal sequencing, FIFO-full backpressure, source
stalling, buffered-data integrity, and continuity across chip-select toggles.

## Remaining Limitation

`spi_counter_stream` now decodes the ready command and the temporary SPI/DIO
test-pattern mode commands. `spi_hw_stream` remains stream-only and ignores
MOSI. A mode is considered selected only after the command-capable image
returns a valid ACK in the follow-up transaction.

The behavioral `SB_SPI` model is not silicon-accurate. Final validation still
requires synthesizing the bitstream and checking the stream on the target
board at 2 MHz.
