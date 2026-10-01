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
Its polling interval is 10 ms. Length-prefixed 64-byte responses now carry up
to 58 payload bytes, giving 5.8 kB/s capacity for the 3 kB/s test source.

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
- Add shared `53 42 52 <maximum>` READ / `53 42 44 <count>` response framing.
- Snapshot each batch's valid-byte count and pop only its reserved payload.
- Keep idle `FF` outside the payload without filtering real binary bytes.
- Make ready commands available in both stream images; mode select remains
  counter-only. The MCU no longer switches modes with BOOT or at startup.

With this behavior, a counter jump or skipped message character is evidence
of a transport or hard-SPI issue rather than an intentional generator drop.

## Verification

Both behavioral simulations pass with Icarus Verilog:

```text
PASS: all spi_counter_stream tests completed successfully
PASS: all spi_hw_stream tests completed successfully
```

The simulations cover normal sequencing, FIFO-full backpressure, source
stalling, buffered-data integrity, continuity across chip-select toggles,
empty/short/full packets, maximum request clamping, binary payloads, and
control ACKs that do not consume FIFO bytes. Both FPGA images also build
through synthesis, placement/routing at the Makefile's 30 MHz target, and
bitstream packing. MCU parser sanitizer tests and the ESP-IDF build pass.

## Remaining Limitation

Both stream images decode READ and ready commands. `spi_counter_stream`
also supports temporary SPI/DIO test-pattern mode commands; `spi_hw_stream`
ignores mode select. Neither implements I2C/CAN/UART capture selection. See
[`SPI_PACKET_PROTOCOL.md`](SPI_PACKET_PROTOCOL.md) for the wire contract.

This format is incompatible with old raw-stream MCU readers. There is no CRC
or retransmission: aborted reads and physical bit errors are not repaired.

The behavioral `SB_SPI` model is not silicon-accurate. Final validation still
requires programming the matching bitstream/MCU firmware and checking packet
counts, prefix bytes, and stream continuity on the target board at 2 MHz.
