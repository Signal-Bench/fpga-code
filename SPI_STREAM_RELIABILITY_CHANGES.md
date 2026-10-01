# SPI Stream Reliability Changes

## Current Contract

Both hard-SPI bring-up images send raw FIFO bytes on MISO whenever the MCU
asserts CS and supplies SCK:

- `spi_counter_stream`: ascending printable ASCII from space through `~`, then LF.
- `spi_hw_stream`: repeating `Hello World!`.

Neither image decodes commands or inserts headers, counts, ACKs, or markers.
MOSI is ignored. The counted request/response version was removed because the
intended architecture keeps the FPGA a raw data source and packages data only
on the MCU. See [`SPI_RAW_STREAM.md`](SPI_RAW_STREAM.md).

## Retained Reliability Behavior

The source advances only when its byte is accepted into the FIFO. A full FIFO
pauses the synthetic source rather than overwriting buffered data or creating
counter/message gaps. The red LED continues to indicate this backpressure.
The SPI service loop pops a FIFO byte only when written to the hard-IP TX
register and drains/discards RX bytes without interpreting them.

The ESP32 defaults to 2 MHz with its default sample point and one clock of CS
setup/hold. A phase-1 override previously increased one-to-zero MISO errors.
It clocks 64 raw bytes every 10 ms, giving 6.4 kB/s of wire capacity for the
3 kB/s test source. No READ, ready, or mode control transfers are sent, and
BOOT switching stays disabled.

## MCU Packaging

The ESP32 preserves every received byte and wraps each read in a full SPI TLV
frame. It does not filter `00`, `FF`, CR, LF, periods, or header-like sequences.
A 64-byte raw read produces an 83-byte mobile frame (ATT MTU at least 86).

Without a data-ready signal or another agreed boundary, idle `FF` and binary
`FF` are indistinguishable. ASCII padding dots may remain; filtering them is
not a lossless solution. No data-ready wiring or error correction is added.

## Verification

Behavioral simulations cover startup reads with no command, source stalling,
FIFO-full backpressure, byte order across CS boundaries, ASCII wrap, binary
bytes, idle clocks, and old command patterns on MOSI that must be ignored.
Both stream images build through Yosys, nextpnr, and icepack. MCU sanitizer
tests verify raw SPI TLVs and sequence behavior; the ESP-IDF build also passes.

The behavioral hard-IP model is not silicon accurate. Final validation requires
programming both devices and checking startup/CS behavior and MISO sampling
on the target board at 2 MHz. Raw streaming and TLV framing do not repair
physical bit errors or recover interrupted reads and failed BLE sends.
