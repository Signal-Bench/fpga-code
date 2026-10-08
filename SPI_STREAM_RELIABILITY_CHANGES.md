# SPI Stream Reliability Changes

## Raw Contract

Both FPGA images remain raw mode-0, MSB-first data sources: printable ASCII
plus LF in `spi_counter_stream`, and `Hello World!` in `spi_hw_stream`.
MOSI is ignored. The MCU alone adds SignalBench TLVs, with no FPGA commands,
headers, counts, ACKs, or new data-ready wiring. See
[`SPI_RAW_STREAM.md`](SPI_RAW_STREAM.md).

## FPGA Transmitter

The hard-IP register feeder is replaced by the shared fabric transmitter in
[`common/spi_raw_tx.v`](common/spi_raw_tx.v). The old tests assumed hard-IP
underflow and preloaded-byte behavior that they could not verify on silicon.
The new implementation makes those boundaries explicit:

- Load a byte at CS assertion or the falling edge after a complete byte.
- Pop the FIFO only at the eighth rising sampling edge.
- Keep a partial byte queued when CS interrupts a transfer.
- Consume nothing when preloading the next byte before CS rises.
- Complete an idle FF byte before accepting data that arrives during it.

All logic runs in the 24 MHz core domain with synchronized SCK and CS. SCK
is limited to 2 MHz, with CS setup/hold/high times of at least four core
clocks. FIFO-full backpressure still pauses the source without advancing
its pattern. Blue now indicates a complete payload byte clocked out; green
indicates the stream is running; red still indicates source backpressure.

## ESP32 And TLVs

The default SCK is 1 MHz with the normal MISO sample point and two clocks of
CS setup/hold. Reads remain 64 bytes every 10 ms. The MCU preserves every
raw byte, including FF padding and binary values, in a complete 83-byte
SPI TLV frame. The app still needs notifications and ATT MTU at least 86.

Output readiness now checks the required SPI MTU, and connection events reset
MTU and congestion state. A rejected BLE enqueue retains the read and retries
an identical frame and sequence before reading again. The polling schedule
resets after pauses/errors instead of issuing rapid catch-up reads. Unready
reads are still drained, with an explicit `skipped` diagnostic counter.

This retries enqueue failures; it does not acknowledge delivery at the phone,
correct electrical errors, or buffer data across disconnects. Idle FF and
binary FF remain indistinguishable without an agreed validity signal.

## Verification

Both bitstreams build through Yosys, nextpnr, and icepack with timing checks.
The actual transmitter RTL is exercised at 1 MHz and 2 MHz with phase offsets.
Tests cover source stalls and wraps, all 256 byte values, partial transfers
after each bit, CS after the last sampling edge, late data after underflow,
repeated one-byte CS transactions, and sustained 64-byte polling.

Sanitizer-enabled MCU tests exercise the production SPI packager, binary byte
preservation, frame limits, invalid TLVs, sequence wrap, disconnected output,
and byte-for-byte identical retries. Run `idf.py build` for the ESP32 firmware.
Program both rebuilt devices and take a fresh capture to validate the physical
link and the complete phone delivery path.
