# Raw FPGA SPI Stream

`spi_counter_stream` and `spi_hw_stream` send raw FIFO bytes over the iCE40
hard SPI slave. There are no READ requests, ready handshakes, mode-select
commands, ACKs, packet headers, or valid-byte counts. MOSI is ignored.

The FPGA continuously produces and queues its source data. The ESP32-H2 is
the SPI master: asserting CS and supplying SCK shifts bytes onto MISO. A
slave cannot transmit without those clocks. Source backpressure pauses the
synthetic generator when the FIFO is full, preserving queued-byte order.

## Wire Contract

- SPI mode 0, MSB first; start at 1-2 MHz.
- One CS-low transfer reads raw data immediately, with zero dummy MOSI bytes.
- The default ESP32 reader clocks 64 bytes every 10 ms at 2 MHz.
- No separate command transfer or command-turnaround delay is needed.
- When the transmit path is empty, the hard IP may shift idle `FF` bytes.

The counter image repeats ASCII space through `~`, then LF. The fixed-message
image repeats `Hello World!` without a terminator. Each produces up to 3 kB/s.
Select an image by programming its bitstream, not by sending a mode command.

## Mobile Frames

Only the ESP32 constructs SignalBench frames: an eight-byte frame header,
Transport SPI (`01 01 01`), CS0 (`20 01 00`), mode0 (`21 01 00`), then SPI
Data (`22 <length> <raw bytes>`). A 64-byte read produces an 83-byte frame
and requires ATT MTU at least 86.

The MCU preserves every received byte, including `00`, `FF`, `2E`, CR, LF,
and sequences resembling old packet headers. It neither parses nor removes
anything from the raw FPGA stream. BOOT switching remains disabled.

Without a data-ready signal or another agreed boundary, idle `FF` cannot be
distinguished from legitimate binary `FF`. Both remain in the mobile payload.
ASCII viewers may show them as dots; never delete literal periods or filter
bytes from a general binary stream. No data-ready wiring is added by this
restoration.

## Verification And Limits

Run `make sim` and `make build` in each stream directory. Behavioral tests
cover startup reads with no command, FIFO-full backpressure, source stalls,
ASCII/message continuity, binary bytes across CS changes, idle clocks, and
old command patterns on MOSI that must not affect the raw output.

The simulation model is not silicon accurate. Verify startup bytes, CS
boundaries, and MISO sampling on hardware at 2 MHz. Neither raw streaming nor
TLV packaging corrects electrical bit errors. An interrupted read, a failed
BLE send, or a disconnected phone is not recovered by this path.

Rebuild and program both FPGA and ESP32 to replace the counted-protocol
version. Building or pushing source does not update either device's firmware.
