# SPI Stream Packet Protocol

Implemented by `spi_counter_stream` and `spi_hw_stream` through
`common/spi_stream_packet.v`. This replaces their old unframed raw streams.
The matching ESP32-H2 reader is `main/fpga_spi_live` in the SignalBench project.
Program matching FPGA and MCU firmware together.

## Transactions

SPI slave, mode 0, MSB first. The ESP32 supplies SCK and CS. At the default
2 MHz, the MCU polls every 10 ms:

1. Assert CS and send `53 42 52 <maximum>` on MOSI. Ignore simultaneous MISO.
2. Deassert CS and wait at least 20 us for command decoding.
3. Assert CS and clock the response with zero MOSI bytes, then deassert CS.

The default response transfer is 64 bytes; the default command is
`53 42 52 3A` (up to 58 payload bytes).

| Response Field | Bytes |
| --- | --- |
| Prefetched idle prefix | Zero to two `FF` bytes |
| Magic/opcode | `53 42 44` (`SBD`) |
| Count | One unsigned byte, 0 through 58 |
| Payload | Exactly `count` bytes |
| Remaining clocks | `FF` idle padding |

The count snapshots `min(maximum, queued FIFO count, 58)` when the fourth
command byte is decoded. Bytes produced afterward wait for the next request.
Idle TX never pops the FIFO, so no padding can appear inside the declared
payload. A real `FF`, `00`, `2E`, or header-like sequence inside the payload
is ordinary data and must be preserved. An empty queue returns count zero.

For example, a three-byte batch containing `00 FF 2E` may appear as:

```text
FF 53 42 44 03 00 FF 2E FF FF ...
```

The mobile SPI Data TLV is only `22 03 00 FF 2E`; the MCU adds the complete
SignalBench frame header and transport, chip-select, and mode TLVs.

For shorter response reads, request at most `response length - 6` bytes.
Always finish the response before issuing another command. The module pops
data when handed to the hard IP, not when acknowledged by the host. An
aborted read or a corrupted response can therefore lose that batch. There is
no CRC, resend, or transaction identifier; length framing is not bit-error
correction.

## Other Commands

Responses use the same separate follow-up transaction and idle prefix, but
have no count or payload. The MCU clocks 16 response bytes for these commands.

| Command | Response | Images |
| --- | --- | --- |
| `53 42 A5 5A` ready | `53 42 4F 4B` (`SBOK`) | Both |
| `53 42 4D 01` ASCII pattern | `53 42 41 01` | Counter only |
| `53 42 4D 02` Hello World pattern | `53 42 41 02` | Counter only |

The counter mode command clears its FIFO and resets the selected generator.
The fixed-message image ignores mode select. I2C `00`, CAN `03`, and UART `04`
mode IDs are not implemented by these images. The MCU no longer sends a mode
command at boot or changes FPGA modes with the BOOT button; all received
payload is packaged as SPI regardless of the selected test pattern.

## Verification

Run `make sim` and `make build` in each stream directory. Tests cover initial
empty polls, queue snapshots, binary-byte preservation, request limits,
FIFO backpressure, sequential batches, ready ACK isolation, and counter
mode ACKs. The behavioral hard-IP model is not silicon accurate: verify the
idle prefix, complete packet counts, and CS-boundary continuity on the board.
Keep the existing ESP32 default sample point and validate at 1-2 MHz before
increasing SCK. Framing changes do not resolve electrical sampling errors.
