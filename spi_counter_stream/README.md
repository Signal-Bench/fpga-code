# spi_counter_stream - Raw SPI Stream

An iCE40 UP5K/UPduino v3 hard-SPI (`SB_SPI`) slave bring-up image.
The fixed source repeats printable ASCII from space (20) through tilde (7E),
then LF (0A).
Bytes are produced at up to 3 kHz into a 64-entry FIFO. FIFO-full backpressure
pauses the synthetic source without advancing or overwriting queued data.

## Raw Transfers

The master asserts CS and supplies mode-0, MSB-first SPI clocks. FIFO bytes
feed the hard-IP TX register directly and appear on MISO without READ commands,
ready handshakes, mode select, ACKs, headers, or valid-byte counts. MOSI is
ignored; the service loop drains received bytes only to keep the RX register
clear. The FPGA cannot shift data without master clocks.

For example: `20 21 22 23 24 ... 7E 0A 20 21 ...`.
To use the other pattern, program the `spi_hw_stream` image instead;
there is no runtime switching or FIFO reset command.

The ESP32 alone wraps raw reads in complete mobile SPI TLV frames. See
[`SPI_RAW_STREAM.md`](../SPI_RAW_STREAM.md) for the wire/MCU contract.

## Wiring

| Signal | FPGA Pin | Direction | Meaning |
| --- | --- | --- | --- |
| SCK | 11 | In | Master clock |
| CS | 19 | In | Active low |
| MOSI | 21 | In | Ignored |
| MISO | 13 | Out | Raw FIFO data |
| Flash CS | 16 | Out | Held high to deselect onboard flash |

These use general-fabric pins, not dedicated configuration-SPI pads.
Start at 1-2 MHz to allow routing and hard-IP refill margin. The companion
ESP32 retains its default sample point and one clock of CS setup/hold; a
phase-1 override regressed previous hardware captures.

## Rates And Idle

The source produces up to 3 kB/s. The MCU must clock at least 24 kbit/s of
data plus margin for gaps. The default 64 bytes every 10 ms gives 6.4 kB/s
of wire capacity. A full FIFO pauses the test generator; it does not model
a real-time capture engine that cannot pause.

An empty hard-IP TX path may shift idle `FF`. The raw wire provides no
valid-byte count, so the MCU preserves those bytes along with legitimate
binary `FF`. Repeated dots in an ASCII view are not literal period bytes.
No byte filtering or data-ready wiring is added.

## LEDs

| LED (Active Low) | Meaning |
| --- | --- |
| Green | Hard SPI IP configured |
| Blue | FIFO byte handed to the SPI IP |
| Red | Synthetic source stalled by a full FIFO |

## Register Configuration

| Register | Setting |
| --- | --- |
| SPICR1 | `80`: SPI enabled, TXEDGE clear |
| SPICR2 | `00`: slave, mode 0, MSB first, SDBRE disabled |
| SPIBR | `01`: valid divider (clock supplied by master) |
| SPISR | TRDY bit 4, RRDY bit 3 |

Registers are configured once before streaming. SDBRE remains off to avoid
inserting its optional hardware framing marker.

## Build And Verification

```sh
make sim
make build
make flash
```

`make flash` programs over the FTDI device in the existing Makefile.
Simulation checks raw output without commands, FIFO stalls, continuity across
CS changes, binary bytes, idle reads, and ignored command patterns on MOSI.
It also checks the ASCII tilde/LF/space wrap.

The behavioral hard-IP model is not silicon accurate. Confirm the first byte
at startup, empty-TX dummy behavior, CS boundaries, and sampling on hardware
at 2 MHz. This change does not correct electrical bit errors or recover
interrupted transfers. Program matching rebuilt FPGA and ESP32 firmware to
replace the counted request/response version.
