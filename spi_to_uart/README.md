# spi_to_uart — SPI slave to picocom

Every byte an external SPI master clocks in on MOSI is printed to the host
terminal as hex. Flash it, open `make pico`, drive the SPI pins, and watch.

It is two pieces of this repo joined together: the hardened `SB_SPI` slave and
its configuration state machine from [`../spi_hw_stream/`](../spi_hw_stream/),
and the UART transmit path (`uart_tx` from `../common/uart.v` at 1 Mbaud) that
[`../host_to_fpga/`](../host_to_fpga/) and [`../uart_to_spi/`](../uart_to_spi/)
use. The FPGA is the SPI **slave**: the master supplies SCK and CS, as the
ESP32-H2 will in the SignalBench state-management link.

## Wiring

Same SPI pins as `spi_hw_stream`, so the same jumper harness works for both.

| Signal | Pin | Direction | Notes |
|---|---|---|---|
| SCK  | gpio_11 | FPGA in  | master supplies the clock, mode 0 |
| CS   | gpio_19 | FPGA in  | active low, one transaction per assertion |
| MOSI | gpio_21 | FPGA in  | the bytes that get printed |
| MISO | gpio_13 | FPGA out | **not meaningful** — see below |
| UART TX | 14 | FPGA out | to the FT232H, shows up on `/dev/ttyUSB0` |
| flash CS | 16 | FPGA out | held high to keep the onboard flash off pin 14 |

SPI mode 0 (CPOL=0, CPHA=0), **MSB-first**, matching `notes.md` and the rest of
the repo. Common ground with the master, and 3.3 V logic (UPduino header
position 2 is `VIO`, not a supply — the 3.3 V rail is position 9).

If the master is another UPduino running `uart_to_spi`, jumper 11→11, 21→21,
19→19 and GND→GND; its 3-byte `[0x48][flag][data]` transactions print one per
line.

**Pin 14 is the FPGA's config-SPI `SPI_SO`.** On the UPduino v3 that pin sits
on the `FLASH_MOSI` net together with the FT232H's `ADBUS1`, which is the
FTDI's receive pin in async serial mode (`UPduino_v3.0.sch`) — so this is the
FPGA-to-host direction, the one `../common/upduino_v3.pcf` labels "FPGA transmit
to USB" and `host_to_fpga` already uses. The same net goes to the flash, which is
why pin 16 must hold the flash deselected. Because the FTDI is shared with the
programmer, **close picocom before `make flash`**.

## Output format

Lowercase hex, one space after every byte, CR LF when CS is released (or after
16 bytes, so a master that holds CS low forever still produces lines):

```
48 01 41
aa 12 34 ff ff 9a bc de
a0 a1 a2 a3 a4 a5 a6 a7 a8 a9 aa ab ac ad ae af
b0 b1 b2 b3
```

The newline is emitted once the bus is idle *and* the IP's receive register has
been drained, so it can never race the last byte of a transaction. Nothing is
printed while the bus is quiet.

## Commands

```
make sim     # testbench: transactions, nibble extremes, line wrap, drop-on-full
make wave    # same, with a GTKWave dump
make build   # bitstream
make time    # static timing (icetime cannot analyze the SPI/HFOSC hard cells — expected warnings)
make flash   # program over FTDI (close picocom first)
make pico    # picocom -b 1000000 /dev/ttyUSB0
```

Current build: 284 LC (5%), 1 EBR, 1 of 2 `SB_SPI` blocks. Core clock 24 MHz,
icetime 47.8 MHz against the 30 MHz constraint.

## LEDs (active low)

| LED | Meaning |
|---|---|
| GREEN | hard SPI IP finished configuring (should light at power-on) |
| BLUE  | pulses when a byte is received — the master is clocking |
| RED   | pulses when a byte was dropped — the UART cannot keep up |

## Limits

- **Throughput.** Three UART characters per SPI byte at 1 Mbaud is
  **~33 kB/s sustained**. A 64-character queue absorbs bursts of ~21 bytes
  beyond that; past it, whole bytes are dropped (never half a hex pair — the
  output stays parseable) and the red LED pulses. The dropped bytes leave no
  marker in the text; the LED is the only indication. `uart_to_spi` at 9600
  baud produces ~2.9 kB/s, well inside the budget.
- **SCK.** The service loop drains `SPIRXDR` within ~11 core cycles (~460 ns),
  against 800 ns per byte at 10 MHz — the same ~10 MHz ceiling as
  `spi_hw_stream`, and the same pad→IP routing detour (these are not the
  dedicated config-SPI pins). Start at 1–2 MHz.
- **MISO carries nothing.** `SPITXDR` is never written, so `SO` shifts out
  whatever the IP holds — on silicon that is unspecified (FPGA-TN-02011
  Figure 15.2's empty-`SPITXDR` case). The pin is driven only to keep the pin
  plan identical to `spi_hw_stream`; leave it unconnected or ignore it.
- **Receive only.** There is no path from picocom back to the SPI side, so
  `tools/send_pattern.c` is not needed here — it generates UART traffic *into*
  the sniffers, and this design has nothing to receive from the host.

## Verification status

**Simulation only — not yet run on hardware.** The testbench drives real SPI
transactions and decodes the UART line the way the host does (start bit, eight
data bits LSB-first, stop bit), comparing against text it builds with its own
hex conversion and line rules. It covers a 3-byte `uart_to_spi`-style
transaction, `0x00`/`0xff` nibble extremes, a single-byte transaction, a 20-byte
transaction wrapping at 16, and a 64-byte burst at 2 MHz that must drop bytes
whole and stay well formed.

Two things the testbench cannot prove, both inherited from `spi_hw_stream`:

1. The `SB_SPI` model is the simplified one from `spi_hw_stream_tb.v` — it has
   no receive-overrun (`ROE`) flag, so it shows that `SPIRXDR` is drained every
   byte but not how the real IP behaves if it ever isn't.
2. Whether the FTDI path on pin 14 works at 1 Mbaud on *this* board is inferred
   from the schematic and from `host_to_fpga` having used it, not re-measured.

One thing that is *not* inherited: the character queue is written as a
register array with a combinational read, but yosys merges its single
registered consumer into the read port and maps it to an EBR. That is only
equivalent because a push and a pop never hit the same address in one cycle
(pops are issued only while the queue is non-empty); the testbench asserts
this on every clock. Add a second reader of `cq_head` and the merge goes away.
