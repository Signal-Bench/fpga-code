# stream_system — framed streaming, decoder chosen by the MCU

The streaming-only variant of [`../final_system/`](../final_system/): no trigger, no
recording. The MCU selects a decoder with `53 42 4D xx`, and everything that decoder
captures comes out on MISO as framed messages:

```
53 42 4D | flags/protocol | length | payload[length]
```

**The wire format and how to parse it are in [`MCU_FRAME_FORMAT.md`](MCU_FRAME_FORMAT.md)**,
written for whoever builds the ESP32 firmware. This file covers building, testing, and
what changed relative to the original sniffers.

**Status:** simulation passes and timing closes. **Not yet run on hardware.** The `SB_SPI`
model used in simulation isn't silicon-accurate.

```
make build      # -> signalbench_stream.bin
make flash
make sim        # 25 checks, MAX_PAYLOAD=16 and a 128-byte FIFO to force splits/overflow
make sim-255    # production limits (255-byte payload, 4 KB FIFO), long I2C transaction
make seeds      # timing across six placement seeds -- all must pass
```

| | |
|---|---|
| Logic | 2391 / 5280 LC (45%) — 1670 LUT4 + 419 carry |
| RAM | 8 / 30 EBR (4 KB stream FIFO) |
| Timing | 29.9 – 33.3 MHz across 6 seeds, all PASS at 24 MHz; icetime 30.05 MHz |

---

## Before you test: hardware checklist

1. **Jumper R16 ("OSC") must be shorted.** The clock comes from the 12 MHz crystal on
   pin 20, as for `can_sniffer`. Without it nothing runs: all LEDs stay dark and there
   are no ACKs. **Green on = clock running and SPI IP configured.**
2. **Probe wiring is unchanged from your existing rigs**; see Pins.
3. **MCU wiring is unchanged from `spi_counter_stream`**: SCK 11, CS 19, MOSI 21,
   MISO 13.
4. **UART is fixed at 9600 baud**, matching `send_pattern.c`. Change `UART_BAUD` in
   `signalbench_stream.v` to rebuild for another rate.
5. **Power-up protocol is I2C.** Select explicitly anyway, so you get the ACK.
6. **Your firmware's parser must handle frames.** The raw unframed stream from the
   previous version of this directory is gone.

---

## Pins

Probe channels are **shared** between protocols (only one decoder runs at a time), which
lets every existing rig plug in unchanged:

| Pin | I2C | UART | CAN | DIO | Same as |
|---|---|---|---|---|---|
| **38** | SCL | line A (target TX), tag `01` | RX via VP230 | input | `i2c_sniffer.pcf`, `uart_to_spi.v`, `can_sniffer.pcf` |
| **42** | SDA | line B (target RX), tag `02` | — | — | `i2c_sniffer.pcf`, `uart_to_spi.v` |

| Pin | Function |
|---|---|
| 37 | SPI probe SCK (GBIN, required) |
| 31 / 32 / 28 | SPI probe MOSI / MISO / CS |
| 11 / 19 / 21 / 13 | MCU SCK / CS / MOSI / MISO |
| 20 | 12 MHz crystal (R16) |
| **35** | **leave unconnected** — the PLL consumes this I/O site |
| 41 / 39 / 40 | LED R / G / B |

**LEDs** (active low): **red** = a frame was dropped because the FIFO was full;
**green** = SPI IP configured, blinking off briefly on decoder activity;
**blue** = a byte was handed to the MCU.

---

## UART — what changed from `uart_to_spi.v`

**Kept:** the same two `uart_rx` receivers from `common/uart.v`, the same 4×
oversampling, the same baud tick (24 MHz ÷ 625 = 48 MHz ÷ 1250 = 38.4 kHz), and the same
pins.

**Changed:**

- **Output format.** Was `[0x48][0x01|0x02][data]` per byte over an FPGA-driven SPI
  master. Now there's one frame per byte, `53 42 4D 04 02 [tag][byte]`, or a length-4
  frame `[01][a][02][b]` when both lines deliver a byte at once. Tags `01`/`02` are the
  same values `uart_to_spi.v` used.
- **"At once" means the same quarter-bit sample tick**: within ~26 µs at 9600. The two
  receivers share one 4×-baud tick, so their outputs are quantized to quarter bits.
  Anything further apart gets separate frames.
- **Clock:** crystal instead of `SB_HFOSC`, which is strictly better for UART (HFOSC is
  ±10%; UART tolerates roughly ±2–3% total).
- **Output path:** SPI slave polled by the MCU, instead of a master the FPGA drove.

**Inherited from `common/uart.v`, unchanged:** each bit is sampled 25–50% of the way in
rather than centred, which reduces tolerance to baud mismatch. Framing errors are
silently dropped.

## I2C — what changed from `i2c_sniffer.v`

**Kept, bit for bit:** the capture state machine, so the raw packed SDA bitstream
(ACK bits included) is identical to what `i2c_sniffer.v` captured. That includes the
3-stage synchronizers, START/STOP detection, the ACK-before-STOP fix and the 1 ms idle
timeout. The testbench drives the bus with `i2c_sniffer_tb.v`'s own tasks; the payload
bytes match that testbench's expected bytes (`6C 05 80`, `6C 05 A0`, `6D 49 C0`).

**Changed:**

- **One frame per START-delimited segment.** A repeated START ends the frame and starts a
  new one, instead of inserting a `0xFF` marker: a marker can't be told apart from a data
  `0xFF`, and a frame boundary can. A register read (write, rSTART, read) is two frames.
- **An info byte ends each payload**: how the segment ended (STOP, repeated START, idle
  timeout, split), whether it began with a repeated START, and **how many bits of the
  last byte are real**. The original had no way to say that last part; its zero padding
  looked like data.
- **No 128-byte limit per transaction.** Long transactions split into frames flagged
  `CONT`; tested with 300 bytes.
- **Clock:** PLL from the crystal instead of `SB_HFOSC` ÷ 2. Both are 24 MHz.
- **Output path:** as for UART.

## SPI, DIO, CAN

Built, synthesized, and covered by the format document, but **not exercised by the
testbench**, and not your focus for testing. SPI mode is fixed at build time (`SPI_MODE`,
default 0).

---

## Known limits

1. **Throughput is set by how often the MCU polls**, not by the link. 64 bytes per 10 ms
   is 6.4 kB/s. A UART line saturated at 9600 produces 6.7 kB/s of frames (7 bytes per
   UART byte), so it slowly overflows; `send_pattern` (~3.1 kB/s) is fine.
   `MCU_FRAME_FORMAT.md` §7 has an adaptive-polling fix.
2. **A ready check sent mid-stream loses the bytes clocked out during it.** A select
   doesn't matter (it flushes anyway).
3. **No checksum.** The framing finds frame *boundaries* reliably, but a bit flipped on
   MISO inside a payload goes undetected. Your branch's hardware captures saw real MISO
   bit errors at some sample-point settings, so this is a real risk.
4. **UART baud and SPI mode are build-time only.**
