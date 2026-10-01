# stream_system — raw streaming, decoder chosen by the MCU

The streaming-only variant of [`../final_system/`](../final_system/): no trigger, no
recording, no record framing. The MCU selects a decoder with `53 42 4D xx`, and from then
on every byte that decoder produces comes out on MISO, raw.

**Status:** simulation passes (`make sim`); builds and closes timing. **Not yet run on
hardware.** The `SB_SPI` model used in simulation is not silicon-accurate.

```
make build    # -> signalbench_stream.bin
make flash
make sim      # ready/mode handshake, I2C x3, UART x2, invalid mode, flush on switch
make seeds    # timing across six placement seeds -- all must pass
```

| | |
|---|---|
| Logic | 1668 / 5280 LC (31%) — 1162 LUT4 + 277 carry |
| RAM | 8 / 30 EBR (4 KB stream FIFO) |
| Timing | 32.4 – 34.7 MHz across 6 seeds, all PASS at 24 MHz; icetime 30.6 MHz |

---

## Before you test: hardware checklist

1. **Jumper R16 ("OSC") must be shorted.** The clock comes from the 12 MHz crystal on
   pin 20, as for `can_sniffer`. Without the jumper nothing runs at all: all three LEDs
   stay dark (green never comes on, because the SPI IP never gets configured) and there
   are no ACKs. Every older image except `can_sniffer` used `SB_HFOSC`, so this is the
   easiest thing to miss. **Green on = clock running and IP configured.**
2. **Probe wiring is unchanged from your existing rigs.** See the pin table below.
3. **MCU wiring is unchanged from `spi_counter_stream`**: SCK 11, CS 19, MOSI 21,
   MISO 13.
4. **UART is fixed at 9600 baud**, matching `send_pattern.c` and `uart_to_spi.v`. There
   is no baud opcode; change `UART_BAUD` in `signalbench_stream.v` and rebuild.
5. **Power-up protocol is I2C.** Select explicitly anyway, so you get the ACK.

---

## MCU protocol

Mirrors the command-capable `spi_counter_stream.v` on `fix/spi-stream-reliability`, so
firmware written against that image should work against this one unchanged.

| MCU → FPGA (one transaction) | FPGA → MCU (next transaction) | |
|---|---|---|
| `53 42 A5 5A` | `53 42 4F 4B` | ready check ("SBOK") |
| `53 42 4D 00` | `53 42 41 00` | select **I2C** |
| `53 42 4D 01` | `53 42 41 01` | select **SPI** |
| `53 42 4D 02` | `53 42 41 02` | select **DIO** |
| `53 42 4D 03` | `53 42 41 03` | select **CAN** |
| `53 42 4D 04` | `53 42 41 04` | select **UART** |

- The ACK comes in the **next** transaction, because a slave can't answer a byte it hasn't
  received yet. **Scan the burst for it** rather than expecting it at byte 0. In
  simulation it has landed at byte 0 and at byte 1, depending on SB_SPI's TXDR pipelining.
- `xx` above `04` is ignored: no ACK, protocol unchanged.
- A valid select **flushes the stream FIFO** and restarts the decoder, so everything after
  the ACK belongs to the new protocol. Anything *before* the ACK in that burst may be one
  leftover byte of the old protocol. Discard up to and including the ACK.
- Commands are matched on a rolling window of received bytes, not framed on CS. Send
  `0x00` on MOSI when you only want to read, since `0x00` can't start a command.

### Reading the stream

Clock bytes with MOSI = `0x00`. Each byte is either stream data or, if the FIFO is empty,
`0xFF`.

> **`0xFF` is ambiguous.** It means "nothing to send" *and* it's a perfectly valid data
> byte. Your `send_pattern` test contains two (`AA 12 34 FF FF 9A BC DE`), and in the I2C
> stream every START marker is `0xFF`. Against a known pattern you can still read it by eye.
> Software can't tell them apart. That's the main open problem with this version; options
> are listed at the bottom.

Bytes clocked out **during a command transaction** are real stream bytes, and they've
left the FIFO. If the firmware discards what it receives while sending a command, those
bytes are lost. This is the same as `spi_counter_stream`.

---

## Pins

Probe channels are **shared** between protocols, since only one decoder runs at a time.
This lets every existing rig plug in unchanged:

| Pin | I2C | UART | CAN | DIO | Same as |
|---|---|---|---|---|---|
| **38** (ch0) | SCL | target **TX** line | RX (via VP230) | input | `i2c_sniffer.pcf`, `uart_to_spi.v`, `can_sniffer.pcf` |
| **42** (ch1) | SDA | target **RX** line | — | — | `i2c_sniffer.pcf`, `uart_to_spi.v` |

| Pin | Function |
|---|---|
| 37 | SPI probe SCK (GBIN, required) |
| 31 / 32 / 28 | SPI probe MOSI / MISO / CS |
| 11 / 19 / 21 / 13 | MCU SCK / CS / MOSI / MISO |
| 20 | 12 MHz crystal (R16) |
| **35** | **leave unconnected** — the PLL consumes this I/O site |
| 41 / 39 / 40 | LED R / G / B |

**LEDs** (active low): **red** = a byte was dropped because the FIFO was full;
**green** = SPI IP configured, blinking off briefly on each decoded byte;
**blue** = a byte was handed to the MCU.

---

## I2C — what you'll see, and what changed

**The stream is `i2c_sniffer.v`'s output.** The capture logic is a direct port of its
capture state machine. The testbench drives the bus with `i2c_sniffer_tb.v`'s own driver
tasks, copied verbatim, and checks against that testbench's own expected bytes:

| Bus traffic | Stream | Matches `i2c_sniffer_tb` |
|---|---|---|
| `START 6C ACK 0B ACK STOP` | `FF 6C 05 80` | yes |
| same, ACK jammed 80 ns before STOP (journal bug 3) | `FF 6C 05 80` | yes |
| `START 6C ACK 0B ACK rSTART 6D ACK 93 NACK STOP` | `FF 6C 05 A0 FF 6D 49 C0` | yes |

So the format is the **raw packed SDA bitstream, ACK bits included**, 8 SCL-rising samples
per byte, MSB first, partial bytes left-aligned at STOP and repeated START, and `0xFF` at
each START.

**Kept unchanged from `i2c_sniffer.v`:** 3-stage synchronizers, START/STOP derivation,
the repeated-START flush-then-mark sequence, the ACK-before-STOP fix, the 1 ms idle
timeout.

**Changed:**

- **Clock:** PLL from the 12 MHz crystal instead of `SB_HFOSC` ÷ 2. Both are 24 MHz, so
  every timing relationship is the same.
- **Where the leading `0xFF` comes from.** The original's *drain* put `0xFF` in front of
  each completed capture. Here it's emitted at the START itself. The per-transaction byte
  sequence is the same.
- **Streams instead of buffering whole transactions.** That has three side effects:
  - a START followed by STOP with no data produces a lone `FF`; the original sent nothing
  - an idle timeout can't take back bytes already streamed; the original dropped the
    whole capture
  - no 128-byte-per-transaction limit; the 4 KB FIFO is the limit instead
- **Output path:** SPI *slave* via `SB_SPI`, polled by the MCU, instead of a bit-banged
  SPI *master* the FPGA drove. Different direction on pins 11/19/21, and MISO is now
  pin 13.

`START_MARKER = 0` in `signalbench_stream.v` removes the `0xFF` markers if you'd rather
have bus bits only.

---

## UART — what you'll see, and what changed

**The stream is the received bytes, in arrival order, nothing added.** With
`send_pattern` on the target-TX line you get `AA 12 34 FF FF 9A BC DE`, as the testbench
checks.

**Kept unchanged from `uart_to_spi.v`:** the same two `uart_rx` instances from
`common/uart.v`, the same 4× oversampling, the same baud tick. 24 MHz ÷ 625 here gives
exactly the 38.4 kHz that 48 MHz ÷ 1250 gave.

**Changed:**

- **Framing removed.** `uart_to_spi.v` sent `[0x48][0x01 or 0x02][data]` per byte. Now
  it's just `[data]`, so **if both lines carry traffic you can't tell which line a byte
  came from.** With only one line wired, the other idles high and contributes nothing.
- **Clock:** crystal instead of `SB_HFOSC`. Strictly better for UART: the HFOSC is
  specified ±10%, and an 8-N-1 receiver tolerates roughly ±2–3% total.
- **Same-cycle collision fixed.** If both lines finish a byte in the same core clock, the
  second byte is held one clock rather than lost. (`uart_to_spi.v` queued both through its
  own FIFO, so it didn't have this problem; a single-stream output does.) Tested by
  driving both lines in lockstep.
- **Output path:** as for I2C.

**Inherited from `common/uart.v`, unchanged, worth knowing:**

- **The sample point isn't centred.** `uart_rx` samples each bit 25–50% of the way in
  (4× oversampling, start detected up to ¼ bit late), not at 50%. That leaves less margin
  for a target whose baud is off. Fine against a crystal-clocked target; marginal against
  a cheap RC-oscillator one.
- **Framing errors are silently dropped.** A byte with a bad stop bit just doesn't
  appear.

---

## SPI, DIO, CAN — not tested, not your focus

- **SPI:** two bytes per 8 probed SCK edges, MOSI then MISO. Mode 0, fixed at build time
  (`SPI_MODE`); there's no opcode for it.
- **DIO:** one byte per edge on pin 38, `01` after rising and `00` after falling. No rate
  limit, so a fast signal will fill the FIFO.
- **CAN:** `can_sniffer.v`'s 20-byte record, same layout, on pin 38 via the VP230. Fixed at
  1 Mbit/s. The `dropped` bit is always 0 here; loss shows on the red LED instead.

---

## Throughput

With the firmware polling 64 bytes every 10 ms you read 6.4 kB/s. A fully loaded 100 kHz
I2C bus produces about 12.5 kB/s of raw stream, so sustained traffic overflows the 4 KB
FIFO in well under a second. AS5600-style polling won't come close. UART at 9600 is under
1 kB/s per line. Overflow drops bytes silently on the wire and lights the red LED.

---

## Open problems

1. **`0xFF` ambiguity** (above). Ways out, cheapest first:
   - a **count query** command, so the MCU reads exactly what's there (a protocol change)
   - a **data-ready GPIO** (FIFO non-empty). Costs a pin; only helps for the first byte
     of a burst
   - **escape encoding** of `0xFF` in the stream, which is framing, i.e. what this version
     set out to avoid
2. **No in-band indication of dropped bytes**, only the red LED.
3. **Both UART lines merge untagged.**
4. **UART baud and SPI mode are build-time only.**
