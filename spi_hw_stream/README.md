# spi_hw_stream — hard SPI IP slave bring-up test

Exercises the iCE40 UP5K's **hardened SPI block** (`SB_SPI`) configured as a
**slave**, streaming a known test pattern so you can confirm the IP works
before building anything real on top of it.

This is the first design in the repo where the FPGA is the SPI *slave* —
`uart_to_spi.v`, `host_to_spi.v`, and `clock_out.v` all bit-bang a soft SPI
*master* out of fabric. It's the role the SignalBench state-management link
needs (see [`../state_management.md`](../state_management.md)).

## What it does

The ASCII string `"Hello World!"` (12 bytes, one character per byte, no
terminator) is emitted one character per tick at **3 kHz** (`DATA_RATE_HZ`),
wrapping from `!` back to `H`. Each character is pushed into a 64-entry FIFO.
A service state machine configures the hard IP and returns length-prefixed
batches on request, then sends idle `FF` without consuming queued data. See
[`../SPI_PACKET_PROTOCOL.md`](../SPI_PACKET_PROTOCOL.md) for the shared wire
contract. Program the matching FPGA and MCU firmware together.

```text
MCU -> FPGA: 53 42 52 3A   request up to 58 queued bytes
FPGA -> MCU: [idle FF prefix] 53 42 44 <count> <payload> [idle FF padding]
```

The command and response use separate transactions, with at least 20 us
between them. Only the declared payload is the repeating message:
`48 65 6c 6c 6f 20 57 6f 72 6c 64 21 48 65 ...`. Empty polls return count
zero. The ready command `53 42 A5 5A` returns `53 42 4F 4B`; mode-select
commands are ignored by this fixed-message image.

`SPITXDR` remains loaded with idle bytes outside responses. Up to two
prefetched idle `FF` bytes may precede the header. `SDBRE` remains disabled;
the packet header/count replaces the hardware's optional zero marker.

## Wiring

SCK, MOSI, and CS keep the same pins `uart_to_spi.v` uses, but their
**directions are reversed** — the master drives all three now. MISO is new: a
slave needs its own data-out line and can't share the master's MOSI, so it
lands on gpio_13 (the pin `host_to_spi.v` already uses for MISO).

| Signal | Pin | Direction | Notes |
|---|---|---|---|
| SCK  | gpio_11 | FPGA in  | master supplies the clock |
| CS   | gpio_19 | FPGA in  | active low |
| MOSI | gpio_21 | FPGA in  | READ and ready commands |
| MISO | gpio_13 | FPGA out | counted message batches and ready ACK |
| flash CS | 16 | FPGA out | held high to keep the onboard flash off the bus |

SPI mode 0 (CPOL=0, CPHA=0), **MSB-first** — matching `notes.md` and the rest
of the repo. Flip `CFG_SPICR2` bit0 in the source for LSB-first.

These are the hard IP's pads routed out through general fabric rather than the
UP5K's dedicated config-SPI pins (14/15/16/17), which keeps the onboard flash
and the FTDI programmer off this bus. The cost is routing delay — see below.

## Rates and backpressure

The message source produces **3 kB/s**. The master only keeps up if it
sustains **≥ 24 kbit/s** of payload clock plus overhead and transaction gaps.
The default 58-byte maximum every 10 ms gives 5.8 kB/s payload capacity.
Below the source rate the FIFO fills and pauses the synthetic message
source. Queued data is never overwritten or reordered, and the message index
does not advance until FIFO space is available.

Because this is a link-integrity generator rather than a real-time source,
backpressure preserves a consecutive message. So:

- **The string repeats cleanly** (`...World!Hello...`) → link is working.
- **Red LED on** → the message source is paused because the master is too slow.
- **The string skips forward mid-word** (`Hello Wold!`) → the SPI path lost
  or skipped a byte.
- **`0xFF` outside the declared payload** is normal idle padding.
- **`0xFF` inside the message / repeated or garbled characters** indicate a link issue.

`DATA_RATE_HZ` in the source sets the rate; the 3 kHz here was chosen so a
slow master can track it without drops.

## LEDs (active low)

| LED | Meaning |
|---|---|
| GREEN | hard SPI IP finished configuring (should light immediately at power-on) |
| BLUE  | pulses when a payload byte is handed to the IP |
| RED   | pulses while FIFO backpressure stalls the message source |

## Commands

```
make sim     # verifies packet counts, binary bytes, commands, sequencing, backpressure
make wave    # same, with a GTKWave dump
make build   # bitstream
make time    # static timing (icetime cannot analyze the SPI/HFOSC hard cells — expected warnings)
make flash   # program over FTDI
```

Run `make build` for current utilization and timing. The packet path still
uses a register-based FIFO, one `SB_SPI` block, and no EBR. Core clock is
24 MHz; the Makefile requests 30 MHz placement margin.

## Register settings, verified against the datasheet

Checked against Lattice **FPGA-TN-02011-1.8** §12 (local copy in
[`../ice_docs/`](../ice_docs/)) — the successor to TN1295:

| Setting | Value | Source |
|---|---|---|
| Register addresses `0x08`–`0x0F` | — | Table 12.1 ✓ |
| `SPICR1[7]` SPE = 1 (enable) | `0x80` | Table 12.3 ✓ |
| `SPICR2[7]` MSTR = 0 → **slave** | `0x00` | Table 12.4 ✓ |
| `SPICR2[2:1]` CPOL/CPHA = 0 → mode 0 | | Table 12.4 ✓ |
| `SPICR2[0]` LSBF = 0 → **MSB-first** | | Table 12.4 ✓ |
| `SPISR[4]` TRDY, `SPISR[3]` RRDY | — | Table 12.7 ✓ |
| `SPIBR` DIVIDER ≥ 1 | `0x01` | Table 12.5 |

Two things worth knowing: **slave mode is selected by `SPICR2` bit 7, not
`SPICR1`** — and a write to *any* of `SPICR0/1/2`, `SPIBR`, or `SPICSR` resets
the SPI core, which is why all five are written up front and never touched
again.

## Still to confirm on hardware

The testbench uses a **simplified behavioral `SB_SPI` model** (no vendor sim
library in this repo, same situation as the `SB_HFOSC`/`SB_GB` stubs in
`i2c_sniffer_tb.v`), so it proves the FIFO/FSM logic, not the IP's real
behavior:

1. **The empty-`SPITXDR` window** (see above) — the model doesn't reproduce the
   forced-`0xFF` silicon limitation, so it looks cleaner at startup than real
   hardware will.
2. **CS-boundary behavior.** The model assumes a byte pre-loaded into the shift
   register survives CS going high and is sent at the start of the next
   transaction. Figure 15.1 is consistent with that, but doesn't state it
   outright. If the real IP reloads from `SPITXDR` on every CS assertion,
   you'll see one character skipped per transaction — watch for a missing
   character at every transaction boundary.
3. **Max usable SCK.** Two limits stack. The refill deadline (Table 12.8) gives
   ~7 bit-times to reload `SPITXDR`, against a ~375 ns worst-case service loop
   — fine below ~10 MHz. On top of that, the hard IP sits at chip corner (0,0)
   and these pins don't, so nextpnr reports ~8.0 ns pad→IP on SCK/CS and
   ~7.6 ns IP→pad on MISO. Start at 1–2 MHz. For a high SCK later, the
   dedicated pins 14/15/16/17 avoid the routing detour (at the cost of sharing
   with the flash and FTDI), and `SPICR1[4]` TXEDGE exists for fast SPI.

   The companion ESP32-H2 firmware retains its default sample point and a
   one-clock CS setup/hold time. A phase-1 override increased one-to-zero
   MISO errors in the user's captures. Packet framing removes idle padding
   from mobile payloads but does not correct physical bit errors.
