# spi_counter_stream — hard SPI IP slave bring-up test

Exercises the iCE40 UP5K's **hardened SPI block** (`SB_SPI`) configured as a
**slave**, streaming a known test pattern so you can confirm the IP works
before building anything real on top of it.

This is the first design in the repo where the FPGA is the SPI *slave* —
`uart_to_spi.v`, `host_to_spi.v`, and `clock_out.v` all bit-bang a soft SPI
*master* out of fabric. It's the role the SignalBench state-management link
needs (see [`../state_management.md`](../state_management.md)).

Relationship to [`../spi_hw_stream/`](../spi_hw_stream/): same design. This
directory keeps the **ascending-byte-counter** payload, which is the better
link-integrity test — a missed, repeated, or reordered byte is visible
immediately. `spi_hw_stream/` has since had its payload replaced with a
repeating `"Hello World!"` string, which is easier to eyeball in a terminal but
much weaker as a check.

## What it does

An 8-bit counter is produced at up to **`DATA_RATE_HZ` (currently 3 kHz)** and
wraps `0xFF -> 0x00`. It pauses while FIFO backpressure is active.
Each value is pushed into a 64-entry FIFO. A service state machine configures
the hard IP, then keeps its transmit register loaded from the FIFO head, so
every byte the master clocks out is the next value in the sequence.

**A working link looks like an unbroken incrementing byte stream** on the
master/analyzer: `00 01 02 03 ...`.

Because this design keeps `SPITXDR` pre-loaded at all times, it matches
FPGA-TN-02011 **Figure 15.1** (*iCE40 UltraPlus as SPI Slave*) — the pre-loaded
byte goes straight to SO when CS asserts, so the counter appears from byte one.
The dummy-byte overhead in Table 12.8 / Figure 15.2 applies to
command-response protocols, where the slave can't know what to send until it
decodes an incoming command. This design has nothing to decode.

**The exception is an empty `SPITXDR` at the start of a transaction.**
Figure 15.2 documents a *silicon limitation* there: the second byte out is
forced to `0xFF` regardless of what you write, and good data only appears in
the third byte period. That window exists at power-on (before the first
first tick, ~333 µs at 3 kHz) or any time the FIFO runs dry. If that's awkward,
`SPICR2[SDBRE]` (bit 5) makes it deterministic: `0xFF` until data is ready,
then a single `0x00` marker, then the real stream — giving the master
something to sync on.

## Wiring

SCK, MOSI, and CS keep the same pins `uart_to_spi.v` uses, but their
**directions are reversed** — the master drives all three now. MISO is new: a
slave needs its own data-out line and can't share the master's MOSI, so it
lands on gpio_13 (the pin `host_to_spi.v` already uses for MISO).

| Signal | Pin | Direction | Notes |
|---|---|---|---|
| SCK  | gpio_11 | FPGA in  | master supplies the clock |
| CS   | gpio_19 | FPGA in  | active low |
| MOSI | gpio_21 | FPGA in  | ignored by this test |
| MISO | gpio_13 | FPGA out | the counter stream |
| flash CS | 16 | FPGA out | held high to keep the onboard flash off the bus |

SPI mode 0 (CPOL=0, CPHA=0), **MSB-first** — matching `notes.md` and the rest
of the repo. Flip `CFG_SPICR2` bit0 in the source for LSB-first.

These are the hard IP's pads routed out through general fabric rather than the
UP5K's dedicated config-SPI pins (14/15/16/17), which keeps the onboard flash
and the FTDI programmer off this bus. The cost is routing delay — see below.

## Rates and backpressure

The counter produces `DATA_RATE_HZ` bytes/s — **3 kB/s** at the current
setting. The master only keeps up if it sustains **≥ 8 × `DATA_RATE_HZ`**
bits/s of SPI clock (**24 kbit/s** at 3 kHz). Below that the FIFO fills and new
samples pause. Queued data is never overwritten or reordered, and the
synthetic counter does not advance until FIFO space is available.

Because this is a link-integrity generator rather than a real-time capture
source, backpressure preserves a consecutive sequence. So:

- **Bytes increment by exactly 1** → link is working.
- **Red LED on** → the counter is paused because the master is too slow.
- **Bytes jump forward** → the SPI path lost or skipped a byte.
- **`0xFF` runs / repeats / decrements** → something is actually wrong.

Change `DATA_RATE_HZ` in the source if you want a different rate. Everything
downstream is derived from it — the tick divider width comes from `TICK_DIV`
via `$clog2`, and the testbench scales its own wait windows off `dut.TICK_DIV`
— so the new value genuinely takes effect. (An earlier revision hardcoded the
tick counter at 6 bits, which silently pinned the usable rate at ≥ 375 kHz:
below that the compare value was unreachable and the tick never fired at all,
so editing `DATA_RATE_HZ` appeared to do nothing.) Verified in simulation from
500 Hz to 2 MHz.

## LEDs (active low)

| LED | Meaning |
|---|---|
| GREEN | hard SPI IP finished configuring (should light immediately at power-on) |
| BLUE  | pulses when a byte is handed to the IP — master is clocking |
| RED   | pulses while FIFO backpressure stalls the counter |

## Commands

```
make sim     # testbench: verifies sequencing, backpressure, and CS-boundary continuity
make wave    # same, with a GTKWave dump
make build   # bitstream
make time    # static timing (icetime cannot analyze the SPI/HFOSC hard cells — expected warnings)
make flash   # program over FTDI
```

Current build: 1180 LC (22%), 1 of 2 `SB_SPI` blocks, no EBR. Core clock
24 MHz, Fmax 35.6 MHz. The LC count is mostly the register-based FIFO's
combinational read; moving it to an inferred EBR would cut it substantially at
the cost of handling the read-during-write hazard.

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
   you'll see one sample skipped per transaction — watch for a consistent +2
   step at transaction boundaries.
3. **Max usable SCK.** Two limits stack. The refill deadline (Table 12.8) gives
   ~7 bit-times to reload `SPITXDR`, against a ~375 ns worst-case service loop
   — fine below ~10 MHz. On top of that, the hard IP sits at chip corner (0,0)
   and these pins don't, so nextpnr reports ~8.0 ns pad→IP on SCK/CS and
   ~7.6 ns IP→pad on MISO. Start at 1–2 MHz. For a high SCK later, the
   dedicated pins 14/15/16/17 avoid the routing detour (at the cost of sharing
   with the flash and FTDI), and `SPICR1[4]` TXEDGE exists for fast SPI.
