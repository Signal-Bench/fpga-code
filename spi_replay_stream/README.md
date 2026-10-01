# spi_replay_stream — recorded-data replay over the hard SPI slave

Streams a **preloaded 4 KB buffer** out of the iCE40 UP5K's hardened SPI block
(`SB_SPI`) as a **slave**, advancing only when the master takes a byte. Nothing
in this design can lose data at any master rate.

This is the third of three SPI slave bring-up images, and the one to reach for
when you want to rule the generator out entirely:

| Design | Models | On a slow master |
|---|---|---|
| [`../spi_hw_stream/`](../spi_hw_stream/) | live bus, `"Hello World!"` | **drops** characters |
| [`../spi_counter_stream/`](../spi_counter_stream/) | live bus, selectable pattern | **drops** samples |
| **`spi_replay_stream/`** | **recorded capture** | **waits** — nothing is lost |

The first two run a synthetic source at a fixed `DATA_RATE_HZ` and drop bytes
the master doesn't collect in time, which is how you measure the sustained rate
the master actually achieves. This one has no producer and no rate: the payload
is already in memory before the first SCK edge, so the master is the sole
flow-control authority. It is the closer model of the **recording drain** mode
in [`../state_management.md`](../state_management.md).

## What it does

4096 bytes sit in inferred EBR, loaded from the bitstream's INIT values at
configuration time — there is no runtime load step. The replay pointer advances
only when the service state machine has handed a byte to the hard IP, so:

- Read at 100 kHz or at 4 MHz and you get the **same byte sequence**, just
  slower or faster.
- Stop clocking for a minute and the pointer does not move.
- Any gap, repeat or reorder you see on the analyzer is a **transport or
  hard-IP fault**, with the generator fully ruled out. That is what makes this
  the strictest of the three link tests.

There is no FIFO, no tick divider and no drop path in the design at all.

## Buffer contents

The default payload is a synthetic 16-byte record, 256 of them:

| Offset | Value |
|---|---|
| 0 | `0xA5` sync |
| 1 | record number, `0x00`..`0xFF` |
| 2–14 | `record + offset`, varies within the record |
| 15 | `0x5A` end |

So a correct read looks like `a5 00 02 03 ... 0e 5a a5 01 03 04 ...`. Framing is
self-evident on an analyzer: **every 16th byte must be `0xA5`**, and record
numbers must ascend by exactly 1 with no repeats or skips.

To replay a real capture instead, define `REPLAY_HEX_FILE`:

```
iverilog -g2012 -DREPLAY_HEX_FILE='"mycapture.hex"' -o tb.out spi_replay_stream.v spi_replay_stream_tb.v
yosys -p "read_verilog -DREPLAY_HEX_FILE='\"mycapture.hex\"' spi_replay_stream.v; synth_ice40 -top top -json spi_replay_stream.json"
```

One hex byte per line, `REPLAY_DEPTH` lines. A short file leaves the tail at `x`
in simulation and `0` in the bitstream, so size it exactly. The bundled
testbench checks the synthetic pattern, so it will fail against a hex file —
that is expected, not a regression.

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `REPLAY_AW` | 12 | address width; buffer is `1 << REPLAY_AW` bytes |
| `REPLAY_LOOP` | 1 | 1 = wrap forever (soak/rate test), 0 = play once then go quiet |

With `REPLAY_LOOP = 0` the pointer stops at the last byte, `SPITXDR` runs dry
and the master reads `0xFF` — which is how to check your master's
end-of-recording handling. The red LED goes on steady.

4096 bytes costs 8 of the UP5K's 30 EBR blocks. Raising `REPLAY_AW` to 13 would
take 16 and still fit; 14 would not.

## Wiring

Identical to [`../spi_hw_stream/`](../spi_hw_stream/) — same pins, same pcf
layout, so the three images are drop-in swaps on the same bench setup.

| Signal | UPduino pin | Direction | Notes |
|---|---|---|---|
| SCK  | gpio_11 | FPGA in  | master supplies the clock |
| CS   | gpio_19 | FPGA in  | active low |
| MOSI | gpio_21 | FPGA in  | ignored (drained to keep ROE clear) |
| MISO | gpio_13 | FPGA out | the replay stream |
| flash CS | 16 | FPGA out | held high to keep the onboard flash off the bus |

SPI mode 0 (CPOL=0, CPHA=0), **MSB-first**. These are the hard SPI IP's pads
routed through general fabric, not the dedicated config-SPI pins.

## LEDs (active low)

| LED | Meaning |
|---|---|
| GREEN | hard SPI IP finished configuring (should light immediately at power-on) |
| BLUE  | pulses when a byte is handed to the IP — master is clocking |
| RED   | pulses on each wrap (`REPLAY_LOOP=1`), or steady once exhausted (`REPLAY_LOOP=0`) |

A **dark red LED with a looping read** means you have not yet read 4096 bytes.

## Commands

```
make sim     # verifies rate independence, idle hold, full-buffer drain, and the wrap
make wave    # same, with a GTKWave dump
make build   # bitstream
make time    # static timing (icetime cannot analyze the SPI/HFOSC hard cells — expected warnings)
make flash   # program over FTDI
make clean   # remove build/sim artifacts
```

Current build: **112 LC (2%)**, **8 of 30 EBR (26%)**, 1 of 2 `SB_SPI`. Core
clock 24 MHz, Fmax 48.6 MHz. Far smaller than the other two designs — the FIFO
and the tick divider were most of their logic.

## Verification status

`make sim` passes: byte-exact against an independently computed copy of the
record layout (the testbench uses division and modulo, the design uses bit
slices), across a 200 kHz burst, a 2 MHz burst, a 4 MHz burst, CS toggles, a
5 ms idle gap, a full 4096-byte drain and the wrap back to record 0. It also
checks that bytes loaded into the IP exceed bytes read by at most the pre-load
pipeline depth of 2, so nothing is handed over and silently lost.

**Two things simulation cannot tell you:**

1. The `SB_SPI` model in the testbench is **not silicon-accurate** — in
   particular it has no leading dummy byte, while FPGA-TN-02011 Table 12.8
   requires `SPITXDR` to be written ≥ 0.5 SCK periods before the first bit
   reaches SO. On hardware, expect the record to begin on byte 2.
2. **4 MHz is the testbench's ceiling, not the design's.** The model
   oversamples SCKI through a 2-flop synchronizer in the 24 MHz `SBCLKI`
   domain, so it needs an SCK half-period of ~3 core cycles (125 ns) to track
   edges at all; above that the *model* aliases and reports corruption that is
   not in the DUT. Rates above 4 MHz have to be checked on the board.

The design's own refill budget is a poll + an RX drain + the EBR settle cycle +
the load, ≈10 core cycles ≈ 417 ns at 24 MHz, which fits under ~10 MHz SCK.
Above that, TXEDGE (SPICR1 bit4) starts to matter.

## EBR read latency

The block RAM read is **synchronous**, so the byte for a new address is not
valid until the cycle after the address changes; `src_valid` drops for exactly
that one cycle. This is the same settle-state hazard that bit `can_sniffer.v`
when its drain was raised to 6 MHz (see the `S_SETTLE` comment there) — a
combinational-read FIFO hides it, an EBR does not. One dead core cycle is
41.7 ns against an 800 ns byte period at 10 MHz SCK, so it costs nothing here,
but it is the reason the pointer logic looks the way it does.
