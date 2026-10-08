# spi_hw_stream - Raw SPI Stream

An iCE40 UP5K/UPduino v3 fabric SPI slave image. The directory name is retained
for compatibility; this image no longer instantiates the hard SPI block.

The source repeats the 12-byte ASCII string `Hello World!` without a terminator.
Bytes are produced at up to 3 kHz into a 64-entry FIFO. A full FIFO pauses
the synthetic source without advancing or overwriting queued data.

## Wire Contract

The master asserts CS and supplies mode-0, MSB-first clocks. Raw bytes appear
on MISO with no commands, handshake, headers, counts, ACKs, or markers. MOSI
is ignored. The ESP32 alone constructs the mobile TLVs; see
[`SPI_RAW_STREAM.md`](../SPI_RAW_STREAM.md).

Both images use [`spi_raw_tx.v`](../common/spi_raw_tx.v). It synchronizes SCK
and CS into the 24 MHz core clock, latches a byte at CS assertion or the
falling edge following a completed byte, and pops the FIFO only at the eighth
rising sampling edge. A byte interrupted by CS stays queued and restarts
from its MSB on the next transfer. A completed byte is committed even when
CS rises before the final falling edge. Preloading the next byte consumes
nothing, so repeated short transactions preserve the stream.

An empty byte is exactly `FF`. Data arriving during that byte waits until
the next boundary and cannot splice into the idle shifter. Binary `FF`
is also valid data; the MCU preserves all bytes unchanged.

## Wiring And Timing

| Signal | FPGA Pin | Direction |
| --- | --- | --- |
| SCK | 11 | In |
| CS | 19 | In, active low |
| MOSI | 21 | In, ignored |
| MISO | 13 | Out |
| Flash CS | 16 | Out, held high |

Use 1 MHz initially; 2 MHz is the maximum supported SCK with the 24 MHz core.
Each SCK high/low period must be at least six core clocks; CS setup, hold,
and high time must each be at least four core clocks (167 ns nominal).
The ESP32 default uses two SCK clocks of CS setup/hold and retains its
normal MISO sample point. Pin assignments are unchanged.

The source produces up to 3 kB/s. Reading 64 bytes every 10 ms provides
6.4 kB/s of wire capacity, so idle FF padding is expected. The generator
can pause under backpressure; a real capture engine may need more buffering.

## LEDs

| LED (Active Low) | Meaning |
| --- | --- |
| Green | Stream image running |
| Blue | A complete FIFO byte sampled by the master |
| Red | Synthetic source stalled by a full FIFO |

## Build And Verification

```sh
make sim
make build
make flash
```

`make flash` programs the FTDI device configured in the Makefile. Simulation
runs the actual fabric transmitter, with only the oscillator modeled. It
checks source backpressure and pattern continuity, all 256 binary values,
idle bytes, MOSI command patterns, partial CS transfers after each bit,
CS rising after the last sample edge, late data after underflow, repeated
one-byte transfers, and sustained 64-byte polling.

Run the compiled testbench with `+half=500 +phase=17` for 1 MHz with a phase
offset, or `+half=250 +phase=37` for 2 MHz. The old
`+drop_preloaded_on_cs` hard-IP model option has been removed.

Program the rebuilt FPGA bitstream and ESP32 firmware before capturing again.
Simulation and timing closure verify the logic; the target board still needs
a fresh capture to confirm MISO electrical timing and signal integrity.
