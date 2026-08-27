# I2C Sniffer Project Notes

## Goal

The design goal was to build an I2C sniffer for the iCE40 UP5K in Verilog that:

- samples `SCL` and `SDA` using the internal 48 MHz oscillator
- detects I2C `START` and `STOP` conditions
- captures the observed I2C bitstream into inferred embedded block RAM
- drains the captured data out over SPI after a `STOP`
- uses the same SPI pins and general structure as `uart_to_spi.v`

The top-level implementation lives in `i2c_sniffer.v`.

## Intended Behavior

The sniffer was designed around four capture states:

- `CAP_IDLE`: wait for a `START`
- `CAP_ACTIVE`: capture SDA on each SCL rising edge and write packed bytes to RAM
- `CAP_STOP_DETECTED`: prepare the SPI dump
- `CAP_SPI_DRAIN`: clock out a sentinel byte followed by the captured RAM contents

Key format choices:

- RAM stores the raw packed SDA bitstream rather than fully decoded I2C fields
- SPI sends a leading `0xFF` sentinel before the captured bytes
- repeated `START` inside an active capture inserts `0xFF` into RAM as a marker

## What Was Reused

The design intentionally reused patterns from `uart_to_spi.v`:

- the `SB_HFOSC` 48 MHz oscillator block
- the SPI state machine structure
- the debug LED pulse stretcher pattern
- the same SPI output pins used by the UART-to-SPI bridge

## Main Roadblocks

### 1. Missing or marginal first START detection

On hardware, the SPI output sometimes showed only the read phase of a repeated-start I2C transaction. That strongly suggested the first `START` for the write phase was missed and the sniffer only armed on the repeated `START`.

What likely contributed:

- `SCL` and `SDA` are asynchronous inputs
- `START` and `STOP` were derived from synchronized edge relationships
- very tight timing between SDA and SCL changes could cause the synchronizer to miss the intended condition

What was done:

- kept the required 3-stage synchronizers
- added a dedicated sweep testbench to vary `START` timing and phase against the FPGA clock

### 2. Repeated START behavior

The user needed support for traffic shaped like:

- `START ... START ... STOP`

The active-state logic already preserved existing data and continued capture across repeated `START`, but repeated-start timing exposed edge-detection sensitivity and packing corner cases.

What was done:

- confirmed that `CAP_ACTIVE` does not reset the buffer on repeated `START`
- inserted a repeated-start marker byte `0xFF` into RAM
- expanded the testbench to explicitly cover write-then-read repeated-start traffic

### 3. ACK immediately before STOP was not stored

Another bug showed up when an ACK bit occurred right before `STOP`. In that case, the code could take the `stop_cond` branch before folding in the last sampled bit, which caused the final ACK to be lost.

What was done:

- updated the stop path to account for the case where `STOP` and the last `scl_rising` sample land in the same observation window
- added a regression test for a tight ACK-to-STOP transition

### 4. Timing closure at 48 MHz

After functional fixes, the design no longer met timing at the full 48 MHz internal oscillator rate. The critical path was in the capture/write logic around stop handling and byte packing.

What was done:

- tried relaxing `nextpnr` frequency alone, but that was not enough because the actual oscillator was still 48 MHz
- moved the heavy capture/SPI logic onto a divided internal core clock derived from the 48 MHz oscillator
- kept the oscillator block itself unchanged
- updated the Makefile timing/build constraint to 30 MHz for the slower core domain

This brought the build back into a passing state.

## Things Tried During Debugging

The following approaches were tried during the design/debug cycle:

- building the initial sniffer around synchronized `START`/`STOP` detection and inferred EBR
- writing a main testbench to validate normal traffic, repeated `START`, SPI drain, and overflow behavior
- changing the first transaction example to use:
  - address `0x36`
  - write, `ACK`, data `0x0B`
- changing repeated-start tests to use:
  - `START 0x36 W ACK 0x0B ACK START 0x36 R ACK 0x93 NACK STOP`
- adding a timing sweep testbench that tightens the gap between SDA and SCL during `START`
- fixing the ACK-before-STOP corner case
- moving the main logic to a slower derived core clock to recover timing margin

## Current Known Design Characteristics

- capture RAM is inferred as one EBR, not explicit SPRAM
- SPI drains the entire capture after `STOP`
- the design captures raw packed SDA bits, so the I2C read/write bit is present but not broken out separately
- repeated `START` is intended to be preserved within one capture session
- there is a 1 ms idle timeout while in `CAP_ACTIVE`

## Files Written or Updated

Primary source files written or updated during this work:

- `i2c_sniffer.v`
  - main RTL for the I2C sniffer
- `i2c_sniffer_tb.v`
  - main regression testbench
- `i2c_sniffer_start_sweep_tb.v`
  - timing-sweep testbench for marginal `START` behavior
- `i2c_sniffer.pcf`
  - UPduino pin constraints for the sniffer
- `Makefile`
  - build, sim, waveform, sweep, and timing targets for the sniffer flow

Reference/input files used during the work:

- `uart_to_spi.v`
  - structural reference for oscillator/SPI/LED patterns
- `notes.md`
  - informal working notes

Generated artifacts produced by the flow:

- `i2c_sniffer.asc`
- `i2c_sniffer.bin`
- `i2c_sniffer.json`
- `i2c_sniffer_tb.out`
- `i2c_sniffer_tb.vcd`
- `i2c_sniffer_start_sweep_tb.out`
- `timing.rpt`
- `i2c_sniffer_timing.rpt`
- `i2c_sniffer_yosys.log`
- `i2c_sniffer_nextpnr.log`

## Pin Assumptions

The current pin constraints assume:

- `scl_pin` on FPGA pin `38`
- `sda_pin` on FPGA pin `42`
- SPI pins reused from the UART design:
  - `spi_sck` on `11`
  - `spi_mosi` on `21`
  - `spi_cs` on `19`

Those assignments are defined in `i2c_sniffer.pcf`.

## Verification Summary

The project was exercised with:

- `make sim`
- `make build`
- `make time`
- `make sweep-start`

The sweep testing was especially useful for showing that tightening the SDA-to-SCL timing could reproduce missing-start behavior in simulation.

## Environment Note

There was an attempt to use:

- `distrobox enter --root ubuntu-22-04`

from the coding environment, but it hit an interactive `sudo` prompt and was not usable through the non-interactive tool path. Verification was therefore completed with the host toolchain from the project workspace.
