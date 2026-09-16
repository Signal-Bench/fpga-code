# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A capstone project (**SignalBench**) targeting the Lattice iCE40 UP5K FPGA on an
UPduino v3 board, paired with an ESP32-H2 MCU and a mobile app over BLE. The active RTL
work in this repo is `i2c_sniffer/` and `can_sniffer/`, both complete and
hardware-tested: bus sniffers that capture traffic and drain it out over a bit-banged
SPI link to a host. The repo also contains several
earlier/simpler UART↔SPI bridge designs built along the way (each its own protocol
analyzer, one per directory), plus vendored example collections used as reference
material. See "Target System Architecture" below for the full planned system this
repo is building toward — most of it (ESP32 link, trigger subsystem, non-I2C decoders)
isn't implemented in code yet.

## Target System Architecture (SignalBench)

This section is summarized from the capstone report and describes the *planned* full
system. `i2c_sniffer/` and `can_sniffer/` are complete, hardware-tested pieces of it
(both decoders verified against real buses, both draining records over SPI). The
ESP32-H2 state-management link, the trigger subsystem, and the digital-I/O / RS-232 /
RS-485 / SPI decoders below are design targets, not modules that exist in this repo
yet, unless a note says otherwise. Don't assume code implementing this exists — check
before referencing it.

Five subsystems: sampling, decoding, trigger, recording, and state management.

- **Sampling** — each GPIO input goes through a 2-stage synchronizer plus a 3rd
  register for edge detection (rising/falling), matching the pattern already used in
  `i2c_sniffer.v`. UART, digital I/O, I2C, and SPI probe the target directly via hook
  probes; CAN, RS-232, and RS-485 go through a line transceiver first for voltage
  translation to 3.3 V logic.
- **Decoding** — one decoder per protocol:
  - *Digital I/O* — simplest decoder: timestamps rising/falling edges on a single pin,
    for signals that don't conform to any framed protocol.
  - *UART / RS-232 / RS-485* — one shared decoder (the RS-232/485 variants only differ
    at the physical layer, before the transceiver output reaches the synchronizer).
    Detects a start bit (falling edge on an idle-high line), samples the first data bit
    at 1.5 bit periods in, then every 1 bit period after, and flags framing errors.
  - *I2C* — matches `i2c_sniffer.v`'s already-implemented behavior: START/STOP via
    SDA/SCL edge relationships, SDA sampled on SCL rising edges, ACK/NACK tracked on
    the 9th clock, address byte + R/W bit split into frame metadata, and a sentinel
    byte inserted on repeated START.
  - *SPI* — **not yet tested on hardware**, planned only: idle until CS asserted low,
    sample MOSI/MISO on the configured clock edge, assemble bytes every 8 clocks, end
    the transaction on CS high.
- **Trigger** — for buses above 1 MHz, where continuous BLE streaming to the mobile app
  can't keep up. The MCU writes a hex trigger pattern over SPI; the FPGA compares it
  against decoder output on a rolling basis, and on a match starts recording payload
  bytes into the UP5K's 120 Kbit embedded block RAM until the buffer fills or a
  protocol-defined end condition hits, then sets a recording-done status flag for the
  MCU to drain.
- **State management** — SPI link between the FPGA and the **ESP32-H2 MCU**, where
  **the MCU is SPI master and the FPGA is SPI slave** — the opposite role from the
  FPGA-as-master pattern used by `uart_to_spi.v` / `host_to_spi.v` / `clock_out.v`
  elsewhere in this repo. Commands are a single byte, optionally followed by a data
  byte for parameterized ops (baud rate, trigger pattern). This SPI slave logic is
  meant to run as its own state machine independent of decode/record, so the MCU can
  reconfigure the FPGA at any time and it takes effect on the next decoder clock cycle.
  Write-only config registers: protocol select, baud rate, trigger pattern. Read-only
  status register: idle / recording-in-progress / streaming-in-progress /
  active-protocol fields, plus a recording-done flag once a triggered capture
  finishes. Two operating modes, each entered via a request-then-start-command
  handshake: **streaming** (decoded frames sent to the MCU as they arrive) and
  **recording** (trigger-armed capture into EBR, drained after the fact).

The closest existing building block for the state-management SPI-slave role is
[`spi_slave/spi_slave.v`](spi_slave/spi_slave.v) (write/read queues, MISO shift-out) —
but its protocol (fixed 32-bit frames, init-opcode handshake) doesn't match the
single-command-byte design described above, so it'd need adapting rather than reuse
as-is. The report's "picocom + companion C utility" integration-testing setup already
corresponds to this repo's `make pico` target and `tools/send_pattern.c`.

[`state_management.md`](state_management.md) works through this SPI command protocol
in more detail: step-by-step transaction traces for recording (with and without a
trigger pattern), streaming, and a proposal for loading multi-byte trigger patterns
given the one-command-byte-plus-one-data-byte format. It's draft/proposal, not spec —
treat any `CMD_...` name in it as a placeholder, and check its "Open questions"
section before assuming a detail (opcode values, drain byte format, MOSI vs. MISO
direction) is settled.

Toolchain: open-source icestorm flow (`yosys`, `nextpnr-ice40`, `icepack`, `icetime`,
`iceprog`) and `iverilog`/`vvp`/`gtkwave` for simulation. `UPduino-v3.0/`,
`ice40_ultraplus_examples/`, and `up5k/` are separately-cloned repos, each with their
own `.git` (vendored reference material) — all three are gitignored here, treat them
as read-only, local-only reference.

Build artifacts (`.blif`/`.asc`/`.bin`/`.json`/`.out`/`.vcd`/`.rpt`/`.log`) are
gitignored throughout the repo — they're regenerated by `make build`/`make sim`/`make
time` in each design directory, don't expect them to be present after a fresh clone.

## Layout

```
common/            shared includes: uart.v, util.v, and the general upduino_v3.pcf
i2c_sniffer/        primary design — see below
can_sniffer/        CAN 2.0B sniffer — complete, hardware-tested against a GM6020 (see below)
uart_to_spi/        UART sniffer -> SPI bridge (structural ancestor of i2c_sniffer)
host_to_spi/        UART-to-host bridge that drives a real SPI *master* transaction
host_to_fpga/        UART echo + inverted-byte-response demo, used to validate the link
clock_out/           UART RX -> bit-banged SPI TX bridge (earlier/simpler than uart_to_spi)
clock_test/          minimal design: divides 48 MHz HFOSC down to ~1 MHz on a pin
spi_hw_stream/       hard-IP (SB_SPI) SLAVE bring-up test — streams a 500 kHz counter
spi_slave/           standalone SPI slave register interface, not wired into any top module
tools/               send_pattern.c — host-side UART test-pattern generator
ice40_ultraplus_examples/   vendored example collection (own Makefiles/README)
up5k/                        earlier UPduino v2 icestorm demos (own Makefile)
UPduino-v3.0/                 vendored board reference repo (gitignored, not pushed)
docs/               vendor datasheets and standards, local-only (gitignored — see below)
notes.md            informal hardware-debugging notes (bit order, polarity, known bugs)
```

Each design directory that has a `top` module (everything except `spi_slave/`, which
is a reusable block not yet wired up) has its own `Makefile` following the same
three-step icestorm pattern, and `` `include``s the shared `common/uart.v` /
`common/util.v` via a relative `../common/` path.

## Primary design: i2c_sniffer

All commands run from `i2c_sniffer/`:

```
make build          # yosys synth -> nextpnr (up5k, sg48, 30 MHz) -> icepack -> i2c_sniffer.bin
make sim            # iverilog + vvp against i2c_sniffer_tb.v
make wave            # same as sim but dumps i2c_sniffer_tb.vcd and opens gtkwave
make sweep-start     # builds i2c_sniffer_start_sweep_tb.v, sweeps start_gap_ns x phase_ns
make time            # icetime timing report -> timing.rpt (check PASS/FAIL at the bottom)
make flash           # iceprog -d i:0x0403:0x6014 i2c_sniffer.bin (specific FTDI VID:PID)
make pico            # picocom -b 1000000 /dev/ttyUSB0
make send_pattern    # gcc send_pattern.c -o send_pattern (source now lives in ../tools/)
make clean           # erases the FPGA flash AND removes build artifacts — destructive, confirm before running
```

Run a single test by editing which testbench file gets compiled, or invoke iverilog
directly from `i2c_sniffer/`, e.g.:
```
iverilog -g2012 -o i2c_sniffer_tb.out i2c_sniffer.v i2c_sniffer_tb.v && vvp i2c_sniffer_tb.out
```
Pass `+vcd` on the `vvp` command line to get a waveform dump from either testbench.

Testbenches (`i2c_sniffer_tb.v`, `i2c_sniffer_start_sweep_tb.v`) define their own
behavioral models for `SB_HFOSC` and `SB_GB` at the top of the file (there's no vendor
sim library in this repo) — keep those in sync with any real usage of those primitives.

### Architecture

`i2c_sniffer.v` is a ping-pong-buffered design with two independent state machines
connected by a small queue (see the module-header comment in the file for the full
picture):

- **Clocking**: `SB_HFOSC` gives 48 MHz; a single toggle flip-flop divides it to a
  24 MHz `clk_core` that all sequential logic runs on. This division was added
  specifically to close timing at 48 MHz — see "Timing closure" in `i2c_sniffer.md`.
- **Input synchronization**: SCL/SDA are asynchronous and go through 3-stage
  synchronizers; START/STOP conditions are derived from the synchronized edges
  (`sda_falling & scl_high` = START, `sda_rising & scl_high` = STOP).
- **Capture RAM**: one inferred EBR, 512 bytes organized as 4×128-byte buffers,
  addressed as `{buf_sel[1:0], offset[6:0]}`. Capture always writes into whichever
  buffer the drain side isn't using; `buf_sel` increments after every STOP.
- **Drain queue**: 3-entry FIFO of `{buf_sel, len}`. Capture pushes on STOP; the SPI
  drain state machine pops when idle. Push always wins over pop in the same cycle to
  avoid reading a slot mid-write. Captures are dropped if the queue is full.
- **SPI drain**: sends a leading `0xFF` sentinel byte, then the raw captured bytes,
  over the same SPI pins reused from `uart_to_spi.v`.
- Repeated START mid-capture inserts an `0xFF` marker byte into RAM rather than
  resetting the capture, so `START ... START ... STOP` sequences are preserved in one
  buffer.

`i2c_sniffer.md` is a design journal (goals, bugs found, fixes applied) written during
development — useful for *why* decisions were made, but treat it as historical notes
rather than a spec: the current `i2c_sniffer.v` (ping-pong/queue variant) is more
sophisticated than the single-buffer description in the "Intended Behavior" section of
that file. When the two disagree, the `.v` is authoritative.

`notes.md` (repo root) has informal hardware-debugging notes (bit order, polarity,
known bugs from specific dates) — check it before re-debugging something already
characterized on hardware.

## CAN sniffer: can_sniffer

Passive CAN 2.0B sniffer, successor to `i2c_sniffer`. **Complete and hardware-tested**
(2026-09-16) against a RoboMaster GM6020 motor driven by a Development Board Type C at
1 Mbit/s: bit timing, CRC-15, frame decoder, 20-byte record packer, 16-record EBR ring,
and a 6 MHz bit-banged SPI master drain, with records verified on a logic analyzer
against the CAN waveform. 1292 LCs (24% of the UP5K), one EBR, timing closes at
~21.8 MHz against the 12 MHz constraint.

All commands run from `can_sniffer/`:

```
make build            # yosys -> nextpnr (up5k, sg48, 12 MHz) -> icepack -> can_sniffer.bin
make flash            # iceprog -d i:0x0403:0x6014 can_sniffer.bin
make time             # icetime timing report -> timing.rpt
make sim              # all three testbenches
make sim-timing       # bit recovery only, at the nominal rate
make sim-frame        # decoder, 16 tests, own CRC + stuffing
make sim-top          # end to end: CAN bits in, SPI records out, sampled like an analyzer
make synth-check      # yosys resource estimate for the top
make sweep-timing     # oscillator-offset sweep, both worst-case patterns
make sweep-segments   # compares candidate bit-time segment configurations
make build-bringup    # reduced top (decoder + LEDs, no drain) for isolating faults
make clean            # removes build/sim artifacts only (non-destructive, unlike i2c_sniffer)
```

Two top modules: `can_sniffer.v` is the real one; `can_bringup.v` is the same decoder
with only LEDs and a scope-trigger pin, kept for isolating "is it the decoder or the
drain". Both share `can_sniffer.pcf`. The SPI drain uses the same pins as
`uart_to_spi`/`spi_hw_stream` (FPGA 11/19/21) but the FPGA is **master** here and
there is no MISO.

**Record format** (20 bytes, MSB first, SPI mode 0, one record per CS assertion):
`AA | flags | id[4] | dlc | data[8] | timestamp[4] | 55`. Flags are
`{ide, rtr, overload, ack_ok, crc_ok, err[2:0]}`; `crc_ok` reports the CRC comparison
alone, so a no-ACK frame reads `err=4, ack_ok=0, crc_ok=1`. Timestamps count **bit
times** latched at SOF, so at 1 Mbit/s the unit is 1 µs. Full layout and the
error-code table are in `can_sniffer/can_sniffer.md`.

`can_frame_fsm_tb.v` and `can_sniffer_tb.v` build real frames from scratch — computing
CRC-15 and applying bit stuffing independently of the DUT — so they are genuine
cross-checks, not round-trips against the same code.

### The one thing that makes CAN different from every other design here

**`can_sniffer` must be clocked from the UPduino's 12 MHz on-board oscillator (short
jumper R16, silkscreen "OSC", arrives on `gpio_20`) — NOT `SB_HFOSC`.** Every other
design in this repo uses `SB_HFOSC`, which is fine for them because I2C and UART either
carry a clock or are heavily oversampled. CAN is NRZ with no clock line, so ISO 11898-1
clause 11.3.2.5 imposes a real accuracy budget: **df < 0.98%** for this configuration,
while `SB_HFOSC` is spec'd at 48 MHz ±10% commercial / ±20% industrial (Lattice
FPGA-DS-02008 Table 4.11). It would fail *data-dependently* — some frames decoding and
some not — which is the expensive kind of hardware bug.

`can_sniffer/clock_choice.md` is the full derivation, written to be self-contained:
what the bit-time segments and SJW are, the ISO equations, the simulation results, and
why 12 tq/bit was chosen over PLL-ing to 24 MHz. Read it before touching any timing
parameter. `can_sniffer/can_sniffer.md` is the design doc for everything else — frame
format, record layout, which errors a passive sniffer can and cannot detect, LED map,
transceiver wiring, pin plan.

### Notes that are easy to get wrong

- **Bit stuffing** covers SOF through the CRC *sequence*; the CRC delimiter, ACK field
  and EOF are fixed-form and not stuffed. **CRC-15** (poly `0x4599`) is computed over
  the *destuffed* stream from SOF through the end of the *data field* — not including
  the CRC sequence itself. Two different spans, easy to conflate.
- Reserved bits `r0`/`r1` must **not** be flagged as form errors — the spec says
  receivers accept dominant and recessive in all combinations.
- A **bit error** cannot be detected passively at all; it can only be inferred by
  elimination and should be reported as "unattributed error".
- **Overload frames** are not errors (they are flow control) but the FSM still has to
  parse them, or it loses frame sync.
- The test bus is a GM6020 motor: 1 Mbit/s, **standard frames only**, DLC 8. Extended
  (29-bit) frames are supported in the decoder but the bench rig will never exercise
  them — they are covered by the synthetic testbenches only.
- **The VP230 transceiver must NOT have RS tied high.** TI's "Standby (Listen Only)"
  mode leaves RXD stuck recessive at 1 Mbit/s, despite what the datasheet implies.
  Leave RS on the breakout's 10 kΩ (slope-control mode) and hard-wire TXD to 3.3 V —
  that strap is the entire passivity guarantee. Verified on hardware; details in
  `notes.md` and `can_sniffer/can_sniffer.md`.
- **SPI drain rate is 6 MHz**, the ceiling from a 12 MHz clock. Raising it exposed a
  latent EBR settle-state bug that the old 1 MHz tick had hidden — see the `S_SETTLE`
  comment in `can_sniffer.v` before touching the drain FSM.
- UPduino header **position 2 is `VIO`, not a 3.3 V supply**; the 3.3 V rail is
  position 9. `VIO` floats by default and meters as ~2.3 V of leakage.

## Other protocol analyzers / stepping-stone designs

These predate `i2c_sniffer.v` and were stepping stones toward it (see "What Was Reused"
in `i2c_sniffer/i2c_sniffer.md`). Each has its own `Makefile` in its directory
following the same three-command pattern (top module is always named `top`, not the
directory name):

```
yosys -p "synth_ice40 -top top -json <name>.json" <name>.v
nextpnr-ice40 --up5k --package sg48 --json <name>.json --pcf ../common/upduino_v3.pcf --asc <name>.asc
icepack <name>.asc <name>.bin
```

- `clock_test/` — minimal design, just divides the 48 MHz HFOSC down to ~1 MHz on a pin.
- `clock_out/` — UART RX -> bit-banged SPI TX bridge (sniffs one UART line, forwards bytes over SPI).
- `uart_to_spi/` — sniffs two UART lines (TX and RX of a target) and sends `[0x48 header][flag][data]` over SPI. Structural ancestor of `i2c_sniffer.v` (oscillator, SPI state machine, LED pulse-stretcher pattern all reused directly).
- `host_to_fpga/` — UART echo + inverted-byte-response demo (`"N <data>\n"` protocol), used to validate the UART link itself.
- `host_to_spi/` — UART-to-host bridge that bit-bangs a real SPI *master* transaction (vs. the sniffers, which only ever drive MOSI) and reports MISO data back over UART.
- `spi_hw_stream/` — bring-up test for the UP5K's **hardened** SPI block (`SB_SPI`) as a **slave**, the only design here where the FPGA is not the SPI master. Streams a 500 kHz incrementing byte counter through a 64-entry FIFO so a master can verify the link. Has its own `README.md` (wiring, LED meanings, rate math, and the hardware questions it's meant to answer) and a `sim` target whose testbench stubs `SB_SPI` behaviorally — that model is *not* silicon-accurate, so don't treat its passing as proof the real IP behaves the same way.
- `spi_slave/` — a fuller-featured **soft** SPI slave register interface (separate read/write queues); not currently wired into any `top` module in this repo, and has no Makefile of its own.
- `common/uart.v` / `common/util.v` — shared building blocks (`uart_tx`/`uart_rx`, `divide_by_n`, `fifo`, `pulse_stretcher`, etc.), pulled in via `` `include "../common/..." `` by nearly every design above.

Pin constraints: `common/upduino_v3.pcf` is the general-purpose UPduino v3 pinout (used
by the designs above); `i2c_sniffer/i2c_sniffer.pcf` is sniffer-specific and documents
its own pin assumptions (SCL=38, SDA=42, SPI pins reused from `uart_to_spi.v`). All
designs explicitly drive the onboard SPI flash chip-select high (`spi_cs = 1` /
`spi_cs_flash = 1'b1`) since it shares pins with other functions and must be disabled
to avoid bus contention.

## Reference documents: docs/

`docs/` holds vendor datasheets and standards used during design — Lattice iCE40
datasheets and technical notes, the Bosch CAN 2.0 spec, ISO 11898-1:2015/2024, the TI
SN65HVD230 and TCAN330 transceiver datasheets, and the GM6020 manual. It is
**gitignored and local-only** (~54 MB), for two reasons: the ISO copies are
institution-licensed and marked "no further reproductions authorized", and vendor PDFs
do not belong in a repo whose RTL is a few hundred KB. The existing `ice_docs/` ignore
line reflects the same decision.

Design docs cite these by filename. If a needed document is missing, fetch the official
version rather than answering hardware questions from memory.

## Vendored/reference subdirectories

All three of these are separately-cloned git repos (each has its own nested `.git`) and
are gitignored here — don't try to commit inside them from this repo.

- `ice40_ultraplus_examples/` — a standalone collection of independent iCE40 UltraPlus
  examples (7seg, BRAM/SPRAM, DSP MAC16, flash, PLL, RISC-V soft core, USB, etc.), each
  with its own `Makefile` (`make`, `make prog`, `make prog_flash`) and `README.md`. Used
  as reference, not part of the sniffer build.
- `up5k/` — earlier UPduino v2 icestorm demos (blink, pulse, serial, spram) with their
  own `Makefile`; note its `build` target still hardcodes `filename = serial`, so other
  demos there need the `filename` var overridden or the Makefile edited to build them.
- `UPduino-v3.0/` — vendored copy of the tinyVision.ai UPduino v3.x board repo (schematics,
  pinout, board rev docs). Reference material for the physical board, not part of this
  project's RTL.
