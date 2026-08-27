# fpga-code

FPGA capstone project targeting the Lattice iCE40 UP5K (UPduino v3 board), built with
the open-source icestorm toolchain (`yosys` / `nextpnr-ice40` / `icepack` / `icetime` /
`iceprog`).

## Protocol analyzers / designs

| Directory | What it does |
|---|---|
| [`i2c_sniffer/`](i2c_sniffer/) | Primary design. Captures I2C SCL/SDA traffic (START/STOP detection, ping-pong buffered capture RAM) and drains it out over SPI to a host. See [`i2c_sniffer/i2c_sniffer.md`](i2c_sniffer/i2c_sniffer.md) for the design journal. |
| [`uart_to_spi/`](uart_to_spi/) | Sniffs two UART lines (TX/RX of a target device) and forwards bytes over SPI with a header/flag. Structural ancestor of `i2c_sniffer`. |
| [`host_to_spi/`](host_to_spi/) | UART-to-host bridge that drives a real SPI *master* transaction and reports MISO data back over UART. |
| [`host_to_fpga/`](host_to_fpga/) | UART echo + inverted-byte-response demo, used to validate the host UART link. |
| [`clock_out/`](clock_out/) | Earlier/simpler UART RX -> bit-banged SPI TX bridge. |
| [`clock_test/`](clock_test/) | Minimal design: divides the 48 MHz internal oscillator down to ~1 MHz on a pin. |
| [`spi_slave/`](spi_slave/) | Standalone SPI slave register interface; not yet wired into a top-level design. |

`common/` holds shared Verilog includes (`uart.v`, `util.v`) and the general UPduino v3
pin constraint file, used by most of the designs above via `` `include "../common/..." ``.

`tools/` has `send_pattern.c`, a small host-side utility that writes a repeating test
pattern out a serial port.

## Building

Each design directory has its own `Makefile`. From inside a design directory:

```
make build   # synth -> place & route -> bitstream
make flash   # program over FTDI (iceprog)
```

`i2c_sniffer/` additionally has `make sim`, `make wave`, `make sweep-start`, and
`make time` for simulation, waveform viewing, START-timing sweeps, and static timing
analysis. See [`CLAUDE.md`](CLAUDE.md) for the full command reference and architecture
notes.

## Reference material (not part of the build)

`ice40_ultraplus_examples/`, `up5k/`, and `UPduino-v3.0/` are each separately-cloned
git repos (vendored iCE40 UltraPlus examples, earlier UPduino v2 icestorm demos, and
the UPduino v3.x board reference repo, respectively). None are tracked in this
repository — clone them separately if needed:

- `ice40_ultraplus_examples/` — https://github.com/damdoy/ice40_ultraplus_examples
- `up5k/` — https://github.com/osresearch/up5k
- `UPduino-v3.0/` — https://github.com/tinyvision-ai-inc/UPduino-v3.0
