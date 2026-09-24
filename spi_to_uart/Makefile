filename = spi_to_uart
top_module = top
pcf_file = $(filename).pcf
tb = $(filename)_tb
pnr_freq_mhz = 30

build:
	yosys -p "synth_ice40 -top $(top_module) -json $(filename).json" $(filename).v
	nextpnr-ice40 --up5k --package sg48 --json $(filename).json --pcf $(pcf_file) --asc $(filename).asc --freq $(pnr_freq_mhz)
	icepack $(filename).asc $(filename).bin

sim:
	iverilog -g2012 -o $(tb).out $(filename).v $(tb).v
	vvp $(tb).out

wave:
	iverilog -g2012 -o $(tb).out $(filename).v $(tb).v
	vvp $(tb).out +vcd
	gtkwave $(tb).vcd

time: $(filename).asc
	icetime -d up5k -c $(pnr_freq_mhz) -mtr timing.rpt $(filename).asc
	@echo "Timing report written to timing.rpt. Check for PASS/FAIL at the bottom."

flash:
	iceprog -d i:0x0403:0x6014 $(filename).bin # Flash to FTDI

# Watch the decoded MOSI bytes.  1 Mbaud matches BAUD in spi_to_uart.v; close
# picocom before running `make flash`, they share the FTDI.
pico:
	picocom -b 1000000 /dev/ttyUSB0

clean:
	rm -rf $(filename).blif $(filename).asc $(filename).bin $(filename).json timing.rpt $(tb).out $(tb).vcd
