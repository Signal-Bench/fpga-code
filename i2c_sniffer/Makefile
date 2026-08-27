filename = i2c_sniffer
top_module = $(filename)
pcf_file = $(filename).pcf
i2c_sim_tb = i2c_sniffer_tb
i2c_sweep_tb = i2c_sniffer_start_sweep_tb
pnr_freq_mhz = 30

build:
	yosys -p "synth_ice40 -top $(top_module) -json $(filename).json" $(filename).v
	nextpnr-ice40 --up5k --package sg48 --json $(filename).json --pcf $(pcf_file) --asc $(filename).asc --freq $(pnr_freq_mhz)
	icepack $(filename).asc $(filename).bin

sim:
	iverilog -g2012 -o $(i2c_sim_tb).out i2c_sniffer.v $(i2c_sim_tb).v
	vvp $(i2c_sim_tb).out

wave:
	iverilog -g2012 -o $(i2c_sim_tb).out i2c_sniffer.v $(i2c_sim_tb).v
	vvp $(i2c_sim_tb).out +vcd
	gtkwave i2c_sniffer_tb.vcd

sweep-start:
	iverilog -g2012 -o $(i2c_sweep_tb).out i2c_sniffer.v $(i2c_sweep_tb).v
	@for gap in 1250 625 312 156 78 39 20 10 5 2 1 0; do \
		for phase in 0 5 10 15 20; do \
			vvp $(i2c_sweep_tb).out +start_gap_ns=$$gap +phase_ns=$$phase; \
		done; \
	done

time: $(filename).asc
	icetime -d up5k -c $(pnr_freq_mhz) -mtr timing.rpt $(filename).asc
	@echo "Timing report written to timing.rpt. Check for PASS/FAIL at the bottom."

flash:
	iceprog -d i:0x0403:0x6014 $(filename).bin # Flash to FTDI

pico:
	picocom -b 1000000 /dev/ttyUSB0

send_pattern: send_pattern.c
	gcc send_pattern.c -o send_pattern

clean:
	iceprog -d i:0x0403:0x6014 -e 128 # Erase flash
	rm -rf $(filename).blif $(filename).asc $(filename).bin $(filename).json timing.rpt send_pattern $(i2c_sim_tb).out $(i2c_sweep_tb).out i2c_sniffer_tb.vcd
