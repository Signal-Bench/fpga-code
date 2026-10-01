# SignalBench STREAM -- raw-byte streaming, decoder chosen by `53 42 4D xx`.
# Derived from ../final_system/.  See README.md.

top_module   = signalbench_stream
pcf_file     = signalbench_stream.pcf
pnr_freq_mhz = 24

# CAN from ../can_sniffer (PIPELINE_RESYNC opt-in), sb_spi from ../final_system.
rtl = ../can_sniffer/can_crc15.v ../can_sniffer/can_bit_timing.v \
      ../can_sniffer/can_frame_fsm.v ../final_system/sb_decoders.v \
      sbs_decoders.v sbs_framer.v sbs_link.v

build:
	yosys -p "synth_ice40 -top $(top_module) -json $(top_module).json" $(rtl) $(top_module).v
	nextpnr-ice40 --up5k --package sg48 --json $(top_module).json --pcf $(pcf_file) --asc $(top_module).asc --freq $(pnr_freq_mhz)
	icepack $(top_module).asc $(top_module).bin

flash:
	iceprog -d i:0x0403:0x6014 $(top_module).bin

time: $(top_module).asc
	icetime -d up5k -c $(pnr_freq_mhz) -mtr timing.rpt $(top_module).asc
	@echo "Timing report written to timing.rpt. Check PASS/FAIL at the bottom."

synth-check:
	yosys -p "read_verilog $(rtl) $(top_module).v; synth_ice40 -top $(top_module)"

# One nextpnr run is not evidence of closure; every seed must pass.
seeds:
	@for s in 1 2 3 4 5 6; do \
		printf "seed %s: " $$s; \
		nextpnr-ice40 --up5k --package sg48 --json $(top_module).json \
		  --pcf $(pcf_file) --asc /dev/null --freq $(pnr_freq_mhz) --seed $$s 2>&1 \
		  | grep -E "Max frequency for clock 'clk" | tail -1; \
	done

sim:
	iverilog -g2012 -o $(top_module)_tb.out $(rtl) $(top_module).v $(top_module)_tb.v
	vvp $(top_module)_tb.out

# Production payload limit, with an I2C transaction long enough to split.
sim-255:
	iverilog -g2012 -P signalbench_stream_tb.MAXP=255 -P signalbench_stream_tb.FIFO_AW=12 -o $(top_module)_tb255.out $(rtl) $(top_module).v $(top_module)_tb.v
	vvp $(top_module)_tb255.out

wave:
	iverilog -g2012 -o $(top_module)_tb.out $(rtl) $(top_module).v $(top_module)_tb.v
	vvp $(top_module)_tb.out +vcd
	gtkwave $(top_module)_tb.vcd

clean:
	rm -f $(top_module).json $(top_module).asc $(top_module).bin timing.rpt *.out *.vcd
