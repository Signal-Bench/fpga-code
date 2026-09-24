# SignalBench unified top -- all four sniffers + MCU link in one bitstream.
#
# See ../unified_top.md for the design and CHANGES.md for what was modified
# in the existing sniffers.

top_module   = signalbench
pcf_file     = signalbench.pcf
pnr_freq_mhz = 24

# CAN decoder is reused from ../can_sniffer/ unmodified apart from the
# opt-in PIPELINE_RESYNC parameter.
can_rtl = ../can_sniffer/can_crc15.v ../can_sniffer/can_bit_timing.v ../can_sniffer/can_frame_fsm.v
rtl     = $(can_rtl) sb_decoders.v sb_capture.v sb_mcu_link.v

build:
	yosys -p "synth_ice40 -top $(top_module) -json $(top_module).json" $(rtl) $(top_module).v
	nextpnr-ice40 --up5k --package sg48 --json $(top_module).json --pcf $(pcf_file) --asc $(top_module).asc --freq $(pnr_freq_mhz)
	icepack $(top_module).asc $(top_module).bin

flash:
	iceprog -d i:0x0403:0x6014 $(top_module).bin

time: $(top_module).asc
	icetime -d up5k -c $(pnr_freq_mhz) -mtr timing.rpt $(top_module).asc
	@echo "Timing report written to timing.rpt. Check PASS/FAIL at the bottom."

# Resource estimate without running place and route.
synth-check:
	yosys -p "read_verilog $(rtl) $(top_module).v; synth_ice40 -top $(top_module)"

# A single nextpnr run is not evidence of timing closure -- placement is
# seed-dependent and this design sits close enough to the limit to care.
# Every seed must pass.
seeds:
	@for s in 1 2 3 4 5 6; do \
		printf "seed %s: " $$s; \
		nextpnr-ice40 --up5k --package sg48 --json $(top_module).json \
		  --pcf $(pcf_file) --asc /dev/null --freq $(pnr_freq_mhz) --seed $$s 2>&1 \
		  | grep -E "Max frequency for clock 'clk" | tail -1; \
	done

clean:
	rm -f $(top_module).json $(top_module).asc $(top_module).bin timing.rpt *.out *.vcd

sim:
	iverilog -g2012 -o signalbench_tb.out $(rtl) $(top_module).v signalbench_tb.v
	vvp signalbench_tb.out

wave:
	iverilog -g2012 -o signalbench_tb.out $(rtl) $(top_module).v signalbench_tb.v
	vvp signalbench_tb.out +vcd
	gtkwave signalbench_tb.vcd
