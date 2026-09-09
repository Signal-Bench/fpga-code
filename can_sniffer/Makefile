# can_sniffer - CAN 2.0B bus sniffer for iCE40 UP5K / UPduino v3
#
# No top module or bitstream yet: only the bit timing block and its
# testbench exist.  Build/flash targets arrive with can_sniffer.v.
#
# NOTE: this design must be clocked from the UPduino's 12 MHz on-board
# oscillator (short jumper R16 / "OSC"), not SB_HFOSC.  See clock_choice.md.

bt_tb        = can_bit_timing_tb
pnr_freq_mhz = 12

sim:
	iverilog -g2012 -o $(bt_tb).out can_bit_timing.v $(bt_tb).v
	vvp $(bt_tb).out

wave:
	iverilog -g2012 -o $(bt_tb).out can_bit_timing.v $(bt_tb).v
	vvp $(bt_tb).out +vcd
	gtkwave can_bit_timing_tb.vcd

# Oscillator-tolerance sweep: drives the bit stream at a deliberately wrong
# rate and checks every bit still samples correctly.  pattern 0 = worst case
# inside the stuffed region (resync edge every 10 bits), pattern 1 = worst
# case across the frame tail (11 recessive bits, no resync edge at all).
sweep-timing:
	iverilog -g2012 -o $(bt_tb).out can_bit_timing.v $(bt_tb).v
	@for pat in 0 1; do \
		for ppm in 0 2000 5000 10000 20000 40000 -2000 -5000 -10000 -14000 -20000; do \
			vvp $(bt_tb).out +pattern=$$pat +offset_ppm=$$ppm; \
		done; \
	done

# Compare candidate segment configurations. Overrides go to the testbench,
# which passes them down to the DUT.
sweep-segments:
	@for cfg in "4 4 3" "3 4 4" "2 5 4" "5 4 2"; do \
		set -- $$cfg; \
		iverilog -g2012 -P $(bt_tb).PROP_SEG=$$1 -P $(bt_tb).PHASE_SEG1=$$2 \
			-P $(bt_tb).PHASE_SEG2=$$3 -o $(bt_tb).out can_bit_timing.v $(bt_tb).v; \
		vvp $(bt_tb).out +pattern=1 +offset_ppm=-14000; \
	done

clean:
	rm -rf $(bt_tb).out can_bit_timing_tb.vcd
