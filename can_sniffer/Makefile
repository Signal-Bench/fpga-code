# can_sniffer - CAN 2.0B bus sniffer for iCE40 UP5K / UPduino v3
#
# No top module or bitstream yet: bit timing, CRC-15 and the frame decoder
# exist and simulate.  Build/flash targets arrive with can_sniffer.v (record
# packing, EBR ring, SPI drain, LEDs).
#
# NOTE: this design must be clocked from the UPduino's 12 MHz on-board
# oscillator (short jumper R16 / "OSC"), not SB_HFOSC.  See clock_choice.md.

bt_tb        = can_bit_timing_tb
fr_tb        = can_frame_fsm_tb
rtl          = can_crc15.v can_frame_fsm.v can_bit_timing.v
top_module   = can_sniffer
pcf_file     = can_sniffer.pcf
pnr_freq_mhz = 12

# Full bitstream: decoder + record packer + EBR ring + SPI drain + LEDs.
build:
	yosys -p "synth_ice40 -top $(top_module) -json $(top_module).json" $(rtl) $(top_module).v
	nextpnr-ice40 --up5k --package sg48 --json $(top_module).json --pcf $(pcf_file) --asc $(top_module).asc --freq $(pnr_freq_mhz)
	icepack $(top_module).asc $(top_module).bin

flash:
	iceprog -d i:0x0403:0x6014 $(top_module).bin

# Reduced bitstream: clock + pin + decoder + LEDs only, no SPI drain. Useful
# for isolating a problem to the decoder when the drain is misbehaving.
build-bringup:
	yosys -p "synth_ice40 -top can_bringup -json can_bringup.json" $(rtl) can_bringup.v
	nextpnr-ice40 --up5k --package sg48 --json can_bringup.json --pcf $(pcf_file) --asc can_bringup.asc --freq $(pnr_freq_mhz)
	icepack can_bringup.asc can_bringup.bin

flash-bringup:
	iceprog -d i:0x0403:0x6014 can_bringup.bin

time: $(top_module).asc
	icetime -d up5k -c $(pnr_freq_mhz) -mtr timing.rpt $(top_module).asc
	@echo "Timing report written to timing.rpt. Check PASS/FAIL at the bottom."

sim: sim-timing sim-frame sim-top

# Bit recovery only: nominal rate, no frames.
sim-timing:
	iverilog -g2012 -o $(bt_tb).out can_bit_timing.v $(bt_tb).v
	vvp $(bt_tb).out

# Full decode chain. The testbench builds real frames -- it computes CRC-15 and
# applies bit stuffing itself -- then checks what the decoder produces.
sim-frame:
	iverilog -g2012 -o $(fr_tb).out $(rtl) $(fr_tb).v
	vvp $(fr_tb).out

# End to end: CAN bit stream in, SPI records out, checked the way a logic
# analyzer would sample them.
sim-top:
	iverilog -g2012 -o can_sniffer_tb.out $(rtl) can_sniffer.v can_sniffer_tb.v
	vvp can_sniffer_tb.out

synth-check:
	yosys -p "read_verilog $(rtl) can_sniffer.v; synth_ice40 -top can_sniffer"

wave:
	iverilog -g2012 -o $(bt_tb).out can_bit_timing.v $(bt_tb).v
	vvp $(bt_tb).out +vcd
	gtkwave can_bit_timing_tb.vcd

wave-frame:
	iverilog -g2012 -o $(fr_tb).out $(rtl) $(fr_tb).v
	vvp $(fr_tb).out +vcd
	gtkwave can_frame_fsm_tb.vcd

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
	rm -rf can_sniffer_tb.out can_sniffer_tb.vcd $(top_module).json $(top_module).asc $(top_module).bin \
	       can_bringup.json can_bringup.asc can_bringup.bin timing.rpt $(bt_tb).out $(fr_tb).out can_bit_timing_tb.vcd can_frame_fsm_tb.vcd
