filename = spi_frame_stream
top_module = top
pcf_file = $(filename).pcf
tb = $(filename)_tb
pnr_freq_mhz = 30

build:
	yosys -p "synth_ice40 -top $(top_module) -json $(filename).json" $(filename).v
	nextpnr-ice40 --up5k --package sg48 --json $(filename).json --pcf $(pcf_file) --asc $(filename).asc --freq $(pnr_freq_mhz)
	icepack $(filename).asc $(filename).bin

# Three configurations:
#   sim-zero   1 Ki-sample buffer, offsets from 0 (the hardware default)
#   sim-small  1 Ki-sample buffer, offsets from just below 2^32, so the
#              stream wraps the 32-bit offset; small enough that the
#              overflow test can fill it
#   sim-full   the real 64 Ki-sample four-SPRAM buffer, also through the
#              2^32 wrap — which is also the crossing from SPRAM bank 3
#              back to bank 0
sim: sim-zero sim-small sim-full

sim-zero:
	iverilog -g2012 -P$(tb).BUF_AW=10 -o $(tb)_zero.out $(filename).v $(tb).v
	vvp $(tb)_zero.out

sim-small:
	iverilog -g2012 -P$(tb).BUF_AW=10 -P$(tb).INITIAL_OFFSET=32\'hFFFFFF00 -o $(tb)_small.out $(filename).v $(tb).v
	vvp $(tb)_small.out

sim-full:
	iverilog -g2012 -P$(tb).BUF_AW=16 -P$(tb).INITIAL_OFFSET=32\'hFFFFFE00 -o $(tb)_full.out $(filename).v $(tb).v
	vvp $(tb)_full.out

# Writes every transfer (MOSI and MISO bytes) to frames.txt — the test
# vectors the MCU's host test replays through its own parser.
vectors:
	iverilog -g2012 -P$(tb).BUF_AW=10 -P$(tb).INITIAL_OFFSET=32\'hFFFFFF00 -o $(tb)_small.out $(filename).v $(tb).v
	vvp $(tb)_small.out +dump=frames.txt

wave:
	iverilog -g2012 -P$(tb).BUF_AW=10 -o $(tb)_small.out $(filename).v $(tb).v
	vvp $(tb)_small.out +vcd
	gtkwave $(tb).vcd

time: $(filename).asc
	icetime -d up5k -c $(pnr_freq_mhz) -mtr timing.rpt $(filename).asc
	@echo "Timing report written to timing.rpt. Check for PASS/FAIL at the bottom."

flash:
	iceprog -d i:0x0403:0x6014 $(filename).bin # Flash to FTDI

clean:
	rm -rf $(filename).blif $(filename).asc $(filename).bin $(filename).json timing.rpt $(tb)*.out $(tb).vcd frames.txt
