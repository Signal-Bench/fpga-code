# can_frame_link_board — CAN + 4 x UART sniffer + PROBE2 edge counter -> framed SPI link, SignalBench PCBA (U7 UP5K-SG48)
#
#   make build     bitstream (can_frame_link_board.bin), 12 MHz constraint, timing report
#   make sim       everything below
#   make sim-partner   partner testbenches for the reused modules, at 12 MHz
#   make sim-top       end-to-end testbench, three configurations (make -j3 sim-top runs them in parallel)
#   make vectors   frames_vectors.txt, frames_vectors_records.txt, frames_vectors_esp.txt (ESP host test)
#   make clean
#
# Toolchain: OSS CAD Suite.  `source ~/opt/oss-cad-suite/environment` first.
# DO NOT flash from here: flashing is done by Charles.

name     = can_frame_link_board
top      = top
pcf      = $(name).pcf
freq_mhz = 12
B        = out

rtl      = can_crc15.v can_bit_timing.v can_frame_fsm.v uart_rx_sniff.v spi_frame_link.v can_frame_link.v
simlib   = ice40_sim_models.v
TT       = can_frame_link_tb
PT       = spi_frame_stream_tb

.PHONY: build time sim sim-partner sim-top sim-top-small sim-top-full sim-top-baud vectors clean

build: $(B)/$(name).bin time

$(B):
	mkdir -p $(B)

$(B)/$(name).json: $(rtl) | $(B)
	yosys -q -l $(B)/yosys.log -p "synth_ice40 -top $(top) -json $@" $(rtl)

$(B)/$(name).asc: $(B)/$(name).json $(pcf)
	nextpnr-ice40 --up5k --package sg48 --json $< --pcf $(pcf) --asc $@ \
		--freq $(freq_mhz) --seed 1 --report $(B)/nextpnr_report.json -l $(B)/nextpnr.log

$(B)/$(name).bin: $(B)/$(name).asc
	icepack $< $@
	cp $@ $(name).bin

time: $(B)/$(name).asc
	icetime -d up5k -P sg48 -p $(pcf) -c $(freq_mhz) -mtr $(B)/timing.rpt $<

# ---- partner testbenches -------------------------------------------------
sim-partner: | $(B)
	iverilog -g2012 -o $(B)/bt.out can_bit_timing.v partner_tb/can_bit_timing_tb.v
	vvp -n $(B)/bt.out
	@for pat in 0 1; do for ppm in 0 5000 -5000 9000 -9000; do \
		vvp -n $(B)/bt.out +pattern=$$pat +offset_ppm=$$ppm | grep -E "PASS|FAIL"; done; done
	iverilog -g2012 -o $(B)/fr.out can_crc15.v can_frame_fsm.v can_bit_timing.v partner_tb/can_frame_fsm_tb.v
	vvp -n $(B)/fr.out
	# baseline: the unmodified partner RTL + testbench (24 MHz), small config
	iverilog -g2012 -P$(PT).BUF_AW=10 -o $(B)/sfs_orig.out partner_tb/spi_frame_stream_orig.v partner_tb/spi_frame_stream_tb_orig.v
	vvp -n $(B)/sfs_orig.out
	# the shipped link at 12 MHz under the partner testbench (rates rescaled)
	sed -f partner_tb/patch_tb_12m.sed partner_tb/spi_frame_stream_tb_orig.v > $(B)/spi_frame_stream_tb_12m.v
	iverilog -g2012 -P$(PT).BUF_AW=10 -o $(B)/sfs_zero.out spi_frame_link.v partner_tb/spi_frame_stream_12m_wrap.v $(B)/spi_frame_stream_tb_12m.v
	vvp -n $(B)/sfs_zero.out
	iverilog -g2012 -P$(PT).BUF_AW=10 -P$(PT).INITIAL_OFFSET=32\'hFFFFFF00 -o $(B)/sfs_small.out spi_frame_link.v partner_tb/spi_frame_stream_12m_wrap.v $(B)/spi_frame_stream_tb_12m.v
	vvp -n $(B)/sfs_small.out
	iverilog -g2012 -P$(PT).BUF_AW=16 -P$(PT).INITIAL_OFFSET=32\'hFFFFFE00 -o $(B)/sfs_full.out spi_frame_link.v partner_tb/spi_frame_stream_12m_wrap.v $(B)/spi_frame_stream_tb_12m.v
	vvp -n $(B)/sfs_full.out

# ---- end-to-end ------------------------------------------------------------
#  Every config also runs UART3 (PROBE1, 115200, tx +1.5% unless set) and the
#  PROBE2 edge counter (bursts, glitches, 100 ms counter records), phase D
#  (Pi packets back to back) and the non-empty-compare checks.
#  small: 1 Ki-sample buffer (overflow test incl. UART3 + COUNTER drops), UART0-2 9600 (tx +2% / -2% / 0)
#  full : real 4-SPRAM buffer, offsets through the 2^32 wrap
#  baud : UART 115200 / 230400 / 57600 / 115200 (tx +2% / -2% / -2% / -2%), CAN +3000 ppm, 2 KB reads
# Each config passes only if the testbench prints its PASS line (exit status is the gate).
sim-top: sim-top-small sim-top-full sim-top-baud

sim-top-small: | $(B)
	iverilog -g2012 -P$(TT).BUF_AW=10 -o $(B)/top_small.out $(simlib) $(rtl) $(TT).v
	vvp -n $(B)/top_small.out | tee $(B)/top_small.log
	@grep -q '^PASS: can_frame_link end-to-end' $(B)/top_small.log

sim-top-full: | $(B)
	iverilog -g2012 -P$(TT).BUF_AW=16 -P$(TT).INITIAL_OFFSET=32\'hFFFFFF00 -o $(B)/top_full.out $(simlib) $(rtl) $(TT).v
	vvp -n $(B)/top_full.out | tee $(B)/top_full.log
	@grep -q '^PASS: can_frame_link end-to-end' $(B)/top_full.log

sim-top-baud: | $(B)
	iverilog -g2012 -P$(TT).BUF_AW=16 -P$(TT).U0_BAUD=115200 -P$(TT).U1_BAUD=230400 -P$(TT).U2_BAUD=57600 \
		-P$(TT).U2_PPM=-20000 -P$(TT).U3_PPM=-20000 -P$(TT).CAN_PPM=3000 -P$(TT).READ_LEN=2048 -o $(B)/top_baud.out $(simlib) $(rtl) $(TT).v
	vvp -n $(B)/top_baud.out | tee $(B)/top_baud.log
	@grep -q '^PASS: can_frame_link end-to-end' $(B)/top_baud.log

sim: sim-partner sim-top

vectors: | $(B)
	iverilog -g2012 -P$(TT).BUF_AW=16 -o $(B)/top_vec.out $(simlib) $(rtl) $(TT).v
	vvp -n $(B)/top_vec.out +dump=frames_vectors.txt +records=frames_vectors_records.txt +esp=frames_vectors_esp.txt | tee $(B)/top_vec.log
	@grep -q '^PASS: can_frame_link end-to-end' $(B)/top_vec.log

clean:
	rm -rf $(B) $(TT).vcd
