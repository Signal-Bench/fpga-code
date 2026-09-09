/** \file
 * Oscillator-tolerance sweep for can_bit_timing.
 *
 * Drives a worst-case CAN-like bit stream at a deliberately WRONG bit rate and
 * checks that every bit is still sampled correctly.  This is the empirical
 * check on the tolerance budget derived in clock_choice.md: ISO 11898-1:2015
 * clause 11.3.2.5 predicts df < 0.98% for the default 12 tq configuration, so
 * the sweep should pass out to roughly +/-9800 ppm and fail beyond it.
 *
 * The stimulus uses the worst-case edge density that bit stuffing permits:
 * 5 dominant then 5 recessive, so recessive->dominant edges (the only ones
 * used for resynchronization) arrive only every 10 bit times.
 *
 *   vvp can_bit_timing_tb.out +offset_ppm=5000 +phase_ps=0
 *   vvp can_bit_timing_tb.out +offset_ppm=-12000
 *   add +vcd for a waveform dump
 */
`timescale 1ns/1ps

module can_bit_timing_tb;
	// Segment configuration under test; override from the command line, e.g.
	//   iverilog -P can_bit_timing_tb.PROP_SEG=3 -P can_bit_timing_tb.PHASE_SEG2=4 ...
	parameter integer PROP_SEG   = 4;
	parameter integer PHASE_SEG1 = 4;
	parameter integer PHASE_SEG2 = 3;
	parameter integer SJW        = 4;
	parameter real    CLK_MHZ    = 12.0;

	localparam integer NBITS       = 120;
	localparam real    CLK_NS      = 1000.0 / CLK_MHZ;
	localparam real    NOM_BIT_NS  = 1000.0;      // 1 Mbit/s

	reg        clk = 1'b0;
	reg        rst = 1'b1;
	reg  [7:0] brp = 8'd0;                        // tq = 1 core clock
	reg        rx_raw   = 1'b1;                   // idle recessive
	reg        bus_idle = 1'b1;

	wire sample_tick, sample_bit, bit_tick, rx_sync, hard_sync, resync;

	can_bit_timing #(
		.PROP_SEG(PROP_SEG), .PHASE_SEG1(PHASE_SEG1),
		.PHASE_SEG2(PHASE_SEG2), .SJW(SJW)
	) uut (
		.clk(clk), .rst(rst), .brp(brp),
		.rx_raw(rx_raw), .bus_idle(bus_idle),
		.sample_tick(sample_tick), .sample_bit(sample_bit),
		.bit_tick(bit_tick), .rx_sync(rx_sync),
		.hard_sync(hard_sync), .resync(resync)
	);

	always #(CLK_NS/2.0) clk = ~clk;

	// ----------------------------------------------------------------
	// Stimulus pattern and capture
	// ----------------------------------------------------------------
	reg        pattern [0:NBITS-1];
	reg        sampled [0:NBITS-1];
	integer    smp_count = 0;
	integer    resync_count = 0;
	reg        capturing = 1'b0;
	integer    i, errors, first_bad;
	integer    offset_ppm, phase_ps, pat_mode;
	real       bit_ns;

	// The bit timer free-runs while the bus is idle and emits sample points
	// the whole time, so only start capturing once the SOF edge has hard
	// synchronized it.
	always @(posedge clk) begin
		if (hard_sync) begin
			bus_idle  <= 1'b0;
			capturing <= 1'b1;
		end
		if (capturing) begin
			if (sample_tick && smp_count < NBITS) begin
				sampled[smp_count] = sample_bit;
				smp_count = smp_count + 1;
			end
			if (resync) resync_count = resync_count + 1;
		end
	end

	initial begin
		if (!$value$plusargs("offset_ppm=%d", offset_ppm)) offset_ppm = 0;
		if (!$value$plusargs("phase_ps=%d",   phase_ps))   phase_ps   = 0;
		if (!$value$plusargs("pattern=%d",    pat_mode))   pat_mode   = 0;

		if ($test$plusargs("vcd")) begin
			$dumpfile("can_bit_timing_tb.vcd");
			$dumpvars(0, can_bit_timing_tb);
		end

		// pattern 0 - worst case the stuffed region allows: 5 dominant then
		//   5 recessive, so a recessive->dominant resync edge every 10 bits.
		// pattern 1 - worst case the whole frame allows: the tail from the ACK
		//   slot through ACK delimiter, 7-bit EOF and 3-bit intermission to the
		//   next SOF is 11 recessive bits with no resync edge at all.  This is
		//   the span ISO 11898-1 eq. (4) is derived against.
		for (i = 0; i < NBITS; i = i + 1) begin
			if (pat_mode == 0)
				pattern[i] = (i % 10) < 5 ? 1'b0 : 1'b1;
			else
				pattern[i] = (i % 12) == 0 ? 1'b0 : 1'b1;
		end

		bit_ns = NOM_BIT_NS * (1.0 + offset_ppm / 1000000.0);

		// Reset, then sit idle recessive so the bit timer free-runs.
		repeat (4) @(posedge clk);
		rst = 1'b0;
		#(3 * NOM_BIT_NS);
		#(phase_ps / 1000.0);

		// pattern[0] is dominant, so this first transition is the SOF edge
		// that triggers hard synchronization.
		for (i = 0; i < NBITS; i = i + 1) begin
			rx_raw = pattern[i];
			#(bit_ns);
		end
		rx_raw = 1'b1;
		#(3 * NOM_BIT_NS);

		// ------------------------------------------------------------
		// Check
		// ------------------------------------------------------------
		errors    = 0;
		first_bad = -1;
		for (i = 0; i < NBITS; i = i + 1) begin
			if (i < smp_count && sampled[i] !== pattern[i]) begin
				errors = errors + 1;
				if (first_bad < 0) first_bad = i;
			end
		end
		if (smp_count < NBITS) begin
			errors = errors + (NBITS - smp_count);
			if (first_bad < 0) first_bad = smp_count;
		end

		$write("cfg %0d/%0d/%0d/%0d @%0.0fMHz  pat %0d  offset %+7d ppm  sampled %3d/%3d  ",
		       1, PROP_SEG, PHASE_SEG1, PHASE_SEG2, CLK_MHZ,
		       pat_mode, offset_ppm, smp_count, NBITS);
		if (errors == 0)
			$display("PASS");
		else
			$display("FAIL (%0d bad, first at bit %0d)", errors, first_bad);

		$finish;
	end
endmodule
