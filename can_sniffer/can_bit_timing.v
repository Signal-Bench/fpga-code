/** \file
 * CAN bit timing and synchronization for iCE40 UP5K.
 *
 * Recovers the bit clock from the NRZ CAN bus.  This is the piece that has no
 * analogue in the other sniffers in this repo: I2C carries its own clock on
 * SCL, so i2c_sniffer.v just samples SDA on scl_rising.  CAN has no clock
 * line, so the sample point has to be reconstructed from the data edges.
 *
 * Bit time is divided into time quanta (tq), per ISO 11898-1:2015 clause 11.3:
 *
 *   | SYNC_SEG | PROP_SEG | PHASE_SEG1 | PHASE_SEG2 |
 *   |   1 tq   |          |            ^            |
 *                                      sample point
 *
 * PROP_SEG + PHASE_SEG1 are merged internally into TSEG1 (they only differ in
 * what they compensate for, not in how they are counted); PHASE_SEG2 is TSEG2.
 * Resynchronization lengthens TSEG1 or shortens TSEG2 by at most SJW tq.
 *
 * Clock: this module MUST be driven from the UPduino's 12 MHz on-board
 * oscillator (jumper R16 / "OSC", pin gpio_20), NOT SB_HFOSC.  SB_HFOSC is
 * spec'd at 48 MHz +/-10% commercial (Lattice FPGA-DS-02008 Table 4.11), and
 * the ISO tolerance budget for this configuration is df < 0.98%.  See
 * clock_choice.md for the full derivation.
 *
 * Defaults: 12 MHz core, BRP=0 (tq = 1 clk), 12 tq/bit -> 1 Mbit/s,
 * sample point at 9/12 = 75%.
 *
 * Parameter constraints (ISO 11898-1:2015 clause 11.3.1.2):
 *   SJW        <= min(PHASE_SEG1, PHASE_SEG2)
 *   PHASE_SEG2 >= max(SJW, information processing time)   (IPT here is 1 tq)
 * With PHASE_SEG2 = 3 the largest conformant SJW is 3.  The resync logic
 * saturates rather than underflowing if these are violated, but the result is
 * asymmetric and non-conformant, so the checks below fail simulation early.
 *
 * Bus polarity: the transceiver's RXD mirrors the bus, so rx_raw is
 * 0 = dominant, 1 = recessive.  Recessive->dominant is a falling edge.
 */
module can_bit_timing #(
	parameter integer PROP_SEG   = 4,
	parameter integer PHASE_SEG1 = 4,
	parameter integer PHASE_SEG2 = 3,
	parameter integer SJW        = 3,

	// Lift the resync adder chain out of the tq_en critical path by
	// precomputing its candidates into registers.  Raises placed fmax from
	// ~24 MHz to ~32 MHz, which is what makes a 24 MHz unified core clock
	// close -- see ../unified_top.md.
	//
	// REQUIRES brp >= 1.  The precomputed values are one core clock behind
	// tq_pos/tseg1_eff/tseg2_eff, so they are only settled in time when a
	// time quantum is 2 or more core clocks.  At brp = 0 they are stale and
	// the module is WRONG, which is why this defaults off: can_sniffer.v
	// runs brp = 0 at 12 MHz and must stay bit-exact with the configuration
	// verified on hardware.
	parameter integer PIPELINE_RESYNC = 0
)(
	input  wire       clk,
	input  wire       rst,
	input  wire [7:0] brp,          // tq = (brp + 1) core clocks
	input  wire       rx_raw,       // straight from transceiver RXD
	input  wire       bus_idle,     // 1 = frame FSM is idle, allow hard sync

	output reg        sample_tick,  // 1-clk pulse at the sample point
	output reg        sample_bit,   // bus value latched at the sample point
	output reg        bit_tick,     // 1-clk pulse at end of bit time
	output wire       rx_sync,      // synchronized bus level
	output reg        hard_sync,    // 1-clk pulse: hard synchronization done
	output reg        resync        // 1-clk pulse: resynchronization done
);
	localparam integer TSEG1_NOM = PROP_SEG + PHASE_SEG1;
	localparam integer TSEG2_NOM = PHASE_SEG2;
	localparam integer IPT       = 1;   // sample-to-decision latency, tq

	// synthesis translate_off
	initial begin
		if (SJW > PHASE_SEG1 || SJW > PHASE_SEG2) begin
			$display("can_bit_timing: SJW=%0d exceeds min(PHASE_SEG1=%0d, PHASE_SEG2=%0d) -- ISO 11898-1 11.3.1.2",
			         SJW, PHASE_SEG1, PHASE_SEG2);
			$finish;
		end
		if (PHASE_SEG2 < IPT) begin
			$display("can_bit_timing: PHASE_SEG2=%0d below information processing time %0d", PHASE_SEG2, IPT);
			$finish;
		end
	end

	// PIPELINE_RESYNC precondition.  Checked on brp itself rather than as a
	// parameter assertion, because brp is a runtime input -- a baud-rate
	// change to brp = 0 would silently break the precomputed path.
	always @(posedge clk) begin
		if (!rst && PIPELINE_RESYNC != 0 && brp == 8'd0) begin
			$display("can_bit_timing: PIPELINE_RESYNC=1 requires brp >= 1 (tq >= 2 core clocks); brp=0 gives stale resync values");
			$finish;
		end
	end
	// synthesis translate_on

	// ----------------------------------------------------------------
	// Time quantum generator
	// ----------------------------------------------------------------
	reg [7:0] brp_ctr = 8'd0;
	wire      tq_en   = (brp_ctr == brp);

	always @(posedge clk) begin
		if (rst)       brp_ctr <= 8'd0;
		else if (tq_en) brp_ctr <= 8'd0;
		else            brp_ctr <= brp_ctr + 8'd1;
	end

	// ----------------------------------------------------------------
	// Input synchronizer (3-stage, same pattern as i2c_sniffer.v)
	// ----------------------------------------------------------------
	reg [2:0] rx_ff = 3'b111;
	always @(posedge clk) begin
		if (rst) rx_ff <= 3'b111;
		else     rx_ff <= {rx_ff[1:0], rx_raw};
	end
	assign rx_sync = rx_ff[1];

	// Edges are evaluated between consecutive time quanta, per ISO 11898-1
	// clause 11.3.2.1 ("a difference in bus states between two consecutive
	// time quanta is an edge"), not on the raw core clock.
	reg  rx_tq_prev = 1'b1;
	wire edge_r2d   = rx_tq_prev & ~rx_ff[1];   // recessive -> dominant

	// ----------------------------------------------------------------
	// Segment state
	// ----------------------------------------------------------------
	reg [7:0] tq_pos    = 8'd0;             // 0 = SYNC_SEG
	reg [7:0] tseg1_eff = TSEG1_NOM[7:0];   // may be lengthened by resync
	reg [7:0] tseg2_eff = TSEG2_NOM[7:0];   // may be shortened by resync
	reg       sync_done = 1'b0;             // one synchronization per bit time
	reg       last_smp_rec = 1'b1;          // bus state at previous sample point

	wire in_tseg1 = (tq_pos != 8'd0) && (tq_pos <= tseg1_eff);
	wire in_tseg2 = (tq_pos > tseg1_eff);

	// ISO 11898-1 clause 11.3.2.1 rules (a) and (b): at most one
	// synchronization per bit time, and only when the previous sample point
	// read recessive.
	wire sync_ok    = ~sync_done & last_smp_rec;
	wire do_hard    = tq_en & edge_r2d & bus_idle;
	wire do_resync  = tq_en & edge_r2d & ~bus_idle & sync_ok & (tq_pos != 8'd0);

	// Positive phase error: edge arrived tq_pos quanta after SYNC_SEG.
	// Lengthen TSEG1 by min(e, SJW).
	wire [7:0] lengthen  = (tq_pos > SJW[7:0]) ? SJW[7:0] : tq_pos;
	wire [7:0] tseg1_new = tseg1_eff + lengthen;

	// Negative phase error: edge arrived during TSEG2, so the bit should end
	// early.  Shorten TSEG2 towards the current position, floored at SJW.
	wire [7:0] q_in2     = tq_pos - tseg1_eff;   // quanta elapsed into TSEG2
	wire [7:0] floor2    = (tseg2_eff > SJW[7:0]) ? (tseg2_eff - SJW[7:0]) : 8'd0;
	wire [7:0] tseg2_new = (q_in2 > floor2) ? q_in2 : floor2;

	// Effective lengths for this tq boundary's segment decisions.
	//
	// Two implementations, selected by PIPELINE_RESYNC and identical in
	// behaviour whenever the parameter's brp precondition holds.

	wire [7:0] tseg1_use, tseg2_use;
	wire       at_sample, at_bitend;

	generate
	if (PIPELINE_RESYNC == 0) begin : g_comb
		// Original: compare -> add -> mux -> add -> compare, all in one
		// core clock.  Valid at any brp, including 0.
		assign tseg1_use = (do_resync & in_tseg1) ? tseg1_new : tseg1_eff;
		assign tseg2_use = (do_resync & in_tseg2) ? tseg2_new : tseg2_eff;
		assign at_sample = (tq_pos == tseg1_use);
		assign at_bitend = (tq_pos == (tseg1_use + tseg2_use));

	end else begin : g_pipe
		// Precomputed.  Every input is a register that only changes at a tq
		// boundary, so with brp >= 1 these are settled before the next
		// boundary consumes them.  Only a mux and a compare remain.
		reg [7:0] tseg1_new_r  = TSEG1_NOM[7:0];
		reg [7:0] tseg2_new_r  = TSEG2_NOM[7:0];
		reg [7:0] bitend_nom_r = TSEG1_NOM[7:0] + TSEG2_NOM[7:0];
		reg [7:0] bitend_rs1_r = TSEG1_NOM[7:0] + TSEG2_NOM[7:0];
		reg [7:0] bitend_rs2_r = TSEG1_NOM[7:0] + TSEG2_NOM[7:0];
		reg       in_tseg1_r   = 1'b0;
		reg       in_tseg2_r   = 1'b0;

		always @(posedge clk) begin
			if (rst) begin
				tseg1_new_r  <= TSEG1_NOM[7:0];
				tseg2_new_r  <= TSEG2_NOM[7:0];
				bitend_nom_r <= TSEG1_NOM[7:0] + TSEG2_NOM[7:0];
				bitend_rs1_r <= TSEG1_NOM[7:0] + TSEG2_NOM[7:0];
				bitend_rs2_r <= TSEG1_NOM[7:0] + TSEG2_NOM[7:0];
				in_tseg1_r   <= 1'b0;
				in_tseg2_r   <= 1'b0;
			end else begin
				tseg1_new_r  <= tseg1_new;
				tseg2_new_r  <= tseg2_new;
				bitend_nom_r <= tseg1_eff + tseg2_eff;
				bitend_rs1_r <= tseg1_new + tseg2_eff;
				bitend_rs2_r <= tseg1_eff + tseg2_new;
				in_tseg1_r   <= in_tseg1;
				in_tseg2_r   <= in_tseg2;
			end
		end

		wire rs1 = do_resync & in_tseg1_r;
		wire rs2 = do_resync & in_tseg2_r;

		assign tseg1_use = rs1 ? tseg1_new_r : tseg1_eff;
		assign tseg2_use = rs2 ? tseg2_new_r : tseg2_eff;
		assign at_sample = (tq_pos == tseg1_use);
		assign at_bitend = (tq_pos == (rs1 ? bitend_rs1_r :
		                               rs2 ? bitend_rs2_r : bitend_nom_r));
	end
	endgenerate

	// ----------------------------------------------------------------
	// Main sequential process
	// ----------------------------------------------------------------
	always @(posedge clk) begin
		sample_tick <= 1'b0;
		bit_tick    <= 1'b0;
		hard_sync   <= 1'b0;
		resync      <= 1'b0;

		if (rst) begin
			tq_pos       <= 8'd0;
			tseg1_eff    <= TSEG1_NOM[7:0];
			tseg2_eff    <= TSEG2_NOM[7:0];
			sync_done    <= 1'b0;
			last_smp_rec <= 1'b1;
			rx_tq_prev   <= 1'b1;
			sample_bit   <= 1'b1;

		end else if (tq_en) begin
			rx_tq_prev <= rx_ff[1];

			if (do_hard) begin
				// Hard synchronization: the bit time restarts with SYNC_SEG,
				// and the tq that just ended was that SYNC_SEG.
				tq_pos     <= 8'd1;
				tseg1_eff  <= TSEG1_NOM[7:0];
				tseg2_eff  <= TSEG2_NOM[7:0];
				sync_done  <= 1'b1;
				hard_sync  <= 1'b1;

			end else begin
				if (do_resync) begin
					tseg1_eff <= tseg1_use;
					tseg2_eff <= tseg2_use;
					sync_done <= 1'b1;
					resync    <= 1'b1;
				end

				if (at_sample) begin
					sample_tick  <= 1'b1;
					sample_bit   <= rx_ff[1];
					last_smp_rec <= rx_ff[1];
				end

				if (at_bitend) begin
					bit_tick   <= 1'b1;
					tq_pos     <= 8'd0;
					tseg1_eff  <= TSEG1_NOM[7:0];
					tseg2_eff  <= TSEG2_NOM[7:0];
					sync_done  <= 1'b0;
				end else begin
					tq_pos <= tq_pos + 8'd1;
				end
			end
		end
	end
endmodule
