/** \file
 * CAN 2.0B frame decoder: destuffing, field FSM, error and overload detection.
 *
 * Consumes one bit per sample_tick from can_bit_timing.v and emits a decoded
 * frame record, or an error record, on frame_strobe.
 *
 * Field order (ISO 11898-1 / CAN 2.0B Part B):
 *
 *   SOF -> 11-bit base ID -> RTR/SRR -> IDE -+- dominant  -> r0 -> DLC -> ...
 *                                            +- recessive -> 18-bit ext ID
 *                                                         -> RTR -> r1 -> r0 -> DLC -> ...
 *   ... -> DATA (0-8 bytes) -> CRC-15 -> CRC delim -> ACK slot -> ACK delim
 *       -> EOF (7 recessive) -> intermission (3 recessive)
 *
 * Standard vs extended is one bit: after the 11 base ID bits, read two more.
 * If the second (IDE) is dominant this is a standard frame and the first was
 * RTR; if recessive it is extended, the first was SRR, and 18 more ID bits
 * follow.
 *
 * Two spans that are easy to conflate:
 *   - BIT STUFFING covers SOF through the end of the CRC SEQUENCE.  A stuff
 *     bit can therefore land between the last CRC bit and the CRC delimiter.
 *   - CRC-15 covers SOF through the end of the DATA FIELD, destuffed.
 *
 * Reserved bits r0/r1 are deliberately NOT form-checked: the spec says
 * receivers accept dominant and recessive in all combinations.
 *
 * Error reporting is honest about what a passive observer can know:
 *   STUFF / FORM / CRC / ACK are detected directly.  A BIT error is not
 *   detectable at all -- it is only inferrable, and shows up here as UNATTR
 *   when the bus is jammed during EOF after we validated everything we could.
 */
module can_frame_fsm (
	input  wire        clk,
	input  wire        rst,
	input  wire        sample_tick,
	input  wire        sample_bit,     // 1 = recessive, 0 = dominant

	output wire        bus_idle,       // to can_bit_timing: enable hard sync
	output reg         frame_strobe,   // 1-clk pulse: record below is valid
	output reg  [28:0] frame_id,
	output reg         frame_ide,
	output reg         frame_rtr,
	output reg  [3:0]  frame_dlc,
	output reg  [63:0] frame_data,     // data[0] in bits [63:56], MSB first
	output reg         frame_crc_ok,
	output reg         frame_ack_ok,
	output reg         frame_overload, // an overload frame preceded this record
	output reg  [2:0]  frame_err
);
	// Error codes
	localparam [2:0] ERR_NONE   = 3'd0;
	localparam [2:0] ERR_STUFF  = 3'd1;
	localparam [2:0] ERR_CRC    = 3'd2;
	localparam [2:0] ERR_FORM   = 3'd3;
	localparam [2:0] ERR_ACK    = 3'd4;
	localparam [2:0] ERR_UNATTR = 3'd5;   // error frame we could not attribute
	localparam [2:0] ERR_DOM    = 3'd6;   // bus stuck dominant

	// States.  S_ID_A..S_CRC is the bit-stuffed region.
	localparam [4:0] S_IDLE      = 5'd0;
	localparam [4:0] S_ID_A      = 5'd1;
	localparam [4:0] S_RTR_SRR   = 5'd2;
	localparam [4:0] S_IDE       = 5'd3;
	localparam [4:0] S_ID_B      = 5'd4;
	localparam [4:0] S_RTR_EXT   = 5'd5;
	localparam [4:0] S_R1        = 5'd6;
	localparam [4:0] S_R0        = 5'd7;
	localparam [4:0] S_DLC       = 5'd8;
	localparam [4:0] S_DATA      = 5'd9;
	localparam [4:0] S_CRC       = 5'd10;
	localparam [4:0] S_CRC_DELIM = 5'd11;
	localparam [4:0] S_ACK       = 5'd12;
	localparam [4:0] S_ACK_DELIM = 5'd13;
	localparam [4:0] S_EOF       = 5'd14;
	localparam [4:0] S_IFS       = 5'd15;
	localparam [4:0] S_RECOVER   = 5'd16;

	reg [4:0]  state      = S_IDLE;
	reg [6:0]  bit_cnt    = 7'd0;
	reg [28:0] id_sr      = 29'd0;
	reg [63:0] data_sr    = 64'd0;
	reg [14:0] crc_rx     = 15'd0;
	reg [3:0]  dlc_r      = 4'd0;
	reg        ide_r      = 1'b0;
	reg        rtr_r      = 1'b0;
	reg        ack_ok_r   = 1'b0;
	reg [2:0]  err_latch  = ERR_NONE;
	reg        overload_r = 1'b0;

	// Bit destuffing state
	reg        stuffing_on = 1'b0;
	reg [2:0]  same_cnt    = 3'd0;
	reg        last_bit    = 1'b1;

	// Consecutive-recessive counter, used to declare the bus idle again after
	// an error or overload frame (delimiter is 8 recessive, intermission 3).
	reg [4:0]  rec_run = 5'd31;
	// Consecutive-dominant counter, for the stuck-bus detector.  An active
	// error flag superposition is legitimately up to 12 dominant bits, so the
	// threshold sits above that and only reports once per stuck episode.
	reg [4:0]  dom_run      = 5'd0;
	reg        dom_reported = 1'b0;

	wire [14:0] crc_calc;
	reg         crc_clear;
	reg         crc_en;
	reg         crc_bit;

	can_crc15 u_crc (
		.clk(clk), .clear(crc_clear), .en(crc_en),
		.bit_in(crc_bit), .crc(crc_calc)
	);

	// A stuff bit is the bit immediately following five identical bits, while
	// stuffing is active.  Six identical bits is a stuff error -- which is
	// also exactly what an active error flag looks like from out here.
	wire is_stuff   = stuffing_on & (same_cnt == 3'd5);
	wire stuff_err  = is_stuff & (sample_bit == last_bit);
	wire take_bit   = sample_tick & ~is_stuff;      // a real (destuffed) bit

	// Number of data bytes.  DLC > 8 is illegal but ISO says treat it as 8.
	wire [3:0] dlc_bytes  = (dlc_r > 4'd8) ? 4'd8 : dlc_r;
	wire [6:0] data_bits  = {dlc_bytes, 3'b000};    // dlc_bytes * 8

	assign bus_idle = (state == S_IDLE);

	// Latch the first error of a frame; later ones are consequences of it.
	task latch_err(input [2:0] code);
		begin
			if (err_latch == ERR_NONE) err_latch <= code;
		end
	endtask

	always @(posedge clk) begin
		frame_strobe <= 1'b0;
		crc_clear    <= 1'b0;
		crc_en       <= 1'b0;

		if (rst) begin
			state        <= S_IDLE;
			stuffing_on  <= 1'b0;
			err_latch    <= ERR_NONE;
			overload_r   <= 1'b0;
			rec_run      <= 5'd31;
			dom_run      <= 5'd0;
			dom_reported <= 1'b0;

		end else if (sample_tick) begin
			// ---------------------------------------------------------
			// Run-length bookkeeping on the raw (still stuffed) stream
			// ---------------------------------------------------------
			if (sample_bit) begin
				rec_run <= (rec_run == 5'd31) ? rec_run : rec_run + 5'd1;
				dom_run <= 5'd0;
			end else begin
				rec_run <= 5'd0;
				dom_run <= (dom_run == 5'd31) ? dom_run : dom_run + 5'd1;
			end
			if (sample_bit) dom_reported <= 1'b0;

			// Destuffing run counter tracks every bit, stuff bits included.
			if (sample_bit == last_bit) same_cnt <= same_cnt + 3'd1;
			else                        same_cnt <= 3'd1;
			last_bit <= sample_bit;

			// ---------------------------------------------------------
			// Stuck-dominant detector (not an ISO error type, but the
			// failure that actually happens on a bench).  11 bit times.
			// ---------------------------------------------------------
			if (dom_run >= 5'd13 && !dom_reported) begin
				dom_reported   <= 1'b1;
				frame_id       <= 29'd0;
				frame_ide      <= 1'b0;
				frame_rtr      <= 1'b0;
				frame_dlc      <= 4'd0;
				frame_data     <= 64'd0;
				frame_crc_ok   <= 1'b0;
				frame_ack_ok   <= 1'b0;
				frame_overload <= 1'b0;
				frame_err      <= ERR_DOM;
				frame_strobe   <= 1'b1;
				state          <= S_RECOVER;
				stuffing_on    <= 1'b0;

			end else if (is_stuff) begin
				// Consume the stuff bit.  It never reaches the field FSM or
				// the CRC.  Six identical bits means the stuffing rule was
				// violated.
				if (stuff_err) begin
					latch_err(ERR_STUFF);
					state       <= S_RECOVER;
					stuffing_on <= 1'b0;
					frame_id       <= id_sr;
					frame_ide      <= ide_r;
					frame_rtr      <= rtr_r;
					frame_dlc      <= dlc_r;
					frame_data     <= data_sr;
					frame_crc_ok   <= 1'b0;
					frame_ack_ok   <= 1'b0;
					frame_overload <= overload_r;
					frame_err      <= (err_latch == ERR_NONE) ? ERR_STUFF : err_latch;
					frame_strobe   <= 1'b1;
				end

			end else begin
				// ------------------------------------------------------
				// Real, destuffed bit -> field FSM
				// ------------------------------------------------------
				// CRC covers SOF..end of DATA.
				if (state == S_ID_A || state == S_RTR_SRR || state == S_IDE ||
				    state == S_ID_B || state == S_RTR_EXT || state == S_R1  ||
				    state == S_R0   || state == S_DLC     || state == S_DATA) begin
					crc_en  <= 1'b1;
					crc_bit <= sample_bit;
				end

				case (state)
				// ------------------------------------------------------
				S_IDLE: begin
					err_latch <= ERR_NONE;
					if (!sample_bit) begin
						// SOF.  The CRC formally covers SOF, but feeding a
						// dominant bit into a zeroed register is a no-op
						// (0 ^ crc[14] = 0, and the shift keeps it zero), so
						// clearing here is equivalent to clearing and feeding.
						crc_clear   <= 1'b1;
						stuffing_on <= 1'b1;
						same_cnt    <= 3'd1;
						last_bit    <= 1'b0;
						id_sr       <= 29'd0;
						data_sr     <= 64'd0;
						ide_r       <= 1'b0;
						rtr_r       <= 1'b0;
						ack_ok_r    <= 1'b0;
						bit_cnt     <= 7'd0;
						state       <= S_ID_A;
					end
				end
				// ------------------------------------------------------
				S_ID_A: begin
					id_sr <= {id_sr[27:0], sample_bit};
					if (bit_cnt == 7'd10) begin
						bit_cnt <= 7'd0;
						state   <= S_RTR_SRR;
					end else
						bit_cnt <= bit_cnt + 7'd1;
				end
				// ------------------------------------------------------
				S_RTR_SRR: begin
					// Either RTR (standard) or SRR (extended); the next bit
					// decides which.  Hold it until then.
					rtr_r <= sample_bit;
					state <= S_IDE;
				end
				// ------------------------------------------------------
				S_IDE: begin
					ide_r <= sample_bit;
					if (sample_bit) begin
						// Extended: the bit we held was SRR, not RTR.
						state   <= S_ID_B;
						bit_cnt <= 7'd0;
					end else begin
						// Standard: the held bit really was RTR.
						state <= S_R0;
					end
				end
				// ------------------------------------------------------
				S_ID_B: begin
					id_sr <= {id_sr[27:0], sample_bit};
					if (bit_cnt == 7'd17) begin
						bit_cnt <= 7'd0;
						state   <= S_RTR_EXT;
					end else
						bit_cnt <= bit_cnt + 7'd1;
				end
				// ------------------------------------------------------
				S_RTR_EXT: begin
					rtr_r <= sample_bit;
					state <= S_R1;
				end
				// r1 and r0 are accepted in any combination -- no form check.
				S_R1: state <= S_R0;
				S_R0: begin
					bit_cnt <= 7'd0;
					state   <= S_DLC;
				end
				// ------------------------------------------------------
				S_DLC: begin
					dlc_r <= {dlc_r[2:0], sample_bit};
					if (bit_cnt == 7'd3) begin
						bit_cnt <= 7'd0;
						// A remote frame carries no data whatever the DLC says.
						if (rtr_r || {dlc_r[2:0], sample_bit} == 4'd0)
							state <= S_CRC;
						else
							state <= S_DATA;
					end else
						bit_cnt <= bit_cnt + 7'd1;
				end
				// ------------------------------------------------------
				S_DATA: begin
					// Written by bit position rather than shifted in, so the
					// field is left-justified for every DLC -- data[0] always
					// lands in [63:56] and the record packer needs no shift.
					data_sr[6'd63 - bit_cnt[5:0]] <= sample_bit;
					if (bit_cnt == data_bits - 7'd1) begin
						bit_cnt <= 7'd0;
						state   <= S_CRC;
					end else
						bit_cnt <= bit_cnt + 7'd1;
				end
				// ------------------------------------------------------
				S_CRC: begin
					crc_rx <= {crc_rx[13:0], sample_bit};
					if (bit_cnt == 7'd14) begin
						bit_cnt <= 7'd0;
						state   <= S_CRC_DELIM;
						// Stuffing ends with the CRC sequence -- but a stuff
						// bit may still follow this one, so only clear the
						// flag when this bit did not complete a run of five.
						if (!((sample_bit == last_bit) && (same_cnt == 3'd4)))
							stuffing_on <= 1'b0;
					end else
						bit_cnt <= bit_cnt + 7'd1;
				end
				// ------------------------------------------------------
				S_CRC_DELIM: begin
					stuffing_on <= 1'b0;
					if (!sample_bit) latch_err(ERR_FORM);
					state <= S_ACK;
				end
				// ------------------------------------------------------
				S_ACK: begin
					// Dominant here means at least one node accepted the
					// frame.  Recessive means nobody did.
					ack_ok_r <= ~sample_bit;
					if (sample_bit) latch_err(ERR_ACK);
					state <= S_ACK_DELIM;
				end
				// ------------------------------------------------------
				S_ACK_DELIM: begin
					if (!sample_bit) latch_err(ERR_FORM);
					bit_cnt <= 7'd0;
					state   <= S_EOF;
				end
				// ------------------------------------------------------
				S_EOF: begin
					if (!sample_bit) begin
						// The bus is being jammed during end of frame.  If we
						// found nothing wrong ourselves, someone else objected
						// for a reason we could not observe -- most likely a
						// bit error, but we cannot prove that from one point
						// on the bus, so report it as unattributed.
						emit_record((err_latch == ERR_NONE) ? ERR_UNATTR : err_latch,
						            1'b0);
						state       <= S_RECOVER;
						stuffing_on <= 1'b0;
					end else if (bit_cnt == 7'd5) begin
						// Valid for a receiver at the last-but-one EOF bit.
						emit_record(err_latch,
						            (crc_calc == crc_rx) && (err_latch == ERR_NONE));
						bit_cnt <= bit_cnt + 7'd1;
					end else if (bit_cnt == 7'd6) begin
						bit_cnt <= 7'd0;
						state   <= S_IFS;
					end else
						bit_cnt <= bit_cnt + 7'd1;
				end
				// ------------------------------------------------------
				S_IFS: begin
					// A dominant bit during intermission is an OVERLOAD
					// condition, not an error.  It gets no error code and no
					// LED colour -- but it still has to be parsed, or frame
					// sync is lost.
					if (!sample_bit) begin
						overload_r <= 1'b1;
						state      <= S_RECOVER;
					end else if (bit_cnt == 7'd2) begin
						state <= S_IDLE;
					end else
						bit_cnt <= bit_cnt + 7'd1;
				end
				// ------------------------------------------------------
				S_RECOVER: begin
					// Error and overload delimiters are 8 recessive bits, then
					// 3 of intermission.  Waiting for 7 clean recessive bits
					// re-establishes frame sync from anywhere in either.
					if (rec_run >= 5'd7) state <= S_IDLE;
				end
				default: state <= S_IDLE;
				endcase
			end
		end
	end

	// Emit a record.  Declared after use above is fine for a task in the same
	// module; it exists to keep the two exit paths from EOF in sync.
	task emit_record(input [2:0] code, input ok);
		begin
			frame_id       <= id_sr;
			frame_ide      <= ide_r;
			frame_rtr      <= rtr_r;
			frame_dlc      <= dlc_r;
			frame_data     <= data_sr;
			frame_crc_ok   <= ok;
			frame_ack_ok   <= ack_ok_r;
			frame_overload <= overload_r;
			overload_r     <= 1'b0;   // consumed by this record
			frame_err      <= (crc_calc != crc_rx) && (code == ERR_NONE)
			                  ? ERR_CRC : code;
			frame_strobe   <= 1'b1;
		end
	endtask
endmodule
