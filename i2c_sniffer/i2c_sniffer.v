/** \file
 * I2C-to-SPI sniffer for iCE40 UP5K — ping-pong buffer variant.
 *
 * Capture and SPI drain run as fully independent state machines connected
 * by a 3-entry drain queue.  Capture returns to CAP_IDLE immediately after
 * each STOP; the SPI drain empties queued captures in the background.
 *
 * RAM layout: 4 × 128-byte buffers = 512 bytes (one inferred EBR).
 *   Address = { buf_sel[1:0], offset[6:0] }
 *   buf_sel increments mod-4 ("flip") after every STOP so capture always
 *   writes into a region that the drain has already finished with.
 *
 * Queue: 3 entries of { buf_sel[1:0], len[6:0] }.
 *   Captures are silently dropped when all 3 slots are occupied.
 *   push and pop are mutually exclusive (push wins) so a simultaneous
 *   push/pop never corrupts the queue head.
 */
module i2c_sniffer (
	input  wire scl_pin,
	input  wire sda_pin,
	output wire spi_sck,
	output wire spi_mosi,
	output wire spi_cs,
	output wire spi_cs_flash,
	output wire led_r,
	output wire led_g,
	output wire led_b
);
	assign spi_cs_flash = 1'b1;

	// ----------------------------------------------------------------
	// 48 MHz oscillator → 24 MHz core clock
	// ----------------------------------------------------------------
	wire clk_48;
	reg  clk_core_r = 0;
	wire clk_core   = clk_core_r;

	localparam integer CORE_HZ                = 24_000_000;
	localparam integer START_LED_CYCLES       = CORE_HZ / 100;
	localparam integer IDLE_TIMEOUT_CYCLES    = CORE_HZ / 1000;
	localparam integer SPI_HALF_PERIOD_CYCLES = CORE_HZ / 2_000_000;

	SB_HFOSC u_hfosc (.CLKHFPU(1'b1), .CLKHFEN(1'b1), .CLKHF(clk_48));
	always @(posedge clk_48) clk_core_r <= ~clk_core_r;

	// ----------------------------------------------------------------
	// Input synchronization (3-stage) + edge detection
	// ----------------------------------------------------------------
	reg [2:0] scl_sync = 3'b111;
	reg [2:0] sda_sync = 3'b111;
	always @(posedge clk_core) begin
		scl_sync <= {scl_sync[1:0], scl_pin};
		sda_sync <= {sda_sync[1:0], sda_pin};
	end

	wire scl_rising   =  scl_sync[1] & ~scl_sync[2];
	wire scl_falling  = ~scl_sync[1] &  scl_sync[2];
	wire sda_rising   =  sda_sync[1] & ~sda_sync[2];
	wire sda_falling  = ~sda_sync[1] &  sda_sync[2];
	wire start_cond   = sda_falling & scl_sync[1];
	wire stop_cond    = sda_rising  & scl_sync[1];
	wire bus_activity = scl_rising | scl_falling | sda_rising | sda_falling;

	// ----------------------------------------------------------------
	// Debug LED pulse stretcher (START indicator)
	// ----------------------------------------------------------------
	reg [18:0] start_led_ctr = 0;
	always @(posedge clk_core) begin
		if (start_cond)
			start_led_ctr <= START_LED_CYCLES[18:0];
		else
			start_led_ctr <= start_led_ctr - (start_led_ctr != 0);
	end
	wire start_led_on = (start_led_ctr != 0);

	// ----------------------------------------------------------------
	// SPI tick divider (~1 MHz half-period from 24 MHz)
	// ----------------------------------------------------------------
	reg [4:0] spi_div = 0;
	wire tick = (spi_div == SPI_HALF_PERIOD_CYCLES - 1);
	always @(posedge clk_core) begin
		if (tick) spi_div <= 0;
		else      spi_div <= spi_div + 1'b1;
	end

	// ----------------------------------------------------------------
	// Capture RAM: 4 × 128 B = 512 B
	// ----------------------------------------------------------------
	reg [7:0] capture_ram [0:511];

	reg       cap_wr_en   = 0;
	reg [8:0] cap_wr_addr = 0;
	reg [7:0] cap_wr_data = 0;

	reg       dr_rd_en   = 0;
	reg [8:0] dr_rd_addr = 0;
	reg [7:0] dr_rd_data = 0;

	always @(posedge clk_core) begin
		if (cap_wr_en) capture_ram[cap_wr_addr] <= cap_wr_data;
		if (dr_rd_en)  dr_rd_data <= capture_ram[dr_rd_addr];
	end

	// ----------------------------------------------------------------
	// Drain queue: 3-entry FIFO, each entry = { buf_sel[1:0], len[6:0] }
	// ----------------------------------------------------------------
	reg [8:0] q_ent [0:2];
	reg [1:0] q_wr    = 0;
	reg [1:0] q_rd    = 0;
	reg [1:0] q_count = 0;

	wire q_full  = (q_count == 2'd3);
	wire q_empty = (q_count == 2'd0);

	// Pointer increment mod-3
	function [1:0] q_inc;
		input [1:0] p;
		q_inc = (p == 2'd2) ? 2'd0 : p + 2'd1;
	endfunction

	// ----------------------------------------------------------------
	// Capture state machine registers
	// ----------------------------------------------------------------
	reg        capture_state     = 0;   // 0 = IDLE, 1 = ACTIVE
	reg [1:0]  buf_wr_sel        = 0;
	reg [6:0]  cap_wr_ptr        = 0;
	reg [2:0]  bit_count         = 0;
	reg [7:0]  shift_reg         = 0;
	reg        buf_overflow      = 0;
	reg [15:0] idle_hi_ctr       = 0;
	// When a repeated START fires mid-byte we need two RAM writes: the
	// partial byte this cycle, then the 0xFF marker next cycle.
	reg        rpt_start_pending = 0;

	// Combinatorials for the STOP corner case:
	// stop_cond and scl_rising can assert in the same cycle when the last
	// ACK/NACK bit lands right as SDA rises for STOP.
	wire [7:0] next_sda      = {shift_reg[6:0], sda_sync[1]};
	wire       stop_also_scl = stop_cond & scl_rising;
	wire [7:0] stop_sr       = stop_also_scl ? next_sda    : shift_reg;
	wire [3:0] stop_bc       = stop_also_scl ? ({1'b0, bit_count} + 4'd1)
	                                         : {1'b0, bit_count};
	wire       stop_full_byte = stop_also_scl & (bit_count == 3'd7);
	wire [7:0] stop_byte      = stop_full_byte ? stop_sr
	                                           : (stop_sr << (4'd8 - stop_bc));

	wire       has_stop_byte  = (stop_bc != 4'd0) & ~buf_overflow
	                            & (cap_wr_ptr != 7'd127);
	wire [6:0] final_len      = cap_wr_ptr + (has_stop_byte ? 7'd1 : 7'd0);

	wire idle_timeout = (capture_state == 1'b1) & scl_sync[1] & ~bus_activity
	                  & (idle_hi_ctr == IDLE_TIMEOUT_CYCLES - 1);

	// ----------------------------------------------------------------
	// SPI drain state machine registers
	// ----------------------------------------------------------------
	localparam S_IDLE      = 3'd0;
	localparam S_CS_LOW    = 3'd1;
	localparam S_CLKING    = 3'd2;
	localparam S_NEXT_BYTE = 3'd3;
	localparam S_CS_HIGH   = 3'd4;

	reg [2:0] spi_state     = S_IDLE;
	reg [1:0] dr_buf_sel    = 0;
	reg [6:0] dr_bytes_left = 0;
	reg [6:0] dr_byte_idx   = 0;
	reg [7:0] spi_shift     = 8'hFF;
	reg [2:0] spi_bit_idx   = 3'd7;
	reg       spi_load_pend = 0;
	reg       sck_phase     = 0;
	reg       sck_out       = 0;
	reg       mosi_out      = 0;
	reg       cs_out        = 1;

	// Queue handshake signals.
	// push_en: capture completed a message and queue has room.
	// pop_en:  SPI is idle, queue is non-empty, a tick fired, AND no push
	//          is happening this cycle (push wins to avoid reading a slot
	//          that is being written when the queue was previously empty).
	wire push_en = (capture_state == 1'b1) & stop_cond & ~q_full
	             & (final_len != 7'd0);
	wire pop_en  = (spi_state == S_IDLE) & ~q_empty & tick & ~push_en;

	// ----------------------------------------------------------------
	// Main clocked process — capture + drain + queue share one block
	// to avoid multi-driver conflicts on q_count.
	// ----------------------------------------------------------------
	always @(posedge clk_core) begin
		cap_wr_en <= 1'b0;
		dr_rd_en  <= 1'b0;

		// -- idle-high timeout counter --
		if (capture_state == 1'b1) begin
			if (scl_sync[1] && !bus_activity)
				idle_hi_ctr <= idle_hi_ctr + 1'b1;
			else
				idle_hi_ctr <= 16'd0;
		end else
			idle_hi_ctr <= 16'd0;

		// -- queue count: one assignment handles all four cases --
		case ({push_en, pop_en})
			2'b10:   q_count <= q_count + 2'd1;
			2'b01:   q_count <= q_count - 2'd1;
			default: q_count <= q_count;
		endcase

		// ==============================================================
		// CAPTURE
		// ==============================================================
		if (idle_timeout) begin
			// Abandon current buffer without queuing it
			buf_wr_sel        <= buf_wr_sel + 2'd1;
			cap_wr_ptr        <= 7'd0;
			bit_count         <= 3'd0;
			shift_reg         <= 8'd0;
			buf_overflow      <= 1'b0;
			rpt_start_pending <= 1'b0;
			capture_state     <= 1'b0;

		end else if (capture_state == 1'b0) begin
			// CAP_IDLE
			cap_wr_ptr        <= 7'd0;
			bit_count         <= 3'd0;
			shift_reg         <= 8'd0;
			buf_overflow      <= 1'b0;
			rpt_start_pending <= 1'b0;
			if (start_cond)
				capture_state <= 1'b1;

		end else begin
			// CAP_ACTIVE
			if (rpt_start_pending) begin
				// Second cycle of repeated-START: write the deferred 0xFF marker
				rpt_start_pending <= 1'b0;
				if (!buf_overflow) begin
					if (cap_wr_ptr == 7'd127)
						buf_overflow <= 1'b1;
					else begin
						cap_wr_addr <= {buf_wr_sel, cap_wr_ptr};
						cap_wr_data <= 8'hFF;
						cap_wr_en   <= 1'b1;
						cap_wr_ptr  <= cap_wr_ptr + 7'd1;
					end
				end

			end else if (stop_cond) begin
				// Flush the last partial/complete byte if any bits collected
				if (has_stop_byte) begin
					cap_wr_addr <= {buf_wr_sel, cap_wr_ptr};
					cap_wr_data <= stop_byte;
					cap_wr_en   <= 1'b1;
				end

				// Push completed capture to queue (push_en guards full check)
				if (push_en) begin
					q_ent[q_wr] <= {buf_wr_sel, final_len};
					q_wr        <= q_inc(q_wr);
				end

				// Flip to next buffer and return to idle immediately —
				// capture is now ready for the very next START.
				buf_wr_sel    <= buf_wr_sel + 2'd1;
				cap_wr_ptr    <= 7'd0;
				bit_count     <= 3'd0;
				shift_reg     <= 8'd0;
				buf_overflow  <= 1'b0;
				capture_state <= 1'b0;

			end else if (start_cond) begin
				// Repeated START: flush any partial byte first, then write 0xFF.
				// A single-port RAM can only do one write per cycle, so if there
				// are pending bits we write the partial byte now and defer 0xFF
				// to the next cycle via rpt_start_pending.
				if (bit_count != 3'd0 && !buf_overflow) begin
					if (cap_wr_ptr == 7'd127) begin
						buf_overflow <= 1'b1;
						// overflow: skip both partial byte and marker
					end else begin
						cap_wr_addr       <= {buf_wr_sel, cap_wr_ptr};
						cap_wr_data       <= shift_reg << (4'd8 - {1'b0, bit_count});
						cap_wr_en         <= 1'b1;
						cap_wr_ptr        <= cap_wr_ptr + 7'd1;
						rpt_start_pending <= 1'b1;  // write 0xFF next cycle
					end
				end else if (!buf_overflow) begin
					// No partial bits — write 0xFF immediately
					if (cap_wr_ptr == 7'd127) begin
						buf_overflow <= 1'b1;
					end else begin
						cap_wr_addr <= {buf_wr_sel, cap_wr_ptr};
						cap_wr_data <= 8'hFF;
						cap_wr_en   <= 1'b1;
						cap_wr_ptr  <= cap_wr_ptr + 7'd1;
					end
				end
				bit_count <= 3'd0;
				shift_reg <= 8'd0;

			end else if (scl_rising) begin
				shift_reg <= next_sda;
				if (bit_count == 3'd7) begin
					if (!buf_overflow) begin
						if (cap_wr_ptr == 7'd127) begin
							buf_overflow <= 1'b1;
						end else begin
							cap_wr_addr <= {buf_wr_sel, cap_wr_ptr};
							cap_wr_data <= next_sda;
							cap_wr_en   <= 1'b1;
							cap_wr_ptr  <= cap_wr_ptr + 7'd1;
						end
					end
					bit_count <= 3'd0;
				end else
					bit_count <= bit_count + 3'd1;
			end
		end

		// ==============================================================
		// SPI DRAIN  (runs only on tick edges)
		// ==============================================================
		if (tick) begin
			case (spi_state)
				S_IDLE: begin
					cs_out   <= 1'b1;
					sck_out  <= 1'b0;
					mosi_out <= 1'b0;
					if (pop_en) begin
						// Dequeue and load sentinel byte
						dr_buf_sel    <= q_ent[q_rd][8:7];
						dr_bytes_left <= q_ent[q_rd][6:0];
						dr_byte_idx   <= 7'd0;
						q_rd          <= q_inc(q_rd);
						spi_shift     <= 8'hFF;
						spi_bit_idx   <= 3'd7;
						spi_load_pend <= 1'b0;
						spi_state     <= S_CS_LOW;
					end
				end

				S_CS_LOW: begin
					cs_out      <= 1'b0;
					mosi_out    <= spi_shift[7];
					sck_phase   <= 1'b0;
					spi_bit_idx <= 3'd7;
					spi_state   <= S_CLKING;
				end

				S_CLKING: begin
					if (!sck_phase) begin
						sck_out   <= 1'b1;
						sck_phase <= 1'b1;
					end else begin
						sck_out   <= 1'b0;
						sck_phase <= 1'b0;
						spi_shift <= {spi_shift[6:0], 1'b0};
						if (spi_bit_idx == 3'd0) begin
							spi_state <= S_NEXT_BYTE;
						end else begin
							mosi_out    <= spi_shift[6];
							spi_bit_idx <= spi_bit_idx - 3'd1;
						end
					end
				end

				S_NEXT_BYTE: begin
					if (!spi_load_pend) begin
						if (dr_bytes_left != 7'd0) begin
							dr_rd_addr    <= {dr_buf_sel, dr_byte_idx};
							dr_rd_en      <= 1'b1;
							dr_byte_idx   <= dr_byte_idx   + 7'd1;
							dr_bytes_left <= dr_bytes_left - 7'd1;
							spi_load_pend <= 1'b1;
						end else
							spi_state <= S_CS_HIGH;
					end else begin
						spi_shift     <= dr_rd_data;
						mosi_out      <= dr_rd_data[7];
						spi_bit_idx   <= 3'd7;
						sck_phase     <= 1'b0;
						spi_load_pend <= 1'b0;
						spi_state     <= S_CLKING;
					end
				end

				S_CS_HIGH: begin
					cs_out    <= 1'b1;
					sck_out   <= 1'b0;
					spi_state <= S_IDLE;
				end
			endcase
		end
	end

	// ----------------------------------------------------------------
	// Output assignments
	// ----------------------------------------------------------------
	assign spi_sck  = sck_out;
	assign spi_mosi = mosi_out;
	assign spi_cs   = cs_out;
	assign led_r    = ~start_led_on;
	assign led_g    = ~capture_state;
	assign led_b    = ~(spi_state != S_IDLE);

endmodule
