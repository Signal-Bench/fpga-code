/** \file
 * CAN 2.0B sniffer top module for iCE40 UP5K / UPduino v3.
 *
 * can_bit_timing -> can_frame_fsm -> record packer -> EBR ring -> SPI master.
 *
 * Clock MUST be the 12 MHz on-board oscillator (short jumper R16, "OSC"), not
 * SB_HFOSC -- see clock_choice.md.
 *
 * Record: 20 bytes, fixed, MSB first, framed by 0xAA ... 0x55 so a logic
 * analyzer or host can resynchronize on any record boundary.
 *
 *   [0]      0xAA   start marker
 *   [1]      flags  {ide, rtr, overload, ack_ok, crc_ok, err[2:0]}
 *   [2..5]   identifier, 29 bits right-justified in 32, big-endian
 *   [6]      {dropped, 3'b000, dlc[3:0]}
 *   [7..14]  data[0..7]  (left-justified: data[0] is always byte 7)
 *   [15..18] timestamp, 32 bits, big-endian, in BIT TIMES latched at SOF
 *   [19]     0x55   end marker
 *
 * err codes: 0 none, 1 stuff, 2 crc, 3 form, 4 ack, 5 unattributed, 6 stuck
 * dominant.  "dropped" means at least one record was lost to a full buffer
 * before this one.
 *
 * Timestamps count bit times, not core clocks, so at 1 Mbit/s the unit is
 * exactly 1 us and the value is bit-rate independent.  Latched on the falling
 * edge of bus_idle, which is the SOF that hard-synchronized the bit timing.
 *
 * Buffer: one EBR as 16 records x 32 bytes (only 20 used, but the power-of-two
 * stride makes addressing a concatenation instead of a multiply).
 *
 * At 6 MHz SCK a record drains in ~32 us.  The shortest possible CAN frame at
 * 1 Mbit/s (DLC 0, worst-case stuffing) is ~57 us, so the drain keeps up with
 * a fully saturated bus -- which it could not at 1 MHz, where a record took
 * 160 us.  The 16-record buffer is burst tolerance on top of that.
 *
 * SCK is not continuous: each byte costs 19 ticks, not 16, because the EBR
 * read takes a cycle to issue, a cycle to land, and a cycle to latch.  That
 * leaves SCK low for ~250 ns between bytes with CS still asserted, which is
 * ordinary SPI and decodes normally -- analyzers count clock edges, not time.
 */
module can_sniffer #(
	// SCK = clk_12 / (2 * SPI_HALF_DIV).  1 -> 6 MHz (the ceiling, since the
	// divider can only halve), 2 -> 3 MHz, 6 -> 1 MHz.  10 MHz is not
	// reachable from a 12 MHz clock by division and would need a PLL.
	parameter integer SPI_HALF_DIV = 1
)(
	input  wire clk_12,
	input  wire can_rx,
	output wire spi_sck,
	output wire spi_mosi,
	output wire spi_cs,
	output wire spi_cs_flash,
	output wire dbg_frame,
	output wire led_r,
	output wire led_g,
	output wire led_b
);
	assign spi_cs_flash = 1'b1;

	localparam integer CLK_HZ  = 12_000_000;
	localparam integer STRETCH = CLK_HZ / 20;      // 50 ms LED stretch

	// ----------------------------------------------------------------
	// Power-on reset
	// ----------------------------------------------------------------
	reg [7:0] por = 8'd0;
	wire      rst = ~por[7];
	always @(posedge clk_12) if (!por[7]) por <= por + 8'd1;

	reg [23:0] hb = 24'd0;
	always @(posedge clk_12) hb <= hb + 24'd1;
	wire heartbeat = hb[22];

	// ----------------------------------------------------------------
	// Decoder chain
	// ----------------------------------------------------------------
	wire bus_idle, sample_tick, sample_bit, bit_tick, rx_sync, hard_sync, resync;
	wire        frame_strobe, frame_ide, frame_rtr, frame_crc_ok;
	wire        frame_ack_ok, frame_overload;
	wire [28:0] frame_id;
	wire [3:0]  frame_dlc;
	wire [63:0] frame_data;
	wire [2:0]  frame_err;

	can_bit_timing u_bt (
		.clk(clk_12), .rst(rst), .brp(8'd0),
		.rx_raw(can_rx), .bus_idle(bus_idle),
		.sample_tick(sample_tick), .sample_bit(sample_bit),
		.bit_tick(bit_tick), .rx_sync(rx_sync),
		.hard_sync(hard_sync), .resync(resync)
	);

	can_frame_fsm u_fsm (
		.clk(clk_12), .rst(rst),
		.sample_tick(sample_tick), .sample_bit(sample_bit),
		.bus_idle(bus_idle), .frame_strobe(frame_strobe),
		.frame_id(frame_id), .frame_ide(frame_ide), .frame_rtr(frame_rtr),
		.frame_dlc(frame_dlc), .frame_data(frame_data),
		.frame_crc_ok(frame_crc_ok), .frame_ack_ok(frame_ack_ok),
		.frame_overload(frame_overload), .frame_err(frame_err)
	);

	// ----------------------------------------------------------------
	// Timestamp, in bit times, latched at start of frame.
	// bus_idle falls exactly when the frame FSM leaves idle on SOF.
	// ----------------------------------------------------------------
	reg [31:0] ts_ctr    = 32'd0;
	reg [31:0] ts_latch  = 32'd0;
	reg        idle_prev = 1'b1;

	always @(posedge clk_12) begin
		if (bit_tick) ts_ctr <= ts_ctr + 32'd1;
		idle_prev <= bus_idle;
		if (idle_prev & ~bus_idle) ts_latch <= ts_ctr;   // SOF
	end

	// ----------------------------------------------------------------
	// Record RAM: 16 records x 32 bytes = 512 B, one inferred EBR
	// ----------------------------------------------------------------
	reg [7:0] rec_ram [0:511];

	reg       ram_we   = 1'b0;
	reg [8:0] ram_wa   = 9'd0;
	reg [7:0] ram_wd   = 8'd0;
	reg       ram_re   = 1'b0;
	reg [8:0] ram_ra   = 9'd0;
	reg [7:0] ram_rd   = 8'd0;

	always @(posedge clk_12) begin
		if (ram_we) rec_ram[ram_wa] <= ram_wd;
		if (ram_re) ram_rd <= rec_ram[ram_ra];
	end

	// ----------------------------------------------------------------
	// Record writer
	// ----------------------------------------------------------------
	reg [3:0]  rec_wr    = 4'd0;
	reg [3:0]  rec_rd    = 4'd0;
	reg [4:0]  wr_byte   = 5'd0;
	reg        wr_busy   = 1'b0;
	reg        dropped   = 1'b0;   // sticky until reported

	// Latched copy of the record being written out
	reg [28:0] l_id;
	reg [63:0] l_data;
	reg [31:0] l_ts;
	reg [3:0]  l_dlc;
	reg [2:0]  l_err;
	reg        l_ide, l_rtr, l_ovl, l_ack, l_crc, l_drop;

	wire [3:0] rec_wr_next = rec_wr + 4'd1;
	wire       fifo_full   = (rec_wr_next == rec_rd);
	wire       fifo_empty  = (rec_wr == rec_rd);

	// 20:1 byte mux over the latched record
	reg [7:0] rec_byte;
	always @(*) begin
		case (wr_byte)
			5'd0:  rec_byte = 8'hAA;
			5'd1:  rec_byte = {l_ide, l_rtr, l_ovl, l_ack, l_crc, l_err};
			5'd2:  rec_byte = {3'b000, l_id[28:24]};
			5'd3:  rec_byte = l_id[23:16];
			5'd4:  rec_byte = l_id[15:8];
			5'd5:  rec_byte = l_id[7:0];
			5'd6:  rec_byte = {l_drop, 3'b000, l_dlc};
			5'd7:  rec_byte = l_data[63:56];
			5'd8:  rec_byte = l_data[55:48];
			5'd9:  rec_byte = l_data[47:40];
			5'd10: rec_byte = l_data[39:32];
			5'd11: rec_byte = l_data[31:24];
			5'd12: rec_byte = l_data[23:16];
			5'd13: rec_byte = l_data[15:8];
			5'd14: rec_byte = l_data[7:0];
			5'd15: rec_byte = l_ts[31:24];
			5'd16: rec_byte = l_ts[23:16];
			5'd17: rec_byte = l_ts[15:8];
			5'd18: rec_byte = l_ts[7:0];
			default: rec_byte = 8'h55;
		endcase
	end

	always @(posedge clk_12) begin
		ram_we <= 1'b0;

		if (rst) begin
			rec_wr  <= 4'd0;
			wr_busy <= 1'b0;
			dropped <= 1'b0;

		end else if (frame_strobe && (wr_busy || fifo_full)) begin
			// No room, or still writing the previous one: lose this record but
			// remember that it happened so the next one can say so.
			dropped <= 1'b1;

		end else if (frame_strobe) begin
			l_id   <= frame_id;
			l_data <= frame_data;
			l_ts   <= ts_latch;
			l_dlc  <= frame_dlc;
			l_err  <= frame_err;
			l_ide  <= frame_ide;
			l_rtr  <= frame_rtr;
			l_ovl  <= frame_overload;
			l_ack  <= frame_ack_ok;
			l_crc  <= frame_crc_ok;
			l_drop <= dropped;
			dropped <= 1'b0;
			wr_byte <= 5'd0;
			wr_busy <= 1'b1;

		end else if (wr_busy) begin
			ram_wa  <= {rec_wr, wr_byte};
			ram_wd  <= rec_byte;
			ram_we  <= 1'b1;
			wr_byte <= wr_byte + 5'd1;
			if (wr_byte == 5'd19) begin
				wr_busy <= 1'b0;
				rec_wr  <= rec_wr_next;
			end
		end
	end

	// ----------------------------------------------------------------
	// SPI master drain -- same bit-banged structure as i2c_sniffer.v, and
	// the same pins, so an existing logic-analyzer setup carries over.
	// Mode 0, MSB first.
	// ----------------------------------------------------------------
	reg [3:0] spi_div = 4'd0;
	wire      tick    = (spi_div == SPI_HALF_DIV[3:0] - 4'd1);
	always @(posedge clk_12) spi_div <= tick ? 4'd0 : spi_div + 4'd1;

	localparam [2:0] S_IDLE   = 3'd0;
	localparam [2:0] S_LOAD   = 3'd1;   // issue the EBR read
	localparam [2:0] S_SETTLE = 3'd2;   // read lands in ram_rd this cycle
	localparam [2:0] S_WAIT   = 3'd3;   // ram_rd valid: latch, drop CS
	localparam [2:0] S_SHIFT  = 3'd4;
	localparam [2:0] S_DONE   = 3'd5;

	reg [2:0] sst       = S_IDLE;
	reg [4:0] dr_byte   = 5'd0;
	reg [7:0] dr_sr     = 8'd0;
	reg [2:0] dr_bit    = 3'd7;
	reg       sck_ph    = 1'b0;
	reg       sck_r     = 1'b0;
	reg       mosi_r    = 1'b0;
	reg       cs_r      = 1'b1;

	always @(posedge clk_12) begin
		ram_re <= 1'b0;

		if (rst) begin
			sst   <= S_IDLE;
			cs_r  <= 1'b1;
			sck_r <= 1'b0;
			rec_rd<= 4'd0;

		end else if (tick) begin
			case (sst)
				S_IDLE: begin
					cs_r <= 1'b1; sck_r <= 1'b0; mosi_r <= 1'b0;
					if (!fifo_empty) begin
						dr_byte <= 5'd0;
						sst     <= S_LOAD;
					end
				end

				S_LOAD: begin
					ram_ra <= {rec_rd, dr_byte};
					ram_re <= 1'b1;
					sst    <= S_SETTLE;
				end

				// ram_re is registered, so the EBR performs the read on the
				// cycle after S_LOAD and ram_rd is only valid the cycle after
				// that.  With SPI_HALF_DIV = 1 the FSM advances every clock,
				// so without this state S_WAIT would latch the previous byte.
				// (At the old 1 MHz setting the six-clock tick hid the bug.)
				S_SETTLE: sst <= S_WAIT;

				S_WAIT: begin
					dr_sr  <= ram_rd;
					mosi_r <= ram_rd[7];
					dr_bit <= 3'd7;
					sck_ph <= 1'b0;
					cs_r   <= 1'b0;
					sst    <= S_SHIFT;
				end

				S_SHIFT: begin
					if (!sck_ph) begin
						sck_r  <= 1'b1;    // sample edge
						sck_ph <= 1'b1;
					end else begin
						sck_r  <= 1'b0;
						sck_ph <= 1'b0;
						dr_sr  <= {dr_sr[6:0], 1'b0};
						if (dr_bit == 3'd0) begin
							if (dr_byte == 5'd19) sst <= S_DONE;
							else begin
								dr_byte <= dr_byte + 5'd1;
								sst     <= S_LOAD;
							end
						end else begin
							mosi_r <= dr_sr[6];
							dr_bit <= dr_bit - 3'd1;
						end
					end
				end

				S_DONE: begin
					cs_r   <= 1'b1;
					sck_r  <= 1'b0;
					rec_rd <= rec_rd + 4'd1;
					sst    <= S_IDLE;
				end
				default: sst <= S_IDLE;
			endcase
		end
	end

	assign spi_sck  = sck_r;
	assign spi_mosi = mosi_r;
	assign spi_cs   = cs_r;

	// ----------------------------------------------------------------
	// Indicators -- priority encoded, one colour per state
	// ----------------------------------------------------------------
	reg [2:0] rx_ff = 3'b111;
	always @(posedge clk_12) rx_ff <= {rx_ff[1:0], can_rx};
	wire rx_edge = rx_ff[1] ^ rx_ff[2];

	reg [19:0] act_ctr = 20'd0;
	reg [19:0] ok_ctr  = 20'd0;
	reg [19:0] err_ctr = 20'd0;
	reg [4:0]  dbg_ctr = 5'd0;

	wire frame_ok  = frame_strobe & (frame_err == 3'd0);
	wire frame_bad = frame_strobe & (frame_err != 3'd0);

	always @(posedge clk_12) begin
		act_ctr <= rx_edge   ? STRETCH[19:0] : (act_ctr - (act_ctr != 0));
		ok_ctr  <= frame_ok  ? STRETCH[19:0] : (ok_ctr  - (ok_ctr  != 0));
		err_ctr <= frame_bad ? STRETCH[19:0] : (err_ctr - (err_ctr != 0));
		dbg_ctr <= frame_strobe ? 5'd12      : (dbg_ctr - (dbg_ctr != 0));
	end

	wire ok_recent  = (ok_ctr  != 0);
	wire bad_recent = (err_ctr != 0);
	wire act_recent = (act_ctr != 0);

	assign led_g     = ~ok_recent;
	assign led_r     = ~(~ok_recent & (bad_recent | (act_recent & heartbeat)));
	assign led_b     = ~(~ok_recent & ~bad_recent & ~act_recent & heartbeat);
	assign dbg_frame = (dbg_ctr != 0);
endmodule
