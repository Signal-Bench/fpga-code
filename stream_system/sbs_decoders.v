/** \file
 * SignalBench STREAM decoders, framed.
 *
 * Each decoder drives the framer (sbs_framer.v) with one op per clock:
 *   op_kind 0 BYTE(b)   append a payload byte
 *   op_kind 2 CLOSE(c)  end the frame; c = 1 marks the next frame CONT
 *
 * The payload each one produces is specified in MCU_FRAME_FORMAT.md.
 *
 * The capture logic underneath is unchanged from the unframed version:
 *   sbs_i2c   port of i2c_sniffer.v's capture FSM
 *   sbs_uart  the two uart_rx receivers from uart_to_spi.v
 *   sbs_spi   ../final_system/sb_decoders.v :: sb_spi
 *   sbs_dio   new
 *   sbs_can   can_bit_timing + can_frame_fsm
 *
 * Decoders that can produce long frames (I2C, SPI) split them themselves
 * rather than letting the framer do it, because only the decoder knows what
 * a payload must end with (I2C's info trailer, SPI's whole pairs).
 */

`define OP_BYTE  2'd0
`define OP_CLOSE 2'd2

// ====================================================================
// I2C -- one frame per START-delimited segment.
//
// Payload:  [packed SDA bits ...] [info]
//
//   Packed bits: every SCL-rising SDA sample, ACK/NACK bits included, 8 per
//   byte, MSB first -- exactly i2c_sniffer.v's raw bitstream.  The last data
//   byte is left-aligned and zero-padded if the segment didn't end on a
//   byte boundary, which it usually doesn't: 9 bits per I2C byte.
//
//   info:  bit7    RSTART   this segment began with a repeated START
//          bit6:5  END      00 STOP, 01 repeated START, 10 idle timeout
//                           (no STOP for 1 ms), 11 split: continues in the
//                           next frame, which has CONT set
//          bit4:3  0
//          bit2:0  number of valid bits in the last data byte; 0 = all 8.
//                  If the payload is only the info byte, there are none.
//
// A repeated START ends the frame and starts a new one with RSTART set,
// instead of i2c_sniffer.v's in-band 0xFF marker -- a marker can't be told
// apart from a data 0xFF, a frame boundary can.  A write-then-read register
// access is therefore two frames: [... END=01] then [RSTART=1 ...].
//
// Capture logic kept from i2c_sniffer.v: 3-stage synchronizers, START/STOP
// derivation, the ACK-before-STOP fix (stop_also_scl), the 1 ms idle timeout.
// ====================================================================
module sbs_i2c #(
	parameter integer CORE_HZ = 24_000_000,
	parameter integer MAXP    = 255
)(
	input  wire       clk,
	input  wire       rst,
	input  wire       scl_pin,
	input  wire       sda_pin,
	output reg        op_stb,
	output reg  [1:0] op_kind,
	output reg  [7:0] op_data
);
	localparam integer IDLE_TIMEOUT_CYCLES = CORE_HZ / 1000;

	reg [2:0] scl_sync = 3'b111;
	reg [2:0] sda_sync = 3'b111;
	always @(posedge clk) begin
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

	reg        capture_state = 1'b0;
	reg [2:0]  bit_count     = 3'd0;
	reg [7:0]  shift_reg     = 8'd0;
	reg [15:0] idle_hi_ctr   = 16'd0;
	reg [8:0]  dcount        = 9'd0;   // data bytes in the current frame
	reg        rstart        = 1'b0;   // current segment began with rSTART

	// i2c_sniffer.v's STOP corner case, unchanged
	wire [7:0] next_sda       = {shift_reg[6:0], sda_sync[1]};
	wire       stop_also_scl  = stop_cond & scl_rising;
	wire [7:0] stop_sr        = stop_also_scl ? next_sda : shift_reg;
	wire [3:0] stop_bc        = stop_also_scl ? ({1'b0, bit_count} + 4'd1)
	                                          : {1'b0, bit_count};
	wire       stop_full_byte = stop_also_scl & (bit_count == 3'd7);
	wire [7:0] stop_byte      = stop_full_byte ? stop_sr
	                                           : (stop_sr << (4'd8 - stop_bc));
	wire       has_stop_byte  = (stop_bc != 4'd0);

	wire idle_timeout = (capture_state == 1'b1) & scl_sync[1] & ~bus_activity
	                  & (idle_hi_ctr == IDLE_TIMEOUT_CYCLES - 1);

	// Tail sequencer: an ending segment needs up to three ops (last byte,
	// info, CLOSE) but the output carries one per clock.  The bus can't
	// produce another event within two clocks of any segment end.
	reg [1:0] tail     = 2'd0;   // ops still to send: 2 = info+close, 1 = close
	reg [7:0] tail_inf = 8'd0;
	reg       tail_c   = 1'b0;

	task emit(input [1:0] kd, input [7:0] dt);
		begin op_stb <= 1'b1; op_kind <= kd; op_data <= dt; end
	endtask

	// End the segment: optional last byte now, then info and CLOSE.
	task end_seg(input has_byte, input [7:0] b, input [7:0] inf, input cont);
		begin
			if (has_byte) begin
				emit(`OP_BYTE, b);
				tail <= 2'd2;
			end else begin
				emit(`OP_BYTE, inf);
				tail <= 2'd1;
			end
			tail_inf <= inf;
			tail_c   <= cont;
			dcount   <= 9'd0;
		end
	endtask

	always @(posedge clk) begin
		op_stb <= 1'b0;

		if (rst) begin
			capture_state <= 1'b0;
			bit_count     <= 3'd0;
			shift_reg     <= 8'd0;
			idle_hi_ctr   <= 16'd0;
			dcount        <= 9'd0;
			rstart        <= 1'b0;
			tail          <= 2'd0;
		end else begin
			if (capture_state == 1'b1 && scl_sync[1] && !bus_activity)
				idle_hi_ctr <= idle_hi_ctr + 16'd1;
			else
				idle_hi_ctr <= 16'd0;

			// -- finish a segment's tail first --
			if (tail == 2'd2) begin
				emit(`OP_BYTE, tail_inf);
				tail <= 2'd1;
			end else if (tail == 2'd1) begin
				emit(`OP_CLOSE, {7'd0, tail_c});
				tail <= 2'd0;
			end

			if (idle_timeout) begin
				// END=10, flushing any partial bits.
				end_seg(bit_count != 3'd0,
				        shift_reg << (4'd8 - {1'b0, bit_count}),
				        {rstart, 2'b10, 2'b00, bit_count}, 1'b0);
				bit_count     <= 3'd0;
				shift_reg     <= 8'd0;
				capture_state <= 1'b0;

			end else if (capture_state == 1'b0) begin
				bit_count <= 3'd0;
				shift_reg <= 8'd0;
				if (start_cond) begin
					capture_state <= 1'b1;
					rstart        <= 1'b0;
					dcount        <= 9'd0;
				end

			end else begin
				if (stop_cond) begin
					// END=00.  lastbits: 0 if the flushed byte is full.
					end_seg(has_stop_byte, stop_byte,
					        {rstart, 2'b00, 2'b00,
					         stop_full_byte ? 3'd0 : stop_bc[2:0]}, 1'b0);
					bit_count     <= 3'd0;
					shift_reg     <= 8'd0;
					capture_state <= 1'b0;

				end else if (start_cond) begin
					// Repeated START: END=01 here, RSTART on the next frame.
					end_seg(bit_count != 3'd0,
					        shift_reg << (4'd8 - {1'b0, bit_count}),
					        {rstart, 2'b01, 2'b00, bit_count}, 1'b0);
					rstart    <= 1'b1;
					bit_count <= 3'd0;
					shift_reg <= 8'd0;

				end else if (scl_rising) begin
					shift_reg <= next_sda;
					if (bit_count == 3'd7) begin
						bit_count <= 3'd0;
						if (dcount + 9'd2 >= MAXP) begin
							// Split: this byte, then info END=11, then CLOSE
							// with CONT for the frame that carries on.
							end_seg(1'b1, next_sda, {rstart, 2'b11, 2'b00, 3'd0}, 1'b1);
							rstart <= 1'b0;
						end else begin
							emit(`OP_BYTE, next_sda);
							dcount <= dcount + 9'd1;
						end
					end else begin
						bit_count <= bit_count + 3'd1;
					end
				end
			end
		end
	end
endmodule


// ====================================================================
// UART -- one frame per received byte, or per simultaneous pair.
//
// Payload is one or two [line][byte] pairs, so the frame length says which:
//
//   length 2   [line][byte]                  one line had a byte
//   length 4   [0x01][byte A][0x02][byte B]  both lines had one at once
//
//   line 0x01 = ch0 / pin 38 (target TX), 0x02 = ch1 / pin 42 (target RX)
//   -- the values uart_to_spi.v used for its direction byte.  Because the
//   pairs sit at fixed offsets, a data byte that happens to equal 0x01 or
//   0x02 can't be mistaken for a line tag.
//
// "At once" means both bytes are waiting when the frame starts, i.e. they
// finished within about a microsecond of each other -- genuinely
// simultaneous full-duplex traffic, including the same-clock case.  A
// request on one line followed by a reply on the other is two frames.
//
// No idle timeout and no splitting: every frame is complete as soon as the
// byte (or pair) is.  Receivers and baud tick are uart_to_spi.v's.
// ====================================================================
module sbs_uart #(
	parameter integer CORE_HZ = 24_000_000,
	parameter integer BAUD    = 9600
)(
	input  wire       clk,
	input  wire       rst,
	input  wire       line_a,
	input  wire       line_b,
	output reg        op_stb,
	output reg  [1:0] op_kind,
	output reg  [7:0] op_data
);
	localparam [7:0] TAG_A = 8'h01, TAG_B = 8'h02;

	wire baud_x4;
	divide_by_n #(.N(CORE_HZ / (4 * BAUD))) u_div (clk, rst, baud_x4);

	wire [7:0] a_data, b_data;
	wire       a_stb,  b_stb;
	uart_rx u_a (.mclk(clk), .reset(rst), .baud_x4(baud_x4),
	             .serial(line_a), .data(a_data), .data_strobe(a_stb));
	uart_rx u_b (.mclk(clk), .reset(rst), .baud_x4(baud_x4),
	             .serial(line_b), .data(b_data), .data_strobe(b_stb));

	// One holding register per line.  A line can't deliver a second byte
	// within a character time; a frame takes at most 5 clocks to emit.
	reg       ha = 1'b0, hb = 1'b0;
	reg [7:0] ha_d = 8'd0, hb_d = 8'd0;

	// Emission: snapshot what's waiting, then send it one op per clock.
	reg [2:0]  n_ops = 3'd0;       // ops left in the current frame, incl. CLOSE
	reg [31:0] seq   = 32'd0;      // pending payload bytes, MSB first
	reg        take_a, take_b;

	always @(posedge clk) begin
		op_stb <= 1'b0;
		take_a = 1'b0; take_b = 1'b0;

		if (rst) begin
			ha <= 1'b0; hb <= 1'b0; n_ops <= 3'd0;
		end else begin
			if (n_ops != 3'd0) begin
				op_stb <= 1'b1;
				if (n_ops == 3'd1) begin
					op_kind <= `OP_CLOSE; op_data <= 8'd0;
				end else begin
					op_kind <= `OP_BYTE;  op_data <= seq[31:24];
					seq <= {seq[23:0], 8'h00};
				end
				n_ops <= n_ops - 3'd1;
			end else if (ha && hb) begin
				seq   <= {TAG_A, ha_d, TAG_B, hb_d};
				n_ops <= 3'd5;                       // 4 bytes + CLOSE
				take_a = 1'b1; take_b = 1'b1;
			end else if (ha) begin
				seq   <= {TAG_A, ha_d, 16'd0};
				n_ops <= 3'd3;                       // 2 bytes + CLOSE
				take_a = 1'b1;
			end else if (hb) begin
				seq   <= {TAG_B, hb_d, 16'd0};
				n_ops <= 3'd3;
				take_b = 1'b1;
			end

			// Consume, then capture: a byte arriving this clock is kept.
			if (take_a) ha <= 1'b0;
			if (take_b) hb <= 1'b0;
			if (a_stb) begin ha <= 1'b1; ha_d <= a_data; end
			if (b_stb) begin hb <= 1'b1; hb_d <= b_data; end
		end
	end
endmodule


// ====================================================================
// SPI -- one frame per CS assertion.  Payload: [mosi][miso] pairs.
//
// The frame closes a few clocks after CS rises (synchronized into the core
// domain), so the last pair -- which crosses through its own synchronizer --
// lands before the CLOSE.  Splits at the last whole pair that fits, with
// CONT on the next frame.  Bits after the last whole byte are dropped, as
// in sb_spi.  Not exercised by the testbench.
// ====================================================================
module sbs_spi #(
	parameter integer MAXP = 255
)(
	input  wire       clk,
	input  wire       rst,
	input  wire       sck_g,
	input  wire       mosi,
	input  wire       miso,
	input  wire       cs,
	input  wire [2:0] spi_mode,
	output reg        op_stb,
	output reg  [1:0] op_kind,
	output reg  [7:0] op_data
);
	wire        p_stb;
	wire [63:0] p_payload;
	sb_spi u_core (
		.clk(clk), .rst(rst), .sck_g(sck_g),
		.mosi(mosi), .miso(miso), .cs(cs), .spi_mode(spi_mode),
		.ev_strobe(p_stb), .ev_flags(), .ev_aux(), .ev_len(),
		.ev_payload(p_payload)
	);

	reg [2:0] cs_s = 3'b111;
	always @(posedge clk) cs_s <= {cs_s[1:0], cs};
	wire cs_rise = cs_s[1] & ~cs_s[2];

	reg [2:0] close_dly = 3'd0;     // counts down to the CS CLOSE
	reg [1:0] st = 2'd0;            // 0 idle, 1 send MISO, 2 send split CLOSE
	reg [7:0] miso_hold = 8'd0;
	reg [8:0] plen = 9'd0;

	always @(posedge clk) begin
		op_stb <= 1'b0;
		if (rst) begin
			st <= 2'd0; plen <= 9'd0; close_dly <= 3'd0;
		end else begin
			if (cs_rise) close_dly <= 3'd4;
			else if (close_dly > 3'd1) close_dly <= close_dly - 3'd1;

			if (p_stb) begin
				op_stb <= 1'b1; op_kind <= `OP_BYTE; op_data <= p_payload[63:56];
				miso_hold <= p_payload[55:48];
				st <= 2'd1;
			end else if (st == 2'd1) begin
				op_stb <= 1'b1; op_kind <= `OP_BYTE; op_data <= miso_hold;
				if (plen + 9'd4 > MAXP) begin
					st <= 2'd2;                    // no room for another pair
				end else begin
					plen <= plen + 9'd2;
					st <= 2'd0;
				end
			end else if (st == 2'd2) begin
				op_stb <= 1'b1; op_kind <= `OP_CLOSE; op_data <= 8'd1;
				plen <= 9'd0; st <= 2'd0;
			end else if (close_dly == 3'd1) begin
				close_dly <= 3'd0;
				if (plen != 9'd0) begin
					op_stb <= 1'b1; op_kind <= `OP_CLOSE; op_data <= 8'd0;
				end
				plen <= 9'd0;
			end
		end
	end
endmodule


// ====================================================================
// DIO -- one frame per edge.  Payload: [level][timestamp, 4 bytes, BE].
//
// level: 0x01 after a rising edge, 0x00 after a falling edge.
// timestamp: microseconds since reset (or since DIO was selected), wraps
// after ~71 minutes.  An edge arriving while the previous frame is still
// being sent (6 clocks) is missed.  Not exercised by the testbench.
// ====================================================================
module sbs_dio #(
	parameter integer CORE_HZ = 24_000_000
)(
	input  wire       clk,
	input  wire       rst,
	input  wire       pin,
	output reg        op_stb,
	output reg  [1:0] op_kind,
	output reg  [7:0] op_data
);
	localparam integer US_DIV = CORE_HZ / 1_000_000;

	reg [4:0]  us_pre = 5'd0;
	reg [31:0] us     = 32'd0;
	always @(posedge clk) begin
		if (rst) begin us_pre <= 0; us <= 0; end
		else if (us_pre == US_DIV - 1) begin us_pre <= 0; us <= us + 32'd1; end
		else us_pre <= us_pre + 5'd1;
	end

	reg [2:0]  s = 3'b000;
	reg [39:0] rec = 40'd0;
	reg [2:0]  left = 3'd0;    // bytes still to send, then a CLOSE
	reg        closing = 1'b0;

	always @(posedge clk) begin
		op_stb <= 1'b0;
		if (rst) begin
			s <= {3{pin}}; left <= 3'd0; closing <= 1'b0;
		end else begin
			s <= {s[1:0], pin};
			if (left != 3'd0) begin
				op_stb <= 1'b1; op_kind <= `OP_BYTE; op_data <= rec[39:32];
				rec  <= {rec[31:0], 8'h00};
				left <= left - 3'd1;
				if (left == 3'd1) closing <= 1'b1;
			end else if (closing) begin
				op_stb <= 1'b1; op_kind <= `OP_CLOSE; op_data <= 8'd0;
				closing <= 1'b0;
			end else if (s[1] ^ s[2]) begin
				rec  <= {7'd0, s[1], us};
				left <= 3'd5;
			end
		end
	end
endmodule


// ====================================================================
// CAN -- one frame per CAN frame.  Payload, 18 bytes:
//
//   flags | id[4] | dlc | data[8] | timestamp[4]
//   flags = {ide, rtr, overload, ack_ok, crc_ok, err[2:0]}
//
// can_sniffer.v's 20-byte record without its AA/55 markers, which the
// frame header now replaces.  Timestamp in bit times, latched at SOF.
// Fixed at 1 Mbit/s.  Not exercised by the testbench.
// ====================================================================
module sbs_can (
	input  wire       clk,
	input  wire       rst,
	input  wire       can_rx,
	output reg        op_stb,
	output reg  [1:0] op_kind,
	output reg  [7:0] op_data
);
	wire smp_tick, smp_bit, bus_idle, bit_tick, f_stb;
	wire [28:0] f_id; wire [3:0] f_dlc; wire [63:0] f_data;
	wire f_ide, f_rtr, f_crc_ok, f_ack_ok, f_ovl; wire [2:0] f_err;

	can_bit_timing #(
		.PROP_SEG(4), .PHASE_SEG1(4), .PHASE_SEG2(3), .SJW(3),
		.PIPELINE_RESYNC(1)
	) u_bt (
		.clk(clk), .rst(rst), .brp(8'd1), .rx_raw(can_rx),
		.bus_idle(bus_idle), .sample_tick(smp_tick), .sample_bit(smp_bit),
		.bit_tick(bit_tick), .rx_sync(), .hard_sync(), .resync()
	);
	can_frame_fsm u_fsm (
		.clk(clk), .rst(rst),
		.sample_tick(smp_tick), .sample_bit(smp_bit), .bus_idle(bus_idle),
		.frame_strobe(f_stb), .frame_id(f_id), .frame_ide(f_ide),
		.frame_rtr(f_rtr), .frame_dlc(f_dlc), .frame_data(f_data),
		.frame_crc_ok(f_crc_ok), .frame_ack_ok(f_ack_ok),
		.frame_overload(f_ovl), .frame_err(f_err)
	);

	reg [31:0] bit_time = 32'd0, sof_ts = 32'd0;
	reg        idle_d   = 1'b1;
	always @(posedge clk) begin
		if (rst) begin bit_time <= 0; idle_d <= 1'b1; end
		else begin
			if (bit_tick) bit_time <= bit_time + 32'd1;
			idle_d <= bus_idle;
			if (idle_d && !bus_idle) sof_ts <= bit_time;
		end
	end

	reg [143:0] rec  = 144'd0;
	reg [4:0]   left = 5'd0;
	reg         closing = 1'b0;
	always @(posedge clk) begin
		op_stb <= 1'b0;
		if (rst) begin left <= 5'd0; closing <= 1'b0; end
		else if (left != 5'd0) begin
			op_stb <= 1'b1; op_kind <= `OP_BYTE; op_data <= rec[143:136];
			rec  <= {rec[135:0], 8'h00};
			left <= left - 5'd1;
			if (left == 5'd1) closing <= 1'b1;
		end else if (closing) begin
			op_stb <= 1'b1; op_kind <= `OP_CLOSE; op_data <= 8'd0;
			closing <= 1'b0;
		end else if (f_stb) begin
			rec  <= {f_ide, f_rtr, f_ovl, f_ack_ok, f_crc_ok, f_err,
			         3'b000, f_id,
			         4'b0000, f_dlc,
			         f_data,
			         sof_ts};
			left <= 5'd18;
		end
	end
endmodule
