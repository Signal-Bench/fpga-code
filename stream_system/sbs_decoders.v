/** \file
 * SignalBench STREAM decoders: raw bytes out, no framing.
 *
 * Every module here emits a plain byte stream -- out_stb pulses for one core
 * clock with the byte on out_byte, at most one byte per clock.  There are no
 * record headers, lengths or timestamps; what goes into the FIFO is what the
 * MCU reads off MISO.  See README.md for what "raw" means per protocol.
 *
 * This is deliberately different from ../final_system/, whose decoders emit
 * structured events (flags + payload) into a framed record packer.  The
 * stream version reuses the PROVEN capture logic of the original sniffers
 * instead, because "raw" here means "what those sniffers produced":
 *
 *   sbs_i2c   port of i2c_sniffer.v's capture FSM, unchanged in behaviour
 *   sbs_uart  the two uart_rx receivers from uart_to_spi.v, unchanged
 *   sbs_spi   ../final_system/sb_decoders.v :: sb_spi, two bytes per pair
 *   sbs_dio   new -- one byte per edge
 *   sbs_can   can_bit_timing + can_frame_fsm, can_sniffer.v's 20-byte record
 */

// ====================================================================
// I2C -- raw packed SDA bitstream, ACK bits included.
//
// A port of i2c_sniffer.v's CAPTURE state machine, keeping its behaviour
// bit for bit, including the three fixes recorded in i2c_sniffer.md:
//   - repeated START flushes a partial byte, then marks the START
//   - ACK-immediately-before-STOP: when STOP and the last scl_rising land
//     in the same sample, the last bit is folded in before flushing
//   - 1 ms idle timeout while active, recovering from a missed STOP
//
// What is removed is everything downstream of capture: the 4x128-byte
// ping-pong RAM, the drain queue and the SPI master.  Bytes go straight
// to the stream instead.
//
// Output byte sequence for one transaction is the same as what
// i2c_sniffer.v drained for it, with ONE intentional difference in where
// the leading 0xFF comes from:
//
//   i2c_sniffer.v:  the DRAIN prepends 0xFF to each completed capture
//   here:           0xFF is emitted at the START itself
//
// Consequences of streaming rather than buffering a whole transaction:
//   - a START followed by STOP with no data emits a lone 0xFF; the original
//     dropped empty captures (final_len == 0) and sent nothing
//   - an idle timeout can't take back bytes already streamed; the original
//     abandoned the whole buffer
//   - there is no 128-byte per-transaction limit; the FIFO bounds it instead
//
// START_MARKER = 0 suppresses the 0xFF markers entirely, if you would rather
// have nothing but bus bits.
// ====================================================================
module sbs_i2c #(
	parameter integer CORE_HZ      = 24_000_000,
	parameter integer START_MARKER = 1
)(
	input  wire       clk,
	input  wire       rst,
	input  wire       scl_pin,
	input  wire       sda_pin,
	output reg        out_stb,
	output reg  [7:0] out_byte
);
	localparam integer IDLE_TIMEOUT_CYCLES = CORE_HZ / 1000;    // 1 ms, as before
	localparam [7:0]   MARKER = 8'hFF;

	// -- 3-stage synchronizers + edges, identical to i2c_sniffer.v --------
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

	reg        capture_state     = 1'b0;   // 0 = IDLE, 1 = ACTIVE
	reg [2:0]  bit_count         = 3'd0;
	reg [7:0]  shift_reg         = 8'd0;
	reg [15:0] idle_hi_ctr       = 16'd0;
	reg        rpt_start_pending = 1'b0;

	// -- STOP corner case, identical to i2c_sniffer.v ---------------------
	// stop_cond and scl_rising can assert in the same cycle when the last
	// ACK/NACK bit lands right as SDA rises for STOP (i2c_sniffer.md bug 3).
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

	always @(posedge clk) begin
		out_stb <= 1'b0;

		if (rst) begin
			capture_state     <= 1'b0;
			bit_count         <= 3'd0;
			shift_reg         <= 8'd0;
			idle_hi_ctr       <= 16'd0;
			rpt_start_pending <= 1'b0;
		end else begin
			// -- idle-high timeout counter --
			if (capture_state == 1'b1 && scl_sync[1] && !bus_activity)
				idle_hi_ctr <= idle_hi_ctr + 16'd1;
			else
				idle_hi_ctr <= 16'd0;

			if (idle_timeout) begin
				bit_count         <= 3'd0;
				shift_reg         <= 8'd0;
				rpt_start_pending <= 1'b0;
				capture_state     <= 1'b0;

			end else if (capture_state == 1'b0) begin
				// CAP_IDLE
				bit_count         <= 3'd0;
				shift_reg         <= 8'd0;
				rpt_start_pending <= 1'b0;
				if (start_cond) begin
					capture_state <= 1'b1;
					// This 0xFF is the one i2c_sniffer.v's drain prepended.
					if (START_MARKER != 0) begin
						out_byte <= MARKER;
						out_stb  <= 1'b1;
					end
				end

			end else begin
				// CAP_ACTIVE
				if (rpt_start_pending) begin
					// Second cycle of a repeated START: the deferred marker.
					rpt_start_pending <= 1'b0;
					if (START_MARKER != 0) begin
						out_byte <= MARKER;
						out_stb  <= 1'b1;
					end

				end else if (stop_cond) begin
					// Flush the last partial/complete byte, left-aligned.
					if (has_stop_byte) begin
						out_byte <= stop_byte;
						out_stb  <= 1'b1;
					end
					bit_count     <= 3'd0;
					shift_reg     <= 8'd0;
					capture_state <= 1'b0;

				end else if (start_cond) begin
					// Repeated START: flush any partial byte, then mark.  One
					// byte per clock, so with a partial byte the marker is
					// deferred one cycle -- same as the original's single
					// RAM write port forced.
					if (bit_count != 3'd0) begin
						out_byte          <= shift_reg << (4'd8 - {1'b0, bit_count});
						out_stb           <= 1'b1;
						rpt_start_pending <= 1'b1;
					end else if (START_MARKER != 0) begin
						out_byte <= MARKER;
						out_stb  <= 1'b1;
					end
					bit_count <= 3'd0;
					shift_reg <= 8'd0;

				end else if (scl_rising) begin
					shift_reg <= next_sda;
					if (bit_count == 3'd7) begin
						out_byte  <= next_sda;
						out_stb   <= 1'b1;
						bit_count <= 3'd0;
					end else begin
						bit_count <= bit_count + 3'd1;
					end
				end
			end
		end
	end
endmodule


// ====================================================================
// UART -- the target's TX and RX lines, merged, untagged.
//
// Same receivers and same baud arrangement as uart_to_spi.v: two uart_rx
// instances from common/uart.v sharing one divide_by_n baud_x4 tick.
// At 24 MHz, N = 24e6 / (4 * 9600) = 625 gives exactly the 38.4 kHz that
// uart_to_spi.v gets from 48 MHz / 1250.
//
// Removed: uart_to_spi.v's per-byte [0x48 header][0x01|0x02 direction]
// framing.  Bytes from both lines are merged in arrival order with no tag,
// so if BOTH lines carry traffic the MCU cannot tell them apart.  With one
// line wired (the other idles high and produces nothing) it is unambiguous.
//
// Added: if both receivers finish a byte in the same core clock, the RX
// byte is held one cycle instead of being lost.  uart_to_spi.v queued both
// through its own FIFO so it never had this problem; a single-strobe
// stream output does.
// ====================================================================
module sbs_uart #(
	parameter integer CORE_HZ = 24_000_000,
	parameter integer BAUD    = 9600
)(
	input  wire       clk,
	input  wire       rst,
	input  wire       line_a,      // target's TX line (uart_to_spi gpio_38)
	input  wire       line_b,      // target's RX line (uart_to_spi gpio_42)
	output wire       out_stb,
	output wire [7:0] out_byte
);
	wire baud_x4;
	divide_by_n #(.N(CORE_HZ / (4 * BAUD))) u_div (clk, rst, baud_x4);

	wire [7:0] a_data, b_data;
	wire       a_stb,  b_stb;
	uart_rx u_a (.mclk(clk), .reset(rst), .baud_x4(baud_x4),
	             .serial(line_a), .data(a_data), .data_strobe(a_stb));
	uart_rx u_b (.mclk(clk), .reset(rst), .baud_x4(baud_x4),
	             .serial(line_b), .data(b_data), .data_strobe(b_stb));

	reg       b_pend   = 1'b0;
	reg [7:0] b_pend_d = 8'd0;
	always @(posedge clk) begin
		if (rst)                   b_pend <= 1'b0;
		else if (a_stb && b_stb)   begin b_pend <= 1'b1; b_pend_d <= b_data; end
		else if (b_pend && !a_stb) b_pend <= 1'b0;   // emitted this cycle
	end

	wire b_out = ~a_stb & (b_stb | b_pend);

	assign out_stb  = a_stb | b_out;
	assign out_byte = a_stb ? a_data : (b_pend ? b_pend_d : b_data);
endmodule


// ====================================================================
// SPI -- MOSI byte then MISO byte, per 8 SCK edges.
//
// Wraps ../final_system/sb_decoders.v :: sb_spi, whose shift registers are
// clocked by the probed SCK (see that module).  Each completed pair is
// serialized as two stream bytes on consecutive core clocks.  Pairs arrive
// at most once per 8 SCK periods, so the second byte is always out before
// the next pair can land.
// ====================================================================
module sbs_spi (
	input  wire       clk,
	input  wire       rst,
	input  wire       sck_g,
	input  wire       mosi,
	input  wire       miso,
	input  wire       cs,
	input  wire [2:0] spi_mode,
	output reg        out_stb,
	output reg  [7:0] out_byte
);
	wire        p_stb;
	wire [63:0] p_payload;
	sb_spi u_core (
		.clk(clk), .rst(rst), .sck_g(sck_g),
		.mosi(mosi), .miso(miso), .cs(cs), .spi_mode(spi_mode),
		.ev_strobe(p_stb), .ev_flags(), .ev_aux(), .ev_len(),
		.ev_payload(p_payload)
	);

	reg       second   = 1'b0;
	reg [7:0] miso_hold = 8'd0;
	always @(posedge clk) begin
		out_stb <= 1'b0;
		if (rst) second <= 1'b0;
		else if (p_stb) begin
			out_byte  <= p_payload[63:56];   // MOSI
			out_stb   <= 1'b1;
			miso_hold <= p_payload[55:48];   // MISO
			second    <= 1'b1;
		end else if (second) begin
			out_byte <= miso_hold;
			out_stb  <= 1'b1;
			second   <= 1'b0;
		end
	end
endmodule


// ====================================================================
// DIO -- one byte per edge: 0x01 after a rising edge, 0x00 after falling.
//
// Not tested on hardware or exercised by the testbench.  A fast-toggling
// input will fill the FIFO; there is no rate limit.
// ====================================================================
module sbs_dio (
	input  wire       clk,
	input  wire       rst,
	input  wire       pin,
	output reg        out_stb,
	output reg  [7:0] out_byte
);
	reg [2:0] s = 3'b000;
	always @(posedge clk) begin
		out_stb <= 1'b0;
		if (rst) s <= {3{pin}};
		else begin
			s <= {s[1:0], pin};
			if (s[1] ^ s[2]) begin
				out_byte <= {7'd0, s[1]};
				out_stb  <= 1'b1;
			end
		end
	end
endmodule


// ====================================================================
// CAN -- can_sniffer.v's 20-byte record, serialized onto the stream.
//
//   AA | flags | id[4] | {dropped,3'b0,dlc} | data[8] | timestamp[4] | 55
//   flags = {ide, rtr, overload, ack_ok, crc_ok, err[2:0]}
//
// Same layout as can_sniffer.v so existing tooling still parses it.  Two
// differences: `dropped` is always 0 (loss happens in the shared FIFO and
// is shown on the red LED instead), and the timestamp counts BIT TIMES
// here just as there -- latched on the falling edge of bus_idle.
//
// Not tested on hardware in this build.  Runs at brp = 1 so that
// can_bit_timing's PIPELINE_RESYNC path is valid -- see unified_top.md.
// ====================================================================
module sbs_can (
	input  wire       clk,
	input  wire       rst,
	input  wire       can_rx,
	output reg        out_stb,
	output reg  [7:0] out_byte
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

	// Bit-time timestamp latched at SOF, as can_sniffer.v does.
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

	reg [159:0] rec  = 160'd0;
	reg [4:0]   left = 5'd0;
	always @(posedge clk) begin
		out_stb <= 1'b0;
		if (rst) left <= 5'd0;
		else if (f_stb) begin
			rec  <= {8'hAA,
			         f_ide, f_rtr, f_ovl, f_ack_ok, f_crc_ok, f_err,
			         3'b000, f_id,
			         4'b0000, f_dlc,
			         f_data,
			         sof_ts,
			         8'h55};
			left <= 5'd20;
		end else if (left != 5'd0) begin
			out_byte <= rec[159:152];
			out_stb  <= 1'b1;
			rec      <= {rec[151:0], 8'h00};
			left     <= left - 5'd1;
		end
	end
endmodule
