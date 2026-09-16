/** \file
 * Minimal CAN sniffer bring-up top module.
 *
 * No SPI drain, no capture RAM, no record packing -- just clock, pin, decoder
 * and LEDs.  Its whole job is to answer, on hardware, how far the chain gets:
 *
 *   dark             no power / not configured
 *   blue blinking    clocked (R16 "OSC" jumper is good), no edges on can_rx
 *   red blinking     edges arriving on can_rx, but nothing decodes
 *   red solid        frames decoding, but every one of them has an error
 *   green            frames decoding cleanly
 *
 * The states are priority-encoded and mutually exclusive, so each shows as one
 * colour rather than a mix -- a healthy bus is plain green.
 *
 * dbg_frame (FPGA pin 42, header 22 -- next to can_rx on 23) pulses high for ~1 us
 * on every decoded record, for a scope trigger.
 *
 * Clock MUST come from the 12 MHz on-board oscillator (short jumper R16,
 * silkscreen "OSC") -- not SB_HFOSC.  See clock_choice.md.
 */
module can_bringup (
	input  wire clk_12,
	input  wire can_rx,
	output wire led_r,
	output wire led_g,
	output wire led_b,
	output wire dbg_frame,
	output wire spi_cs_flash
);
	assign spi_cs_flash = 1'b1;   // disable the on-board flash

	localparam integer CLK_HZ     = 12_000_000;
	localparam integer STRETCH    = CLK_HZ / 20;   // 50 ms LED stretch

	// ----------------------------------------------------------------
	// Power-on reset
	// ----------------------------------------------------------------
	reg [7:0] por = 8'd0;
	wire      rst = ~por[7];
	always @(posedge clk_12) if (!por[7]) por <= por + 8'd1;

	// ----------------------------------------------------------------
	// Heartbeat -- proves the 12 MHz clock is actually running
	// ----------------------------------------------------------------
	reg [23:0] hb = 24'd0;
	always @(posedge clk_12) hb <= hb + 24'd1;
	wire heartbeat = hb[22];      // ~1.4 Hz

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
	// Raw pin activity -- deliberately independent of the decoder, so a
	// wiring fault can be told apart from a decode fault.
	// ----------------------------------------------------------------
	reg [2:0] rx_ff = 3'b111;
	always @(posedge clk_12) rx_ff <= {rx_ff[1:0], can_rx};
	wire rx_edge = rx_ff[1] ^ rx_ff[2];

	// ----------------------------------------------------------------
	// Pulse stretchers
	// ----------------------------------------------------------------
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

	// ----------------------------------------------------------------
	// LED priority encoder.  Highest state wins outright so each condition
	// shows as a single colour instead of the three channels summing to
	// white.  Blinking versus solid separates the two red cases.
	// ----------------------------------------------------------------
	wire ok_recent  = (ok_ctr  != 0);
	wire bad_recent = (err_ctr != 0);
	wire act_recent = (act_ctr != 0);

	wire show_green = ok_recent;
	wire show_red   = ~ok_recent & (bad_recent | (act_recent & heartbeat));
	wire show_blue  = ~ok_recent & ~bad_recent & ~act_recent & heartbeat;

	// LEDs are active low on the UPduino.
	assign led_r     = ~show_red;
	assign led_g     = ~show_green;
	assign led_b     = ~show_blue;
	assign dbg_frame = (dbg_ctr != 0);
endmodule
