/*
 * spi_frame_stream_12m_wrap.v — SIMULATION ONLY.
 *
 * Rebuilds the partner's `top` from spi_frame_stream.v (fake counter sensor +
 * link) around the SHIPPED spi_frame_link.v, but clocked at 12 MHz (the
 * SignalBench board clock) instead of HFOSC/2 = 24 MHz, so the partner's own
 * testbench (spi_frame_stream_tb, patched for hierarchy and SCK rates by the
 * Makefile) exercises the exact link RTL that goes into the bitstream.
 *
 * The partner testbench's SB_HFOSC stand-in runs at 48 MHz; it is divided by
 * 4 here to make the 12 MHz core.  (On the board the clock is pin 44.)
 *
 * Each sensor tick is offered as a one-sample record (rec_len = 1), which
 * reproduces the original sensor exactly: `dropped` counts records, and with
 * one-sample records that equals the samples the partner testbench counts.
 * RECORD_SAMPLES must stay 1 here (the patched testbench passes it through).
 * Multi-sample, mixed-length records are covered by can_frame_link_tb.v.
 */
`default_nettype none

module top #(
	parameter integer SAMPLE_RATE_HZ    = 31_250,
	parameter integer BUF_AW            = 16,
	parameter integer MAX_FRAME_SAMPLES = 2036,
	parameter integer RECORD_SAMPLES    = 1,
	parameter [31:0]  INITIAL_OFFSET    = 32'd0
) (
	input  wire spi_sck,
	input  wire spi_cs,
	input  wire spi_mosi,
	output wire spi_miso,
	output wire spi_cs_flash,
	output wire led_r,
	output wire led_g,
	output wire led_b
);
	assign spi_cs_flash = 1'b1;

	wire clk_48;
	reg  [1:0] div4 = 2'd0;
	wire clk = div4[1];
	SB_HFOSC u_hfosc (.CLKHFPU(1'b1), .CLKHFEN(1'b1), .CLKHF(clk_48));
	always @(posedge clk_48) div4 <= div4 + 2'd1;

	localparam integer CORE_HZ  = 12_000_000;
	localparam integer TICK_DIV = (CORE_HZ / SAMPLE_RATE_HZ) * RECORD_SAMPLES;
	localparam integer TICK_W   = $clog2(TICK_DIV + 1);

	reg [TICK_W-1:0] tick_ctr = 0;
	wire tick = (tick_ctr == TICK_DIV - 1);
	always @(posedge clk)
		tick_ctr <= tick ? {TICK_W{1'b0}} : tick_ctr + 1'b1;

	// Counter sensor.  It advances by a whole record per tick whether or not
	// the record fits, so a drop shows as a jump of RECORD_SAMPLES in value.
	reg  [15:0] sensor  = INITIAL_OFFSET[15:0];
	reg  [15:0] rec_base = 16'd0;
	wire [3:0]  rec_idx;
	wire        rec_busy, rec_accept, rec_dropped;
	wire        ctrl_good, frame_sent;

	always @(posedge clk)
		if (tick) begin
			sensor <= sensor + RECORD_SAMPLES[15:0];
			if (!rec_busy)
				rec_base <= sensor;            // held until the next accepted tick
		end

	spi_frame_link #(
		.BUF_AW(BUF_AW),
		.MAX_FRAME_SAMPLES(MAX_FRAME_SAMPLES),
		.MAX_REC_SAMPLES(RECORD_SAMPLES),
		.INITIAL_OFFSET(INITIAL_OFFSET)
	) u_link (
		.clk(clk),
		.spi_sck(spi_sck), .spi_cs(spi_cs), .spi_mosi(spi_mosi), .spi_miso(spi_miso),
		.rec_req(tick), .rec_len(RECORD_SAMPLES[3:0]), .rec_idx(rec_idx), .rec_sample(rec_base + {12'd0, rec_idx}),
		.rec_busy(rec_busy), .rec_accept(rec_accept), .rec_dropped(rec_dropped), .drop_add(3'd0),
		.ctrl_good(ctrl_good), .frame_sent(frame_sent)
	);

	// LEDs as in the original (active low): red pulses on a drop, green =
	// control block valid, blue pulses on a frame with samples.
	localparam integer LED_CYCLES = CORE_HZ / 10;
	reg [21:0] drop_led_ctr = 0, frame_led_ctr = 0;
	always @(posedge clk) begin
		drop_led_ctr  <= rec_dropped ? LED_CYCLES[21:0] : drop_led_ctr  - (drop_led_ctr  != 0);
		frame_led_ctr <= frame_sent  ? LED_CYCLES[21:0] : frame_led_ctr - (frame_led_ctr != 0);
	end
	assign led_r = ~(drop_led_ctr != 0);
	assign led_g = ~ctrl_good;
	assign led_b = ~(frame_led_ctr != 0);
endmodule
