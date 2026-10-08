// Raw SPI slave stream for the iCE40 UP5K, mode 0 and MSB first.
// Ascending ASCII from space through '~', then LF.
// A 3 kB/s producer pauses when its 64-byte FIFO is full.
// The shared fabric transmitter retires data only after eight sampling edges.
// Empty bytes are FF; MOSI is ignored. All TLV framing belongs to the MCU.
// LEDs (active low): red = source stalled, green = running, blue = byte sent.
module top (
	input  wire spi_sck,       // gpio_11 — SCK in from master
	input  wire spi_cs,        // gpio_19 — CS in from master (active low)
	input  wire spi_mosi,      // gpio_21 - ignored by the raw stream
	output wire spi_miso,      // gpio_13 - raw FIFO bytes out to master
	output wire spi_cs_flash,  // pin 16  — hold the onboard flash deselected
	output wire led_r,
	output wire led_g,
	output wire led_b
);
	assign spi_cs_flash = 1'b1;

	// ----------------------------------------------------------------
	// 48 MHz oscillator -> 24 MHz core clock
	// Same divide-by-2 as i2c_sniffer.v, for timing margin.
	// ----------------------------------------------------------------
	wire clk_48;
	reg  clk_core_r = 0;
	wire clk_core   = clk_core_r;

	SB_HFOSC u_hfosc (.CLKHFPU(1'b1), .CLKHFEN(1'b1), .CLKHF(clk_48));
	always @(posedge clk_48) clk_core_r <= ~clk_core_r;

	localparam integer CORE_HZ      = 24_000_000;
	localparam integer DATA_RATE_HZ = 3_000;
	localparam integer TICK_DIV     = CORE_HZ / DATA_RATE_HZ;
	localparam integer LED_CYCLES   = CORE_HZ / 10;            // ~100 ms

	// ----------------------------------------------------------------
	// Sample tick at DATA_RATE_HZ.
	//
	// The counter width MUST be derived from TICK_DIV, not hardcoded.  A
	// fixed 6-bit counter works only while TICK_DIV <= 64 (DATA_RATE_HZ >=
	// 375 kHz); below that the compare value is unreachable, so the tick
	// never fires at all.  Symptom: FIFO stays empty forever, green LED
	// still on (the IP configured fine), red LED off (nothing to drop),
	// blue LED off, and the master reads nothing but 0xFF.
	// ----------------------------------------------------------------
	localparam integer TICK_W = (TICK_DIV <= 2) ? 1 : $clog2(TICK_DIV);

	reg [TICK_W-1:0] tick_ctr = 0;
	wire             tick = (tick_ctr == TICK_DIV - 1);
	always @(posedge clk_core)
		tick_ctr <= tick ? {TICK_W{1'b0}} : tick_ctr + 1'b1;

	// ----------------------------------------------------------------
	// Ascending ASCII source
	//
	// Register-based with a combinational read of the head, deliberately
	// NOT an inferred EBR: the read-during-write hazard of a synchronous
	// RAM read would need an extra guard, and keeping the EBRs free
	// matters for the real capture design.  64 entries buys FIFO_DEPTH /
	// DATA_RATE_HZ of buffering (21 ms at 3 kHz); bump FIFO_AW if you
	// need to absorb longer gaps between master reads.
	// ----------------------------------------------------------------
	localparam integer FIFO_AW    = 6;
	localparam integer FIFO_DEPTH = 1 << FIFO_AW;   // 64

	reg [7:0] ascii_char = 8'h20;

	reg  [7:0]         fifo_mem [0:FIFO_DEPTH-1];
	reg  [FIFO_AW-1:0] fifo_wr    = 0;
	reg  [FIFO_AW-1:0] fifo_rd    = 0;
	reg  [FIFO_AW:0]   fifo_count = 0;

	wire       fifo_full  = (fifo_count == FIFO_DEPTH);
	wire       fifo_empty = (fifo_count == 0);
	wire [7:0] fifo_head  = fifo_mem[fifo_rd];

	wire fifo_push = tick & ~fifo_full;
	wire source_stalled = tick & fifo_full;
	wire fifo_pop;

	always @(posedge clk_core) begin
		if (fifo_push) begin
			fifo_mem[fifo_wr] <= ascii_char;
			fifo_wr <= fifo_wr + 1'b1;
			if (ascii_char == 8'h7E)
				ascii_char <= 8'h0A;
			else if (ascii_char == 8'h0A)
				ascii_char <= 8'h20;
			else
				ascii_char <= ascii_char + 1'b1;
		end
		if (fifo_pop)
			fifo_rd <= fifo_rd + 1'b1;
		case ({fifo_push, fifo_pop})
			2'b10:   fifo_count <= fifo_count + 1'b1;
			2'b01:   fifo_count <= fifo_count - 1'b1;
			default: fifo_count <= fifo_count;
		endcase
	end

	spi_raw_tx u_tx (
		.clk(clk_core), .spi_sck(spi_sck), .spi_cs(spi_cs),
		.data(fifo_head), .empty(fifo_empty), .pop(fifo_pop),
		.spi_miso(spi_miso)
	);

	// ----------------------------------------------------------------
	// Debug LED pulse stretchers
	// ----------------------------------------------------------------
	reg [21:0] drop_led_ctr = 0;
	reg [21:0] tx_led_ctr   = 0;

	always @(posedge clk_core) begin
		if (source_stalled)
			drop_led_ctr <= LED_CYCLES[21:0];
		else
			drop_led_ctr <= drop_led_ctr - (drop_led_ctr != 0);

		if (fifo_pop)
			tx_led_ctr <= LED_CYCLES[21:0];
		else
			tx_led_ctr <= tx_led_ctr - (tx_led_ctr != 0);
	end

	assign led_r = ~(drop_led_ctr != 0);
	assign led_g = 1'b0;
	assign led_b = ~(tx_led_ctr != 0);

endmodule
