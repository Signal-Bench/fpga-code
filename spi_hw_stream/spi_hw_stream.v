/** \file
 * Hard-IP SPI slave streaming test for the iCE40 UP5K.
 *
 * Bring-up test for the FPGA's *hardened* SPI block (SB_SPI), wired as a
 * SLAVE — an external master (ESP32 / logic analyzer / MCU) supplies SCK
 * and CS.  This is the opposite role from uart_to_spi.v, which bit-bangs a
 * soft SPI master out of fabric.
 *
 * Test payload: the ASCII string "Hello World!" (MSG_LEN = 12 bytes, one
 * character per byte, no terminator), emitted one character per tick at
 * DATA_RATE_HZ and wrapping from '!' back to 'H'.  Each character is pushed
 * into a FIFO; the SPI service state machine keeps the hard IP's transmit
 * register loaded from the FIFO head, so whatever byte the master clocks out
 * is the next character of the message.  A correctly working link shows the
 * string repeating back-to-back on the master side:
 *   48 65 6c 6c 6f 20 57 6f 72 6c 64 21 48 65 ...
 *
 * Leading bytes: this design keeps SPITXDR pre-loaded at all times, which is
 * the "fully specified" case of FPGA-TN-02011 Figure 15.1 (iCE40 UltraPlus
 * as SPI Slave) — the pre-loaded byte goes straight to SO when CS asserts, so
 * the message should appear from the very first byte.  The dummy-byte
 * overhead described in Table 12.8 / Figure 15.2 applies to command-response
 * protocols, where the slave cannot know what to send until it has decoded an
 * incoming command; this design has nothing to decode.
 *
 * BUT if SPITXDR is empty when a transaction starts, Figure 15.2 documents a
 * silicon limitation: the second byte out is forced to 0xFF regardless, and
 * good data only appears in the third byte period.  That can happen here at
 * power-on (before the first DATA_RATE_HZ tick lands, ~333 us at 3 kHz) or
 * any time the FIFO runs dry.  SPICR2[SDBRE] turns that window into a
 * deterministic 0xFF.../0x00-marker/data framing instead — see the
 * CFG_SPICR2 comment.
 *
 * Producer/consumer rates: the message source produces DATA_RATE_HZ bytes/s,
 * so the master must sustain at least 8 * DATA_RATE_HZ bits/s of SPI clock to
 * keep up (24 kbit/s at the current 3 kHz), plus margin for inter-transaction
 * gaps.  Below that the FIFO fills and new characters are DROPPED
 * (drop-on-full, never overwrite), which shows up on the master side as a
 * skip forward within the message (e.g. "Hello Wo" then "rld!" missing) rather
 * than corrupted or out-of-order data.  The red LED flags that this is
 * happening.
 *
 * Debug LEDs (active low on the UPduino):
 *   RED:   pulses when a sample was dropped (FIFO full — master too slow)
 *   GREEN: on once the hard SPI IP has been configured
 *   BLUE:  pulses when a byte is handed to the IP (master is clocking)
 */
module top (
	input  wire spi_sck,       // gpio_11 — SCK in from master
	input  wire spi_cs,        // gpio_19 — CS in from master (active low)
	input  wire spi_mosi,      // gpio_21 — MOSI in from master (ignored here)
	output wire spi_miso,      // gpio_13 — MISO out to master (message stream)
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
	// Test message
	//
	// A Verilog string literal packs its first character into the most
	// significant byte, so character i lives at bits [8*(MSG_LEN-1-i) +: 8].
	// msg_idx walks 0 .. MSG_LEN-1 and wraps, one step per tick.
	// ----------------------------------------------------------------
	localparam integer          MSG_LEN = 12;
	localparam [8*MSG_LEN-1:0]  MSG     = "Hello World!";
	localparam integer          MSG_IW  = $clog2(MSG_LEN);

	reg  [MSG_IW-1:0] msg_idx  = 0;
	wire [7:0]        msg_byte = MSG[8*(MSG_LEN-1-msg_idx) +: 8];

	// ----------------------------------------------------------------
	// Test-pattern FIFO
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

	reg  [7:0]         fifo_mem [0:FIFO_DEPTH-1];
	reg  [FIFO_AW-1:0] fifo_wr    = 0;
	reg  [FIFO_AW-1:0] fifo_rd    = 0;
	reg  [FIFO_AW:0]   fifo_count = 0;

	wire       fifo_full  = (fifo_count == FIFO_DEPTH);
	wire       fifo_empty = (fifo_count == 0);
	wire [7:0] fifo_head  = fifo_mem[fifo_rd];

	wire fifo_push = tick & ~fifo_full;
	wire sample_dropped = tick & fifo_full;
	reg  fifo_pop;   // single-cycle pulse, driven by the SPI service FSM

	always @(posedge clk_core) begin
		if (tick)
			msg_idx <= (msg_idx == MSG_LEN - 1) ? {MSG_IW{1'b0}} : msg_idx + 1'b1;

		if (fifo_push) begin
			fifo_mem[fifo_wr] <= msg_byte;
			fifo_wr           <= fifo_wr + 1'b1;
		end

		if (fifo_pop)
			fifo_rd <= fifo_rd + 1'b1;

		// push and pop can land in the same cycle
		case ({fifo_push, fifo_pop})
			2'b10:   fifo_count <= fifo_count + 1'b1;
			2'b01:   fifo_count <= fifo_count - 1'b1;
			default: fifo_count <= fifo_count;
		endcase
	end

	// ----------------------------------------------------------------
	// SB_SPI register map — verified against Lattice FPGA-TN-02011-1.8
	// ("Advanced iCE40 I2C and SPI Hardened IP User Guide"), section 12,
	// Tables 12.1-12.9.  A local copy is in ../ice_docs/.
	//
	// The upper address nibble is the BUS_ADDR74 parameter, left at its
	// "0b0000" default.
	//
	// NOTE: a write to SPICR0/SPICR1/SPICR2/SPIBR/SPICSR resets the SPI
	// core (Tables 12.2-12.6), so all five are written before the IP is
	// used and never touched again.
	// ----------------------------------------------------------------
	localparam [7:0] ADDR_SPICR0  = 8'h08;  // lead/trail/idle delays (master mode only)
	localparam [7:0] ADDR_SPICR1  = 8'h09;  // bit7 SPE (enable), bit4 TXEDGE
	localparam [7:0] ADDR_SPICR2  = 8'h0A;  // bit7 MSTR, bit5 SDBRE, bit2 CPOL, bit1 CPHA, bit0 LSBF
	localparam [7:0] ADDR_SPIBR   = 8'h0B;  // clock prescale, DIVIDER[5:0]
	localparam [7:0] ADDR_SPISR   = 8'h0C;  // status
	localparam [7:0] ADDR_SPITXDR = 8'h0D;  // transmit data
	localparam [7:0] ADDR_SPIRXDR = 8'h0E;  // receive data
	localparam [7:0] ADDR_SPICSR  = 8'h0F;  // master chip-select

	// SPISR, Table 12.7.  RRDY is cleared by reading SPIRXDR; leaving it
	// set until the next byte arrives sets ROE (bit1, receive overrun),
	// which is why S_DRAIN_RX exists.
	localparam integer SR_RRDY = 3;   // SPIRXDR holds valid receive data
	localparam integer SR_TRDY = 4;   // SPITXDR is empty, ready for a byte

	// SPICR1 (Table 12.3): SPE=1 enables the core.  TXEDGE=0 keeps the
	// standard "receive on rising / transmit on falling" behaviour; it
	// only needs setting for fast SPI, and must stay clear while CPHA is 0.
	localparam [7:0] CFG_SPICR1 = 8'b1000_0000;

	// SPICR2 (Table 12.4): MSTR=0 is what actually selects SLAVE mode —
	// it lives here, not in SPICR1.  CPOL=0 + CPHA=0 = SPI mode 0, and
	// LSBF=0 = MSB-first, matching "MSB SPI" in notes.md and the rest of
	// this repo.  Set bit0 for LSB-first.
	//
	// Bit5 (SDBRE) is worth knowing about: it turns on Lattice's dummy
	// byte response, where the slave sends 0xFF until the first SPITXDR
	// write, then a single 0x00 marker, then real data — giving the master
	// a deterministic "data starts here" byte to sync on.  Left off here to
	// keep the raw stream simple; set 8'b0010_0000 to try it.
	localparam [7:0] CFG_SPICR2 = 8'b0000_0000;

	// SPIBR (Table 12.5): DIVIDER must be >= 1 per the datasheet.  Only
	// meaningful in master mode (the master supplies SCK here), but the
	// constraint is stated unconditionally, so use 1 rather than 0.
	localparam [7:0] CFG_SPIBR = 8'b0000_0001;

	reg        spi_stb  = 0;
	reg        spi_rw   = 0;
	reg  [7:0] spi_adr  = 0;
	reg  [7:0] spi_dati = 0;
	wire [7:0] spi_dato;
	wire       spi_ack;
	wire       so_data;

	SB_SPI u_spi (
		.SBCLKI(clk_core), .SBSTBI(spi_stb), .SBRWI(spi_rw),
		.SBADRI0(spi_adr[0]), .SBADRI1(spi_adr[1]), .SBADRI2(spi_adr[2]), .SBADRI3(spi_adr[3]),
		.SBADRI4(spi_adr[4]), .SBADRI5(spi_adr[5]), .SBADRI6(spi_adr[6]), .SBADRI7(spi_adr[7]),
		.SBDATI0(spi_dati[0]), .SBDATI1(spi_dati[1]), .SBDATI2(spi_dati[2]), .SBDATI3(spi_dati[3]),
		.SBDATI4(spi_dati[4]), .SBDATI5(spi_dati[5]), .SBDATI6(spi_dati[6]), .SBDATI7(spi_dati[7]),
		.SBDATO0(spi_dato[0]), .SBDATO1(spi_dato[1]), .SBDATO2(spi_dato[2]), .SBDATO3(spi_dato[3]),
		.SBDATO4(spi_dato[4]), .SBDATO5(spi_dato[5]), .SBDATO6(spi_dato[6]), .SBDATO7(spi_dato[7]),
		.SBACKO(spi_ack),
		.MI(1'b0),           // master-mode input, unused as a slave
		.SI(spi_mosi),
		.SCKI(spi_sck),
		.SCSNI(spi_cs),
		.SO(so_data)
	);

	// MISO is driven unconditionally, which is correct for a point-to-point
	// link with a single slave (what this test targets).  On a shared bus,
	// gate it with the IP's SOE output instead:
	//   wire so_oe;  ... .SOE(so_oe) ...
	//   SB_IO #(.PIN_TYPE(6'b1010_01)) u_miso (
	//       .PACKAGE_PIN(spi_miso), .OUTPUT_ENABLE(so_oe), .D_OUT_0(so_data));
	// and declare spi_miso as inout.
	assign spi_miso = so_data;

	// ----------------------------------------------------------------
	// Service state machine: configure the IP, then keep its transmit
	// register fed from the FIFO and drain its receive register so the
	// overrun flag never latches up.
	//
	// Refill deadline (FPGA-TN-02011 Table 12.8): as a slave, SPITXDR must
	// be written before the SCK edge that samples the last bit of the
	// PREVIOUS byte — so roughly 7 bit-times after TRDY asserts.  Worst
	// case here is a poll + an RX drain + the load, ~9 core cycles = ~375 ns
	// at 24 MHz, which fits comfortably below ~10 MHz SCK.  Above that the
	// margin gets thin and TXEDGE (SPICR1 bit4) starts to matter.
	// ----------------------------------------------------------------
	localparam [3:0] S_CR0     = 4'd0,
	                 S_CR1     = 4'd1,
	                 S_CR2     = 4'd2,
	                 S_BR      = 4'd3,
	                 S_CSR     = 4'd4,
	                 S_POLL    = 4'd5,
	                 S_LOAD_TX = 4'd6,
	                 S_DRAIN_RX = 4'd7;

	reg [3:0] state = S_CR0;
	wire      spi_ready = (state >= S_POLL);

	always @(posedge clk_core) begin
		spi_stb  <= 1'b0;
		fifo_pop <= 1'b0;

		case (state)
			// -- configuration writes --
			S_CR0: begin
				spi_adr <= ADDR_SPICR0; spi_dati <= 8'h00;
				spi_stb <= 1'b1; spi_rw <= 1'b1;
				if (spi_ack) begin spi_stb <= 1'b0; state <= S_CR1; end
			end
			S_CR1: begin
				spi_adr <= ADDR_SPICR1; spi_dati <= CFG_SPICR1;
				spi_stb <= 1'b1; spi_rw <= 1'b1;
				if (spi_ack) begin spi_stb <= 1'b0; state <= S_CR2; end
			end
			S_CR2: begin
				spi_adr <= ADDR_SPICR2; spi_dati <= CFG_SPICR2;
				spi_stb <= 1'b1; spi_rw <= 1'b1;
				if (spi_ack) begin spi_stb <= 1'b0; state <= S_BR; end
			end
			S_BR: begin
				spi_adr <= ADDR_SPIBR; spi_dati <= CFG_SPIBR;
				spi_stb <= 1'b1; spi_rw <= 1'b1;
				if (spi_ack) begin spi_stb <= 1'b0; state <= S_CSR; end
			end
			S_CSR: begin
				spi_adr <= ADDR_SPICSR; spi_dati <= 8'h00;
				spi_stb <= 1'b1; spi_rw <= 1'b1;
				if (spi_ack) begin spi_stb <= 1'b0; state <= S_POLL; end
			end

			// -- steady state --
			S_POLL: begin
				spi_adr <= ADDR_SPISR;
				spi_stb <= 1'b1; spi_rw <= 1'b0;
				if (spi_ack) begin
					spi_stb <= 1'b0;
					if (spi_dato[SR_TRDY] && !fifo_empty)
						state <= S_LOAD_TX;
					else if (spi_dato[SR_RRDY])
						state <= S_DRAIN_RX;
					else
						state <= S_POLL;
				end
			end

			S_LOAD_TX: begin
				spi_adr <= ADDR_SPITXDR; spi_dati <= fifo_head;
				spi_stb <= 1'b1; spi_rw <= 1'b1;
				if (spi_ack) begin
					spi_stb  <= 1'b0;
					fifo_pop <= 1'b1;   // byte now belongs to the IP
					state    <= S_POLL;
				end
			end

			S_DRAIN_RX: begin
				spi_adr <= ADDR_SPIRXDR;
				spi_stb <= 1'b1; spi_rw <= 1'b0;
				if (spi_ack) begin spi_stb <= 1'b0; state <= S_POLL; end
			end

			default: state <= S_CR0;
		endcase
	end

	// ----------------------------------------------------------------
	// Debug LED pulse stretchers
	// ----------------------------------------------------------------
	reg [21:0] drop_led_ctr = 0;
	reg [21:0] tx_led_ctr   = 0;

	always @(posedge clk_core) begin
		if (sample_dropped)
			drop_led_ctr <= LED_CYCLES[21:0];
		else
			drop_led_ctr <= drop_led_ctr - (drop_led_ctr != 0);

		if (fifo_pop)
			tx_led_ctr <= LED_CYCLES[21:0];
		else
			tx_led_ctr <= tx_led_ctr - (tx_led_ctr != 0);
	end

	assign led_r = ~(drop_led_ctr != 0);
	assign led_g = ~spi_ready;
	assign led_b = ~(tx_led_ctr != 0);

endmodule
