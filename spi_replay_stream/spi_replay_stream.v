/*
 * spi_replay_stream.v — recorded-data replay over the hardened SPI slave
 *
 * Companion to spi_hw_stream/ and spi_counter_stream/.  Those two model a
 * LIVE bus: a synthetic source runs at its own fixed DATA_RATE_HZ and DROPS
 * bytes when the master cannot keep up, which is how you measure the
 * sustained rate the master actually achieves.
 *
 * This design models the other half of SignalBench: draining a RECORDING.
 * The payload is already sitting in memory before the first SCK edge, so
 * there is no producer and no rate to outrun.  The replay pointer advances
 * only when the hard IP has accepted a byte, which makes the master the sole
 * flow-control authority:
 *
 *   - No tick divider, no DATA_RATE_HZ, no FIFO, no drop path.  Nothing in
 *     this design can lose a byte; the pointer physically cannot advance
 *     past a byte the IP has not taken.
 *   - Read it at 100 kHz or 8 MHz and you get the same byte sequence, just
 *     slower or faster.  Any gap or repeat you observe on the analyzer is a
 *     transport or hard-IP fault, with the generator fully ruled out.
 *
 * That makes this the stricter link-integrity test of the three, and the
 * closer model of the recording-drain mode in state_management.md.
 *
 * Buffer: REPLAY_DEPTH bytes in inferred EBR (4096 = 8 of the UP5K's 30
 * 4-kbit blocks), preloaded at configuration time from the bitstream's INIT
 * values — no runtime load step.  Contents default to a synthetic 16-byte
 * record pattern (see REPLAY PATTERN below); define REPLAY_HEX_FILE to
 * replay a real capture instead.
 *
 * EBR read latency: the block RAM read is SYNCHRONOUS, so the byte for a new
 * address is not valid until the cycle after the address changes.  src_valid
 * drops for exactly that one cycle.  This is the same settle-state hazard
 * that bit can_sniffer.v when its drain was raised to 6 MHz (see the
 * S_SETTLE comment there) — a combinational-read FIFO hides it, an EBR does
 * not.  One dead core cycle is 41.7 ns at 24 MHz against a 800 ns byte
 * period at 10 MHz SCK, so it costs nothing.
 *
 * REPLAY PATTERN (default contents, 16-byte records):
 *   byte 0     0xA5                 sync
 *   byte 1     record number        0..(REPLAY_DEPTH/16 - 1)
 *   bytes 2-14 record + position    varies within the record
 *   byte 15    0x5A                 end
 * Framing is self-evident on an analyzer: every 16th byte must be 0xA5, and
 * record numbers must ascend by exactly 1 with no repeats.
 *
 * Leading bytes: SPITXDR is kept pre-loaded at all times, the "fully
 * specified" case of FPGA-TN-02011 Figure 15.1 (iCE40 UltraPlus as SPI
 * Slave), so byte 0 of the buffer goes straight to SO when CS asserts.
 *
 * Debug LEDs (active low on the UPduino):
 *   RED:   pulses when the replay wraps (REPLAY_LOOP=1), or on steady once
 *          the buffer is exhausted (REPLAY_LOOP=0)
 *   GREEN: on once the hard SPI IP has been configured
 *   BLUE:  pulses when a byte is handed to the IP (master is clocking)
 */

module top (
	input  wire spi_sck,       // gpio_11 — SCK in from master
	input  wire spi_cs,        // gpio_19 — CS in from master (active low)
	input  wire spi_mosi,      // gpio_21 — MOSI in from master (ignored here)
	output wire spi_miso,      // gpio_13 — MISO out to master (replay stream)
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

	localparam integer CORE_HZ    = 24_000_000;
	localparam integer LED_CYCLES = CORE_HZ / 10;             // ~100 ms

	// ----------------------------------------------------------------
	// Replay buffer
	//
	// REPLAY_LOOP = 1 wraps forever, which is what you want for a soak or
	// rate test.  REPLAY_LOOP = 0 plays the buffer once and then goes
	// quiet (SPITXDR runs dry, so the master reads 0xFF), which is what the
	// real recording drain does — use it to check end-of-recording
	// behaviour on the master side.
	// ----------------------------------------------------------------
	localparam integer REPLAY_AW    = 12;
	localparam integer REPLAY_DEPTH = 1 << REPLAY_AW;          // 4096 bytes
	localparam         REPLAY_LOOP  = 1'b1;

	localparam [7:0] SYNC_BYTE = 8'hA5;
	localparam [7:0] END_BYTE  = 8'h5A;

	reg [7:0] replay_mem [0:REPLAY_DEPTH-1];

	integer init_i;
	initial begin
`ifdef REPLAY_HEX_FILE
		// A short file leaves the tail at 'x in simulation and 0 in the
		// bitstream; size the file to REPLAY_DEPTH to avoid both.
		$readmemh(`REPLAY_HEX_FILE, replay_mem);
`else
		for (init_i = 0; init_i < REPLAY_DEPTH; init_i = init_i + 1) begin
			if (init_i[3:0] == 4'h0)
				replay_mem[init_i] = SYNC_BYTE;
			else if (init_i[3:0] == 4'h1)
				replay_mem[init_i] = init_i[REPLAY_AW-1:4];
			else if (init_i[3:0] == 4'hF)
				replay_mem[init_i] = END_BYTE;
			else
				replay_mem[init_i] = init_i[REPLAY_AW-1:4] + {4'd0, init_i[3:0]};
		end
`endif
	end

	// ----------------------------------------------------------------
	// Replay pointer and one-byte prefetch
	//
	// src_advance is the only thing that moves the pointer, and it is
	// pulsed by the service FSM exactly when the IP has taken a byte.
	// That is what makes the stream lossless regardless of master rate.
	// ----------------------------------------------------------------
	reg [REPLAY_AW-1:0] rd_addr     = 0;
	reg [7:0]           src_byte    = 0;
	reg                 src_valid   = 1'b0;
	reg                 replay_done = 1'b0;
	reg                 replay_wrap = 1'b0;
	reg                 src_advance;   // single-cycle pulse from the FSM

	wire at_last = (rd_addr == REPLAY_DEPTH - 1);

	always @(posedge clk_core) begin
		// Unconditional synchronous read.  On an advance cycle this latches
		// the byte for the OLD address, which is why src_valid drops below.
		src_byte    <= replay_mem[rd_addr];
		replay_wrap <= 1'b0;

		if (src_advance) begin
			src_valid <= 1'b0;             // new address not settled yet
			if (at_last) begin
				if (REPLAY_LOOP) begin
					rd_addr     <= 0;
					replay_wrap <= 1'b1;
				end else begin
					replay_done <= 1'b1;   // hold the pointer, stop offering bytes
				end
			end else begin
				rd_addr <= rd_addr + 1'b1;
			end
		end else if (!replay_done) begin
			src_valid <= 1'b1;
		end
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
	// which is why S_DRAIN_RX exists even though MOSI carries nothing.
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
	// Bit5 (SDBRE) turns on Lattice's dummy byte response.  Left off here
	// so the first byte off the link is buffer offset 0; set 8'b0010_0000
	// if you want the deterministic 0xFF.../0x00-marker framing instead.
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
	// gate it with the IP's SOE output instead — see the same comment in
	// ../spi_hw_stream/spi_hw_stream.v for the SB_IO version.
	assign spi_miso = so_data;

	// ----------------------------------------------------------------
	// Service state machine: configure the IP, then keep its transmit
	// register fed from the replay buffer and drain its receive register so
	// the overrun flag never latches up.
	//
	// RRDY is checked BEFORE TRDY: prioritizing TRDY can leave SPIRXDR
	// occupied until the following byte arrives, setting ROE on real
	// silicon.  MOSI is ignored here, but the drain still has to happen.
	//
	// Refill deadline (FPGA-TN-02011 Table 12.8): as a slave, SPITXDR must
	// be written before the SCK edge that samples the last bit of the
	// PREVIOUS byte — so roughly 7 bit-times after TRDY asserts.  Worst
	// case here is a poll + an RX drain + the EBR settle cycle + the load,
	// ~10 core cycles = ~417 ns at 24 MHz, which fits below ~10 MHz SCK.
	// Above that the margin gets thin and TXEDGE (SPICR1 bit4) starts to
	// matter.
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
		spi_stb     <= 1'b0;
		src_advance <= 1'b0;

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
					if (spi_dato[SR_RRDY])
						state <= S_DRAIN_RX;
					else if (spi_dato[SR_TRDY] && src_valid)
						state <= S_LOAD_TX;
					else
						state <= S_POLL;
				end
			end

			S_LOAD_TX: begin
				spi_adr <= ADDR_SPITXDR; spi_dati <= src_byte;
				spi_stb <= 1'b1; spi_rw <= 1'b1;
				if (spi_ack) begin
					spi_stb     <= 1'b0;
					src_advance <= 1'b1;   // byte now belongs to the IP
					state       <= S_POLL;
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
	reg [21:0] wrap_led_ctr = 0;
	reg [21:0] tx_led_ctr   = 0;

	always @(posedge clk_core) begin
		if (replay_wrap)
			wrap_led_ctr <= LED_CYCLES[21:0];
		else
			wrap_led_ctr <= wrap_led_ctr - (wrap_led_ctr != 0);

		if (src_advance)
			tx_led_ctr <= LED_CYCLES[21:0];
		else
			tx_led_ctr <= tx_led_ctr - (tx_led_ctr != 0);
	end

	assign led_r = ~(replay_done || (wrap_led_ctr != 0));
	assign led_g = ~spi_ready;
	assign led_b = ~(tx_led_ctr != 0);

endmodule
