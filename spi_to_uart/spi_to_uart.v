/** \file
 * SPI slave -> UART hex printer.
 *
 * Every byte the external master clocks in on MOSI is printed to the host
 * terminal as two hex characters, so `make pico` shows the SPI traffic live.
 *
 * This is two existing pieces of the repo glued together:
 *   * the hardened SPI slave (SB_SPI) and its configuration state machine
 *     from spi_hw_stream.v — the FPGA is the SLAVE here, the master supplies
 *     SCK and CS;
 *   * the UART transmit path from uart_to_spi.v / host_to_fpga.v — the same
 *     `uart_tx` out of ../common/uart.v at 1 Mbaud on pin 14, which is the
 *     baud `make pico` expects.
 *
 * The counter/FIFO test pattern of spi_hw_stream.v is gone: nothing is ever
 * written to SPITXDR, so this design only ever receives.  MISO is still
 * wired out (same pin plan, same jumper harness) but carries nothing
 * meaningful — see the README.
 *
 * Output format: lowercase hex, space separated, one line per SPI
 * transaction (CS deasserted) or per BYTES_PER_LINE bytes, whichever comes
 * first.  A three-byte transaction from uart_to_spi.v prints as
 *
 *   48 01 41
 *
 * Sustained throughput is limited by the UART, not the SPI: three UART
 * characters per SPI byte at 1 Mbaud is ~33 kB/s.  Above that the character
 * queue fills and whole bytes are dropped (never half a hex pair), which the
 * red LED flags.
 *
 * Debug LEDs (active low on the UPduino):
 *   RED:   pulses when a byte was dropped (character queue full)
 *   GREEN: on once the hard SPI IP has been configured
 *   BLUE:  pulses when a byte is received from the master
 */
`include "../common/util.v"
`include "../common/uart.v"

module top (
	input  wire spi_sck,       // gpio_11 — SCK in from master
	input  wire spi_cs,        // gpio_19 — CS in from master (active low)
	input  wire spi_mosi,      // gpio_21 — MOSI in from master
	output wire spi_miso,      // gpio_13 — MISO out (not meaningful here)
	output wire serial_txd,    // pin 14  — UART to the host (picocom)
	output wire spi_cs_flash,  // pin 16  — hold the onboard flash deselected
	output wire led_r,
	output wire led_g,
	output wire led_b
);
	// Pin 14 reaches the FT232H's ADBUS1, which is its receive pin in async
	// serial mode, so this is the direction that shows up in picocom.  That
	// net is shared with the flash, so pin 16 must hold the flash
	// deselected — the same reason every other design here drives it high.
	assign spi_cs_flash = 1'b1;

	// ----------------------------------------------------------------
	// 48 MHz oscillator -> 24 MHz core clock
	// Same divide-by-2 as spi_hw_stream.v / i2c_sniffer.v.
	// ----------------------------------------------------------------
	wire clk_48;
	reg  clk_core_r = 0;
	wire clk_core   = clk_core_r;

	SB_HFOSC u_hfosc (.CLKHFPU(1'b1), .CLKHFEN(1'b1), .CLKHF(clk_48));
	always @(posedge clk_48) clk_core_r <= ~clk_core_r;

	localparam integer CORE_HZ       = 24_000_000;
	localparam integer BAUD          = 1_000_000;   // matches `make pico`
	localparam integer LED_CYCLES    = CORE_HZ / 10;          // ~100 ms
	localparam integer BYTES_PER_LINE = 16;

	// ----------------------------------------------------------------
	// Power-on reset.
	//
	// uart_tx's shift register is only cleared by `reset`, so the other
	// designs here that tie it to 0 rely on the iCE40 bringing registers up
	// at 0.  That is true in silicon but not in simulation, where the
	// shifter would stay X forever.  A few cycles of real reset costs one
	// counter and makes the testbench meaningful.
	// ----------------------------------------------------------------
	reg [3:0] por_ctr = 0;
	wire      reset = ~por_ctr[3];
	always @(posedge clk_core)
		if (!por_ctr[3]) por_ctr <= por_ctr + 1'b1;

	// ----------------------------------------------------------------
	// UART transmitter, 1 Mbaud, 8-N-1
	// ----------------------------------------------------------------
	wire baud_x1;
	divide_by_n #(.N(CORE_HZ / BAUD)) div_tx(clk_core, reset, baud_x1);

	reg  [7:0] uart_txd        = 0;
	reg        uart_txd_strobe = 0;
	wire       uart_txd_ready;

	uart_tx txd (
		.mclk(clk_core),
		.reset(reset),
		.baud_x1(baud_x1),
		.serial(serial_txd),
		.ready(uart_txd_ready),
		.data(uart_txd),
		.data_strobe(uart_txd_strobe)
	);

	// ----------------------------------------------------------------
	// Character queue
	//
	// Absorbs SPI bursts that are faster than the UART can print: 64
	// characters is 21 hex-formatted bytes, ~640 us of UART time at 1 Mbaud.
	//
	// Written as a register array with a combinational read of the head,
	// like spi_hw_stream.v's FIFO, but it does NOT end up in LUTs: the only
	// consumer of `cq_head` registers it (`uart_txd <= cq_head`), so yosys
	// merges that register into the read port and maps the queue to one EBR
	// (284 LCs total, against ~1180 for spi_hw_stream).  That merge is only
	// equivalent because a push and a pop can never hit the same address in
	// the same cycle — a pop is issued only while the queue is non-empty,
	// i.e. while cq_rd != cq_wr — so the EBR's undefined same-address
	// read-during-write case is unreachable.  The testbench checks this.
	// Add a second reader of `cq_head` and the merge no longer applies.
	// ----------------------------------------------------------------
	localparam integer CQ_AW    = 6;
	localparam integer CQ_DEPTH = 1 << CQ_AW;   // 64

	reg  [7:0]       cq_mem [0:CQ_DEPTH-1];
	reg  [CQ_AW-1:0] cq_wr    = 0;
	reg  [CQ_AW-1:0] cq_rd    = 0;
	reg  [CQ_AW:0]   cq_count = 0;

	wire       cq_empty = (cq_count == 0);
	wire [7:0] cq_head  = cq_mem[cq_rd];

	// A whole byte is either printed or dropped, never split across a hex
	// pair, so the format sequence only starts when there is room for its
	// three characters plus a possible CR/LF.
	wire cq_room = (cq_count <= CQ_DEPTH - 5);

	reg       cq_push      = 0;   // driven by the SPI service FSM
	reg [7:0] cq_push_data = 0;
	reg       cq_pop       = 0;   // driven by the UART drain below

	always @(posedge clk_core) begin
		if (reset) begin
			cq_wr    <= 0;
			cq_rd    <= 0;
			cq_count <= 0;
		end else begin
			if (cq_push) begin
				cq_mem[cq_wr] <= cq_push_data;
				cq_wr         <= cq_wr + 1'b1;
			end

			if (cq_pop)
				cq_rd <= cq_rd + 1'b1;

			// push and pop can land in the same cycle
			case ({cq_push, cq_pop})
				2'b10:   cq_count <= cq_count + 1'b1;
				2'b01:   cq_count <= cq_count - 1'b1;
				default: cq_count <= cq_count;
			endcase
		end
	end

	// Drain the queue into the UART.  Same handshake as uart_tx_fifo in
	// ../common/uart.v: hand over one character per `ready`, and hold off
	// while a strobe or a pop from the previous cycle is still settling.
	always @(posedge clk_core) begin
		uart_txd_strobe <= 1'b0;
		cq_pop          <= 1'b0;

		if (!cq_empty && uart_txd_ready && !uart_txd_strobe && !cq_pop) begin
			uart_txd        <= cq_head;
			uart_txd_strobe <= 1'b1;
			cq_pop          <= 1'b1;
		end
	end

	// ----------------------------------------------------------------
	// CS synchronizer
	//
	// CS is asynchronous to clk_core, so it gets the two-stage synchronizer
	// every async input in this repo gets.  Only the level is needed: the
	// end-of-line is emitted once the bus is idle AND the receive register
	// has been drained, which sidesteps any race between CS rising and the
	// last byte arriving.
	// ----------------------------------------------------------------
	reg [1:0] cs_sync = 2'b11;
	always @(posedge clk_core) cs_sync <= {cs_sync[0], spi_cs};
	wire cs_idle = cs_sync[1];

	// ----------------------------------------------------------------
	// SB_SPI register map — verified against Lattice FPGA-TN-02011-1.8
	// ("Advanced iCE40 I2C and SPI Hardened IP User Guide"), section 12,
	// Tables 12.1-12.9.  A local copy is in ../docs/.
	//
	// Identical configuration to spi_hw_stream.v: slave, SPI mode 0,
	// MSB-first.  A write to SPICR0/SPICR1/SPICR2/SPIBR/SPICSR resets the
	// SPI core, so all five are written up front and never touched again.
	// ----------------------------------------------------------------
	localparam [7:0] ADDR_SPICR0  = 8'h08;
	localparam [7:0] ADDR_SPICR1  = 8'h09;  // bit7 SPE (enable)
	localparam [7:0] ADDR_SPICR2  = 8'h0A;  // bit7 MSTR, bit2 CPOL, bit1 CPHA, bit0 LSBF
	localparam [7:0] ADDR_SPIBR   = 8'h0B;
	localparam [7:0] ADDR_SPISR   = 8'h0C;
	localparam [7:0] ADDR_SPIRXDR = 8'h0E;
	localparam [7:0] ADDR_SPICSR  = 8'h0F;

	// SPISR, Table 12.7.  RRDY is cleared by reading SPIRXDR; leaving it set
	// until the next byte arrives sets ROE (receive overrun), so the read
	// always takes priority over printing.
	localparam integer SR_RRDY = 3;

	localparam [7:0] CFG_SPICR1 = 8'b1000_0000;  // SPE=1
	localparam [7:0] CFG_SPICR2 = 8'b0000_0000;  // MSTR=0 -> slave, mode 0, MSB-first
	localparam [7:0] CFG_SPIBR  = 8'b0000_0001;  // DIVIDER >= 1 per Table 12.5

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

	// SPITXDR is never written, so SO carries no useful data — see the
	// README.  It is still driven out for pin-plan compatibility with
	// spi_hw_stream.v, and because a point-to-point link needs no output
	// enable.
	assign spi_miso = so_data;

	// ----------------------------------------------------------------
	// Service state machine: configure the IP, then drain each received
	// byte and format it into the character queue.
	//
	// Formatting lives in this FSM rather than a separate one so there is no
	// handshake to get wrong: the read and its three characters are
	// consecutive states, and SPIRXDR is drained before the next byte can
	// complete.  Worst case is a poll + a read + five pushes, ~11 cycles =
	// ~460 ns at 24 MHz, against 800 ns per byte at 10 MHz SCK.  Above that
	// the refill/drain deadline in Table 12.8 gets thin; in practice the
	// UART is the binding limit long before SCK is.
	// ----------------------------------------------------------------
	localparam [3:0] S_CR0      = 4'd0,
	                 S_CR1      = 4'd1,
	                 S_CR2      = 4'd2,
	                 S_BR       = 4'd3,
	                 S_CSR      = 4'd4,
	                 S_POLL     = 4'd5,
	                 S_READ_RX  = 4'd6,
	                 S_PUSH_HI  = 4'd7,
	                 S_PUSH_LO  = 4'd8,
	                 S_PUSH_SEP = 4'd9,
	                 S_PUSH_CR  = 4'd10,
	                 S_PUSH_LF  = 4'd11;

	reg [3:0] state = S_CR0;
	wire      spi_ready = (state >= S_POLL);

	reg [7:0] rx_byte  = 0;
	reg       line_open = 0;                 // characters printed since the last newline
	reg [CQ_AW-1:0] line_len = 0;            // bytes on the current line

	wire rx_accepted = (state == S_READ_RX) && spi_ack;
	reg  byte_dropped = 0;

	always @(posedge clk_core) begin
		spi_stb      <= 1'b0;
		cq_push      <= 1'b0;
		byte_dropped <= 1'b0;

		if (reset) begin
			state     <= S_CR0;
			line_open <= 1'b0;
			line_len  <= 0;
		end else
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
			// Draining SPIRXDR always wins over printing, so a received byte
			// can never be lost to a newline in flight.
			S_POLL: begin
				spi_adr <= ADDR_SPISR;
				spi_stb <= 1'b1; spi_rw <= 1'b0;
				if (spi_ack) begin
					spi_stb <= 1'b0;
					if (spi_dato[SR_RRDY])
						state <= S_READ_RX;
					else if (line_open && cs_idle)
						state <= S_PUSH_CR;
					else
						state <= S_POLL;
				end
			end

			S_READ_RX: begin
				spi_adr <= ADDR_SPIRXDR;
				spi_stb <= 1'b1; spi_rw <= 1'b0;
				if (spi_ack) begin
					spi_stb <= 1'b0;
					rx_byte <= spi_dato;
					// The byte leaves the IP either way — dropping it here
					// keeps ROE from latching when the UART is the bottleneck.
					if (cq_room)
						state <= S_PUSH_HI;
					else begin
						byte_dropped <= 1'b1;
						state        <= S_POLL;
					end
				end
			end

			S_PUSH_HI: begin
				cq_push      <= 1'b1;
				cq_push_data <= hexdigit(rx_byte[7:4]);
				line_len     <= line_len + 1'b1;
				state        <= S_PUSH_LO;
			end
			S_PUSH_LO: begin
				cq_push      <= 1'b1;
				cq_push_data <= hexdigit(rx_byte[3:0]);
				state        <= S_PUSH_SEP;
			end
			S_PUSH_SEP: begin
				cq_push      <= 1'b1;
				cq_push_data <= " ";
				line_open    <= 1'b1;
				// Wrap long transactions so a master that holds CS low
				// forever still produces readable lines.
				state <= (line_len >= BYTES_PER_LINE) ? S_PUSH_CR : S_POLL;
			end

			S_PUSH_CR: begin
				cq_push      <= 1'b1;
				cq_push_data <= 8'h0D;
				state        <= S_PUSH_LF;
			end
			S_PUSH_LF: begin
				cq_push      <= 1'b1;
				cq_push_data <= 8'h0A;
				line_open    <= 1'b0;
				line_len     <= 0;
				state        <= S_POLL;
			end

			default: state <= S_CR0;
		endcase
	end

	// ----------------------------------------------------------------
	// Debug LED pulse stretchers
	// ----------------------------------------------------------------
	reg [21:0] drop_led_ctr = 0;
	reg [21:0] rx_led_ctr   = 0;

	always @(posedge clk_core) begin
		if (byte_dropped)
			drop_led_ctr <= LED_CYCLES[21:0];
		else
			drop_led_ctr <= drop_led_ctr - (drop_led_ctr != 0);

		if (rx_accepted)
			rx_led_ctr <= LED_CYCLES[21:0];
		else
			rx_led_ctr <= rx_led_ctr - (rx_led_ctr != 0);
	end

	assign led_r = ~(drop_led_ctr != 0);
	assign led_g = ~spi_ready;
	assign led_b = ~(rx_led_ctr != 0);

endmodule
