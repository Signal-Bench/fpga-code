/** \file
 * SignalBench STREAM link: byte FIFO + SB_SPI slave + mode commands.
 *
 * The command protocol and the SB_SPI service loop deliberately MIRROR the
 * command-capable spi_counter_stream.v on the fix/spi-stream-reliability
 * branch, so the ESP32 firmware written against that image works against
 * this one unchanged:
 *
 *   MCU -> FPGA, one transaction:      FPGA -> MCU, front of the NEXT one:
 *     53 42 A5 5A   ready check          53 42 4F 4B   "SBOK"
 *     53 42 4D xx   select decoder xx    53 42 41 xx   "SBA" + xx
 *
 *   xx:  00 I2C   01 SPI   02 DIO   03 CAN   04 UART
 *
 * An out-of-range xx is ignored and gets no ACK, as in spi_counter_stream.
 * A valid select flushes the FIFO and holds the old decoder in reset, so
 * every byte after the ACK belongs to the new protocol.  Re-selecting the
 * current protocol does the same.
 *
 * Responses go in the NEXT transaction because an SPI slave cannot answer
 * a byte it has not received yet -- the same reason FPGA-TN-02011 Table
 * 12.8 requires a dummy byte for slave reads.
 *
 * Commands are matched on a rolling window of received bytes, not framed on
 * CS -- again as spi_counter_stream does.  The MCU sends 0x00 while it only
 * wants to read, and 0x00 can never start a command.
 *
 * Stream semantics -- what the MCU sees on MISO:
 *   - pending ACK bytes first, then published frames, byte by byte
 *   - SPITXDR is only loaded when there is something to send.  When there
 *     is not, it goes empty and the master reads 0xFF -- harmless now that
 *     frames carry their own length.  See MCU_FRAME_FORMAT.md.
 *
 * The FIFO itself lives in sbs_framer.v; this module only reads it.
 */
module sbs_link #(
	parameter [2:0] DEFAULT_PROTO = 3'd0     // I2C after power-up
)(
	input  wire       clk,
	input  wire       rst,

	// SPI slave pads
	input  wire       mcu_sck,
	input  wire       mcu_cs,
	input  wire       mcu_mosi,
	output wire       mcu_miso,

	// framer read port
	input  wire [7:0] head,
	input  wire       empty,
	output reg        pop,

	output reg  [2:0] proto,
	output reg        proto_switch,    // 1-clk pulse: flush framer, reset decoders
	output wire       spi_ready,       // IP configured
	output reg        tx_pulse         // 1-clk pulse: a byte handed to the IP
);
	// ----------------------------------------------------------------
	// SB_SPI register map and configuration -- identical to
	// spi_counter_stream.v.  A write to any config register resets the SPI
	// core, so all five are written once at start-up and never again.
	// ----------------------------------------------------------------
	localparam [7:0] ADDR_SPICR0  = 8'h08,
	                 ADDR_SPICR1  = 8'h09,
	                 ADDR_SPICR2  = 8'h0A,
	                 ADDR_SPIBR   = 8'h0B,
	                 ADDR_SPISR   = 8'h0C,
	                 ADDR_SPITXDR = 8'h0D,
	                 ADDR_SPIRXDR = 8'h0E,
	                 ADDR_SPICSR  = 8'h0F;

	localparam integer SR_RRDY = 3;
	localparam integer SR_TRDY = 4;

	localparam [7:0] CFG_SPICR1 = 8'b1000_0000;   // SPE=1
	localparam [7:0] CFG_SPICR2 = 8'b0000_0000;   // slave, mode 0, MSB first, SDBRE off
	localparam [7:0] CFG_SPIBR  = 8'b0000_0001;   // >= 1 per datasheet

	reg        spi_stb = 1'b0, spi_rw = 1'b0;
	reg  [7:0] spi_adr = 8'd0, spi_dati = 8'd0;
	wire [7:0] spi_dato;
	wire       spi_ack;

	SB_SPI #(.BUS_ADDR74("0b0000")) u_spi (
		.SBCLKI(clk), .SBSTBI(spi_stb), .SBRWI(spi_rw),
		.SBADRI0(spi_adr[0]), .SBADRI1(spi_adr[1]), .SBADRI2(spi_adr[2]), .SBADRI3(spi_adr[3]),
		.SBADRI4(spi_adr[4]), .SBADRI5(spi_adr[5]), .SBADRI6(spi_adr[6]), .SBADRI7(spi_adr[7]),
		.SBDATI0(spi_dati[0]), .SBDATI1(spi_dati[1]), .SBDATI2(spi_dati[2]), .SBDATI3(spi_dati[3]),
		.SBDATI4(spi_dati[4]), .SBDATI5(spi_dati[5]), .SBDATI6(spi_dati[6]), .SBDATI7(spi_dati[7]),
		.SBDATO0(spi_dato[0]), .SBDATO1(spi_dato[1]), .SBDATO2(spi_dato[2]), .SBDATO3(spi_dato[3]),
		.SBDATO4(spi_dato[4]), .SBDATO5(spi_dato[5]), .SBDATO6(spi_dato[6]), .SBDATO7(spi_dato[7]),
		.SBACKO(spi_ack),
		.MI(1'b0), .SI(mcu_mosi), .SCKI(mcu_sck), .SCSNI(mcu_cs),
		.SO(mcu_miso)
	);

	// ----------------------------------------------------------------
	// Command protocol -- constants as in spi_counter_stream.v
	// ----------------------------------------------------------------
	localparam [7:0] PROTO_MAGIC_0     = 8'h53,   // 'S'
	                 PROTO_MAGIC_1     = 8'h42,   // 'B'
	                 PROTO_READY_0     = 8'hA5,
	                 PROTO_READY_1     = 8'h5A,
	                 PROTO_MODE_CMD    = 8'h4D,   // 'M'
	                 PROTO_READY_0_ACK = 8'h4F,   // 'O'
	                 PROTO_READY_1_ACK = 8'h4B,   // 'K'
	                 PROTO_MODE_ACK    = 8'h41;   // 'A'
	localparam [7:0] PROTO_MAX = 8'h04;           // 00..04 valid

	reg [1:0]  command_pos    = 2'd0;
	reg [7:0]  command_opcode = 8'd0;
	reg [31:0] response_word  = 32'd0;
	reg [2:0]  response_count = 3'd0;
	wire [7:0] response_byte    = response_word[31:24];
	wire       response_pending = (response_count != 3'd0);

	// ----------------------------------------------------------------
	// Service state machine -- same shape as spi_counter_stream.v.
	// RX is drained before TX is refilled: prioritising TRDY can leave
	// SPIRXDR occupied until the next byte arrives, which sets ROE and
	// loses MOSI command bytes on real silicon (found on your branch).
	// ----------------------------------------------------------------
	localparam [3:0] S_CR0 = 4'd0, S_CR1 = 4'd1, S_CR2 = 4'd2, S_BR = 4'd3,
	                 S_CSR = 4'd4, S_POLL = 4'd5, S_LOAD_TX = 4'd6,
	                 S_DRAIN_RX = 4'd7;

	reg [3:0] state = S_CR0;
	assign spi_ready = (state >= S_POLL);

	always @(posedge clk) begin
		spi_stb      <= 1'b0;
		pop          <= 1'b0;
		proto_switch <= 1'b0;
		tx_pulse     <= 1'b0;

		if (rst) begin
			state          <= S_CR0;
			proto          <= DEFAULT_PROTO;
			command_pos    <= 2'd0;
			response_count <= 3'd0;
		end else case (state)
			S_CR0: begin
				spi_adr <= ADDR_SPICR0; spi_dati <= 8'h00; spi_stb <= 1'b1; spi_rw <= 1'b1;
				if (spi_ack) begin spi_stb <= 1'b0; state <= S_CR1; end
			end
			S_CR1: begin
				spi_adr <= ADDR_SPICR1; spi_dati <= CFG_SPICR1; spi_stb <= 1'b1; spi_rw <= 1'b1;
				if (spi_ack) begin spi_stb <= 1'b0; state <= S_CR2; end
			end
			S_CR2: begin
				spi_adr <= ADDR_SPICR2; spi_dati <= CFG_SPICR2; spi_stb <= 1'b1; spi_rw <= 1'b1;
				if (spi_ack) begin spi_stb <= 1'b0; state <= S_BR; end
			end
			S_BR: begin
				spi_adr <= ADDR_SPIBR; spi_dati <= CFG_SPIBR; spi_stb <= 1'b1; spi_rw <= 1'b1;
				if (spi_ack) begin spi_stb <= 1'b0; state <= S_CSR; end
			end
			S_CSR: begin
				spi_adr <= ADDR_SPICSR; spi_dati <= 8'h00; spi_stb <= 1'b1; spi_rw <= 1'b1;
				if (spi_ack) begin spi_stb <= 1'b0; state <= S_POLL; end
			end

			S_POLL: begin
				spi_adr <= ADDR_SPISR; spi_stb <= 1'b1; spi_rw <= 1'b0;
				if (spi_ack) begin
					spi_stb <= 1'b0;
					if (spi_dato[SR_RRDY])
						state <= S_DRAIN_RX;
					else if (spi_dato[SR_TRDY] && (response_pending || !empty))
						state <= S_LOAD_TX;
					else
						state <= S_POLL;
				end
			end

			S_LOAD_TX: begin
				spi_adr  <= ADDR_SPITXDR;
				spi_dati <= response_pending ? response_byte : head;
				spi_stb  <= 1'b1; spi_rw <= 1'b1;
				if (spi_ack) begin
					spi_stb  <= 1'b0;
					tx_pulse <= 1'b1;
					if (response_pending) begin
						response_word  <= {response_word[23:0], 8'h00};
						response_count <= response_count - 3'd1;
					end else begin
						pop <= 1'b1;
					end
					state <= S_POLL;
				end
			end

			S_DRAIN_RX: begin
				spi_adr <= ADDR_SPIRXDR; spi_stb <= 1'b1; spi_rw <= 1'b0;
				if (spi_ack) begin
					spi_stb <= 1'b0;
					state   <= S_POLL;
					case (command_pos)
						2'd0: if (spi_dato == PROTO_MAGIC_0) command_pos <= 2'd1;
						2'd1: begin
							if (spi_dato == PROTO_MAGIC_1) command_pos <= 2'd2;
							else command_pos <= (spi_dato == PROTO_MAGIC_0) ? 2'd1 : 2'd0;
						end
						2'd2: begin
							if (spi_dato == PROTO_READY_0 || spi_dato == PROTO_MODE_CMD) begin
								command_opcode <= spi_dato;
								command_pos    <= 2'd3;
							end else begin
								command_pos <= (spi_dato == PROTO_MAGIC_0) ? 2'd1 : 2'd0;
							end
						end
						2'd3: begin
							command_pos <= 2'd0;
							if (command_opcode == PROTO_READY_0 && spi_dato == PROTO_READY_1) begin
								response_word  <= {PROTO_MAGIC_0, PROTO_MAGIC_1,
								                   PROTO_READY_0_ACK, PROTO_READY_1_ACK};
								response_count <= 3'd4;
							end else if (command_opcode == PROTO_MODE_CMD &&
							             spi_dato <= PROTO_MAX) begin
								proto          <= spi_dato[2:0];
								proto_switch   <= 1'b1;
								response_word  <= {PROTO_MAGIC_0, PROTO_MAGIC_1,
								                   PROTO_MODE_ACK, spi_dato};
								response_count <= 3'd4;
							end
						end
					endcase
				end
			end

			default: state <= S_CR0;
		endcase
	end
endmodule
