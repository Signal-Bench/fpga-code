/** \file
 * SignalBench MCU link: SB_SPI hard IP as slave, plus command decode.
 *
 * *** PLACEHOLDER PROTOCOL ***
 * The real command set is not settled -- see ../state_management.md, whose
 * CMD_ names are themselves placeholders and whose "Open questions" section
 * lists what is still undecided.  The opcodes below are INVENTED so the
 * system can be built, synthesized and measured end to end.  They are
 * deliberately simple and are expected to change.  What is NOT invented is
 * the two silicon constraints below, which any real protocol must respect.
 *
 * Why the hard IP rather than a soft slave: SCK clocks the shift register
 * inside SB_SPI, so the crossing into the core domain is handled by the
 * vendor rather than by us, and there is no SCK <= core/4 oversampling
 * ceiling.  Measured cheaper AND faster than spi_slave.v -- see
 * ../unified_top.md.
 *
 * CONSTRAINT 1 -- the dummy byte is mandatory.  FPGA-TN-02011 Table 12.8:
 * as a slave, SPITXDR must be written before the SCK edge that samples the
 * LSB of the PREVIOUS byte, and Lattice states this "requires at least one
 * protocol dummy byte be included for all slave SPI read operations".  So a
 * read is: command byte, dummy byte, then response.
 *
 * CONSTRAINT 2 -- the refill deadline.  SPITXDR must be reloaded within
 * ~7 bit times of TRDY.  The service loop below is a poll plus an RX drain
 * plus a TX load, roughly 9 core clocks, ~375 ns at 24 MHz.  That fits under
 * 7 bit times down to about 10 MHz SCK.
 *
 * Transaction shape (MCU is master, FPGA is slave):
 *
 *   write:  [opcode] [data]
 *   read:   [opcode] [dummy] [response] [response] ...
 *
 * Opcodes (all placeholders):
 *   0x01 SET_PROTOCOL  + data  0=I2C 1=CAN 2=UART 3=SPI
 *   0x02 SET_BAUD_LO   + data  low byte of the UART divisor
 *   0x03 SET_BAUD_HI   + data  high byte
 *   0x04 SET_CAN_BRP   + data  CAN prescaler; MUST be >= 1, see note below
 *   0x05 SET_SPI_MODE  + data  {5'b0, lsbf, cpha, cpol}
 *   0x06 SET_TRIG_LEN  + data  0..4; 0 means "record now", no pattern
 *   0x08 SET_TRIG_B0   + data  pattern byte 0  (0x09/0x0A/0x0B = bytes 1/2/3)
 *   0x10 ARM_RECORD            arm; capture starts on the trigger match
 *   0x11 START_STREAM          free-run capture, no trigger
 *   0x12 STOP                  back to idle
 *   0x20 READ_STATUS           -> dummy, then the 3 status bytes
 *   0x21 DRAIN                 -> dummy, then capture RAM bytes, auto-advancing
 *
 * Status bytes returned by 0x20:
 *   [0] {done, dropped, streaming, recording, armed, 1'b0, proto[1:0]}
 *   [1] fill_level[15:8]
 *   [2] fill_level[7:0]
 */
module sb_mcu_link (
	input  wire clk,
	input  wire rst,

	// SPI slave pads (MCU drives SCK/CS/MOSI)
	input  wire mcu_sck,
	input  wire mcu_cs,
	input  wire mcu_mosi,
	output wire mcu_miso,
	output wire mcu_miso_oe,

	// configuration outputs
	output reg  [1:0]  proto,
	output reg  [15:0] baud_div,
	output reg  [7:0]  can_brp,
	output reg  [2:0]  spi_mode,
	output reg  [7:0]  trig0, trig1, trig2, trig3,
	output reg  [2:0]  trig_len,

	// control pulses to sb_capture
	output reg         arm_record,
	output reg         start_stream,
	output reg         stop,
	output reg         drain_advance,

	// status in
	input  wire        armed,
	input  wire        recording,
	input  wire        streaming,
	input  wire        done,
	input  wire        dropped,
	input  wire [15:0] fill_level,
	input  wire [7:0]  drain_data
);
	// ----------------------------------------------------------------
	// SB_SPI register map -- FPGA-TN-02011.  Upper address nibble is the
	// BUS_ADDR74 parameter; the low nibble selects the register.
	// ----------------------------------------------------------------
	localparam [7:0] ADDR_SPICR0 = 8'h08,
	                 ADDR_SPICR1 = 8'h09,
	                 ADDR_SPICR2 = 8'h0A,
	                 ADDR_SPIBR  = 8'h0B,
	                 ADDR_SPITXDR= 8'h0D,
	                 ADDR_SPIRXDR= 8'h0E,
	                 ADDR_SPISR  = 8'h0C;

	// SPICR1: SPE (enable) = bit7, MSTR (master) = bit6 -> slave, enabled.
	localparam [7:0] CFG_SPICR1 = 8'b1000_0000;
	// SPICR2[SDBRE] = bit5: Lattice dummy-byte response.  Gives the MCU a
	// deterministic 0xFF.../0x00-marker/data framing to sync on, which is
	// also the answer to state_management.md's streaming-backpressure
	// question.
	localparam [7:0] CFG_SPICR2 = 8'b0010_0000;
	// SPIBR: divider is master-mode only, but the datasheet states the
	// >= 1 constraint unconditionally.
	localparam [7:0] CFG_SPIBR  = 8'b0000_0001;

	reg  [7:0] sb_adr = 8'd0, sb_dati = 8'd0;
	reg        sb_stb = 1'b0, sb_rw = 1'b0;
	wire [7:0] sb_dato;
	wire       sb_ack, so_data, so_oe;

	SB_SPI #(.BUS_ADDR74("0b0000")) u_spi (
		.SBCLKI(clk), .SBSTBI(sb_stb), .SBRWI(sb_rw),
		.SBADRI0(sb_adr[0]), .SBADRI1(sb_adr[1]), .SBADRI2(sb_adr[2]), .SBADRI3(sb_adr[3]),
		.SBADRI4(sb_adr[4]), .SBADRI5(sb_adr[5]), .SBADRI6(sb_adr[6]), .SBADRI7(sb_adr[7]),
		.SBDATI0(sb_dati[0]), .SBDATI1(sb_dati[1]), .SBDATI2(sb_dati[2]), .SBDATI3(sb_dati[3]),
		.SBDATI4(sb_dati[4]), .SBDATI5(sb_dati[5]), .SBDATI6(sb_dati[6]), .SBDATI7(sb_dati[7]),
		.SBDATO0(sb_dato[0]), .SBDATO1(sb_dato[1]), .SBDATO2(sb_dato[2]), .SBDATO3(sb_dato[3]),
		.SBDATO4(sb_dato[4]), .SBDATO5(sb_dato[5]), .SBDATO6(sb_dato[6]), .SBDATO7(sb_dato[7]),
		.SBACKO(sb_ack),
		.MI(1'b0), .SI(mcu_mosi), .SCKI(mcu_sck), .SCSNI(mcu_cs),
		.SO(so_data), .SOE(so_oe)
	);
	assign mcu_miso    = so_data;
	assign mcu_miso_oe = so_oe;

	// ----------------------------------------------------------------
	// Response source.  What the next TX load should send, chosen by the
	// opcode currently being serviced.
	// ----------------------------------------------------------------
	localparam [7:0] OP_NONE = 8'h00, OP_STATUS = 8'h20, OP_DRAIN = 8'h21;

	reg [7:0] resp_op  = OP_NONE;
	reg [1:0] resp_idx = 2'd0;

	reg [7:0] tx_byte;
	always @* begin
		case (resp_op)
			OP_STATUS: case (resp_idx)
				2'd0: tx_byte = {done, dropped, streaming, recording, armed, 1'b0, proto};
				2'd1: tx_byte = fill_level[15:8];
				default: tx_byte = fill_level[7:0];
			endcase
			OP_DRAIN:  tx_byte = drain_data;
			default:   tx_byte = 8'h00;
		endcase
	end

	// ----------------------------------------------------------------
	// Service state machine: configure the IP once, then loop polling
	// status, draining RX, and keeping TX loaded.
	// ----------------------------------------------------------------
	localparam [3:0] S_CR1 = 4'd0, S_CR2 = 4'd1, S_BR = 4'd2,
	                 S_POLL = 4'd3, S_RX = 4'd4, S_TX = 4'd5;

	reg [3:0] state   = S_CR1;
	reg [7:0] rx_byte  = 8'd0;
	reg       rx_valid = 1'b0;
	reg       tx_loaded= 1'b0;

	always @(posedge clk) begin
		sb_stb    <= 1'b0;
		rx_valid  <= 1'b0;
		tx_loaded <= 1'b0;

		if (rst) begin
			state <= S_CR1; sb_stb <= 1'b0;
		end else case (state)
			S_CR1: begin
				sb_adr <= ADDR_SPICR1; sb_dati <= CFG_SPICR1; sb_rw <= 1'b1; sb_stb <= 1'b1;
				if (sb_ack) begin sb_stb <= 1'b0; state <= S_CR2; end
			end
			S_CR2: begin
				sb_adr <= ADDR_SPICR2; sb_dati <= CFG_SPICR2; sb_rw <= 1'b1; sb_stb <= 1'b1;
				if (sb_ack) begin sb_stb <= 1'b0; state <= S_BR; end
			end
			S_BR: begin
				sb_adr <= ADDR_SPIBR; sb_dati <= CFG_SPIBR; sb_rw <= 1'b1; sb_stb <= 1'b1;
				if (sb_ack) begin sb_stb <= 1'b0; state <= S_POLL; end
			end
			S_POLL: begin
				sb_adr <= ADDR_SPISR; sb_rw <= 1'b0; sb_stb <= 1'b1;
				if (sb_ack) begin
					sb_stb <= 1'b0;
					// SPISR: bit3 = RRDY (receive ready), bit4 = TRDY.
					// RX first -- draining it also stops the overrun flag
					// latching up.
					if      (sb_dato[3]) state <= S_RX;
					else if (sb_dato[4]) state <= S_TX;
				end
			end
			S_RX: begin
				sb_adr <= ADDR_SPIRXDR; sb_rw <= 1'b0; sb_stb <= 1'b1;
				if (sb_ack) begin
					sb_stb   <= 1'b0;
					rx_byte  <= sb_dato;
					rx_valid <= 1'b1;
					state    <= S_POLL;
				end
			end
			S_TX: begin
				sb_adr <= ADDR_SPITXDR; sb_dati <= tx_byte; sb_rw <= 1'b1; sb_stb <= 1'b1;
				if (sb_ack) begin
					sb_stb   <= 1'b0;
					state    <= S_POLL;
					tx_loaded <= 1'b1;     // consumed by the command block
				end
			end
			default: state <= S_POLL;
		endcase
	end

	// ----------------------------------------------------------------
	// Command decode.
	//
	// Framing is on CS, not on a byte count.  SB_SPI does not bring CS out
	// to the fabric, so the pad is tapped directly -- it drives both SCSNI
	// and the synchronizer below.  Without a frame delimiter, an MCU that
	// aborts a transaction mid-command would leave cmd_phase set and every
	// later opcode would be misread as a data byte.
	//
	// Byte 0 of a transaction is the opcode.  For a write, byte 1 is its
	// data.  For a read, byte 1 is the mandatory protocol dummy and bytes
	// 2.. are the response.
	// ----------------------------------------------------------------
	reg [2:0] cs_sync    = 3'b111;
	always @(posedge clk) cs_sync <= {cs_sync[1:0], mcu_cs};
	wire cs_falling = ~cs_sync[1] & cs_sync[2];

	reg       cmd_phase = 1'b0;
	reg [7:0] cmd_op    = 8'd0;
	reg       have_op   = 1'b0;   // opcode of this transaction already seen

	always @(posedge clk) begin
		arm_record    <= 1'b0;
		start_stream  <= 1'b0;
		stop          <= 1'b0;
		drain_advance <= 1'b0;

		if (rst) begin
			proto <= 2'd0; baud_div <= 16'd208; can_brp <= 8'd1;
			spi_mode <= 3'd0; trig_len <= 3'd0;
			trig0 <= 8'd0; trig1 <= 8'd0; trig2 <= 8'd0; trig3 <= 8'd0;
			cmd_phase <= 1'b0; cmd_op <= 8'd0; have_op <= 1'b0;
			resp_op <= OP_NONE; resp_idx <= 2'd0;

		end else if (cs_falling) begin
			cmd_phase <= 1'b0;
			have_op   <= 1'b0;
			resp_op   <= OP_NONE;
			resp_idx  <= 2'd0;

		end else if (rx_valid) begin
			// Response pointers advance per byte ACTUALLY EXCHANGED, which is
			// what a received byte represents -- never per TX load.  The
			// service FSM reloads SPITXDR whenever TRDY is set, and that
			// happens continuously whether or not a transaction is in
			// progress; advancing on the load would walk the drain pointer
			// through the entire buffer while the bus sits idle.  Reloading
			// the same byte repeatedly is harmless, so the load stays free-
			// running and only the pointer is gated.
			if (have_op) begin
				if (resp_op == OP_DRAIN)  drain_advance <= 1'b1;
				if (resp_op == OP_STATUS)
					resp_idx <= (resp_idx == 2'd2) ? 2'd0 : resp_idx + 2'd1;
			end

			if (!have_op) begin
				have_op <= 1'b1;
				cmd_op  <= rx_byte;
				// Opcodes with no data byte act immediately.
				case (rx_byte)
					8'h10: begin arm_record   <= 1'b1; resp_op <= OP_NONE; end
					8'h11: begin start_stream <= 1'b1; resp_op <= OP_NONE; end
					8'h12: begin stop         <= 1'b1; resp_op <= OP_NONE; end
					8'h20: begin resp_op <= OP_STATUS; resp_idx <= 2'd0; end
					8'h21: begin resp_op <= OP_DRAIN;                     end
					default: cmd_phase <= 1'b1;     // expects a data byte
				endcase
			end else if (cmd_phase) begin
				cmd_phase <= 1'b0;
				case (cmd_op)
					8'h01: proto         <= rx_byte[1:0];
					8'h02: baud_div[7:0] <= rx_byte;
					8'h03: baud_div[15:8]<= rx_byte;
					// can_brp is clamped to >= 1 because can_bit_timing is built
					// here with PIPELINE_RESYNC=1, which is WRONG at brp = 0.
					// Clamping means a bad MCU write degrades the bit rate rather
					// than silently corrupting every frame.
					8'h04: can_brp       <= (rx_byte == 8'd0) ? 8'd1 : rx_byte;
					8'h05: spi_mode      <= rx_byte[2:0];
					8'h06: trig_len      <= (rx_byte > 8'd4) ? 3'd4 : rx_byte[2:0];
					8'h08: trig0         <= rx_byte;
					8'h09: trig1         <= rx_byte;
					8'h0A: trig2         <= rx_byte;
					8'h0B: trig3         <= rx_byte;
					default: ;
				endcase
			end
		end
	end
endmodule
