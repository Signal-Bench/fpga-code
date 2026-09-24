/** \file
 * SignalBench protocol decoders: I2C, UART (two probed lines), SPI.
 *
 * CAN is not here -- it is can_sniffer/can_bit_timing.v + can_frame_fsm.v,
 * instantiated directly by signalbench.v.  Those are already complete and
 * hardware-tested and are reused unmodified apart from the opt-in
 * PIPELINE_RESYNC parameter (see ../unified_top.md).
 *
 * Every decoder presents the SAME event interface so the top module can mux
 * them with no per-protocol special cases:
 *
 *   ev_strobe    1-clk pulse, an event is valid this cycle
 *   ev_flags[5:0]  decoder-specific status bits
 *   ev_aux[31:0]   decoder-specific word (CAN uses it for the identifier)
 *   ev_len[3:0]    how many bytes of ev_payload are meaningful
 *   ev_payload     up to 8 bytes, MSB first (payload[0] in bits [63:56])
 *
 * All three decoders here run in the core clock domain EXCEPT sb_spi's
 * shift registers, which are clocked by the probed SCK -- see that module.
 */

// ====================================================================
// I2C -- byte assembly from a clocked bus.
//
// Lifted from i2c_sniffer.v's capture front end.  Same 3-stage
// synchronizers and the same START/STOP derivation; what is dropped is
// that design's RAM, ping-pong buffers and drain, which the shared back
// end now provides.
// ====================================================================
module sb_i2c (
	input  wire clk,
	input  wire rst,
	input  wire scl_pin,
	input  wire sda_pin,

	output reg        ev_strobe,
	output reg  [5:0] ev_flags,
	output wire [31:0] ev_aux,
	output wire [3:0]  ev_len,
	output reg  [63:0] ev_payload
);
	assign ev_aux = 32'd0;
	assign ev_len = 4'd1;

	reg [2:0] scl_s = 3'b111;
	reg [2:0] sda_s = 3'b111;
	always @(posedge clk) begin
		if (rst) begin scl_s <= 3'b111; sda_s <= 3'b111; end
		else begin
			scl_s <= {scl_s[1:0], scl_pin};
			sda_s <= {sda_s[1:0], sda_pin};
		end
	end

	wire scl_rising  =  scl_s[1] & ~scl_s[2];
	wire sda_rising  =  sda_s[1] & ~sda_s[2];
	wire sda_falling = ~sda_s[1] &  sda_s[2];
	wire start_cond  = sda_falling & scl_s[1];
	wire stop_cond   = sda_rising  & scl_s[1];

	reg [7:0] sh  = 8'd0;
	reg [3:0] cnt = 4'd0;
	reg       saw_start = 1'b0;

	always @(posedge clk) begin
		ev_strobe <= 1'b0;
		if (rst) begin
			cnt <= 0; sh <= 0; saw_start <= 0; ev_flags <= 0; ev_payload <= 0;
		end else begin
			if (start_cond) begin
				cnt <= 0; saw_start <= 1'b1;
			end else if (stop_cond) begin
				// STOP is reported as a zero-length marker event so the host
				// can bound a transaction without a separate sentinel byte.
				cnt <= 0; saw_start <= 1'b0;
				ev_flags   <= 6'b010000;      // {start, stop, ack, 3'b0}
				ev_payload <= 64'd0;
				ev_strobe  <= 1'b1;
			end else if (scl_rising) begin
				if (cnt == 4'd8) begin
					// 9th clock is the ACK slot: SDA low = ACK.
					cnt        <= 0;
					ev_flags   <= {saw_start, 1'b0, ~sda_s[1], 3'b000};
					ev_payload <= {sh, 56'd0};
					ev_strobe  <= 1'b1;
					saw_start  <= 1'b0;
				end else begin
					sh  <= {sh[6:0], sda_s[1]};
					cnt <= cnt + 4'd1;
				end
			end
		end
	end
endmodule


// ====================================================================
// UART -- two probed lines (the target's TX and RX).
//
// Same arrangement as uart_to_spi.v: two uart_rx instances sharing one
// baud generator, with a direction flag distinguishing them.  uart_to_spi
// tags its output with a 0x01/0x02 byte; here the tag moves into
// ev_flags[5] so it costs no payload space.
//
// RS-232 and RS-485 reuse this unchanged -- they differ only at the
// physical layer, ahead of the transceiver.
//
// Simultaneous bytes on both lines: TX wins and RX is dropped for that
// cycle.  Full duplex at the same instant is possible on a real link, so
// this is a real (if rare) lossy case -- see ../unified_top.md.
// ====================================================================
module sb_uart (
	input  wire clk,
	input  wire rst,
	input  wire [15:0] baud_div,     // core clocks per baud_x4 tick
	input  wire tx_pin,              // target's TX line
	input  wire rx_pin,              // target's RX line

	output wire        ev_strobe,
	output wire [5:0]  ev_flags,
	output wire [31:0] ev_aux,
	output wire [3:0]  ev_len,
	output wire [63:0] ev_payload
);
	assign ev_aux = 32'd0;
	assign ev_len = 4'd1;

	reg [15:0] bd = 16'd0;
	reg        baud_x4 = 1'b0;
	always @(posedge clk) begin
		if (rst) begin bd <= 0; baud_x4 <= 0; end
		else if (bd >= baud_div) begin bd <= 0; baud_x4 <= 1'b1; end
		else begin bd <= bd + 16'd1; baud_x4 <= 1'b0; end
	end

	wire [7:0] tx_data, rx_data;
	wire       tx_stb,  rx_stb;

	uart_rx u_tx (.mclk(clk), .reset(rst), .baud_x4(baud_x4),
	              .serial(tx_pin), .data(tx_data), .data_strobe(tx_stb));
	uart_rx u_rx (.mclk(clk), .reset(rst), .baud_x4(baud_x4),
	              .serial(rx_pin), .data(rx_data), .data_strobe(rx_stb));

	assign ev_strobe  = tx_stb | rx_stb;
	assign ev_flags   = {rx_stb & ~tx_stb, 5'b00000};   // bit5: 0 = TX, 1 = RX
	assign ev_payload = {tx_stb ? tx_data : rx_data, 56'd0};
endmodule


// ====================================================================
// SPI -- the one decoder whose sampling leaves the core clock domain.
//
// The shift registers are clocked by the PROBED SCK, not oversampled in
// the core domain.  Oversampling needs ~4 core clocks per SCK half period,
// which would cap the sniffable target at a few MHz; real SPI devices run
// 1-50 MHz and there is nothing to negotiate with a target.  See
// ../spi_sniffer/spi_sniffer.md and ../unified_top.md.
//
// Both clock edges are captured into separate register sets and the core
// domain selects between them.  The alternative -- a combinational mux on
// the clock net -- costs fewer flops but puts a glitch source on a clock
// and creates an ordering rule ("only change mode while disarmed") that
// has to be enforced in both FPGA logic and MCU firmware.
//
// Only a toggle bit crosses into the core domain.  The captured pair is
// held stable for the next 7 SCK periods, so a 2-FF-plus-edge-detect read
// of the toggle is safe and the data needs no synchronizer of its own.
//
// NOTE: sck_g must come from a global buffer -- see signalbench.v.
// ====================================================================
module sb_spi (
	input  wire clk,                 // core clock
	input  wire rst,
	input  wire sck_g,               // probed SCK, off a global buffer
	input  wire mosi,
	input  wire miso,
	input  wire cs,                  // active low, idle high
	input  wire [2:0] spi_mode,      // {lsbf, cpha, cpol}

	output wire        ev_strobe,
	output wire [5:0]  ev_flags,
	output wire [31:0] ev_aux,
	output wire [3:0]  ev_len,
	output wire [63:0] ev_payload
);
	assign ev_aux = 32'd0;
	assign ev_len = 4'd2;            // one MOSI byte + one MISO byte

	// ---- probed-SCK domain, rising edge ----------------------------
	reg [7:0]  mo_p = 0, mi_p = 0;
	reg [2:0]  bc_p = 0;
	reg [15:0] pair_p = 0;
	reg        tog_p = 0;
	always @(posedge sck_g or posedge cs) begin
		if (cs) begin
			bc_p <= 3'd0;            // CS idle-high resets the bit counter
		end else begin
			mo_p <= {mo_p[6:0], mosi};
			mi_p <= {mi_p[6:0], miso};
			bc_p <= bc_p + 3'd1;
			if (bc_p == 3'd7) begin
				pair_p <= {{mo_p[6:0], mosi}, {mi_p[6:0], miso}};
				tog_p  <= ~tog_p;
			end
		end
	end

	// ---- probed-SCK domain, falling edge ---------------------------
	reg [7:0]  mo_n = 0, mi_n = 0;
	reg [2:0]  bc_n = 0;
	reg [15:0] pair_n = 0;
	reg        tog_n = 0;
	always @(negedge sck_g or posedge cs) begin
		if (cs) begin
			bc_n <= 3'd0;
		end else begin
			mo_n <= {mo_n[6:0], mosi};
			mi_n <= {mi_n[6:0], miso};
			bc_n <= bc_n + 3'd1;
			if (bc_n == 3'd7) begin
				pair_n <= {{mo_n[6:0], mosi}, {mi_n[6:0], miso}};
				tog_n  <= ~tog_n;
			end
		end
	end

	// ---- core domain: toggle handoff -------------------------------
	// Sample edge is rising when CPOL == CPHA, falling otherwise.
	wire use_neg = spi_mode[0] ^ spi_mode[1];

	reg [2:0] sp = 3'd0, sn = 3'd0;
	always @(posedge clk) begin
		if (rst) begin sp <= 0; sn <= 0; end
		else begin
			sp <= {sp[1:0], tog_p};
			sn <= {sn[1:0], tog_n};
		end
	end

	wire [15:0] pair_sel = use_neg ? pair_n : pair_p;

	assign ev_strobe  = use_neg ? (sn[2] ^ sn[1]) : (sp[2] ^ sp[1]);
	assign ev_flags   = {spi_mode, 3'b000};       // {lsbf, cpha, cpol, 3'b0}
	assign ev_payload = {pair_sel[15:8], pair_sel[7:0], 48'd0};
endmodule
