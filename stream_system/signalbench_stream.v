/** \file
 * SignalBench STREAM top -- framed streaming for every decoder, selected by
 * the MCU with `53 42 4D xx`.  No trigger, no recording.
 *
 * Every frame on MISO is  53 42 4D | flags/protocol | length | payload.
 * MCU_FRAME_FORMAT.md is the parsing spec for the firmware side.
 *
 * Derived from ../final_system/signalbench.v.  Kept from it: the 12 MHz
 * crystal -> PLL -> 24 MHz clocking, decoder selection by reset rather than
 * clock gating, the SPI sniffer's probed-SCK domain, and the SB_SPI slave.
 * Removed: the trigger comparator, the record packer and the capture RAM --
 * replaced by sbs_framer.v.  See README.md for every difference.
 *
 * PROBE PINS ARE SHARED, because only one decoder runs at a time and it
 * makes every existing bench rig plug straight in:
 *
 *   ch0 (pin 38): I2C SCL | UART target-TX line | CAN RX | DIO
 *   ch1 (pin 42): I2C SDA | UART target-RX line
 *
 *   i2c_sniffer.pcf  used SCL 38 / SDA 42
 *   uart_to_spi.v    used TX  38 / RX  42
 *   can_sniffer.pcf  used RX  38
 *
 * SPI needs four lines including a GBIN clock, so it has its own pins.
 *
 * CLOCK: pin 20 needs jumper R16 ("OSC") shorted, as for can_sniffer.
 * Without it there is no clock and nothing runs.  PIN 35 must stay free --
 * the PLL consumes that I/O site.
 */
`include "../common/util.v"
`include "../common/uart.v"

module signalbench_stream #(
	parameter integer UART_BAUD    = 9600,   // no baud opcode; build-time only
	parameter [2:0]   SPI_MODE     = 3'd0,   // {lsbf, cpha, cpol}; build-time only
	parameter integer MAX_PAYLOAD  = 255,    // <= 255; length is one byte
	parameter integer FIFO_AW      = 12      // stream FIFO = 2^FIFO_AW bytes (8 EBR at 12)
)(
	input  wire clk_12,          // pin 20, GBIN, 12 MHz crystal via R16

	input  wire ch0,             // pin 38
	input  wire ch1,             // pin 42
	input  wire p_sck,           // pin 37, GBIN, probed SPI clock
	input  wire p_mosi,
	input  wire p_miso,
	input  wire p_cs,

	input  wire mcu_sck,         // pin 11
	input  wire mcu_cs,          // pin 19
	input  wire mcu_mosi,        // pin 21
	output wire mcu_miso,        // pin 13

	output wire spi_cs_flash,
	output wire led_r,
	output wire led_g,
	output wire led_b
);
	assign spi_cs_flash = 1'b1;

	localparam integer CORE_HZ = 24_000_000;
	localparam [2:0] P_I2C = 3'd0, P_SPI = 3'd1, P_DIO = 3'd2,
	                 P_CAN = 3'd3, P_UART = 3'd4;

	// ----------------------------------------------------------------
	// 12 -> 24 MHz.  `icepll -i 12 -o 24`; VCO 768 MHz.
	// ----------------------------------------------------------------
	wire clk, pll_lock;
	SB_PLL40_CORE #(
		.FEEDBACK_PATH("SIMPLE"),
		.DIVR(4'd0), .DIVF(7'd63), .DIVQ(3'd5),
		.FILTER_RANGE(3'b001)
	) u_pll (
		.REFERENCECLK(clk_12), .PLLOUTCORE(clk), .LOCK(pll_lock),
		.RESETB(1'b1), .BYPASS(1'b0)
	);

	reg [7:0] por = 8'd0;
	always @(posedge clk) if (!por[7]) por <= por + 8'd1;
	wire rst = ~por[7] | ~pll_lock;

	// ----------------------------------------------------------------
	// MCU link, and the framer that owns the stream FIFO
	// ----------------------------------------------------------------
	wire [2:0] proto;
	wire       proto_switch, spi_ready, tx_pulse, dropped_pulse;
	wire [7:0] f_head;
	wire       f_empty, f_pop;
	reg        o_stb;
	reg  [1:0] o_kind;
	reg  [7:0] o_data;

	sbs_link #(.DEFAULT_PROTO(P_I2C)) u_link (
		.clk(clk), .rst(rst),
		.mcu_sck(mcu_sck), .mcu_cs(mcu_cs), .mcu_mosi(mcu_mosi), .mcu_miso(mcu_miso),
		.head(f_head), .empty(f_empty), .pop(f_pop),
		.proto(proto), .proto_switch(proto_switch),
		.spi_ready(spi_ready), .tx_pulse(tx_pulse)
	);

	sbs_framer #(.AW(FIFO_AW), .MAXP(MAX_PAYLOAD)) u_framer (
		.clk(clk), .rst(rst), .flush(proto_switch), .proto(proto),
		.op_stb(o_stb), .op_kind(o_kind), .op_data(o_data),
		.head(f_head), .empty(f_empty), .pop(f_pop),
		.dropped_pulse(dropped_pulse)
	);

	// Unselected decoders are held in reset; a switch also resets the newly
	// selected one, so it starts from a clean state (I2C waits for a START,
	// UART for a start bit, CAN for 11 recessive bits, SPI for CS).
	wire rst_i2c  = rst | proto_switch | (proto != P_I2C);
	wire rst_spi  = rst | proto_switch | (proto != P_SPI);
	wire rst_dio  = rst | proto_switch | (proto != P_DIO);
	wire rst_can  = rst | proto_switch | (proto != P_CAN);
	wire rst_uart = rst | proto_switch | (proto != P_UART);

	// ----------------------------------------------------------------
	// Decoders
	// ----------------------------------------------------------------
	wire       i2c_stb, uart_stb, spi_stb, dio_stb, can_stb;
	wire [1:0] i2c_k,   uart_k,   spi_k,   dio_k,   can_k;
	wire [7:0] i2c_d,   uart_d,   spi_d,   dio_d,   can_d;

	sbs_i2c #(.CORE_HZ(CORE_HZ), .MAXP(MAX_PAYLOAD)) u_i2c (
		.clk(clk), .rst(rst_i2c), .scl_pin(ch0), .sda_pin(ch1),
		.op_stb(i2c_stb), .op_kind(i2c_k), .op_data(i2c_d)
	);

	sbs_uart #(.CORE_HZ(CORE_HZ), .BAUD(UART_BAUD)) u_uart (
		.clk(clk), .rst(rst_uart), .line_a(ch0), .line_b(ch1),
		.op_stb(uart_stb), .op_kind(uart_k), .op_data(uart_d)
	);

	wire sck_g;
	SB_GB_IO #(.PIN_TYPE(6'b0000_01)) u_sck_gb (
		.PACKAGE_PIN(p_sck), .GLOBAL_BUFFER_OUTPUT(sck_g)
	);
	sbs_spi #(.MAXP(MAX_PAYLOAD)) u_spi (
		.clk(clk), .rst(rst_spi), .sck_g(sck_g),
		.mosi(p_mosi), .miso(p_miso), .cs(p_cs), .spi_mode(SPI_MODE),
		.op_stb(spi_stb), .op_kind(spi_k), .op_data(spi_d)
	);

	sbs_dio #(.CORE_HZ(CORE_HZ)) u_dio (
		.clk(clk), .rst(rst_dio), .pin(ch0),
		.op_stb(dio_stb), .op_kind(dio_k), .op_data(dio_d)
	);

	sbs_can u_can (
		.clk(clk), .rst(rst_can), .can_rx(ch0),
		.op_stb(can_stb), .op_kind(can_k), .op_data(can_d)
	);

	// Registered mux: keeps proto -> mux -> framer off one combinational
	// path, the same reason final_system registers its event.
	always @(posedge clk) begin
		case (proto)
			P_SPI:   begin o_stb <= spi_stb;  o_kind <= spi_k;  o_data <= spi_d;  end
			P_DIO:   begin o_stb <= dio_stb;  o_kind <= dio_k;  o_data <= dio_d;  end
			P_CAN:   begin o_stb <= can_stb;  o_kind <= can_k;  o_data <= can_d;  end
			P_UART:  begin o_stb <= uart_stb; o_kind <= uart_k; o_data <= uart_d; end
			default: begin o_stb <= i2c_stb;  o_kind <= i2c_k;  o_data <= i2c_d;  end
		endcase
		if (rst || proto_switch) o_stb <= 1'b0;
	end

	// ----------------------------------------------------------------
	// LEDs, active low.  Same meanings as spi_counter_stream where they
	// overlap: green = IP configured, blue = bytes going to the MCU.
	//   red    on ~100 ms after a frame is lost (FIFO full)
	//   green  IP configured; blinks off ~50 ms on decoder activity
	//   blue   on ~100 ms after a byte is handed to the IP
	// ----------------------------------------------------------------
	localparam [21:0] T100 = CORE_HZ / 10;
	localparam [21:0] T50  = CORE_HZ / 20;
	reg [21:0] r_ctr = 0, g_ctr = 0, b_ctr = 0;
	always @(posedge clk) begin
		r_ctr <= dropped_pulse ? T100 : r_ctr - (r_ctr != 0);
		g_ctr <= o_stb    ? T50  : g_ctr - (g_ctr != 0);
		b_ctr <= tx_pulse ? T100 : b_ctr - (b_ctr != 0);
	end

	assign led_r = ~(r_ctr != 0);
	assign led_g = ~(spi_ready & (g_ctr == 0));
	assign led_b = ~(b_ctr != 0);
endmodule
