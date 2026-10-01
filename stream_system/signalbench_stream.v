/** \file
 * SignalBench STREAM top -- raw-byte streaming for every decoder, selected
 * by the MCU with `53 42 4D xx`.  No trigger, no recording.
 *
 * Derived from ../final_system/signalbench.v.  Kept from it: the 12 MHz
 * crystal -> PLL -> 24 MHz clocking, decoder selection by reset rather than
 * clock gating, the SPI sniffer's probed-SCK domain, and the SB_SPI slave.
 * Removed: the trigger comparator, the record packer and the capture RAM --
 * replaced by a plain byte FIFO.  See README.md for every difference.
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
	parameter integer START_MARKER = 1       // I2C 0xFF at each START, as i2c_sniffer
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
	// MCU link + FIFO
	// ----------------------------------------------------------------
	wire [2:0] proto;
	wire       proto_switch, overflow, spi_ready, tx_pulse;
	reg        s_stb;
	reg  [7:0] s_byte;

	sbs_link #(.FIFO_AW(12), .DEFAULT_PROTO(P_I2C)) u_link (
		.clk(clk), .rst(rst),
		.mcu_sck(mcu_sck), .mcu_cs(mcu_cs), .mcu_mosi(mcu_mosi), .mcu_miso(mcu_miso),
		.in_stb(s_stb), .in_byte(s_byte),
		.proto(proto), .proto_switch(proto_switch),
		.overflow(overflow), .spi_ready(spi_ready), .tx_pulse(tx_pulse)
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
	wire [7:0] i2c_b,   uart_b,   spi_b,   dio_b,   can_b;

	sbs_i2c #(.CORE_HZ(CORE_HZ), .START_MARKER(START_MARKER)) u_i2c (
		.clk(clk), .rst(rst_i2c), .scl_pin(ch0), .sda_pin(ch1),
		.out_stb(i2c_stb), .out_byte(i2c_b)
	);

	sbs_uart #(.CORE_HZ(CORE_HZ), .BAUD(UART_BAUD)) u_uart (
		.clk(clk), .rst(rst_uart), .line_a(ch0), .line_b(ch1),
		.out_stb(uart_stb), .out_byte(uart_b)
	);

	wire sck_g;
	SB_GB_IO #(.PIN_TYPE(6'b0000_01)) u_sck_gb (
		.PACKAGE_PIN(p_sck), .GLOBAL_BUFFER_OUTPUT(sck_g)
	);
	sbs_spi u_spi (
		.clk(clk), .rst(rst_spi), .sck_g(sck_g),
		.mosi(p_mosi), .miso(p_miso), .cs(p_cs), .spi_mode(SPI_MODE),
		.out_stb(spi_stb), .out_byte(spi_b)
	);

	sbs_dio u_dio (
		.clk(clk), .rst(rst_dio), .pin(ch0),
		.out_stb(dio_stb), .out_byte(dio_b)
	);

	sbs_can u_can (
		.clk(clk), .rst(rst_can), .can_rx(ch0),
		.out_stb(can_stb), .out_byte(can_b)
	);

	// Registered mux: keeps proto -> mux -> FIFO write off one combinational
	// path, the same reason final_system registers its event.
	always @(posedge clk) begin
		case (proto)
			P_SPI:   begin s_stb <= spi_stb;  s_byte <= spi_b;  end
			P_DIO:   begin s_stb <= dio_stb;  s_byte <= dio_b;  end
			P_CAN:   begin s_stb <= can_stb;  s_byte <= can_b;  end
			P_UART:  begin s_stb <= uart_stb; s_byte <= uart_b; end
			default: begin s_stb <= i2c_stb;  s_byte <= i2c_b;  end
		endcase
		if (rst || proto_switch) s_stb <= 1'b0;
	end

	// ----------------------------------------------------------------
	// LEDs, active low.  Same meanings as spi_counter_stream where they
	// overlap: green = IP configured, blue = bytes going to the MCU.
	//   red    on ~100 ms after any byte is lost to a full FIFO
	//   green  IP configured; blinks off ~50 ms on each decoded byte
	//   blue   on ~100 ms after a byte is handed to the IP
	// ----------------------------------------------------------------
	localparam [21:0] T100 = CORE_HZ / 10;
	localparam [21:0] T50  = CORE_HZ / 20;
	reg [21:0] r_ctr = 0, g_ctr = 0, b_ctr = 0;
	always @(posedge clk) begin
		r_ctr <= overflow ? T100 : r_ctr - (r_ctr != 0);
		g_ctr <= s_stb    ? T50  : g_ctr - (g_ctr != 0);
		b_ctr <= tx_pulse ? T100 : b_ctr - (b_ctr != 0);
	end

	assign led_r = ~(r_ctr != 0);
	assign led_g = ~(spi_ready & (g_ctr == 0));
	assign led_b = ~(b_ctr != 0);
endmodule
