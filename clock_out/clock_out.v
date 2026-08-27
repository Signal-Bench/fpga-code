/** \file
 * UART-to-SPI Bridge.
 * Receives data from UART and sends it over SPI.
 *
 * Pin assignments:
 *   gpio_11  -> SPI SCK   (output)
 *   gpio_21  -> SPI MOSI  (output)
 *   gpio_19  -> SPI CS    (output, active low)
 *   serial_rxd -> UART input
 */
`include "../common/util.v"
`include "../common/uart.v"

module top(
	output gpio_11,      // SCK
	output gpio_21,      // MOSI
	output gpio_19,      // CS (active low)
	// input  serial_rxd,
	input gpio_38,      // UART RX input (sniff)	
	output spi_cs        // onboard flash disable
);
	assign spi_cs = 1;

	wire clk_48;
	wire reset = 0;
	SB_HFOSC u_hfosc (
		.CLKHFPU(1'b1),
		.CLKHFEN(1'b1),
		.CLKHF(clk_48)
	);

	// // UART baud clocks (1 Mbaud)
	// wire clk_4;  // 4 MHz for RX (baud_x4)
	// divide_by_n #(.N(12)) div_rx(clk_48, reset, clk_4);

		// UART baud clocks (1 Mbaud)
	wire clk_4;  // 4 MHz for RX (baud_x4)
	divide_by_n #(.N(1250)) div_rx(clk_48, reset, clk_4);

	// UART RX
	wire [7:0] uart_rxd;
	wire       uart_rxd_strobe;

	uart_rx rxd(
		.mclk(clk_48),
		.reset(reset),
		.baud_x4(clk_4),
		.serial(gpio_38),
		.data(uart_rxd),
		.data_strobe(uart_rxd_strobe)
	);

	// SPI clock divider (~1 MHz)
	reg [4:0] div = 0;
	wire tick = (div == 23);
	always @(posedge clk_48) begin
		if (tick) div <= 0;
		else div <= div + 1;
	end

	// State machine
	localparam S_IDLE      = 3'd0;
	localparam S_CS_LOW    = 3'd1;
	localparam S_CLKING    = 3'd2;
	localparam S_CS_HIGH   = 3'd3;

	reg [2:0] state = S_IDLE;
	reg [2:0] bit_idx = 0;
	reg [7:0] shift_reg = 0;
	reg       sck_out = 0;
	reg       mosi_out = 0;
	reg       cs_out = 1;
	reg       sck_phase = 0;

	always @(posedge clk_48) begin
		if (state == S_IDLE) begin
			cs_out   <= 1;
			sck_out  <= 0;
			mosi_out <= 0;
			if (uart_rxd_strobe) begin
				shift_reg <= uart_rxd;
				state     <= S_CS_LOW;
			end
		end else if (tick) begin
			case (state)
				S_CS_LOW: begin
					cs_out    <= 0;
					bit_idx   <= 7;
					sck_phase <= 0;
					state     <= S_CLKING;
				end

				S_CLKING: begin
					if (sck_phase == 0) begin
						sck_out   <= 1;
						mosi_out  <= shift_reg[7];
						sck_phase <= 1;
					end else begin
						sck_out   <= 0;
						sck_phase <= 0;
						shift_reg <= {shift_reg[6:0], 1'b0};
						if (bit_idx == 0) state <= S_CS_HIGH;
						else bit_idx <= bit_idx - 1;
					end
				end

				S_CS_HIGH: begin
					cs_out <= 1;
					state  <= S_IDLE;
				end
			endcase
		end
	end

	assign gpio_11 = sck_out;
	assign gpio_21 = mosi_out;
	assign gpio_19 = cs_out;
endmodule
