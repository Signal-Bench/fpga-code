/** \file
 * Host-to-FPGA serial protocol with local echo.
 *
 * Protocol: send "N <data>\n"
 *   - 'N' starts a command
 *   - ' ' space separator
 *   - <data> is any sequence of bytes (up to 8)
 *   - '\r' or '\n' terminates the data
 *
 * FPGA echoes all typed characters back so they are visible,
 * then responds with "FPGA: 0x<hex inverted bytes>\r\n"
 * where each data byte is bitwise inverted (~byte) and printed as hex.
 *
 * Example: type "N hello\n" -> FPGA replies "FPGA: 0x979a939390\r\n"
 *
 * The SPI flash chip select *MUST* be pulled high to disable the
 * flash chip, otherwise they will both be driving the bus.
 */
`include "../common/util.v"
`include "../common/uart.v"

module top(
	output led_r,
	output led_g,
	output led_b,
	output serial_txd,
	input serial_rxd,
	output spi_cs,
	output gpio_2
);
	assign spi_cs = 1; // it is necessary to turn off the SPI flash chip
	wire debug0 = gpio_2;

	wire clk_48;
	wire reset = 0;
	SB_HFOSC u_hfosc (
		.CLKHFPU(1'b1),
		.CLKHFEN(1'b1),
		.CLKHF(clk_48)
	);

	assign led_g = 1;
	assign led_b = serial_rxd; // idles high

	// generate a 1 MHz baud clock from the 48 MHz clock (for TX, baud_x1)
	wire clk_1;
	divide_by_n #(.N(48)) div_tx(clk_48, reset, clk_1);

	// generate a 4 MHz baud clock from the 48 MHz clock (for RX, baud_x4)
	wire clk_4;
	divide_by_n #(.N(12)) div_rx(clk_48, reset, clk_4);

	// ---- UART TX ----
	reg [7:0] uart_txd;
	reg uart_txd_strobe;
	wire uart_txd_ready;

	uart_tx txd(
		.mclk(clk_48),
		.reset(reset),
		.baud_x1(clk_1),
		.serial(serial_txd),
		.ready(uart_txd_ready),
		.data(uart_txd),
		.data_strobe(uart_txd_strobe)
	);

	// ---- UART RX ----
	wire [7:0] uart_rxd;
	wire uart_rxd_strobe;

	uart_rx rxd(
		.mclk(clk_48),
		.reset(reset),
		.baud_x4(clk_4),
		.serial(serial_rxd),
		.data(uart_rxd),
		.data_strobe(uart_rxd_strobe)
	);

	assign debug0 = serial_txd;

	// ---- State machine ----
	localparam S_IDLE        = 4'd0;
	localparam S_WAIT_SPACE  = 4'd1;
	localparam S_READ_DATA   = 4'd2;
	localparam S_ECHO        = 4'd3;
	localparam S_SEND_PREFIX = 4'd4;
	localparam S_LOAD_BYTE   = 4'd5;  // latch buffer byte into cur_byte
	localparam S_SEND_HI     = 4'd6;  // send high nibble hex char
	localparam S_SEND_LO     = 4'd7;  // send low nibble hex char
	localparam S_SEND_CR     = 4'd8;
	localparam S_SEND_LF     = 4'd9;

	reg [3:0] state;
	reg [3:0] count;       // number of data bytes stored so far
	reg [3:0] send_idx;    // index while sending data
	reg [3:0] prefix_idx;  // index while sending "FPGA: 0x" prefix
	reg [3:0] return_state; // state to return to after echo
	reg [7:0] cur_byte;   // latched current byte for hex output

	// buffer for up to 8 received bytes (stored already inverted)
	reg [7:0] buffer_0, buffer_1, buffer_2, buffer_3;
	reg [7:0] buffer_4, buffer_5, buffer_6, buffer_7;

	// echo support
	reg [7:0] echo_byte;

	reg led_r_reg;
	assign led_r = led_r_reg;

	// helper to read from buffer by index
	function [7:0] buffer_read;
		input [3:0] idx;
		begin
			case (idx)
				0: buffer_read = buffer_0;
				1: buffer_read = buffer_1;
				2: buffer_read = buffer_2;
				3: buffer_read = buffer_3;
				4: buffer_read = buffer_4;
				5: buffer_read = buffer_5;
				6: buffer_read = buffer_6;
				7: buffer_read = buffer_7;
				default: buffer_read = 8'h00;
			endcase
		end
	endfunction

	always @(posedge clk_48) begin
		uart_txd_strobe <= 0;
		led_r_reg <= 1; // LED off (active low)

		if (reset) begin
			state <= S_IDLE;
			count <= 0;
		end else begin
			case (state)

			// ---- Wait for 'N' to start a command ----
			S_IDLE: begin
				if (uart_rxd_strobe) begin
					// echo every character
					echo_byte <= uart_rxd;
					return_state <= (uart_rxd == "N") ? S_WAIT_SPACE : S_IDLE;
					state <= S_ECHO;
					if (uart_rxd == "N")
						led_r_reg <= 0;
				end
			end

			// ---- Expect a space after 'N' ----
			S_WAIT_SPACE: begin
				if (uart_rxd_strobe) begin
					echo_byte <= uart_rxd;
					if (uart_rxd == " ") begin
						return_state <= S_READ_DATA;
						count <= 0;
					end else begin
						return_state <= S_IDLE; // unexpected, abort
					end
					state <= S_ECHO;
				end
			end

			// ---- Read data bytes until \r or \n ----
			S_READ_DATA: begin
				led_r_reg <= 0;
				if (uart_rxd_strobe) begin
					if (uart_rxd == 8'h0D || uart_rxd == 8'h0A) begin
						// newline terminates input -> send response
						// echo the newline
						echo_byte <= uart_rxd;
						return_state <= S_SEND_PREFIX;
						state <= S_ECHO;
						prefix_idx <= 0;
						send_idx <= 0;
					end else if (count < 8) begin
						// store inverted byte in buffer
						case (count)
							0: buffer_0 <= ~uart_rxd;
							1: buffer_1 <= ~uart_rxd;
							2: buffer_2 <= ~uart_rxd;
							3: buffer_3 <= ~uart_rxd;
							4: buffer_4 <= ~uart_rxd;
							5: buffer_5 <= ~uart_rxd;
							6: buffer_6 <= ~uart_rxd;
							7: buffer_7 <= ~uart_rxd;
						endcase
						count <= count + 1;
						// echo the character
						echo_byte <= uart_rxd;
						return_state <= S_READ_DATA;
						state <= S_ECHO;
					end
					// if count >= 15, silently drop (buffer full)
				end
			end

			// ---- Echo a received character, then go to return_state ----
			S_ECHO: begin
				if (uart_txd_ready && !uart_txd_strobe) begin
					uart_txd_strobe <= 1;
					uart_txd <= echo_byte;
					state <= return_state;
				end
			end

			// ---- Send "FPGA: 0x" prefix (8 chars) ----
			S_SEND_PREFIX: begin
				led_r_reg <= 0;
				if (uart_txd_ready && !uart_txd_strobe) begin
					uart_txd_strobe <= 1;
					case (prefix_idx)
						0: uart_txd <= "F";
						1: uart_txd <= "P";
						2: uart_txd <= "G";
						3: uart_txd <= "A";
						4: uart_txd <= ":";
						5: uart_txd <= " ";
						6: uart_txd <= "0";
						7: uart_txd <= "x";
					endcase
					if (prefix_idx == 7) begin
						if (count == 0)
							state <= S_SEND_CR;
						else
							state <= S_LOAD_BYTE;
					end
					prefix_idx <= prefix_idx + 1;
				end
			end

			// ---- Latch buffer[send_idx] into cur_byte (1 cycle) ----
			S_LOAD_BYTE: begin
				led_r_reg <= 0;
				cur_byte <= buffer_read(send_idx);
				state <= S_SEND_HI;
			end

			// ---- Send high nibble of cur_byte as hex ----
			S_SEND_HI: begin
				led_r_reg <= 0;
				if (uart_txd_ready && !uart_txd_strobe) begin
					uart_txd_strobe <= 1;
					uart_txd <= hexdigit(cur_byte[7:4]);
					state <= S_SEND_LO;
				end
			end

			// ---- Send low nibble of cur_byte as hex, advance ----
			S_SEND_LO: begin
				led_r_reg <= 0;
				if (uart_txd_ready && !uart_txd_strobe) begin
					uart_txd_strobe <= 1;
					uart_txd <= hexdigit(cur_byte[3:0]);
					send_idx <= send_idx + 1;
					if (send_idx + 1 == count)
						state <= S_SEND_CR;
					else
						state <= S_LOAD_BYTE;
				end
			end

			// ---- Send carriage return ----
			S_SEND_CR: begin
				if (uart_txd_ready && !uart_txd_strobe) begin
					uart_txd_strobe <= 1;
					uart_txd <= "\r";
					state <= S_SEND_LF;
				end
			end

			// ---- Send line feed, return to idle ----
			S_SEND_LF: begin
				if (uart_txd_ready && !uart_txd_strobe) begin
					uart_txd_strobe <= 1;
					uart_txd <= "\n";
					state <= S_IDLE;
				end
			end

			default: state <= S_IDLE;
			endcase
		end
	end
endmodule
