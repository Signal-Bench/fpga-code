/** \file
 * Host-to-SPI bridge with local echo.
 *
 * Type text in picocom; each character is echoed back immediately.
 * When you press Enter (\r or \n), the buffered text (up to 16 bytes)
 * is sent out over SPI as a master, clocking one byte at a time.
 * The bytes received on MISO during the transaction are then sent
 * back to the host over UART, followed by \r\n.
 *
 * SPI Mode 0 (CPOL=0, CPHA=0), MSB-first, ~1 MHz clock.
 * Directly bit-bangs the SPI signals, adapted from the ice40
 * ultraplus spi_slave.v approach of edge-based shifting.
 *
 * LED status (active-low accent on UPduino):
 *   GREEN  = idle, ready for input
 *   BLUE   = has buffered data (typing in progress)
 *   RED    = SPI transaction active / sending UART response
 *
 * Pin assignments (directly named after PCF gpio names):
 *   gpio_11  -> SPI SCK   (output)
 *   gpio_21  -> SPI MOSI  (output)
 *   gpio_18  -> SPI MOSI2 (output, duplicate of MOSI)
 *   gpio_13  -> SPI MISO  (input)
 *   gpio_19  -> SPI CS    (output, active low)
 *   spi_cs   -> pin 16, held high to disable onboard SPI flash
 */
`include "../common/util.v"
`include "../common/uart.v"

module top(
	output led_r,
	output led_g,
	output led_b,
	output serial_txd,
	input  serial_rxd,
	output spi_cs,       // pin 16 – onboard flash disable

	// SPI master signals on GPIO header
	output gpio_11,      // SCK
	output gpio_21,      // MOSI
	output gpio_18,      // MOSI2 (duplicate)
	input  gpio_13,      // MISO
	output gpio_19       // CS (active low)
);
	// ---- Keep onboard SPI flash disabled ----
	assign spi_cs = 1;

	// ---- 48 MHz internal oscillator ----
	wire clk_48;
	wire reset = 0;
	SB_HFOSC u_hfosc (
		.CLKHFPU(1'b1),
		.CLKHFEN(1'b1),
		.CLKHF(clk_48)
	);

	// ---- LEDs (active-low: 0 = on, 1 = off) ----
	reg led_r_reg;
	reg led_g_reg;
	reg led_b_reg;
	assign led_r = led_r_reg;
	assign led_g = led_g_reg;
	assign led_b = led_b_reg;

	// ---- UART baud clocks ----
	wire clk_1;  // 1 MHz  for TX (baud_x1)
	wire clk_4;  // 4 MHz  for RX (baud_x4)
	divide_by_n #(.N(48)) div_tx(clk_48, reset, clk_1);
	divide_by_n #(.N(12)) div_rx(clk_48, reset, clk_4);

	// ---- UART TX ----
	reg  [7:0] uart_txd;
	reg        uart_txd_strobe;
	wire       uart_txd_ready;

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
	wire       uart_rxd_strobe;

	uart_rx rxd(
		.mclk(clk_48),
		.reset(reset),
		.baud_x4(clk_4),
		.serial(serial_rxd),
		.data(uart_rxd),
		.data_strobe(uart_rxd_strobe)
	);

	// ---- SPI output registers ----
	reg spi_cs_out;   // directly drives gpio_19 (active low)
	reg spi_sck_out;  // directly drives gpio_12
	reg spi_mosi_out; // directly drives gpio_21 & gpio_18

	assign gpio_19 = spi_cs_out;
	assign gpio_11 = spi_sck_out;
	assign gpio_21 = spi_mosi_out;
	assign gpio_18 = spi_mosi_out;  // MOSI duplicate

	// ---- SPI clock divider (~1 MHz from 48 MHz) ----
	// Divide 48 MHz by 24 => 2 MHz tick, giving a 1 MHz SCK.
	reg [4:0] spi_div;
	wire spi_tick = (spi_div == 23);

	// ---- Data buffers (16 bytes, explicit registers) ----
	reg [7:0] tx_buf_0,  tx_buf_1,  tx_buf_2,  tx_buf_3;
	reg [7:0] tx_buf_4,  tx_buf_5,  tx_buf_6,  tx_buf_7;
	reg [7:0] tx_buf_8,  tx_buf_9,  tx_buf_10, tx_buf_11;
	reg [7:0] tx_buf_12, tx_buf_13, tx_buf_14, tx_buf_15;
	reg [3:0] tx_count;

	reg [7:0] rx_buf_0,  rx_buf_1,  rx_buf_2,  rx_buf_3;
	reg [7:0] rx_buf_4,  rx_buf_5,  rx_buf_6,  rx_buf_7;
	reg [7:0] rx_buf_8,  rx_buf_9,  rx_buf_10, rx_buf_11;
	reg [7:0] rx_buf_12, rx_buf_13, rx_buf_14, rx_buf_15;

	// Helper functions for buffer access
	function [7:0] tx_read;
		input [3:0] idx;
		begin
			case (idx)
				0:  tx_read = tx_buf_0;
				1:  tx_read = tx_buf_1;
				2:  tx_read = tx_buf_2;
				3:  tx_read = tx_buf_3;
				4:  tx_read = tx_buf_4;
				5:  tx_read = tx_buf_5;
				6:  tx_read = tx_buf_6;
				7:  tx_read = tx_buf_7;
				8:  tx_read = tx_buf_8;
				9:  tx_read = tx_buf_9;
				10: tx_read = tx_buf_10;
				11: tx_read = tx_buf_11;
				12: tx_read = tx_buf_12;
				13: tx_read = tx_buf_13;
				14: tx_read = tx_buf_14;
				15: tx_read = tx_buf_15;
			endcase
		end
	endfunction

	function [7:0] rx_read;
		input [3:0] idx;
		begin
			case (idx)
				0:  rx_read = rx_buf_0;
				1:  rx_read = rx_buf_1;
				2:  rx_read = rx_buf_2;
				3:  rx_read = rx_buf_3;
				4:  rx_read = rx_buf_4;
				5:  rx_read = rx_buf_5;
				6:  rx_read = rx_buf_6;
				7:  rx_read = rx_buf_7;
				8:  rx_read = rx_buf_8;
				9:  rx_read = rx_buf_9;
				10: rx_read = rx_buf_10;
				11: rx_read = rx_buf_11;
				12: rx_read = rx_buf_12;
				13: rx_read = rx_buf_13;
				14: rx_read = rx_buf_14;
				15: rx_read = rx_buf_15;
			endcase
		end
	endfunction

	// ---- State machine ----
	localparam S_IDLE          = 4'd0;
	localparam S_ECHO          = 4'd1;
	localparam S_SPI_CS_LOW    = 4'd2;
	localparam S_SPI_LOAD_BYTE = 4'd3;  // latch byte from buffer into shift reg
	localparam S_SPI_CLKING    = 4'd4;  // two-phase SCK: rising sets MOSI+SCK, falling samples MISO
	localparam S_SPI_NEXT_BIT  = 4'd5;  // advance bit or byte
	localparam S_SPI_CS_HIGH   = 4'd6;
	localparam S_TX_LOAD_BYTE  = 4'd7;  // latch rx byte for UART send
	localparam S_TX_RESPONSE   = 4'd8;
	localparam S_TX_CR         = 4'd9;
	localparam S_TX_LF         = 4'd10;

	reg [3:0] state;
	reg [3:0] return_state;
	reg [7:0] echo_byte;

	reg [3:0] spi_byte_idx;
	reg [2:0] spi_bit_idx;
	reg [7:0] spi_shift_out;
	reg [7:0] spi_shift_in;
	reg       spi_phase;      // 0 = next tick is rising, 1 = falling
	reg [3:0] uart_send_idx;
	reg [7:0] cur_tx_byte;   // latched from rx_buf for UART response

	// ---- Pipeline register for UART RX (breaks critical path) ----
	reg       rx_valid;
	reg [7:0] rx_data;

	// ---- Main logic ----
	always @(posedge clk_48) begin
		uart_txd_strobe <= 0;

		// Pipeline: latch UART RX data one cycle early
		rx_valid <= uart_rxd_strobe;
		rx_data  <= uart_rxd;

		// Default LED state: all off
		led_r_reg <= 1;
		led_g_reg <= 1;
		led_b_reg <= 1;

		if (reset) begin
			state       <= S_IDLE;
			tx_count    <= 0;
			spi_cs_out  <= 1;
			spi_sck_out <= 0;
			spi_mosi_out<= 0;
			spi_div     <= 0;
			rx_valid    <= 0;
		end else begin

			// Free-running SPI divider
			if (spi_tick)
				spi_div <= 0;
			else
				spi_div <= spi_div + 1;

			case (state)

			// ============================================================
			// IDLE: buffer typed characters, trigger SPI on Enter
			// ============================================================
			S_IDLE: begin
				if (tx_count != 0)
					led_b_reg <= 0;  // blue: has buffered data
				else
					led_g_reg <= 0;  // green: idle, ready

				if (rx_valid) begin
					if (rx_data == 8'h0D || rx_data == 8'h0A) begin
						echo_byte    <= rx_data;
						return_state <= (tx_count != 0) ? S_SPI_CS_LOW : S_IDLE;
						state        <= S_ECHO;
					end else if (tx_count < 16) begin
						case (tx_count)
							0:  tx_buf_0  <= rx_data;
							1:  tx_buf_1  <= rx_data;
							2:  tx_buf_2  <= rx_data;
							3:  tx_buf_3  <= rx_data;
							4:  tx_buf_4  <= rx_data;
							5:  tx_buf_5  <= rx_data;
							6:  tx_buf_6  <= rx_data;
							7:  tx_buf_7  <= rx_data;
							8:  tx_buf_8  <= rx_data;
							9:  tx_buf_9  <= rx_data;
							10: tx_buf_10 <= rx_data;
							11: tx_buf_11 <= rx_data;
							12: tx_buf_12 <= rx_data;
							13: tx_buf_13 <= rx_data;
							14: tx_buf_14 <= rx_data;
							15: tx_buf_15 <= rx_data;
						endcase
						tx_count     <= tx_count + 1;
						echo_byte    <= rx_data;
						return_state <= S_IDLE;
						state        <= S_ECHO;
					end
				end
			end

			// ============================================================
			// ECHO: send one character back to picocom
			// ============================================================
			S_ECHO: begin
				if (uart_txd_ready && !uart_txd_strobe) begin
					uart_txd        <= echo_byte;
					uart_txd_strobe <= 1;
					state           <= return_state;
				end
			end

			// ============================================================
			// SPI_CS_LOW: pull CS low, prepare first byte
			// ============================================================
			S_SPI_CS_LOW: begin
				led_r_reg <= 0;  // red: SPI active
				if (spi_tick) begin
					spi_cs_out   <= 0;
					spi_byte_idx <= 0;
					spi_sck_out  <= 0;
					state        <= S_SPI_LOAD_BYTE;
				end
			end

			// ============================================================
			// SPI_LOAD_BYTE: latch tx_buf[spi_byte_idx], prepare for clocking
			// ============================================================
			S_SPI_LOAD_BYTE: begin
				led_r_reg     <= 0;  // red: SPI active
				spi_shift_out <= tx_read(spi_byte_idx);
				spi_shift_in  <= 0;
				spi_bit_idx   <= 7;
				spi_phase     <= 0;
				spi_sck_out   <= 0;
				state         <= S_SPI_CLKING;
			end

			// ============================================================
			// SPI_CLKING: two-phase SCK, matching clock_out.v pattern
			//   phase 0 (rising): SCK high + set MOSI to current bit
			//   phase 1 (falling): SCK low, sample MISO, shift data
			// ============================================================
			S_SPI_CLKING: begin
				led_r_reg <= 0;  // red: SPI active
				if (spi_tick) begin
					if (spi_phase == 0) begin
						// Rising edge — SCK high, set MOSI simultaneously
						spi_sck_out  <= 1;
						spi_mosi_out <= spi_shift_out[7];
						spi_phase    <= 1;
					end else begin
						// Falling edge — SCK low, sample MISO, shift
						spi_sck_out   <= 0;
						spi_shift_in  <= {spi_shift_in[6:0], gpio_13};
						spi_shift_out <= {spi_shift_out[6:0], 1'b0};
						spi_phase     <= 0;
						state         <= S_SPI_NEXT_BIT;
					end
				end
			end

			// ============================================================
			// SPI_NEXT_BIT: advance bit or byte
			// ============================================================
			S_SPI_NEXT_BIT: begin
				led_r_reg <= 0;  // red: SPI active
				if (spi_bit_idx == 0) begin
					// Byte complete: store received byte
					case (spi_byte_idx)
						0:  rx_buf_0  <= spi_shift_in;
						1:  rx_buf_1  <= spi_shift_in;
						2:  rx_buf_2  <= spi_shift_in;
						3:  rx_buf_3  <= spi_shift_in;
						4:  rx_buf_4  <= spi_shift_in;
						5:  rx_buf_5  <= spi_shift_in;
						6:  rx_buf_6  <= spi_shift_in;
						7:  rx_buf_7  <= spi_shift_in;
						8:  rx_buf_8  <= spi_shift_in;
						9:  rx_buf_9  <= spi_shift_in;
						10: rx_buf_10 <= spi_shift_in;
						11: rx_buf_11 <= spi_shift_in;
						12: rx_buf_12 <= spi_shift_in;
						13: rx_buf_13 <= spi_shift_in;
						14: rx_buf_14 <= spi_shift_in;
						15: rx_buf_15 <= spi_shift_in;
					endcase
					if (spi_byte_idx + 1 == tx_count) begin
						state <= S_SPI_CS_HIGH;
					end else begin
						spi_byte_idx <= spi_byte_idx + 1;
						state        <= S_SPI_LOAD_BYTE;
					end
				end else begin
					spi_bit_idx <= spi_bit_idx - 1;
					state       <= S_SPI_CLKING;
				end
			end

			// ============================================================
			// SPI_CS_HIGH: deassert CS
			// ============================================================
			S_SPI_CS_HIGH: begin
				if (spi_tick) begin
					spi_cs_out    <= 1;
					spi_sck_out   <= 0;
					spi_mosi_out  <= 0;
					uart_send_idx <= 0;
					state         <= S_TX_LOAD_BYTE;
				end
			end

			// ============================================================
			// TX_LOAD_BYTE: latch rx_buf[uart_send_idx] (1 cycle)
			// ============================================================
			S_TX_LOAD_BYTE: begin
				led_r_reg <= 0;  // red: sending response
				led_b_reg <= 0;  // blue: UART TX active
				if (uart_send_idx == tx_count) begin
					state <= S_TX_CR;
				end else begin
					cur_tx_byte <= rx_read(uart_send_idx);
					state       <= S_TX_RESPONSE;
				end
			end

			// ============================================================
			// TX_RESPONSE: send latched byte over UART
			// ============================================================
			S_TX_RESPONSE: begin
				led_r_reg <= 0;  // red: sending response
				led_b_reg <= 0;  // blue: UART TX active
				if (uart_txd_ready && !uart_txd_strobe) begin
					uart_txd        <= cur_tx_byte;
					uart_txd_strobe <= 1;
					uart_send_idx   <= uart_send_idx + 1;
					state           <= S_TX_LOAD_BYTE;
				end
			end

			// ============================================================
			// TX_CR: send carriage return
			// ============================================================
			S_TX_CR: begin
				led_r_reg <= 0;  // red: sending response
				if (uart_txd_ready && !uart_txd_strobe) begin
					uart_txd        <= 8'h0D;
					uart_txd_strobe <= 1;
					state           <= S_TX_LF;
				end
			end

			// ============================================================
			// TX_LF: send line feed, reset, back to IDLE
			// ============================================================
			S_TX_LF: begin
				led_r_reg <= 0;  // red: sending response
				if (uart_txd_ready && !uart_txd_strobe) begin
					uart_txd        <= 8'h0A;
					uart_txd_strobe <= 1;
					tx_count        <= 0;
					state           <= S_IDLE;
				end
			end

			default: state <= S_IDLE;

			endcase
		end
	end
endmodule
