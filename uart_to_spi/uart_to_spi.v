/** \file
 * UART-to-SPI Sniffer Bridge.
 * Sniffs two UART lines (TX and RX of a target device) and sends
 * the data sequentially over SPI with a header and flag.
 *
 * Format: [0x48 Header] [0x01 (TX) or 0x02 (RX)] [Data Byte]
 *
 * Debug LEDs (accent low accent active):
 *   RED:   Pulses when UART byte received
 *   GREEN: On when SPI state machine is active (not idle)
 *   BLUE:  Directly shows SCK output
 */
`include "../common/util.v"
`include "../common/uart.v"

module top(
	output gpio_11,      // SPI SCK   (output)
	output gpio_21,      // SPI MOSI  (output)
	output gpio_19,      // SPI CS    (output, active low)
	input  gpio_38,      // UART TX input (sniff)
	input  gpio_42,      // UART RX input (sniff)
	output spi_cs,       // Onboard flash disable
	output led_r,        // Debug: UART received (active low)
	output led_g,        // Debug: SPI active (active low)
	output led_b         // Debug: SCK activity (active low)
);
	assign spi_cs = 1;

	//---------------------------------------------------------
	// 48 MHz Clock Generation (Internal HFOSC)
	//---------------------------------------------------------
	wire clk_48;
	wire reset = 0;

	SB_HFOSC u_hfosc (
		.CLKHFPU(1'b1),
		.CLKHFEN(1'b1),
		.CLKHF(clk_48)
	);

	//---------------------------------------------------------
	// UART Receivers (9600 Baud)
	//---------------------------------------------------------
	// 48 MHz / (9600 * 4) = 1250
	wire clk_baud_x4;
	divide_by_n #(.N(1250)) div_rx(clk_48, reset, clk_baud_x4);

	wire [7:0] uart_tx_data;
	wire       uart_tx_strobe;
	uart_rx rx_tx_sniff(
		.mclk(clk_48),
		.reset(reset),
		.baud_x4(clk_baud_x4),
		.serial(gpio_38),
		.data(uart_tx_data),
		.data_strobe(uart_tx_strobe)
	);

	wire [7:0] uart_rx_data;
	wire       uart_rx_strobe;
	uart_rx rx_rx_sniff(
		.mclk(clk_48),
		.reset(reset),
		.baud_x4(clk_baud_x4),
		.serial(gpio_42),
		.data(uart_rx_data),
		.data_strobe(uart_rx_strobe)
	);

	//---------------------------------------------------------
	// Debug LED: Pulse stretcher for UART receive indication
	//---------------------------------------------------------
	// Stretch the single-cycle strobe to ~10ms so it's visible
	// 48MHz * 0.01s = 480,000 cycles, use 19-bit counter
	reg [18:0] uart_led_counter = 0;
	wire uart_received = uart_tx_strobe | uart_rx_strobe;
	
	always @(posedge clk_48) begin
		if (uart_received)
			uart_led_counter <= 19'd480000;
		else if (uart_led_counter != 0)
			uart_led_counter <= uart_led_counter - 1;
	end
	
	wire uart_led_on = (uart_led_counter != 0);

	//---------------------------------------------------------
	// SPI Transmission Logic (adapted from clock_out.v)
	//---------------------------------------------------------
	// SPI clock divider (~1 MHz from 48 MHz)
	reg [4:0] div = 0;
	wire tick = (div == 23);
	always @(posedge clk_48) begin
		if (tick) div <= 0;
		else div <= div + 1;
	end

	// Small FIFO/Buffer to handle simultaneous RX/TX
	reg [7:0] pending_data = 0;
	reg [7:0] pending_flag = 0;
	reg       has_pending  = 0;

	// State machine (matching clock_out.v structure)
	localparam S_IDLE      = 3'd0;
	localparam S_CS_LOW    = 3'd1;
	localparam S_CLKING    = 3'd2;
	localparam S_NEXT_BYTE = 3'd3;
	localparam S_CS_HIGH   = 3'd4;

	reg [2:0] state = S_IDLE;
	reg [2:0] bit_idx = 0;
	reg [1:0] byte_idx = 0;
	reg [7:0] shift_reg = 0;
	
	reg [7:0] out_byte_2 = 0;
	reg [7:0] out_byte_3 = 0;

	reg       sck_out = 0;
	reg       mosi_out = 0;
	reg       cs_out = 1;
	reg       sck_phase = 0;

	always @(posedge clk_48) begin
		// Capture incoming UART data to pending buffer if busy
		if (uart_tx_strobe && state != S_IDLE) begin
			has_pending   <= 1;
			pending_data  <= uart_tx_data;
			pending_flag  <= 8'h01;
		end else if (uart_rx_strobe && state != S_IDLE) begin
			has_pending   <= 1;
			pending_data  <= uart_rx_data;
			pending_flag  <= 8'h02;
		end

		if (state == S_IDLE) begin
			cs_out   <= 1;
			sck_out  <= 0;
			mosi_out <= 0;
			
			if (uart_tx_strobe && uart_rx_strobe) begin
				shift_reg  <= 8'h48;  // Header byte
				out_byte_2 <= uart_tx_data;
				out_byte_3 <= uart_rx_data;
				byte_idx   <= 1;
				state      <= S_CS_LOW;
			end else if (uart_tx_strobe) begin
				shift_reg  <= 8'h48;  // Header byte
				out_byte_2 <= uart_tx_data;
				out_byte_3 <= 8'h00;  // Padding or previous state
				byte_idx   <= 1;
				state      <= S_CS_LOW;
			end else if (uart_rx_strobe) begin
				shift_reg  <= 8'h48;  // Header byte
				out_byte_2 <= 8'h00;  // Padding
				out_byte_3 <= uart_rx_data;
				byte_idx   <= 1;
				state      <= S_CS_LOW;
			end else if (has_pending) begin
				shift_reg  <= 8'h48;
				out_byte_2 <= (pending_flag == 8'h01) ? pending_data : 8'h00;
				out_byte_3 <= (pending_flag == 8'h02) ? pending_data : 8'h00;
				byte_idx   <= 1;
				has_pending <= 0;
				state      <= S_CS_LOW;
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
						if (bit_idx == 0) state <= S_NEXT_BYTE;
						else bit_idx <= bit_idx - 1;
					end
				end

				S_NEXT_BYTE: begin
					if (byte_idx < 3) begin
						byte_idx <= byte_idx + 1;
						// Load next byte into shift register
						if (byte_idx == 1) shift_reg <= out_byte_2;
						else if (byte_idx == 2) shift_reg <= out_byte_3;
						bit_idx   <= 7;
						sck_phase <= 0;
						state     <= S_CLKING;  // Continue clocking without CS toggle
					end else begin
						state <= S_CS_HIGH;
					end
				end

				S_CS_HIGH: begin
					cs_out <= 1;
					state  <= S_IDLE;
				end
			endcase
		end
	end

	//---------------------------------------------------------
	// Output Assignments
	//---------------------------------------------------------
	assign gpio_11 = sck_out;
	assign gpio_21 = mosi_out;
	assign gpio_19 = cs_out;
	
	// Debug LEDs (active low accent accent accent accent accent accent accent accent drive accent 0 to turn ON)
	assign led_r = ~uart_led_on;           // RED: pulses on UART receive
	assign led_g = ~(state != S_IDLE);     // GREEN: on when SPI active
	assign led_b = ~sck_out;               // BLUE: shows SCK directly
endmodule
