/**
 * clock_test.v
 *
 * Simple test design: outputs a ~1 MHz clock on gpio_11
 * using the iCE40 UP5K internal 48 MHz HFOSC.
 */

module top(
	output gpio_11,  // 1 MHz clock output
	output spi_cs    // Onboard flash disable (active low)
);

	// Disable onboard SPI flash so we can use the pins freely
	assign spi_cs = 1'b1;

	//---------------------------------------------------------
	// 48 MHz Clock Generation (Internal HFOSC)
	//---------------------------------------------------------
	wire clk_48;

	SB_HFOSC u_hfosc (
		.CLKHFPU(1'b1),
		.CLKHFEN(1'b1),
		.CLKHF(clk_48)
	);

	//---------------------------------------------------------
	// Clock Divider: 48 MHz -> ~1 MHz
	//---------------------------------------------------------
	// 48 MHz / (2 * 24) = 1 MHz (approx. 50% duty cycle)
	reg [4:0] div = 5'd0;
	reg       clk_out = 1'b0;

	always @(posedge clk_48) begin
		if (div == 5'd23) begin
			div     <= 5'd0;
			clk_out <= ~clk_out;
		end else begin
			div <= div + 5'd1;
		end
	end

	assign gpio_11 = clk_out;

endmodule
