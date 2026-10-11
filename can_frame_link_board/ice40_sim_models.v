`timescale 1ns/1ps
/*
 * ice40_sim_models.v — SIMULATION ONLY stand-ins for the two iCE40 primitives
 * the design instantiates (same convention as the partner testbenches).
 *
 * SB_IO: only the two modes used here, with continuous assigns.  yosys's
 *   cells_sim.v model computes D_IN_0 and the pad output in `always @*`
 *   blocks, which iverilog does not evaluate at time 0, so a pin that never
 *   toggles reads X (found 2026-10-11: CAN idle-high read as dominant, first
 *   MISO bit X).  Behaviour is otherwise identical for these PIN_TYPEs:
 *     6'b0000_01  input, unregistered        D_IN_0 = pad (pull-up: z -> 1)
 *     6'b1010_01  output, unregistered, OE   pad = OE ? D_OUT_0 : z
 * SB_SPRAM256KA: behaviour copied from yosys techlibs/ice40/cells_sim.v (via
 *   the partner's spi_frame_stream_tb.v): registered read, one cycle latency,
 *   DATAOUT undefined after a write cycle.
 */
module SB_IO (
	inout  PACKAGE_PIN,
	input  LATCH_INPUT_VALUE,
	input  CLOCK_ENABLE,
	input  INPUT_CLK,
	input  OUTPUT_CLK,
	input  OUTPUT_ENABLE,
	input  D_OUT_0,
	input  D_OUT_1,
	output D_IN_0,
	output D_IN_1
);
	parameter [5:0] PIN_TYPE = 6'b000000;
	parameter [0:0] PULLUP = 1'b0;
	parameter [0:0] NEG_TRIGGER = 1'b0;
	parameter IO_STANDARD = "SB_LVCMOS";

	initial if (PIN_TYPE != 6'b0000_01 && PIN_TYPE != 6'b1010_01) begin
		$display("ice40_sim_models SB_IO: PIN_TYPE %b not modelled", PIN_TYPE);
		$finish;
	end
	generate
		if (PIN_TYPE[5:4] == 2'b10) assign PACKAGE_PIN = OUTPUT_ENABLE ? D_OUT_0 : 1'bz;
		if (PULLUP) assign (pull1, highz0) PACKAGE_PIN = 1'b1;
	endgenerate
	assign D_IN_0 = PACKAGE_PIN;
	assign D_IN_1 = 1'b0;
endmodule

module SB_SPRAM256KA (
	input [13:0] ADDRESS,
	input [15:0] DATAIN,
	input [3:0]  MASKWREN,
	input WREN, CHIPSELECT, CLOCK, STANDBY, SLEEP, POWEROFF,
	output reg [15:0] DATAOUT
);
	reg [15:0] mem [0:16383];
	wire off = SLEEP || !POWEROFF;

	always @(posedge CLOCK, posedge off) begin
		if (off) begin
			DATAOUT <= 0;
		end else if (STANDBY) begin
			DATAOUT <= 16'bx;
		end else if (CHIPSELECT) begin
			if (!WREN) begin
				DATAOUT <= mem[ADDRESS];
			end else begin
				if (MASKWREN[0]) mem[ADDRESS][ 3: 0] <= DATAIN[ 3: 0];
				if (MASKWREN[1]) mem[ADDRESS][ 7: 4] <= DATAIN[ 7: 4];
				if (MASKWREN[2]) mem[ADDRESS][11: 8] <= DATAIN[11: 8];
				if (MASKWREN[3]) mem[ADDRESS][15:12] <= DATAIN[15:12];
				DATAOUT <= 16'bx;
			end
		end
	end
endmodule
