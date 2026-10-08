`timescale 1ns/1ps

// ---------------------------------------------------------------------------
// Behavioral stand-ins for the iCE40 hard primitives.  Same convention as
// i2c_sniffer_tb.v: there is no vendor simulation library in this repo, so
// the testbench models what it needs.
// ---------------------------------------------------------------------------

module SB_HFOSC (
	input  wire CLKHFPU,
	input  wire CLKHFEN,
	output reg  CLKHF
);
	initial CLKHF = 1'b0;
	always begin
		#10.416667;                     // 48 MHz
		if (CLKHFPU && CLKHFEN)
			CLKHF = ~CLKHF;
		else
			CLKHF = 1'b0;
	end
endmodule


// ---------------------------------------------------------------------------
// Testbench
// ---------------------------------------------------------------------------
module spi_hw_stream_tb;

	reg  spi_sck  = 1'b0;
	reg  spi_cs   = 1'b1;
	reg  spi_mosi = 1'b0;
	wire spi_miso;
	wire spi_cs_flash;
	wire led_r, led_g, led_b;

	integer errors = 0;

	localparam integer HALF = 250;   // ns -> 2 MHz SCK

	// The expected message is spelled out here independently of the DUT
	// so the check is a real cross-check, not a round-trip.  Tick timing is
	// pulled from the DUT so the waits track DATA_RATE_HZ.
	localparam integer         MSG_LEN = 12;
	localparam [8*MSG_LEN-1:0] MSG     = "Hello World!";
	integer                    TICK_NS = 1_000_000_000 / dut.DATA_RATE_HZ;

	function [7:0] msg_char(input integer i);
		msg_char = MSG[8*(MSG_LEN-1-(i % MSG_LEN)) +: 8];
	endfunction

	// Position in the endless "Hello World!Hello World!..." stream that the
	// next byte off the link must correspond to.  Advances with every byte
	// read, across bursts — the stream must stay continuous through CS
	// toggles and through a FIFO-full stall.
	integer stream_pos = 0;

	top dut (
		.spi_sck(spi_sck),
		.spi_cs(spi_cs),
		.spi_mosi(spi_mosi),
		.spi_miso(spi_miso),
		.spi_cs_flash(spi_cs_flash),
		.led_r(led_r), .led_g(led_g), .led_b(led_b)
	);

	`include "../common/spi_raw_stream_tb_tasks.vh"

	task reset_pattern;
		begin
			@(negedge dut.clk_core);
			dut.msg_idx = 0;
		end
	endtask

	// Every byte in the burst must be the next character of the message,
	// continuing from wherever the stream left off.
	task check_stream(input integer n, input [127:0] label);
		integer i;
		reg [7:0] expect_next;
		begin
			for (i = 0; i < n; i = i + 1) begin
				expect_next = msg_char(stream_pos);
				if (burst[i] !== expect_next) begin
					$display("FAIL [%0s]: byte %0d = 0x%02x '%c', expected 0x%02x '%c' (stream pos %0d)",
					         label, i, burst[i], burst[i], expect_next, expect_next, stream_pos);
					errors = errors + 1;
				end
				stream_pos = stream_pos + 1;
			end
		end
	endtask

	task dump_burst(input integer n, input [127:0] label);
		integer i;
		begin
			$write("  %0s:", label);
			for (i = 0; i < n; i = i + 1) $write(" %02x", burst[i]);
			$write("   \"");
			for (i = 0; i < n; i = i + 1) $write("%c", burst[i]);
			$write("\"\n");
		end
	endtask

	// Count source stalls straight off the design's internal strobe.
	integer stall_count = 0;
	integer stalls_before = 0;
	integer msg_idx_before_stall;
	always @(posedge dut.clk_core)
		if (dut.source_stalled) stall_count = stall_count + 1;

	initial begin
		if ($test$plusargs("vcd")) begin
			$dumpfile("spi_hw_stream_tb.vcd");
			$dumpvars(0, spi_hw_stream_tb);
		end

		$display("=== spi_hw_stream testbench ===");

		// ------------------------------------------------------------
		// Let the IP get configured, then let the FIFO build a backlog
		// that is well short of full (FIFO_DEPTH ticks to fill).
		// ------------------------------------------------------------
		#5000;
		spi_burst(16);
		check_empty_stream(16);
		#(20 * TICK_NS);   // ~20 characters queued, no drops yet

		if (led_g !== 1'b0) begin
			$display("FAIL: green LED not lit — hard SPI IP never finished configuring");
			errors = errors + 1;
		end

		// ------------------------------------------------------------
		// Test 1: with backlog available and no drops, the master must
		// see the message from its first character, "Hello World!Hell".
		// ------------------------------------------------------------
		spi_burst(16);
		dump_burst(16, "burst 1");
		if (burst[0] !== "H") begin
			$display("FAIL: first streamed byte = 0x%02x, expected 'H'", burst[0]);
			errors = errors + 1;
		end
		check_stream(16, "burst 1");

		// ------------------------------------------------------------
		// Test 2: stop reading long enough for the FIFO to fill and stall the
		// synthetic source (FIFO_DEPTH ticks to fill, then some margin).
		//
		// The source must stop advancing while full. Otherwise this test
		// generator creates the same message gaps it is intended to detect.
		// ------------------------------------------------------------
		stalls_before = stall_count;
		#((dut.FIFO_DEPTH + 16) * TICK_NS);   // full FIFO plus 16 blocked ticks

		if (dut.fifo_count !== dut.FIFO_DEPTH) begin
			$display("FAIL: FIFO not full after stall (count = %0d, expected %0d)",
			         dut.fifo_count, dut.FIFO_DEPTH);
			errors = errors + 1;
		end
		if (stall_count <= stalls_before) begin
			$display("FAIL: source was not stalled while the FIFO was full");
			errors = errors + 1;
		end
		if (led_r !== 1'b0) begin
			$display("FAIL: red LED not lit — FIFO backpressure never signalled");
			errors = errors + 1;
		end
		msg_idx_before_stall = dut.msg_idx;
		#(8 * TICK_NS);
		if (dut.msg_idx !== msg_idx_before_stall) begin
			$display("FAIL: message source advanced while FIFO was full (%0d -> %0d)",
			         msg_idx_before_stall, dut.msg_idx);
			errors = errors + 1;
		end
		$display("  stall window: %0d blocked characters, fifo_count = %0d",
		         stall_count - stalls_before, dut.fifo_count);

		commands_on_mosi = 1;
		spi_burst(16);
		dump_burst(16, "burst 2");

		// Buffered data must survive backpressure intact and pick up exactly
		// where burst 1 stopped. A full FIFO must never corrupt or reorder
		// what is already queued.
		check_stream(16, "burst 2");

		// ...and the master should still be reading well behind the live
		// source, which is what "the master is too slow" actually looks like:
		// the FIFO was full, we took 16, so most of the backlog remains.
		if (dut.fifo_count < dut.FIFO_DEPTH - 16) begin
			$display("FAIL: expected a backlog after the stall, fifo_count = %0d",
			         dut.fifo_count);
			errors = errors + 1;
		end else begin
			$display("  master is %0d characters behind the live source", dut.fifo_count);
		end

		// ------------------------------------------------------------
		// Test 3: the stream keeps running after CS toggles, i.e. a new
		// transaction picks up where the previous one left off.
		// ------------------------------------------------------------
		spi_burst(8);
		dump_burst(8, "burst 3");
		check_stream(8, "burst 3");
		check_binary_stream;
		check_byte_boundaries;
		check_poll_stream(0);

		#1000;
		if (errors == 0)
			$display("PASS: all spi_hw_stream tests completed successfully");
		else
			$fatal(1, "FAIL: %0d error(s)", errors);
		$finish;
	end

	// safety net
	initial begin
		#((dut.FIFO_DEPTH * 4) * TICK_NS + 200_000_000);
		$fatal(1, "testbench timeout");
	end

endmodule
