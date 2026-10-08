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
module spi_counter_stream_tb;

	reg  spi_sck  = 1'b0;
	reg  spi_cs   = 1'b1;
	reg  spi_mosi = 1'b0;
	wire spi_miso;
	wire spi_cs_flash;
	wire led_r, led_g, led_b;

	integer errors = 0;

	localparam integer HALF = 250;   // ns -> 2 MHz SCK

	// Every wait below is expressed in SAMPLE periods, derived from the DUT's
	// own TICK_DIV, so changing DATA_RATE_HZ in the design re-times the whole
	// testbench instead of silently invalidating it.  Hardcoded microsecond
	// waits were the old failure mode: they were written for a 375 kHz sample
	// rate and quietly tested nothing once the rate dropped.
	localparam real CORE_NS   = 1000.0 / 24.0;   // 24 MHz core clock
	localparam real CFG_NS    = 5_000.0;         // IP config takes << 1 us
	real SAMPLE_NS;                              // set at the top of the run

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
			dut.ascii_char = 8'h20;
		end
	endtask

	// Every byte in the burst must be exactly one more than the previous.
	task check_consecutive(input integer n, input [127:0] label);
		integer i;
		reg [7:0] expect_next;
		begin
			for (i = 1; i < n; i = i + 1) begin
				expect_next = burst[i-1] + 8'd1;
				if (burst[i] !== expect_next) begin
					$display("FAIL [%0s]: byte %0d = 0x%02x, expected 0x%02x (prev 0x%02x)",
					         label, i, burst[i], expect_next, burst[i-1]);
					errors = errors + 1;
				end
			end
		end
	endtask



	task check_ascii_sequence(input integer n);
		integer i;
		reg [7:0] expected;
		begin
			for (i = 1; i < n; i = i + 1) begin
				if (burst[i-1] == 8'h7E)
					expected = 8'h0A;
				else if (burst[i-1] == 8'h0A)
					expected = 8'h20;
				else
					expected = burst[i-1] + 1'b1;
				if (burst[i] !== expected) begin
					$display("FAIL [ASCII mode]: byte %0d = %02x, expected %02x", i, burst[i], expected);
					errors = errors + 1;
				end
			end
		end
	endtask

	task dump_burst(input integer n, input [127:0] label);
		integer i;
		begin
			$write("  %0s:", label);
			for (i = 0; i < n; i = i + 1) $write(" %02x", burst[i]);
			$write("\n");
		end
	endtask

	reg [7:0] last_of_burst1;
	integer   lag_samples;

	// Count source stalls straight off the design's internal strobe.
	integer stall_count = 0;
	integer stalls_before = 0;
	reg [7:0] counter_before_stall;
	always @(posedge dut.clk_core)
		if (dut.source_stalled) stall_count = stall_count + 1;

	// Total samples produced, in a width that does not wrap.  The design's own
	// ascii_char is 8 bits, so measuring how far the master has fallen behind
	// with it aliases as soon as the stall exceeds 256 samples — which is what
	// happens at high DATA_RATE_HZ.  Count ticks here instead.
	integer tick_count = 0;
	integer ticks_before = 0;
	always @(posedge dut.clk_core)
		if (dut.tick) tick_count = tick_count + 1;

	initial begin
		if ($test$plusargs("vcd")) begin
			$dumpfile("spi_counter_stream_tb.vcd");
			$dumpvars(0, spi_counter_stream_tb);
		end

		SAMPLE_NS = dut.TICK_DIV * CORE_NS;

		$display("=== spi_counter_stream testbench ===");
		$display("  TICK_DIV = %0d -> one sample every %0.3f us",
		         dut.TICK_DIV, SAMPLE_NS / 1000.0);

		// ------------------------------------------------------------
		// Let the IP get configured, then let the FIFO build a backlog
		// deep enough to serve a 16-byte burst but well short of the
		// 64-entry depth, so the source is not stalled yet.
		// ------------------------------------------------------------
		#(CFG_NS);
		spi_burst(16);
		check_empty_stream(16);
		#(24 * SAMPLE_NS);

		if (led_g !== 1'b0) begin
			$display("FAIL: green LED not lit — hard SPI IP never finished configuring");
			errors = errors + 1;
		end

		// ------------------------------------------------------------
		// Test 1: with backlog available and no drops, the master must
		// see a strictly incrementing sequence starting from ASCII space.
		// ------------------------------------------------------------
		spi_burst(16);
		dump_burst(16, "burst 1");
		if (burst[0] !== 8'h20) begin
			$display("FAIL: first streamed byte = 0x%02x, expected 0x20", burst[0]);
			errors = errors + 1;
		end
		check_consecutive(16, "burst 1");
		last_of_burst1 = burst[15];

		// ------------------------------------------------------------
		// Test 2: stop reading long enough for the FIFO to fill and stall the
		// synthetic source. FIFO_DEPTH samples fill it; the extra margin below
		// guarantees backpressure actually happens at any DATA_RATE_HZ.
		//
		// The source must stop advancing while full. Otherwise this test
		// generator creates the same counter gaps it is intended to detect.
		// ------------------------------------------------------------
		stalls_before = stall_count;
		ticks_before = tick_count;
		#(2 * dut.FIFO_DEPTH * SAMPLE_NS);   // no master activity

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
		counter_before_stall = dut.ascii_char;
		#(8 * SAMPLE_NS);
		if (dut.ascii_char !== counter_before_stall) begin
			$display("FAIL: counter advanced while FIFO was full (0x%02x -> 0x%02x)",
			         counter_before_stall, dut.ascii_char);
			errors = errors + 1;
		end
		$display("  stall window: %0d blocked samples, fifo_count = %0d",
		         stall_count - stalls_before, dut.fifo_count);

		commands_on_mosi = 1;
		spi_burst(16);
		dump_burst(16, "burst 2");

		// Buffered data must survive backpressure intact and pick up exactly
		// where burst 1 stopped. A full FIFO must never corrupt or reorder
		// what is already queued.
		check_consecutive(16, "burst 2");
		if (burst[0] !== last_of_burst1 + 8'd1) begin
			$display("FAIL: stream not continuous across the stall (0x%02x -> 0x%02x)",
			         last_of_burst1, burst[0]);
			errors = errors + 1;
		end

		// ...and the master should be reading data well behind the live
		// counter, which is what "the master is too slow" actually looks like.
		// Measured off the non-wrapping tick count, not the 8-bit source state.
		lag_samples = tick_count - ticks_before;
		if (lag_samples < dut.FIFO_DEPTH) begin
			$display("FAIL: expected the master to lag the live counter by at least %0d samples, got %0d",
			         dut.FIFO_DEPTH, lag_samples);
			errors = errors + 1;
		end else begin
			$display("  master is >= %0d samples behind the live counter (resuming at 0x%02x)",
			         lag_samples, burst[0]);
		end

		// ------------------------------------------------------------
		// Test 3: the stream keeps running after CS toggles, i.e. a new
		// transaction picks up where the previous one left off.
		// ------------------------------------------------------------
		spi_burst(8);
		dump_burst(8, "burst 3");
		check_consecutive(8, "burst 3");

		#(dut.FIFO_DEPTH * SAMPLE_NS);
		spi_burst(58);
		check_ascii_sequence(58);
		#(dut.FIFO_DEPTH * SAMPLE_NS);
		spi_burst(58);
		check_ascii_sequence(58);
		check_binary_stream;
		check_byte_boundaries;
		check_poll_stream(1);

		#1000;
		if (errors == 0)
			$display("PASS: all spi_counter_stream tests completed successfully");
		else
			$fatal(1, "FAIL: %0d error(s)", errors);
		$finish;
	end

	// safety net
	initial begin
		// Scales with the design's sample rate for the same reason the waits
		// above do; the fixed term covers the SPI bursts and config.
		#(400 * dut.TICK_DIV * (1000.0 / 24.0) + 200_000_000);
		$fatal(1, "testbench timeout");
	end

endmodule
