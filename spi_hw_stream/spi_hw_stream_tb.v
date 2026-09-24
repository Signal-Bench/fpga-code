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

// Simplified SB_SPI model — enough of the register interface and slave shift
// engine to exercise the service state machine.  NOT silicon-accurate:
//   * MSB-first mode 0 only (LSBF/CPOL/CPHA bits are stored, not honoured)
//   * the SPI side is oversampled in the SBCLKI domain rather than being a
//     true asynchronous SCK domain
//   * transmit underrun shifts out 0xFF as a distinctive marker
//   * NO LEADING DUMMY BYTE.  Real hardware needs one: FPGA-TN-02011
//     Table 12.8 requires SPITXDR to be written >= 0.5 SCK periods before
//     the first bit appears on SO, so a slave read always starts with a
//     dummy.  This model hands over the first real byte immediately, so the
//     "burst starts at 'H'" checks below are one byte optimistic versus
//     silicon — on hardware, expect the message to begin on byte 2.
// It models the one behaviour this design depends on: a single-deep transmit
// holding register (SPITXDR) that feeds the shifter at each byte boundary,
// with TRDY/RRDY status bits driving the handshake.
module SB_SPI (
	input  SBCLKI, SBRWI, SBSTBI,
	input  SBADRI7, SBADRI6, SBADRI5, SBADRI4, SBADRI3, SBADRI2, SBADRI1, SBADRI0,
	input  SBDATI7, SBDATI6, SBDATI5, SBDATI4, SBDATI3, SBDATI2, SBDATI1, SBDATI0,
	input  MI, SI, SCKI, SCSNI,
	output SBDATO7, SBDATO6, SBDATO5, SBDATO4, SBDATO3, SBDATO2, SBDATO1, SBDATO0,
	output SBACKO, SPIIRQ, SPIWKUP,
	output SO, SOE, MO, MOE, SCKO, SCKOE,
	output MCSNO3, MCSNO2, MCSNO1, MCSNO0,
	output MCSNOE3, MCSNOE2, MCSNOE1, MCSNOE0
);
	parameter BUS_ADDR74 = "0b0000";

	reg [7:0] cr0 = 0, cr1 = 0, cr2 = 0, br = 0, csr = 0;
	reg [7:0] tx_data = 0;
	reg       tx_full = 0;
	reg [7:0] rx_data = 0;
	reg       rx_full = 0;
	reg [7:0] shifter = 8'hFF;
	reg       shifter_valid = 0;   // shifter holds a byte that has not been sent yet
	reg [7:0] rx_shift = 0;
	reg [2:0] bit_idx = 0;
	reg [7:0] dato_r = 0;
	reg       ack_r = 0;
	reg [1:0] sck_s = 2'b00;
	reg [1:0] cs_s  = 2'b11;

	wire [7:0] adr  = {SBADRI7, SBADRI6, SBADRI5, SBADRI4, SBADRI3, SBADRI2, SBADRI1, SBADRI0};
	wire [7:0] dati = {SBDATI7, SBDATI6, SBDATI5, SBDATI4, SBDATI3, SBDATI2, SBDATI1, SBDATI0};

	wire spe = cr1[7];
	// bit4 = TRDY (transmit register free), bit3 = RRDY (receive data ready)
	wire [7:0] status = {3'b000, ~tx_full, rx_full, 3'b000};

	wire sck_rise = (sck_s == 2'b01);
	wire sck_fall = (sck_s == 2'b10);
	wire cs_fall  = (cs_s  == 2'b10);
	wire cs_low   = ~cs_s[0];

	// bus events
	wire bus_active = SBSTBI && !ack_r;
	wire bus_txwr   = bus_active &&  SBRWI && (adr[3:0] == 4'hD);
	wire bus_rxrd   = bus_active && !SBRWI && (adr[3:0] == 4'hE);
	// shift-engine events.  A byte pre-loaded into the shifter survives CS
	// going high and is sent at the start of the next transaction, so no
	// sample is silently dropped at a transaction boundary.  (Whether the
	// real IP behaves this way is one of the things this test is meant to
	// find out on hardware — see the README.)
	wire eng_byte_done = spe && cs_low && sck_fall && (bit_idx == 3'd7);
	wire eng_cs_start  = spe && cs_fall && !shifter_valid;
	wire eng_load      = eng_byte_done || eng_cs_start;
	wire eng_rx_done   = spe && cs_low && sck_rise && (bit_idx == 3'd7);

	always @(posedge SBCLKI) begin
		ack_r <= 1'b0;
		sck_s <= {sck_s[0], SCKI};
		cs_s  <= {cs_s[0],  SCSNI};

		// ---- register interface ----
		if (bus_active) begin
			ack_r <= 1'b1;
			if (SBRWI) begin
				case (adr[3:0])
					4'h8: cr0 <= dati;
					4'h9: cr1 <= dati;
					4'hA: cr2 <= dati;
					4'hB: br  <= dati;
					4'hD: tx_data <= dati;
					4'hF: csr <= dati;
				endcase
			end else begin
				case (adr[3:0])
					4'hC: dato_r <= status;
					4'hE: dato_r <= rx_data;
					default: dato_r <= 8'h00;
				endcase
			end
		end

		// ---- transmit holding register occupancy ----
		// A bus write landing in the same cycle the engine consumes the byte
		// must not lose the new byte, so resolve both events together.
		case ({bus_txwr, eng_load})
			2'b10:   tx_full <= 1'b1;
			2'b01:   tx_full <= 1'b0;
			2'b11:   tx_full <= 1'b1;
			default: tx_full <= tx_full;
		endcase

		case ({eng_rx_done, bus_rxrd})
			2'b10:   rx_full <= 1'b1;
			2'b01:   rx_full <= 1'b0;
			2'b11:   rx_full <= 1'b1;
			default: rx_full <= rx_full;
		endcase

		// ---- slave shift engine ----
		if (spe) begin
			if (eng_load) begin
				shifter       <= tx_full ? tx_data : 8'hFF;   // 0xFF = underrun marker
				shifter_valid <= tx_full;
				bit_idx       <= 3'd0;
			end else if (cs_low && sck_fall) begin
				shifter <= {shifter[6:0], 1'b1};
				bit_idx <= bit_idx + 3'd1;
			end

			if (cs_low && sck_rise) begin
				rx_shift <= {rx_shift[6:0], SI};
				if (bit_idx == 3'd7)
					rx_data <= {rx_shift[6:0], SI};
			end
		end
	end

	assign {SBDATO7, SBDATO6, SBDATO5, SBDATO4, SBDATO3, SBDATO2, SBDATO1, SBDATO0} = dato_r;
	assign SBACKO = ack_r;
	assign SO     = shifter[7];
	assign SOE    = cs_low;
	assign SPIIRQ = 1'b0;
	assign SPIWKUP = 1'b0;
	assign MO = 1'b0, MOE = 1'b0, SCKO = 1'b0, SCKOE = 1'b0;
	assign MCSNO3 = 1'b1, MCSNO2 = 1'b1, MCSNO1 = 1'b1, MCSNO0 = 1'b1;
	assign MCSNOE3 = 1'b0, MCSNOE2 = 1'b0, MCSNOE1 = 1'b0, MCSNOE0 = 1'b0;
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

	// One SPI burst: assert CS, clock n bytes MSB-first (mode 0), release CS.
	reg [7:0] burst [0:63];
	task spi_burst(input integer n);
		integer i, b;
		reg [7:0] rx;
		begin
			spi_cs = 1'b0;
			#(HALF);                     // CS setup before the first clock
			for (i = 0; i < n; i = i + 1) begin
				rx = 8'h00;
				for (b = 7; b >= 0; b = b - 1) begin
					spi_sck = 1'b1;
					#1;
					rx[b] = spi_miso;    // master samples on the rising edge
					#(HALF - 1);
					spi_sck = 1'b0;
					#(HALF);
				end
				burst[i] = rx;
			end
			spi_cs = 1'b1;
			#(HALF);
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
	reg [dut.MSG_IW-1:0] msg_idx_before_stall;
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

		#1000;
		if (errors == 0)
			$display("PASS: all spi_hw_stream tests completed successfully");
		else
			$display("FAIL: %0d error(s)", errors);
		$finish;
	end

	// safety net
	initial begin
		#((dut.FIFO_DEPTH * 4) * TICK_NS);
		$display("FAIL: testbench timeout");
		$finish;
	end

endmodule
