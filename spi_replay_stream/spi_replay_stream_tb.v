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
//
// The point of this design is that the REPLAY IS LOSSLESS AT ANY MASTER RATE,
// so that is what gets tested: the same expected byte sequence is checked
// across a slow burst, a fast burst, CS toggles, and a long idle gap, with
// one continuously advancing stream position.  The expected contents are
// computed here from the record layout using division and modulo, while the
// DUT builds them from bit slices — different arithmetic, so this is a real
// cross-check rather than a round-trip.
// ---------------------------------------------------------------------------
module spi_replay_stream_tb;

	reg  spi_sck  = 1'b0;
	reg  spi_cs   = 1'b1;
	reg  spi_mosi = 1'b0;
	wire spi_miso;
	wire spi_cs_flash;
	wire led_r, led_g, led_b;

	integer errors = 0;

	// Half an SCK period in ns.  A variable, not a parameter: the whole
	// point is that the stream does not care what this is.
	integer HALF = 250;                  // 2 MHz to start

	localparam integer DEPTH   = 4096;   // must match dut.REPLAY_DEPTH
	localparam integer REC_LEN = 16;

	// Independently computed expected buffer contents.
	function [7:0] expect_byte(input integer pos);
		integer off, rec, slot;
		begin
			off  = pos % DEPTH;
			rec  = off / REC_LEN;
			slot = off % REC_LEN;
			if (slot == 0)
				expect_byte = 8'hA5;
			else if (slot == 1)
				expect_byte = rec % 256;
			else if (slot == REC_LEN - 1)
				expect_byte = 8'h5A;
			else
				expect_byte = (rec + slot) % 256;
		end
	endfunction

	// Position in the replay stream that the next byte off the link must
	// correspond to.  Advances with every byte read and never resets — the
	// stream must stay continuous across everything the master does to it.
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

	// Every byte must be the next byte of the replay buffer.
	task check_stream(input integer n, input [127:0] label);
		integer i;
		reg [7:0] want;
		begin
			for (i = 0; i < n; i = i + 1) begin
				want = expect_byte(stream_pos);
				if (burst[i] !== want) begin
					$display("FAIL [%0s]: byte %0d = 0x%02x, expected 0x%02x (stream pos %0d)",
					         label, i, burst[i], want, stream_pos);
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
			$write("\n");
		end
	endtask

	integer tx_count = 0;
	always @(posedge dut.clk_core)
		if (dut.src_advance) tx_count = tx_count + 1;

	integer wrap_count = 0;
	always @(posedge dut.clk_core)
		if (dut.replay_wrap) wrap_count = wrap_count + 1;

	integer i, pass, chunk;
	integer addr_before, pos_before, tx_before;

	initial begin
		if ($test$plusargs("vcd")) begin
			$dumpfile("spi_replay_stream_tb.vcd");
			$dumpvars(0, spi_replay_stream_tb);
		end

		$display("=== spi_replay_stream testbench ===");
		if (dut.REPLAY_DEPTH !== DEPTH) begin
			$display("FAIL: testbench DEPTH %0d does not match dut.REPLAY_DEPTH %0d",
			         DEPTH, dut.REPLAY_DEPTH);
			errors = errors + 1;
		end
		$display("  buffer = %0d bytes, %0d records of %0d",
		         DEPTH, DEPTH / REC_LEN, REC_LEN);

		#2000;   // let the service FSM finish configuring the IP

		if (led_g !== 1'b0) begin
			$display("FAIL: green LED not lit — the hard SPI IP never configured");
			errors = errors + 1;
		end

		// ------------------------------------------------------------
		// Test 1: the buffer replays from offset 0, starting at byte 1.
		// SPITXDR is pre-loaded, so there is no dummy byte in this model.
		// ------------------------------------------------------------
		spi_burst(32);
		dump_burst(32, "burst 1 @2MHz");
		check_stream(32, "burst 1");

		// ------------------------------------------------------------
		// Test 2: a long idle gap must change nothing.  This is the whole
		// difference from spi_hw_stream / spi_counter_stream: there is no
		// producer running in the background, so the pointer cannot move
		// and no byte can be lost while the master is away.
		// ------------------------------------------------------------
		addr_before = dut.rd_addr;
		tx_before   = tx_count;
		#5_000_000;                      // 5 ms of nothing
		if (dut.rd_addr !== addr_before) begin
			$display("FAIL: replay pointer moved while idle (%0d -> %0d)",
			         addr_before, dut.rd_addr);
			errors = errors + 1;
		end
		if (tx_count !== tx_before) begin
			$display("FAIL: %0d bytes consumed while the master was idle",
			         tx_count - tx_before);
			errors = errors + 1;
		end
		$display("  5 ms idle: pointer held at %0d, nothing consumed", dut.rd_addr);

		spi_burst(32);
		dump_burst(32, "burst2 postidle");
		check_stream(32, "burst 2");

		// ------------------------------------------------------------
		// Test 3: rate independence.  Drop to 200 kHz, then go up to
		// 4 MHz; the sequence must not care.
		//
		// 4 MHz is the ceiling this TESTBENCH can represent, not the
		// design's.  The SB_SPI model above oversamples SCKI through a
		// 2-flop synchronizer in the 24 MHz SBCLKI domain, so it needs an
		// SCK half-period of ~3 core cycles (125 ns) to track edges at all.
		// Above that the model itself aliases and reports corruption that
		// is not in the DUT.  Rates above 4 MHz have to be checked on
		// hardware — see the README.
		// ------------------------------------------------------------
		HALF = 2500;                     // 200 kHz
		spi_burst(16);
		dump_burst(16, "burst 3 @200kHz");
		check_stream(16, "burst 3");

		HALF = 125;                      // 4 MHz
		spi_burst(16);
		dump_burst(16, "burst 4 @4MHz");
		check_stream(16, "burst 4");

		HALF = 250;                      // back to 2 MHz
		if (led_b !== 1'b0) begin
			$display("FAIL: blue LED not lit — no byte was handed to the IP");
			errors = errors + 1;
		end

		// ------------------------------------------------------------
		// Test 4: read the rest of the buffer and over the wrap.  Every
		// byte of all %0d records gets checked, then the stream must come
		// back to record 0 and the red LED must flag the wrap.
		// ------------------------------------------------------------
		pos_before = stream_pos;
		while (stream_pos < DEPTH) begin
			chunk = (DEPTH - stream_pos > 64) ? 64 : (DEPTH - stream_pos);
			spi_burst(chunk);
			check_stream(chunk, "drain");
		end
		$display("  drained to end of buffer: %0d bytes checked in this phase",
		         stream_pos - pos_before);

		// The design pre-loads SPITXDR, so by the time the master has read
		// byte DEPTH-1 the pointer may already have wrapped.  Being ahead
		// by the pipeline depth is correct; being ahead by more is not.
		if (wrap_count > 1) begin
			$display("FAIL: replay wrapped %0d times before the master reached the end",
			         wrap_count);
			errors = errors + 1;
		end

		spi_burst(32);
		dump_burst(32, "burst5 postwrap");
		check_stream(32, "burst 5");

		if (wrap_count !== 1) begin
			$display("FAIL: expected exactly 1 wrap after a full pass, saw %0d", wrap_count);
			errors = errors + 1;
		end
		if (led_r !== 1'b0) begin
			$display("FAIL: red LED not lit — the wrap was never signalled");
			errors = errors + 1;
		end
		$display("  wrapped once and resumed at record 0");

		// ------------------------------------------------------------
		// Test 5: not one byte lost anywhere.  tx_count counts loads into
		// SPITXDR, so it leads the master's read count by the pre-load
		// pipeline (SPITXDR + shifter) and by nothing else.  A larger gap
		// would mean a byte was handed over and never shifted out.
		// ------------------------------------------------------------
		if (tx_count < stream_pos || tx_count - stream_pos > 2) begin
			$display("FAIL: %0d bytes loaded into the IP but %0d read by the master",
			         tx_count, stream_pos);
			errors = errors + 1;
		end else begin
			$display("  %0d bytes read, %0d loaded (%0d in flight), zero lost",
			         stream_pos, tx_count, tx_count - stream_pos);
		end

		if (errors == 0)
			$display("PASS: all spi_replay_stream tests completed successfully");
		else
			$display("FAIL: %0d error(s)", errors);
		$finish;
	end

endmodule
