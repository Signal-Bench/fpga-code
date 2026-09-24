`timescale 1ns/1ps

// ---------------------------------------------------------------------------
// Behavioral stand-ins for the iCE40 hard primitives.  Same convention as
// spi_hw_stream_tb.v / i2c_sniffer_tb.v: there is no vendor simulation library
// in this repo, so the testbench models what it needs.
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

// Simplified SB_SPI model, carried over from spi_hw_stream_tb.v.  NOT
// silicon-accurate:
//   * MSB-first mode 0 only (LSBF/CPOL/CPHA bits are stored, not honoured)
//   * the SPI side is oversampled in the SBCLKI domain rather than being a
//     true asynchronous SCK domain
//   * no receive-overrun (ROE) flag, so the model cannot prove that the real
//     IP's overrun is avoided — only that SPIRXDR is drained every byte
//   * the transmit side is modelled but unused here: this design never writes
//     SPITXDR, so the shifter runs permanently underrun and SO carries the
//     0xFF marker.  On silicon SO is unspecified in that case
//     (FPGA-TN-02011 Figure 15.2); nothing reads it.
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
	reg       shifter_valid = 0;
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
	// shift-engine events
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
// Drives real SPI transactions into the slave and decodes the UART line the
// way the host does — start bit, eight data bits LSB-first, stop bit — then
// compares the characters against text built independently of the DUT (its own
// hex conversion, its own line rules).  That makes it a cross-check rather
// than a round-trip through the same code.
// ---------------------------------------------------------------------------
module spi_to_uart_tb;

	reg  spi_sck  = 1'b0;
	reg  spi_cs   = 1'b1;
	reg  spi_mosi = 1'b0;
	wire spi_miso;
	wire serial_txd;
	wire spi_cs_flash;
	wire led_r, led_g, led_b;

	integer errors = 0;

	localparam integer HALF   = 250;    // ns -> 2 MHz SCK
	localparam integer BIT_NS = 1000;   // 1 Mbaud UART bit period

	top dut (
		.spi_sck(spi_sck),
		.spi_cs(spi_cs),
		.spi_mosi(spi_mosi),
		.spi_miso(spi_miso),
		.serial_txd(serial_txd),
		.spi_cs_flash(spi_cs_flash),
		.led_r(led_r), .led_g(led_g), .led_b(led_b)
	);

	// ------------------------------------------------------------------
	// SPI master: assert CS, clock n bytes MSB-first (mode 0), release CS.
	// gap_ns spaces the bytes out; with gap_ns = 0 the burst runs flat out,
	// which is faster than the UART can print and so exercises drop-on-full.
	// ------------------------------------------------------------------
	reg [7:0] mosi_bytes [0:127];

	task spi_burst(input integer n, input integer gap_ns);
		integer i, b;
		begin
			spi_cs = 1'b0;
			#(HALF);                             // CS setup before the first clock
			for (i = 0; i < n; i = i + 1) begin
				for (b = 7; b >= 0; b = b - 1) begin
					spi_mosi = mosi_bytes[i][b];
					#(HALF);                     // setup before the sampling edge
					spi_sck = 1'b1;
					#(HALF);
					spi_sck = 1'b0;
				end
				if (gap_ns > 0) #(gap_ns);
			end
			#(HALF);
			spi_cs = 1'b1;
			#(HALF);
		end
	endtask

	// ------------------------------------------------------------------
	// Host-side UART receiver on serial_txd
	// ------------------------------------------------------------------
	reg [7:0] rxbuf [0:1023];
	integer   rx_n = 0;
	reg [7:0] ubyte;
	integer   ui;

	initial begin
		#5_000;                                  // let the power-on reset settle
		forever begin
			@(negedge serial_txd);                // start bit
			#(BIT_NS * 3 / 2);                    // centre of bit 0
			for (ui = 0; ui < 8; ui = ui + 1) begin
				ubyte[ui] = serial_txd;           // 8-N-1, LSB first
				#(BIT_NS);
			end
			if (serial_txd !== 1'b1) begin        // centre of the stop bit
				$display("FAIL: UART framing error on character %0d", rx_n);
				errors = errors + 1;
			end
			rxbuf[rx_n] = ubyte;
			rx_n = rx_n + 1;
		end
	end

	// ------------------------------------------------------------------
	// Expected text, built here rather than taken from the DUT
	// ------------------------------------------------------------------
	reg [7:0] exp [0:1023];
	integer   exp_n = 0;

	function [7:0] hexc(input [3:0] nib);
		hexc = (nib < 4'd10) ? (8'h30 + {4'd0, nib}) : (8'h61 + {4'd0, nib} - 8'd10);
	endfunction

	function is_hex(input [7:0] c);
		is_hex = ((c >= "0") && (c <= "9")) || ((c >= "a") && (c <= "f"));
	endfunction

	task exp_byte(input [7:0] b);
		begin
			exp[exp_n]     = hexc(b[7:4]);
			exp[exp_n + 1] = hexc(b[3:0]);
			exp[exp_n + 2] = " ";
			exp_n = exp_n + 3;
		end
	endtask

	task exp_eol;
		begin
			exp[exp_n]     = 8'h0D;
			exp[exp_n + 1] = 8'h0A;
			exp_n = exp_n + 2;
		end
	endtask

	// Wait until the UART has delivered n characters, or give up.
	task wait_chars(input integer n);
		integer guard;
		begin
			guard = 0;
			while ((rx_n < n) && (guard < 20000)) begin
				#500;
				guard = guard + 1;
			end
		end
	endtask

	task dump_text(input [255:0] what, input integer n, input integer which);
		integer i;
		begin
			$write("  %0s: \"", what);
			for (i = 0; i < n; i = i + 1) begin
				if (which == 0) begin
					if (rxbuf[i] == 8'h0D) $write("\\r");
					else if (rxbuf[i] == 8'h0A) $write("\\n");
					else $write("%c", rxbuf[i]);
				end else begin
					if (exp[i] == 8'h0D) $write("\\r");
					else if (exp[i] == 8'h0A) $write("\\n");
					else $write("%c", exp[i]);
				end
			end
			$write("\"\n");
		end
	endtask

	// Compare what came out of the UART against the expected text, then clear
	// both buffers so the next test compares only its own output.
	task check_stream(input [255:0] label);
		integer i;
		begin
			wait_chars(exp_n);
			#(BIT_NS * 12);                       // catch any extra trailing character
			if (rx_n !== exp_n) begin
				$display("FAIL [%0s]: got %0d characters, expected %0d", label, rx_n, exp_n);
				errors = errors + 1;
			end
			for (i = 0; (i < rx_n) && (i < exp_n); i = i + 1) begin
				if (rxbuf[i] !== exp[i]) begin
					$display("FAIL [%0s]: character %0d = 0x%02x, expected 0x%02x",
					         label, i, rxbuf[i], exp[i]);
					errors = errors + 1;
				end
			end
			dump_text("got     ", rx_n,  0);
			dump_text("expected", exp_n, 1);
			rx_n  = 0;
			exp_n = 0;
		end
	endtask

	// Whatever survives a dropped byte must still be well formed: groups of
	// two hex digits plus a space, with CR always followed by LF.  A partial
	// hex pair would mean the queue dropped characters rather than bytes.
	task check_wellformed(input [255:0] label);
		integer i, phase;
		begin
			phase = 0;
			for (i = 0; i < rx_n; i = i + 1) begin
				case (phase)
					0: if (is_hex(rxbuf[i]))       phase = 1;
					   else if (rxbuf[i] == 8'h0D) phase = 3;
					   else begin
						$display("FAIL [%0s]: character %0d = 0x%02x, expected a hex digit or CR",
						         label, i, rxbuf[i]);
						errors = errors + 1;
					   end
					1: if (is_hex(rxbuf[i]))       phase = 2;
					   else begin
						$display("FAIL [%0s]: character %0d = 0x%02x, expected the second hex digit",
						         label, i, rxbuf[i]);
						errors = errors + 1;
					   end
					2: if (rxbuf[i] == " ")        phase = 0;
					   else begin
						$display("FAIL [%0s]: character %0d = 0x%02x, expected a separator",
						         label, i, rxbuf[i]);
						errors = errors + 1;
					   end
					3: if (rxbuf[i] == 8'h0A)      phase = 0;
					   else begin
						$display("FAIL [%0s]: character %0d = 0x%02x, expected LF after CR",
						         label, i, rxbuf[i]);
						errors = errors + 1;
					   end
				endcase
			end
			if (phase != 0) begin
				$display("FAIL [%0s]: output ends mid-token (phase %0d)", label, phase);
				errors = errors + 1;
			end
		end
	endtask

	// Number of bytes actually printed: one separator per formatted byte.
	task count_printed(output integer n);
		integer i;
		begin
			n = 0;
			for (i = 0; i < rx_n; i = i + 1)
				if (rxbuf[i] == " ") n = n + 1;
		end
	endtask

	// Count dropped bytes straight off the design's internal strobe.
	integer drops = 0;
	always @(posedge dut.clk_core)
		if (dut.byte_dropped) drops = drops + 1;

	// The character queue reads combinationally and its consumer registers the
	// result, so yosys merges the two and maps the queue into an EBR (see the
	// queue comment in spi_to_uart.v).  That is only equivalent while a push
	// and a pop never touch the same address in the same cycle, because the
	// iCE40 EBR does not define read data on a same-address collision.  The
	// pop condition should make it impossible; check it rather than assume it.
	integer collisions = 0;
	always @(posedge dut.clk_core)
		if (dut.cq_push && dut.cq_pop && (dut.cq_wr === dut.cq_rd)) begin
			$display("FAIL: queue read/write collision at address %0d", dut.cq_rd);
			collisions = collisions + 1;
			errors = errors + 1;
		end

	integer i;
	integer printed;

	initial begin
		if ($test$plusargs("vcd")) begin
			$dumpfile("spi_to_uart_tb.vcd");
			$dumpvars(0, spi_to_uart_tb);
		end

		$display("=== spi_to_uart testbench ===");

		#20_000;   // let the hard IP get configured

		if (led_g !== 1'b0) begin
			$display("FAIL: green LED not lit - hard SPI IP never finished configuring");
			errors = errors + 1;
		end
		if (serial_txd !== 1'b1) begin
			$display("FAIL: UART line not idle high before any traffic");
			errors = errors + 1;
		end
		if (rx_n != 0) begin
			$display("FAIL: %0d characters printed before any SPI traffic", rx_n);
			errors = errors + 1;
		end

		// ------------------------------------------------------------
		// Test 1: the three bytes uart_to_spi.v sends per sniffed UART
		// byte, as one transaction.
		// ------------------------------------------------------------
		mosi_bytes[0] = 8'h48; mosi_bytes[1] = 8'h01; mosi_bytes[2] = 8'h41;
		spi_burst(3, 0);
		exp_byte(8'h48); exp_byte(8'h01); exp_byte(8'h41); exp_eol;
		check_stream("3-byte transaction");

		if (led_b !== 1'b0) begin
			$display("FAIL: blue LED not lit after receiving bytes");
			errors = errors + 1;
		end

		// ------------------------------------------------------------
		// Test 2: both hex nibble extremes, and a second transaction
		// starting a fresh line.
		// ------------------------------------------------------------
		mosi_bytes[0] = 8'h00; mosi_bytes[1] = 8'hFF; mosi_bytes[2] = 8'h5A;
		spi_burst(3, 0);
		exp_byte(8'h00); exp_byte(8'hFF); exp_byte(8'h5A); exp_eol;
		check_stream("nibble extremes");

		// ------------------------------------------------------------
		// Test 3: a single byte per transaction still terminates its line.
		// ------------------------------------------------------------
		mosi_bytes[0] = 8'h7E;
		spi_burst(1, 0);
		exp_byte(8'h7E); exp_eol;
		check_stream("single byte");

		// ------------------------------------------------------------
		// Test 4: line wrapping.  20 bytes in one transaction, paced slower
		// than the UART so nothing is dropped, must break after 16.
		// ------------------------------------------------------------
		for (i = 0; i < 20; i = i + 1)
			mosi_bytes[i] = 8'hA0 + i[7:0];
		spi_burst(20, 40_000);                    // 40 us/byte > 3 chars of UART
		for (i = 0; i < 20; i = i + 1) begin
			exp_byte(8'hA0 + i[7:0]);
			if (i == 15) exp_eol;                 // BYTES_PER_LINE
		end
		exp_eol;
		check_stream("20 bytes, wrapped at 16");

		if (drops != 0) begin
			$display("FAIL: %0d byte(s) dropped while the master was pacing itself", drops);
			errors = errors + 1;
		end

		// ------------------------------------------------------------
		// Test 5: a burst far faster than the UART.  Bytes must be dropped
		// whole - the output stays well formed, and every byte is either
		// printed or counted as a drop.
		// ------------------------------------------------------------
		for (i = 0; i < 64; i = i + 1)
			mosi_bytes[i] = 8'h10 + i[7:0];
		drops = 0;
		spi_burst(64, 0);
		#3_000_000;                               // let the queue drain

		if (drops == 0) begin
			$display("FAIL: a 64-byte burst at 2 MHz should outrun a 1 Mbaud UART");
			errors = errors + 1;
		end
		if (led_r !== 1'b0) begin
			$display("FAIL: red LED not lit - drop was never signalled");
			errors = errors + 1;
		end
		check_wellformed("fast burst");
		count_printed(printed);
		if (printed + drops != 64) begin
			$display("FAIL: %0d bytes printed + %0d dropped != 64", printed, drops);
			errors = errors + 1;
		end else begin
			$display("  fast burst: %0d bytes printed, %0d dropped", printed, drops);
		end
		dump_text("got     ", rx_n, 0);
		rx_n = 0;

		#1000;
		if (errors == 0)
			$display("PASS: all spi_to_uart tests completed successfully");
		else
			$display("FAIL: %0d error(s)", errors);
		$finish;
	end

	// safety net
	initial begin
		#50_000_000;
		$display("FAIL: testbench timeout");
		$finish;
	end

endmodule
