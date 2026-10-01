/** \file
 * Testbench for the framed SignalBench STREAM top.
 *
 * The parser below is the algorithm in MCU_FRAME_FORMAT.md, transcribed
 * byte for byte, fed one received byte at a time across burst boundaries
 * -- so these tests check the document as well as the RTL.
 *
 * MAXP is the DUT's MAX_PAYLOAD.  `make sim` runs 16 with a 128-byte FIFO,
 * which forces frame splits and overflow with short stimulus; `make sim-255`
 * runs the production values (255, 4 KB) with an I2C transaction long
 * enough to split at 255.
 *
 * The I2C driver tasks are copied verbatim from ../i2c_sniffer/i2c_sniffer_tb.v
 * and the payload bytes are checked against that testbench's expected bytes.
 *
 * SB_SPI model: from ../final_system/signalbench_tb.v, NOT silicon-accurate.
 */
`timescale 1ns/1ps

module signalbench_stream_tb;
	parameter integer MAXP = 16;
	// 128-byte FIFO in the main run so overflow can be forced quickly;
	// sim-255 uses the production 4 KB.
	parameter integer FIFO_AW = 7;

	reg clk_12 = 1'b0;
	always #41.667 clk_12 = ~clk_12;

	reg  scl_pin = 1'b1, sda_pin = 1'b1;
	reg  uart_a = 1'b1, uart_b = 1'b1;
	reg  mode_uart = 1'b0;              // which rig is "plugged in" to ch0/ch1
	wire ch0 = mode_uart ? uart_a : scl_pin;
	wire ch1 = mode_uart ? uart_b : sda_pin;

	reg  p_sck = 1'b0, p_mosi = 1'b0, p_miso = 1'b0, p_cs = 1'b1;
	reg  mcu_sck = 1'b0, mcu_cs = 1'b1, mcu_mosi = 1'b0;
	wire mcu_miso, spi_cs_flash, led_r, led_g, led_b;

	signalbench_stream #(.MAX_PAYLOAD(MAXP), .FIFO_AW(FIFO_AW)) dut (
		.clk_12(clk_12), .ch0(ch0), .ch1(ch1),
		.p_sck(p_sck), .p_mosi(p_mosi), .p_miso(p_miso), .p_cs(p_cs),
		.mcu_sck(mcu_sck), .mcu_cs(mcu_cs), .mcu_mosi(mcu_mosi),
		.mcu_miso(mcu_miso), .spi_cs_flash(spi_cs_flash),
		.led_r(led_r), .led_g(led_g), .led_b(led_b)
	);

	integer errors = 0;
	// +dump writes every byte the MCU receives while reading, plus a RESET
	// line after each command, to miso_dump.txt -- used to check the C
	// parser in MCU_FRAME_FORMAT.md against real DUT output.
	integer dump_fd = 0;
	task fail(input [511:0] msg);
		begin $display("  FAIL  %0s", msg); errors = errors + 1; end
	endtask
	task pass(input [511:0] msg);
		begin $display("  PASS  %0s", msg); end
	endtask

	// ================================================================
	// MCU_FRAME_FORMAT.md parser, transcribed
	// ================================================================
	localparam [3:0] P_HUNT0 = 0, P_HUNT1 = 1, P_TYPE = 2, P_FLAGS = 3,
	                 P_LEN = 4, P_PAYLOAD = 5, P_MODEACK = 6, P_READYACK = 7;
	reg [3:0] pst = P_HUNT0;
	reg [7:0] p_flags, p_len;
	integer   p_i;

	// parsed results
	integer   nf = 0;                    // frames
	reg [7:0] fr_flags [0:63];
	reg [7:0] fr_len   [0:63];
	integer   fr_off   [0:63];
	reg [7:0] fr_pay   [0:16383];
	integer   pay_n = 0;
	integer   n_ready = 0, n_mode = 0;
	reg [7:0] last_mode = 8'hEE;

	task parser_reset; begin pst = P_HUNT0; end endtask

	task feed(input [7:0] b);
		begin
			case (pst)
			P_HUNT0:    if (b == 8'h53) pst = P_HUNT1;
			P_HUNT1:    if (b == 8'h42) pst = P_TYPE;
			            else if (b != 8'h53) pst = P_HUNT0;
			P_TYPE: begin
				if      (b == 8'h4D) pst = P_FLAGS;
				else if (b == 8'h41) pst = P_MODEACK;
				else if (b == 8'h4F) pst = P_READYACK;
				else if (b == 8'h53) pst = P_HUNT1;
				else                 pst = P_HUNT0;
			end
			P_FLAGS: begin p_flags = b; pst = P_LEN; end
			P_LEN: begin
				p_len = b;
				// Sanity: protocol id 0..4, reserved bits 0, length 1..255.
				// Anything else was a false 53 42 4D inside payload: resync.
				if ((p_flags[3:0] > 4'd4) || (p_flags[5:4] != 2'b00) || (b == 8'd0))
					pst = P_HUNT0;
				else begin
					fr_flags[nf] = p_flags; fr_len[nf] = b; fr_off[nf] = pay_n;
					p_i = 0; pst = P_PAYLOAD;
				end
			end
			P_PAYLOAD: begin
				fr_pay[pay_n] = b; pay_n = pay_n + 1; p_i = p_i + 1;
				if (p_i == p_len) begin nf = nf + 1; pst = P_HUNT0; end
			end
			P_MODEACK:  begin last_mode = b; n_mode = n_mode + 1; pst = P_HUNT0; end
			P_READYACK: begin if (b == 8'h4B) n_ready = n_ready + 1; pst = P_HUNT0; end
			default:    pst = P_HUNT0;
			endcase
		end
	endtask

	// ================================================================
	// MCU-side SPI master, mode 0, 2 MHz -- the firmware's default
	// ================================================================
	localparam real SCK_HALF = 250.0;

	task spi_xfer(input [7:0] tx, output [7:0] rx);
		integer i;
		begin
			for (i = 7; i >= 0; i = i - 1) begin
				mcu_mosi = tx[i];
				#SCK_HALF mcu_sck = 1'b1;
				rx[i] = mcu_miso;
				#SCK_HALF mcu_sck = 1'b0;
			end
		end
	endtask

	// Per MCU_FRAME_FORMAT.md: whatever is received while sending a command
	// is discarded, and the parser is reset afterwards.
	reg [7:0] junk;
	task send_cmd(input [7:0] b0, input [7:0] b1, input [7:0] b2, input [7:0] b3);
		begin
			mcu_cs = 1'b0; #SCK_HALF;
			spi_xfer(b0, junk); spi_xfer(b1, junk);
			spi_xfer(b2, junk); spi_xfer(b3, junk);
			#SCK_HALF mcu_cs = 1'b1;
			parser_reset;
			if (dump_fd) $fdisplay(dump_fd, "RESET");
			#5000;
		end
	endtask

	integer burst_live;   // non-0xFF bytes in the last burst
	task read_burst(input integer n);
		integer i;
		reg [7:0] b;
		begin
			mcu_cs = 1'b0; #SCK_HALF;
			burst_live = 0;
			for (i = 0; i < n; i = i + 1) begin
				spi_xfer(8'h00, b);
				if (b != 8'hFF) burst_live = burst_live + 1;
				if (dump_fd) $fdisplay(dump_fd, "%02x", b);
				feed(b);
			end
			#SCK_HALF mcu_cs = 1'b1;
			#5000;
		end
	endtask

	// Read until `want` more frames have been parsed (or give up).
	task read_frames(input integer want);
		integer target, tries;
		begin
			target = nf + want; tries = 0;
			while (nf < target && tries < 40) begin
				read_burst(32); tries = tries + 1;
			end
			if (nf < target) fail("timed out waiting for frames");
		end
	endtask

	// Read until the FIFO is empty: a whole burst of idle 0xFF AND the parser
	// between frames.  NOT "until a burst completes no frame" -- a frame can
	// be longer than one burst (up to 260 bytes), so that stops mid-frame.
	task drain;
		integer tries;
		begin
			tries = 0;
			read_burst(32);
			while ((burst_live != 0 || pst != P_HUNT0) && tries < 200) begin
				read_burst(32); tries = tries + 1;
			end
		end
	endtask

	task show(input integer f);
		integer i;
		begin
			$write("   frame %0d: 53 42 4D %02x %02x |", f, fr_flags[f], fr_len[f]);
			for (i = 0; i < fr_len[f]; i = i + 1) $write(" %02x", fr_pay[fr_off[f] + i]);
			$write("\n");
		end
	endtask

	// Check frame f exactly.
	reg [7:0] e [0:31];
	task check_frame(input integer f, input [7:0] flags, input integer len, input [511:0] label);
		integer i, ok;
		begin
			show(f);
			ok = (fr_flags[f] == flags) && (fr_len[f] == len);
			for (i = 0; i < len; i = i + 1)
				if (fr_pay[fr_off[f] + i] !== e[i]) ok = 0;
			if (ok) pass(label); else fail(label);
		end
	endtask

	// ================================================================
	// I2C stimulus -- VERBATIM from ../i2c_sniffer/i2c_sniffer_tb.v
	// ================================================================
	localparam integer I2C_HALF_PERIOD_NS = 1250;

	task i2c_start;
		begin
			sda_pin = 1'b1;
			scl_pin = 1'b1;
			#(I2C_HALF_PERIOD_NS);
			sda_pin = 1'b0;
			#(I2C_HALF_PERIOD_NS);
			scl_pin = 1'b0;
			#(I2C_HALF_PERIOD_NS);
		end
	endtask

	task i2c_stop;
		begin
			sda_pin = 1'b0;
			scl_pin = 1'b0;
			#(I2C_HALF_PERIOD_NS);
			scl_pin = 1'b1;
			#(I2C_HALF_PERIOD_NS);
			sda_pin = 1'b1;
		end
	endtask

	task i2c_send_bit_then_stop;
		input       bit_value;
		input integer stop_gap_ns;
		begin
			scl_pin = 1'b0;
			sda_pin = bit_value;
			#(I2C_HALF_PERIOD_NS);
			scl_pin = 1'b1;
			#(stop_gap_ns);
			sda_pin = 1'b1;
			#(I2C_HALF_PERIOD_NS);
		end
	endtask

	task i2c_repeated_start;
		begin
			scl_pin = 1'b0;
			sda_pin = 1'b1;
			#(I2C_HALF_PERIOD_NS);
			scl_pin = 1'b1;
			#(I2C_HALF_PERIOD_NS);
			sda_pin = 1'b0;
			#(I2C_HALF_PERIOD_NS);
			scl_pin = 1'b0;
			#(I2C_HALF_PERIOD_NS);
		end
	endtask

	task i2c_send_bit;
		input bit_value;
		begin
			scl_pin = 1'b0;
			sda_pin = bit_value;
			#(I2C_HALF_PERIOD_NS);
			scl_pin = 1'b1;
			#(I2C_HALF_PERIOD_NS);
			scl_pin = 1'b0;
			#(I2C_HALF_PERIOD_NS);
		end
	endtask

	task i2c_send_byte;
		input [7:0] data_value;
		integer bit_idx;
		begin
			for (bit_idx = 7; bit_idx >= 0; bit_idx = bit_idx - 1)
				i2c_send_bit(data_value[bit_idx]);
		end
	endtask

	// ================================================================
	// UART stimulus, 8-N-1, LSB first, 9600
	// ================================================================
	localparam real BIT_NS = 1.0e9 / 9600.0;

	// automatic: two calls can run concurrently under fork/join.
	task automatic uart_send(input [7:0] d, input on_a, input on_b);
		integer i;
		begin
			if (on_a) uart_a = 1'b0;
			if (on_b) uart_b = 1'b0;
			#BIT_NS;
			for (i = 0; i < 8; i = i + 1) begin
				if (on_a) uart_a = d[i];
				if (on_b) uart_b = d[i];
				#BIT_NS;
			end
			if (on_a) uart_a = 1'b1;           // stop bit -- only our lines
			if (on_b) uart_b = 1'b1;
			#BIT_NS;
		end
	endtask


	// ================================================================
	integer k, f0, i, j, nbits, total_bits, ok, nlong, nsplit;
	reg [7:0] pattern [0:7];
	reg [4095:0] want_bits, got_bits;

	initial begin
		if ($test$plusargs("vcd")) begin
			$dumpfile("signalbench_stream_tb.vcd");
			$dumpvars(0, signalbench_stream_tb);
		end
		if ($test$plusargs("dump")) dump_fd = $fopen("miso_dump.txt", "w");
		pattern[0]=8'hAA; pattern[1]=8'h12; pattern[2]=8'h34; pattern[3]=8'hFF;
		pattern[4]=8'hFF; pattern[5]=8'h9A; pattern[6]=8'hBC; pattern[7]=8'hDE;

		$display("MAX_PAYLOAD = %0d", MAXP);
		#30000;

		$display("== ready check ==");
		send_cmd(8'h53, 8'h42, 8'hA5, 8'h5A);
		read_burst(16);
		if (n_ready == 1) pass("ready ACK 53 42 4F 4B"); else fail("no ready ACK");

		$display("== select I2C ==");
		send_cmd(8'h53, 8'h42, 8'h4D, 8'h00);
		read_burst(16);
		if (n_mode == 1 && last_mode == 8'h00) pass("mode ACK 53 42 41 00"); else fail("no I2C mode ACK");

		if (MAXP >= 16) begin
		$display("== I2C T1: START 6C ACK 0B ACK STOP ==");
		i2c_start();
		i2c_send_byte(8'h6C); i2c_send_bit(1'b0);
		i2c_send_byte(8'h0B); i2c_send_bit(1'b0);
		i2c_stop();
		#20000;
		f0 = nf; read_frames(1);
		// i2c_sniffer_tb expects 6C 05 80; 19 bits -> last byte has 3.
		e[0]=8'h6C; e[1]=8'h05; e[2]=8'h80; e[3]=8'h03;
		check_frame(f0, 8'h00, 4, "T1: 6C 05 80 + info 03 (STOP, 3 bits)");

		$display("== I2C T1b: ACK jammed 80 ns against STOP ==");
		i2c_start();
		i2c_send_byte(8'h6C); i2c_send_bit(1'b0);
		i2c_send_byte(8'h0B);
		i2c_send_bit_then_stop(1'b0, 80);
		#20000;
		f0 = nf; read_frames(1);
		// Same bytes as T1, but no separate STOP-setup clock: 18 bits.
		e[0]=8'h6C; e[1]=8'h05; e[2]=8'h80; e[3]=8'h02;
		check_frame(f0, 8'h00, 4, "T1b: 6C 05 80 + info 02 (STOP, 2 bits)");

		$display("== I2C T2: write, repeated START, read, NACK, STOP ==");
		i2c_start();
		i2c_send_byte(8'h6C); i2c_send_bit(1'b0);
		i2c_send_byte(8'h0B); i2c_send_bit(1'b0);
		i2c_repeated_start();
		i2c_send_byte(8'h6D); i2c_send_bit(1'b0);
		i2c_send_byte(8'h93); i2c_send_bit(1'b1);
		i2c_stop();
		#20000;
		f0 = nf; read_frames(2);
		e[0]=8'h6C; e[1]=8'h05; e[2]=8'hA0; e[3]=8'h23;
		check_frame(f0,   8'h00, 4, "T2 seg 1: 6C 05 A0 + info 23 (ended by rSTART)");
		e[0]=8'h6D; e[1]=8'h49; e[2]=8'hC0; e[3]=8'h83;
		check_frame(f0+1, 8'h00, 4, "T2 seg 2: 6D 49 C0 + info 83 (began with rSTART)");
		end

		$display("== I2C long transaction: must split across CONT frames ==");
		nlong = (MAXP == 16) ? 20 : 300;
		// Expected raw bitstream: per byte 8 data bits + ACK(0), then the
		// STOP-setup clock samples SDA = 0.
		total_bits = 0; want_bits = 0;
		for (k = 0; k < nlong; k = k + 1) begin
			for (j = 7; j >= 0; j = j - 1) begin
				want_bits[4095 - total_bits] = ((8'h30 + k) >> j) & 1; total_bits = total_bits + 1;
			end
			want_bits[4095 - total_bits] = 1'b0; total_bits = total_bits + 1;
		end
		want_bits[4095 - total_bits] = 1'b0; total_bits = total_bits + 1;

		f0 = nf;
		i2c_start();
		for (k = 0; k < nlong; k = k + 1) begin i2c_send_byte(8'h30 + k); i2c_send_bit(1'b0); end
		i2c_stop();
		#20000;
		drain;
		// Reassemble: data bytes of every frame up to the one with END != 11.
		got_bits = 0; nbits = 0; ok = 1; nsplit = 0;
		for (i = f0; i < nf; i = i + 1) begin
			if (i > f0 && !fr_flags[i][6]) ok = 0;                 // CONT expected
			if (i == f0 && fr_flags[i][6]) ok = 0;
			if (fr_pay[fr_off[i] + fr_len[i] - 1][6:5] == 2'b11) begin
				nsplit = nsplit + 1;
				if (fr_len[i] != MAXP) ok = 0;                     // split frames are full
			end
			for (j = 0; j < fr_len[i] - 1; j = j + 1) begin
				for (k = 7; k >= 0; k = k - 1) begin
					// last data byte of the final frame: only `lastbits` count
					if (!(i == nf - 1 && j == fr_len[i] - 2 &&
					      fr_pay[fr_off[i] + fr_len[i] - 1][2:0] != 0 &&
					      (7 - k) >= fr_pay[fr_off[i] + fr_len[i] - 1][2:0])) begin
						got_bits[4095 - nbits] = fr_pay[fr_off[i] + j][k];
						nbits = nbits + 1;
					end
				end
			end
		end
		$display("   %0d frames, %0d split, %0d bits reassembled (expected %0d)",
		         nf - f0, nsplit, nbits, total_bits);
		if (ok && nsplit > 0 && nbits == total_bits && got_bits == want_bits)
			pass("long I2C: split frames full, CONT set, bitstream exact");
		else fail("long I2C reassembly");

		if (MAXP == 16) begin
		$display("== select UART ==");
		send_cmd(8'h53, 8'h42, 8'h4D, 8'h04);
		read_burst(16);
		if (last_mode == 8'h04) pass("mode ACK 53 42 41 04"); else fail("no UART mode ACK");
		mode_uart = 1'b1;
		#20000;

		$display("== UART: send_pattern on target TX -> one frame per byte ==");
		f0 = nf;
		for (k = 0; k < 8; k = k + 1) uart_send(pattern[k], 1'b1, 1'b0);
		#20000;
		drain;
		ok = (nf - f0 == 8);
		for (k = 0; k < 8 && ok; k = k + 1) begin
			if (k < 2 || k == 7) show(f0 + k);
			if (fr_flags[f0+k] != 8'h04 || fr_len[f0+k] != 2 ||
			    fr_pay[fr_off[f0+k]] != 8'h01 || fr_pay[fr_off[f0+k]+1] != pattern[k]) ok = 0;
		end
		if (ok) pass("8 frames: [01 AA] [01 12] [01 34] [01 FF] [01 FF] [01 9A] [01 BC] [01 DE]");
		else fail("send_pattern frames");

		$display("== UART: request on TX, reply on RX -> separate frames ==");
		f0 = nf;
		uart_send(8'h41, 1'b1, 1'b0); uart_send(8'h42, 1'b1, 1'b0);
		uart_send(8'h43, 1'b0, 1'b1); uart_send(8'h44, 1'b0, 1'b1);
		#20000;
		drain;
		e[0]=8'h01; e[1]=8'h41; check_frame(f0,   8'h04, 2, "[01 41]");
		e[0]=8'h01; e[1]=8'h42; check_frame(f0+1, 8'h04, 2, "[01 42]");
		e[0]=8'h02; e[1]=8'h43; check_frame(f0+2, 8'h04, 2, "[02 43]");
		e[0]=8'h02; e[1]=8'h44; check_frame(f0+3, 8'h04, 2, "[02 44]");

		$display("== UART: data bytes equal to the line tags ==");
		f0 = nf;
		uart_send(8'h02, 1'b1, 1'b0); uart_send(8'h01, 1'b0, 1'b1);
		#20000;
		drain;
		e[0]=8'h01; e[1]=8'h02; check_frame(f0,   8'h04, 2, "[01 02]: data 02 on line 01");
		e[0]=8'h02; e[1]=8'h01; check_frame(f0+1, 8'h04, 2, "[02 01]: data 01 on line 02");

		$display("== UART: both lines in the same clock -> one frame, length 4 ==");
		f0 = nf;
		uart_send(8'h5C, 1'b1, 1'b1);
		#20000;
		drain;
		e[0]=8'h01; e[1]=8'h5C; e[2]=8'h02; e[3]=8'h5C;
		check_frame(f0, 8'h04, 4, "[01 5C 02 5C]");
		if (nf - f0 == 1) pass("exactly one frame"); else fail("expected exactly one frame");

		// The two receivers share one 4x-baud tick, so strobes are quantized
		// to quarter bits (26 us at 9600).  Same tick -> one frame.
		$display("== UART: lines 2 us apart -> same quarter-bit tick, one frame ==");
		f0 = nf;
		fork
			uart_send(8'h66, 1'b1, 1'b0);
			begin #2000; uart_send(8'h67, 1'b0, 1'b1); end
		join
		#20000;
		drain;
		e[0]=8'h01; e[1]=8'h66; e[2]=8'h02; e[3]=8'h67;
		check_frame(f0, 8'h04, 4, "[01 66 02 67]");

		$display("== UART: lines 60 us apart (> 2 ticks) -> always two frames ==");
		f0 = nf;
		fork
			uart_send(8'h68, 1'b1, 1'b0);
			begin #60000; uart_send(8'h69, 1'b0, 1'b1); end
		join
		#20000;
		drain;
		e[0]=8'h01; e[1]=8'h68; check_frame(f0,   8'h04, 2, "[01 68]");
		e[0]=8'h02; e[1]=8'h69; check_frame(f0+1, 8'h04, 2, "[02 69]");

		$display("== command mid-frame: partial frame discarded, parser resyncs ==");
		for (k = 0; k < 8; k = k + 1) uart_send(pattern[k], 1'b1, 1'b0);
		#20000;
		read_burst(3);                         // stop inside the first frame
		send_cmd(8'h53, 8'h42, 8'hA5, 8'h5A);  // ready check; parser reset
		f0 = nf;
		uart_send(8'h77, 1'b1, 1'b0);
		#20000;
		drain;
		ok = (n_ready == 2) && (nf > f0);
		for (i = f0; i < nf; i = i + 1)        // everything parsed must be well formed
			if (fr_flags[i] != 8'h04 || fr_len[i] != 2 || fr_pay[fr_off[i]] != 8'h01) ok = 0;
		if (ok && fr_pay[fr_off[nf-1]+1] == 8'h77)
			pass("ready ACK found, every later frame well formed, [01 77] last");
		else fail("resync after mid-frame command");

		$display("== overflow: unread frames fill the FIFO, then DROPPED ==");
		// 25 frames x 7 bytes = 175 > ~120 usable bytes; nothing is read.
		f0 = nf;
		for (k = 0; k < 25; k = k + 1) uart_send(8'h80 + k, 1'b1, 1'b0);
		#20000;
		drain;
		$display("   %0d of 25 frames delivered", nf - f0);
		// ~120 usable bytes / 7 per frame = 17.  This test also runs the
		// write pointer across its wrap, which is where a sizing bug in the
		// space check used to drop frames with the FIFO half empty.
		ok = (nf - f0 >= 15) && (nf - f0 < 25);
		for (i = f0; i < nf; i = i + 1)              // complete, in order, no flag yet
			if (fr_flags[i] != 8'h04 || fr_len[i] != 2 || fr_pay[fr_off[i]+1] != 8'h80 + (i - f0)) ok = 0;
		if (ok) pass("FIFO filled to capacity across the pointer wrap; rest dropped whole");
		else fail("overflow: delivered frames");
		f0 = nf;
		uart_send(8'h7E, 1'b1, 1'b0);
		#20000;
		drain;
		e[0]=8'h01; e[1]=8'h7E;
		check_frame(f0, 8'h84, 2, "next frame carries DROPPED: flags 84");

		$display("== invalid mode 53 42 4D 07 ==");
		k = n_mode;
		send_cmd(8'h53, 8'h42, 8'h4D, 8'h07);
		read_burst(16);
		if (n_mode == k && dut.proto == 3'd4) pass("not ACKed, still UART");
		else fail("invalid mode handling");

		$display("== switch flushes queued frames ==");
		for (k = 0; k < 8; k = k + 1) uart_send(pattern[k], 1'b1, 1'b0);
		uart_send(8'h33, 1'b1, 1'b0);
		#20000;
		mode_uart = 1'b0;
		f0 = nf;
		send_cmd(8'h53, 8'h42, 8'h4D, 8'h00);
		drain;
		if (last_mode == 8'h00 && nf == f0) pass("ACK received, no stale UART frames");
		else fail("flush on switch");
		end

		$display("");
		$display("parser totals: %0d frames, %0d mode ACKs, %0d ready ACKs", nf, n_mode, n_ready);
		if (dump_fd) $fclose(dump_fd);
		if (errors == 0) $display("ALL TESTS PASSED");
		else             $display("%0d FAILURE(S)", errors);
		$finish;
	end

	initial begin
		#900_000_000;
		$display("TIMEOUT");
		$finish;
	end
endmodule

// ====================================================================
// Behavioural primitive models
// ====================================================================
module SB_PLL40_CORE (
	input REFERENCECLK, input RESETB, input BYPASS,
	output PLLOUTCORE, output PLLOUTGLOBAL, output LOCK
);
	parameter FEEDBACK_PATH = "SIMPLE";
	parameter DIVR = 4'd0;
	parameter DIVF = 7'd63;
	parameter DIVQ = 3'd5;
	parameter FILTER_RANGE = 3'b001;
	reg clk2 = 1'b0;
	always #20.8333 clk2 = ~clk2;        // 24 MHz
	reg locked = 1'b0;
	initial begin #5000; locked = 1'b1; end
	assign PLLOUTCORE = clk2, PLLOUTGLOBAL = clk2, LOCK = locked;
endmodule

module SB_GB_IO (input PACKAGE_PIN, output GLOBAL_BUFFER_OUTPUT, output D_IN_0);
	parameter PIN_TYPE = 6'b0000_01;
	parameter PULLUP = 1'b0;
	assign GLOBAL_BUFFER_OUTPUT = PACKAGE_PIN, D_IN_0 = PACKAGE_PIN;
endmodule

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
