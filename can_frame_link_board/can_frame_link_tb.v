`timescale 1ns/1ps
/*
 * can_frame_link_tb.v — end-to-end testbench for can_frame_link.v
 *   (CAN + UART0/1/2 + UART3 on PROBE1 + PROBE2 edge counter -> records ->
 *    framed SPI link)
 *
 * Bus side: the testbench is a CAN transmitter (plus an acknowledging node),
 * four UART transmitters and a PROBE2 edge source, all running concurrently.
 * CAN frames are built bit by bit here — CRC-15 and bit stuffing computed
 * independently of the RTL — at 1 Mbit/s (optionally off-nominal by CAN_PPM).
 * Each UART transmitter runs at its own baud and ppm offset (default: UART0-2
 * at 9600 baud, transmitters at +2%, -2% and exact; UART3 at 115200, tx
 * +1.5%; a second run uses other bauds) and sends random bytes, back-to-back
 * bursts, a framing error, a break and a genuine 0x00.  UART3 also carries
 * the Pi's 16-byte 'PB' packets back to back (CONTRACT §7.3).
 *
 * PROBE2: bursts of known rising-edge counts (1..1000, periods 3-40 us),
 * injected sub-microsecond glitches (100-850 ns, in both levels, in idle and
 * inside bursts, as contact bounce before an edge, and one run of 300 in one
 * heartbeat window to saturate the u8), all kept >= 3 us away from a heartbeat
 * boundary so the expected values are exact.  The testbench predicts every
 * counter record on its own (level, rise, glitch, timestamp = k * 100000 us)
 * at t_por + k * 100 ms - 2 us; the DUT must deliver exactly those bytes, and
 * load each one within 1 us of t_por + k * 100 ms (heartbeat period).
 *
 * SPI side: the testbench is the ESP32-H2 following the MCU rules of
 * spi_frame_stream/README.md (control block, A5 5A 01 + fit + CRC-16 check,
 * resync on first contact / ACK_REJECTED, skip already-received samples, ack
 * start+count).  Payload bytes are appended to a stream and split into records
 * by marker (0xAA -> 20 bytes, 0xC5 -> 8 bytes, 0xE7 -> 12 bytes), because a
 * record may continue in the next frame when read_len limits the frame.
 *
 * Checks:
 *  - every record is byte-exact against what the testbench transmitted
 *    (timestamps: mirrored from the DUT's latch, and independently within
 *    +-2 units of elapsed time since reset);
 *  - each source's records arrive exactly once and in order;
 *  - GLOBAL order: the DUT's holding-slot loads are numbered as they happen
 *    (ties in one clock in CAN, UART0..3, COUNTER order); delivered records
 *    must carry strictly increasing numbers, i.e. arrival order is preserved
 *    across record types and the arbiter never reorders or splits;
 *  - a gap in those numbers is a dropped record (overflow phase only); the
 *    next delivered record must carry `dropped`=1, and only then; the frame
 *    header's `dropped` (records) must equal the total gap;
 *  - the arbiter's slots never overrun, the link is never offered a record
 *    while busy, SPRAM reads/writes stay inside the buffered region;
 *  - every compare is non-vacuous: each source must deliver > 0 records, and
 *    the phases that exercise UART3 / the counter must deliver some.
 *
 * Source numbering (also the `R <src>` field of +records): 0 CAN, 1..4 UART
 * chan 0..3, 5 COUNTER.
 *
 * Plusargs: +dump=<file> (phase-A transfers, T/M/S/E format of the partner's
 * frames.txt), +records=<file> (delivered records in order), +esp=<file>
 * (C/U/E/P replay vectors), +vcd.
 */
module can_frame_link_tb;

	parameter integer BUF_AW         = 16;
	parameter [31:0]  INITIAL_OFFSET = 32'd0;
	parameter integer CAN_PPM        = 0;
	parameter integer READ_LEN       = 1024;
	parameter integer U0_BAUD        = 9_600;     // DUT parameters (board default, CONTRACT §6)
	parameter integer U1_BAUD        = 9_600;
	parameter integer U2_BAUD        = 9_600;
	parameter integer U3_BAUD        = 115_200;   // PROBE1 (CONTRACT §7.1)
	parameter integer U0_PPM         = 20_000;    // transmitter offsets: +2%
	parameter integer U1_PPM         = -20_000;   //                      -2%
	parameter integer U2_PPM         = 0;
	parameter integer U3_PPM         = 15_000;    //                      +1.5%
	parameter integer PI_PACKETS     = 80;        // phase D: back-to-back Pi packets on UART3

	localparam real    BIT_NS   = 1000.0 * (1.0 - CAN_PPM * 1.0e-6);
	localparam real    ppm_abs  = ((CAN_PPM < 0) ? -CAN_PPM : CAN_PPM) * 1.0e-6;
	localparam integer NSRC     = 9;               // 0 CAN, 1..4 UART0..3, 5 COUNTER, 6 I2C, 7 SPI, 8 DIO
	localparam integer CNT      = 5;
	localparam integer I2C      = 6;
	localparam integer SPI      = 7;
	localparam integer DIO      = 8;
	localparam integer MAXR     = 4096;            // records tracked per source
	localparam real    HB_NS    = 1.0e8;           // heartbeat period, 100 ms

	localparam [7:0] ST_ACK_APPLIED  = 8'h01;
	localparam [7:0] ST_CTRL_BAD     = 8'h02;
	localparam [7:0] ST_ACK_REJECTED = 8'h04;
	localparam [7:0] ST_DROPPED      = 8'h08;

	// 12 MHz with no accumulated error at 1 ps resolution: a flat #41.6666667
	// rounds to 41.667 ns (8 ppm slow, 2 us by 250 ms, which the timestamp
	// checks catch).  Six half-periods sum to exactly 250.000 ns = 3 periods.
	reg  clk_12m = 1'b0;
	always begin
		#41.667 clk_12m = ~clk_12m;  #41.667 clk_12m = ~clk_12m;
		#41.666 clk_12m = ~clk_12m;  #41.667 clk_12m = ~clk_12m;
		#41.667 clk_12m = ~clk_12m;  #41.666 clk_12m = ~clk_12m;
	end

	reg  can_rx   = 1'b1;
	reg  [3:0] urx = 4'b1111;
	reg  p2       = 1'b0;                           // PROBE2, rests low (Pi GPIO5)
	reg  [5:0] p38 = 6'd0;                          // PROBE3..8 (bit 0 = PROBE3)
	reg  spi_sck  = 1'b0;
	reg  spi_cs   = 1'b1;
	reg  spi_mosi = 1'b0;
	wire spi_miso;

	top #(
		.BUF_AW(BUF_AW), .INITIAL_OFFSET(INITIAL_OFFSET)
	) dut (
		.clk_12m(clk_12m), .can_rx_pin(can_rx),
		.uart0_pin(urx[0]), .uart1_pin(urx[1]), .uart2_pin(urx[2]),
		.probe1_pin(urx[3]), .probe2_pin(p2),
		.probe3_pin(p38[0]), .probe4_pin(p38[1]), .probe5_pin(p38[2]),
		.probe6_pin(p38[3]), .probe7_pin(p38[4]), .probe8_pin(p38[5]),
		.spi_sck(spi_sck), .spi_cs(spi_cs), .spi_mosi(spi_mosi), .spi_miso(spi_miso)
	);

	integer errors = 0;
	integer esp_fd = 0;                             // +esp=<file>: P/C/U/E vectors
	integer HALF   = 500;                           // ns: 1 MHz SCK
	integer RL     = READ_LEN;                      // read length the ESP model uses
	real    t_por  = 0.0;
	reg     por_seen = 1'b0;
	real    t_drain_chk = 0.0;                      // when drain() last found everything drained

	// ------------------------------------------------------------------
	// Expected records per source (0 = CAN, 1..4 = UART chan 0..3,
	// 5 = COUNTER).  Content is built by the transmitters (the counter record
	// entirely, timestamp included, by the PROBE2 model); the CAN/UART
	// timestamp and the global load number are filled in when the DUT loads
	// its holding slot.
	// ------------------------------------------------------------------
	reg [7:0]  exp_rec [0:NSRC-1][0:MAXR-1][0:19];
	reg [31:0] exp_ts  [0:NSRC-1][0:MAXR-1];
	real       exp_t0  [0:NSRC-1][0:MAXR-1];   // tb time the record's timestamp refers to
	real       exp_tl  [0:NSRC-1][0:MAXR-1];   // tb time the DUT loaded it
	integer    exp_seq [0:NSRC-1][0:MAXR-1];
	integer    n_exp   [0:NSRC-1];             // records the transmitter produced
	integer    n_load  [0:NSRC-1];             // records the DUT loaded into its slot
	integer    n_rx    [0:NSRC-1];             // next record expected at the MCU
	integer    gseq    = 0;
	real       can_sof_t [0:MAXR-1];
	integer    n_sof   = 0;
	integer    k0;
	initial for (k0 = 0; k0 < NSRC; k0 = k0 + 1) begin n_exp[k0] = 0; n_load[k0] = 0; n_rx[k0] = 0; end

	function integer rec_len_of(input integer src);
		rec_len_of = (src == 0) ? 20 : (src == CNT) ? 12 : (src == SPI || src == DIO) ? 10 : 8;
	endfunction

	// Source of the PROBE slot (DUT slot 4) by the DUT's probe mode.
	function integer probe_src(input [1:0] m);
		probe_src = (m == 2'd0) ? 4 : (m == 2'd1) ? DIO : (m == 2'd2) ? SPI : I2C;
	endfunction

	// CAN SOF time (mirror of the RTL latch condition).
	always @(posedge clk_12m)
		if (dut.idle_prev && !dut.bus_idle) begin
			can_sof_t[n_sof % MAXR] = $realtime;
			n_sof = n_sof + 1;
		end

	// Number slot loads in the order the DUT performs them.
	integer sl, s;
	real    hb_err;
	always @(posedge clk_12m) begin
		for (sl = 0; sl < 6; sl = sl + 1)
			if (dut.load[sl]) begin
				s = (sl == 4) ? probe_src(dut.mode_q) : sl;
				if (n_load[s] >= n_exp[s]) begin
					$display("FAIL: source %0d loaded a record the testbench never sent (t = %0.3f ms)", s, $realtime / 1e6);
					errors = errors + 1;
				end else begin
					exp_seq[s][n_load[s]] = gseq;
					exp_tl[s][n_load[s]]  = $realtime;
					case (s)
						0: exp_ts[s][n_load[s]] = dut.ts_latch;
						1: exp_ts[s][n_load[s]] = dut.u_uart0.ts;
						2: exp_ts[s][n_load[s]] = dut.u_uart1.ts;
						3: exp_ts[s][n_load[s]] = dut.u_uart2.ts;
						4: exp_ts[s][n_load[s]] = dut.u_uart3.ts;
						CNT: begin
							// Counter: content predicted by the model.  Heartbeat
							// period: each record must load within 1 us of the
							// 100 ms boundary the model predicted it for.
							exp_ts[s][n_load[s]] = dut.us_ctr;
							hb_err = $realtime - exp_t0[s][n_load[s]];
							if (hb_err < 0.0 || hb_err > 1000.0) begin
								$display("FAIL: counter record %0d loaded %0.1f ns from its 100 ms boundary", n_load[s], hb_err);
								errors = errors + 1;
							end
						end
						default: begin
							// I2C / SPI / DIO: content from the stimulus models,
							// timestamp mirrored from the DUT (checked against
							// elapsed time on delivery).
							exp_ts[s][n_load[s]] = dut.pr_ts;
							if (s == DIO && exp_rec[DIO][n_load[s]][1][0]) begin
								hb_err = $realtime - exp_t0[s][n_load[s]];
								if (hb_err < 0.0 || hb_err > 1000.0) begin
									$display("FAIL: DIO heartbeat %0d loaded %0.1f ns from its 100 ms boundary", n_load[s], hb_err);
									errors = errors + 1;
								end
							end
						end
					endcase
					if (s == 0) exp_t0[0][n_load[0]] = can_sof_t[(n_sof - 1) % MAXR];
				end
				n_load[s] = n_load[s] + 1;
				gseq = gseq + 1;
			end
		if (dut.overrun != 6'd0) begin
			$display("FAIL: holding slot overrun %b", dut.overrun); errors = errors + 1;
		end
		if (dut.u_link.rec_req && dut.u_link.rec_busy) begin
			$display("FAIL: link offered a record while busy"); errors = errors + 1;
		end
		if (dut.u_link.ram_re &&
		    ((dut.u_link.fetch_off - dut.u_link.rd_off) >= (dut.u_link.wr_off - dut.u_link.rd_off))) begin
			$display("FAIL: SPRAM read outside buffered data"); errors = errors + 1;
		end
		if (dut.u_link.ram_we && ((dut.u_link.wr_ptr - dut.u_link.rd_off) >= (32'd1 << BUF_AW))) begin
			$display("FAIL: SPRAM write over unacknowledged data"); errors = errors + 1;
		end
	end

	// Records as the DUT pushes them, captured at the SPRAM write port
	// (C = 20-byte CAN, U = 8-byte UART, E = 12-byte edge counter), for the
	// ESP replay file.
	reg [7:0] push_b [0:19];
	integer   push_n = 0, pq;
	always @(posedge clk_12m)
		if (dut.u_link.ram_we) begin
			push_b[2 * dut.u_link.w_idx]     = dut.u_link.rec_sample[7:0];
			push_b[2 * dut.u_link.w_idx + 1] = dut.u_link.rec_sample[15:8];
			if (dut.u_link.w_idx == dut.u_link.w_len - 1) begin
				push_n = push_n + 1;
				if (esp_fd != 0) begin
					case (push_b[0])                                       // (a ternary of strings
						8'hAA:   $fwrite(esp_fd, "C");                       //  prints as a number)
						8'hE7:   $fwrite(esp_fd, "E");
						8'h1C:   $fwrite(esp_fd, "I");
						8'h5B:   $fwrite(esp_fd, "S");
						8'hD1:   $fwrite(esp_fd, "D");
						default: $fwrite(esp_fd, "U");
					endcase
					for (pq = 0; pq < 2 * dut.u_link.w_len; pq = pq + 1) $fwrite(esp_fd, " %02x", push_b[pq]);
					$fwrite(esp_fd, "\n");
				end
			end
		end

	// ------------------------------------------------------------------
	// CAN transmitter
	// ------------------------------------------------------------------
	function [14:0] crc15_step(input [14:0] crc, input b);
		reg fb;
		begin
			fb = b ^ crc[14];
			crc15_step = {crc[13:0], 1'b0} ^ (fb ? 15'h4599 : 15'h0000);
		end
	endfunction

	reg     raw [0:255];
	integer n_raw;
	task push_raw(input b); begin raw[n_raw] = b; n_raw = n_raw + 1; end endtask
	task drive_bit(input b); begin can_rx = b; #(BIT_NS); end endtask

	task send_frame(input ide, input rtr, input [28:0] id, input [3:0] dlc,
	                input [63:0] data, input bad_crc);
		integer i, nbytes, run, n;
		reg [14:0] crc;
		reg last;
		reg [63:0] dexp;
		begin
			n_raw  = 0;
			nbytes = rtr ? 0 : ((dlc > 8) ? 8 : dlc);
			push_raw(1'b0);
			if (!ide) begin
				for (i = 10; i >= 0; i = i - 1) push_raw(id[i]);
				push_raw(rtr); push_raw(1'b0); push_raw(1'b0);
			end else begin
				for (i = 28; i >= 18; i = i - 1) push_raw(id[i]);
				push_raw(1'b1); push_raw(1'b1);
				for (i = 17; i >= 0; i = i - 1) push_raw(id[i]);
				push_raw(rtr); push_raw(1'b0); push_raw(1'b0);
			end
			for (i = 3; i >= 0; i = i - 1) push_raw(dlc[i]);
			for (i = 63; i >= 64 - 8 * nbytes; i = i - 1) push_raw(data[i]);
			crc = 15'd0;
			for (i = 0; i < n_raw; i = i + 1) crc = crc15_step(crc, raw[i]);
			if (bad_crc) crc = crc ^ 15'h0001;
			for (i = 14; i >= 0; i = i - 1) push_raw(crc[i]);

			n = n_exp[0];
			dexp = 64'd0;
			for (i = 0; i < nbytes; i = i + 1) dexp[63 - 8 * i -: 8] = data[63 - 8 * i -: 8];
			exp_rec[0][n][0]  = 8'hAA;
			exp_rec[0][n][1]  = {ide, rtr, 1'b0, 1'b1, ~bad_crc, bad_crc ? 3'd2 : 3'd0};
			exp_rec[0][n][2]  = {3'b000, ide ? id[28:24] : 5'd0};
			exp_rec[0][n][3]  = ide ? id[23:16] : 8'h00;
			exp_rec[0][n][4]  = ide ? id[15:8]  : {5'd0, id[10:8]};
			exp_rec[0][n][5]  = id[7:0];
			exp_rec[0][n][6]  = {4'b0000, dlc};
			for (i = 0; i < 8; i = i + 1) exp_rec[0][n][7 + i] = dexp[63 - 8 * i -: 8];
			exp_rec[0][n][19] = 8'h55;
			n_exp[0] = n + 1;

			run = 0; last = 1'b1;
			for (i = 0; i < n_raw; i = i + 1) begin
				drive_bit(raw[i]);
				if (raw[i] == last) run = run + 1; else run = 1;
				last = raw[i];
				if (run == 5) begin drive_bit(~last); last = ~last; run = 1; end
			end
			drive_bit(1'b1);                                 // CRC delimiter
			drive_bit(1'b0);                                 // ACK from another node
			drive_bit(1'b1);                                 // ACK delimiter
			for (i = 0; i < 10; i = i + 1) drive_bit(1'b1);  // EOF + intermission
		end
	endtask

	reg [31:0] lfsr = 32'h1BADF00D;
	function [31:0] rnd(input dummy);
		begin
			lfsr = {lfsr[30:0], lfsr[31] ^ lfsr[21] ^ lfsr[1] ^ lfsr[0]};
			rnd  = lfsr;
		end
	endfunction

	task send_mix(input integer n, input integer bad_at, input integer idle_bits);
		integer k;
		reg [31:0] r;
		reg [3:0]  dlc;
		reg        ide, rtr;
		reg [28:0] id;
		reg [63:0] d;
		begin
			for (k = 0; k < n; k = k + 1) begin
				r   = rnd(0);
				ide = r[0];
				rtr = (r[3:1] == 3'd5);
				dlc = (r[7:4] > 4'd8) ? ((r[7:4] == 4'd12) ? 4'd12 : r[7:4] - 4'd8) : r[7:4];
				id  = ide ? rnd(0) : {18'd0, rnd(0) & 32'h7FF};
				d   = {rnd(0), rnd(0)};
				if (k % 7 == 3) d = 64'h0000_0000_FFFF_FFFF;
				send_frame(ide, rtr, id, dlc, d, (k == bad_at));
				repeat (idle_bits) drive_bit(1'b1);
			end
		end
	endtask

	// ------------------------------------------------------------------
	// UART transmitters.  kind: 0 normal, 1 framing error (stop bit low),
	// 2 break (line low for 2.5 frames).
	// ------------------------------------------------------------------
	reg [31:0] ulfsr [0:3];
	initial begin ulfsr[0] = 32'h00C0FFEE; ulfsr[1] = 32'h0BADCAFE; ulfsr[2] = 32'h5EED1234; ulfsr[3] = 32'h7E57D00D; end

	// Live transmitter bauds (the DUT's divisors are set to match by C3
	// config blocks; see send_cfg).
	integer u_baud [0:3];
	initial begin u_baud[0] = U0_BAUD; u_baud[1] = U1_BAUD; u_baud[2] = U2_BAUD; u_baud[3] = U3_BAUD; end
	function real ubit_ns(input integer ch);
		begin
			case (ch)
				0: ubit_ns = 1.0e9 / u_baud[0] / (1.0 + U0_PPM * 1.0e-6);
				1: ubit_ns = 1.0e9 / u_baud[1] / (1.0 + U1_PPM * 1.0e-6);
				2: ubit_ns = 1.0e9 / u_baud[2] / (1.0 + U2_PPM * 1.0e-6);
				default: ubit_ns = 1.0e9 / u_baud[3] / (1.0 + U3_PPM * 1.0e-6);
			endcase
		end
	endfunction

	task automatic uart_send(input integer ch, input [7:0] d, input integer kind);
		integer i, n;
		real tb;
		begin
			tb = ubit_ns(ch);
			n  = n_exp[ch + 1];
			exp_rec[ch + 1][n][0] = 8'hC5;
			exp_rec[ch + 1][n][1] = {ch[1:0], kind == 1, kind == 2, 1'b0, 3'b000};
			exp_rec[ch + 1][n][2] = (kind == 2) ? 8'h00 : d;
			exp_rec[ch + 1][n][7] = 8'h5A;
			exp_t0[ch + 1][n]     = $realtime;
			n_exp[ch + 1] = n + 1;
			if (kind == 2) begin
				urx[ch] = 1'b0; #(25.0 * tb);
				urx[ch] = 1'b1; #(2.0 * tb);
			end else begin
				urx[ch] = 1'b0; #(tb);
				for (i = 0; i < 8; i = i + 1) begin urx[ch] = d[i]; #(tb); end
				urx[ch] = (kind == 1) ? 1'b0 : 1'b1; #(tb);
				if (kind == 1) begin urx[ch] = 1'b1; #(2.0 * tb); end
			end
		end
	endtask

	// n bytes: random data, random gaps (0 = back to back), special kinds at
	// the given positions (-1 = none).  A data byte 0x00 is sent normally at
	// zero_at to show it is not mistaken for a break.
	task automatic uart_script(input integer ch, input integer n, input integer fe_at,
	                 input integer brk_at, input integer zero_at);
		integer k, gap;
		reg [7:0] d;
		begin
			for (k = 0; k < n; k = k + 1) begin
				ulfsr[ch] = {ulfsr[ch][30:0], ulfsr[ch][31] ^ ulfsr[ch][21] ^ ulfsr[ch][1] ^ ulfsr[ch][0]};
				d   = ulfsr[ch][7:0];
				if (k == zero_at) d = 8'h00;
				if (k == fe_at && d == 8'h00) d = 8'h5A;   // a framing error needs a nonzero byte
				gap = (ulfsr[ch][11:8] < 6) ? 0 : ulfsr[ch][11:8];
				uart_send(ch, d, (k == fe_at) ? 1 : (k == brk_at) ? 2 : 0);
				#(gap * ubit_ns(ch));
			end
		end
	endtask

	// Pi P1.9 stream (CONTRACT §7.3): 'P' 'B' | seq u32 LE | t_ms u32 LE |
	// xorshift32(seq) u32 LE | CRC-16/CCITT-FALSE over bytes 0-13, LE.
	integer pi_seq = 0;
	function [31:0] xorshift32(input [31:0] x0);
		reg [31:0] x;
		begin
			x = x0;
			x = x ^ (x << 13);
			x = x ^ (x >> 17);
			x = x ^ (x << 5);
			xorshift32 = x;
		end
	endfunction

	task automatic pi_packets(input integer npk, input integer gap_bytes);
		integer p, i;
		reg [7:0]  b [0:15];
		reg [31:0] t_ms, pat;
		reg [15:0] c;
		begin
			for (p = 0; p < npk; p = p + 1) begin
				t_ms = $realtime / 1.0e6;
				pat  = xorshift32(pi_seq);
				b[0] = 8'h50; b[1] = 8'h42;
				for (i = 0; i < 4; i = i + 1) begin
					b[2 + i]  = pi_seq >> (8 * i);
					b[6 + i]  = t_ms   >> (8 * i);
					b[10 + i] = pat    >> (8 * i);
				end
				c = 16'hFFFF;
				for (i = 0; i < 14; i = i + 1) c = tb_crc(c, b[i]);
				b[14] = c[7:0]; b[15] = c[15:8];
				for (i = 0; i < 16; i = i + 1) uart_send(3, b[i], 0);
				pi_seq = pi_seq + 1;
				#(gap_bytes * 10.0 * ubit_ns(3));
			end
		end
	endtask

	// ------------------------------------------------------------------
	// PROBE2 source and the testbench's own counter model.  Every pin
	// change goes through p2_guard so that nothing is in flight within
	// 8 us before / 3 us after a heartbeat boundary: the snapshot taken
	// 2 us before the boundary is then exactly what the DUT must report.
	// ------------------------------------------------------------------
	integer tb_rise = 0, tb_glw = 0, tb_gl_total = 0, tb_bursts = 0;
	reg     tb_lvl  = 1'b0;
	reg [7:0] tb_dio = 8'd0;                        // DIO model: levels of the last expected D1 record
	integer n_dio_hb = 0;
	reg     p2_en   = 1'b1;                         // PROBE2 random bursts allowed (mode 0 only)
	reg     p2_busy = 1'b0;
	reg     p2_det_done = 1'b0;                     // the scripted PROBE2 sequence has finished
	reg [31:0] plfsr = 32'h0DDBA11F;
	function [31:0] prnd(input dummy);
		begin
			plfsr = {plfsr[30:0], plfsr[31] ^ plfsr[21] ^ plfsr[1] ^ plfsr[0]};
			prnd  = plfsr;
		end
	endfunction

	function real hb_phase(input dummy);
		real e;
		begin
			e = $realtime - t_por;
			hb_phase = e - HB_NS * $floor(e / HB_NS);
		end
	endfunction

	task p2_guard;
		real ph;
		begin
			ph = hb_phase(0);
			if (ph > HB_NS - 8000.0) #(HB_NS - ph + 3000.0);
			else if (ph < 3000.0)    #(3000.0 - ph);
		end
	endtask

	task p2_set(input v);              // a real (filter-accepted) level change
		begin
			p2_guard;
			p2 = v;
			if (v && !tb_lvl) tb_rise = tb_rise + 1;
			tb_lvl = v;
		end
	endtask

	task p2_glitch(input real w);      // excursion of w ns (100..850) away from the level, then back
		begin
			p2_guard;
			p2 = ~tb_lvl;  #(w);
			p2 = tb_lvl;   #(150.0);
			tb_glw = tb_glw + 1;
			tb_gl_total = tb_gl_total + 1;
		end
	endtask

	function real gw(input dummy);     // glitch width 100..850 ns
		gw = 100.0 + (prnd(0) % 751);
	endfunction

	// n rising edges, hi/lo ns per half; every gl_every-th period gets a glitch
	// in the middle of its high half and of its low half (0 = none).
	task p2_burst(input integer n, input real hi, input real lo, input integer gl_every);
		integer i;
		begin
			for (i = 0; i < n; i = i + 1) begin
				p2_set(1'b1);
				if (gl_every != 0 && i % gl_every == gl_every - 1) begin
					#(hi / 2.0); p2_glitch(gw(0)); #(hi / 2.0);
				end else #(hi);
				p2_set(1'b0);
				if (gl_every != 0 && i % gl_every == gl_every - 1) begin
					#(lo / 2.0); p2_glitch(gw(0)); #(lo / 2.0);
				end else #(lo);
			end
			tb_bursts = tb_bursts + 1;
		end
	endtask

	// Heartbeat model: snapshot 2 us before each boundary (no stimulus then).
	// Mode 0: a counter record (CONTRACT §7.1).  Mode 1: a DIO heartbeat
	// (levels now, changed 0, ts = k * 100000).  Modes 2/3: nothing.  Mode
	// changes are kept >= 10 us away from a boundary (cfg_xfer).
	integer hb_k = 0, hb_n;
	reg [1:0] hb_m;
	reg [31:0] hb_ts;
	always begin : hb_model
		wait (por_seen);
		hb_k = hb_k + 1;
		#(t_por + hb_k * HB_NS - 2000.0 - $realtime);
		hb_m  = dut.mode_q;
		hb_ts = hb_k * 100000;
		if (hb_m == 2'd0) begin
			hb_n = n_exp[CNT];
			exp_rec[CNT][hb_n][0]  = 8'hE7;
			exp_rec[CNT][hb_n][1]  = {tb_lvl, 7'b000_0000};
			exp_rec[CNT][hb_n][2]  = tb_rise >> 24;
			exp_rec[CNT][hb_n][3]  = tb_rise >> 16;
			exp_rec[CNT][hb_n][4]  = tb_rise >> 8;
			exp_rec[CNT][hb_n][5]  = tb_rise;
			exp_rec[CNT][hb_n][6]  = hb_ts >> 24;
			exp_rec[CNT][hb_n][7]  = hb_ts >> 16;
			exp_rec[CNT][hb_n][8]  = hb_ts >> 8;
			exp_rec[CNT][hb_n][9]  = hb_ts;
			exp_rec[CNT][hb_n][10] = (tb_glw > 255) ? 8'd255 : tb_glw;
			exp_rec[CNT][hb_n][11] = 8'h7E;
			exp_ts[CNT][hb_n]      = hb_ts;
			exp_t0[CNT][hb_n]      = t_por + hb_k * HB_NS;
			tb_glw = 0;
			n_exp[CNT] = hb_n + 1;
		end else if (hb_m == 2'd1) begin
			hb_n = n_exp[DIO];
			exp_rec[DIO][hb_n][0] = 8'hD1;
			exp_rec[DIO][hb_n][1] = 8'h01;              // heartbeat
			exp_rec[DIO][hb_n][2] = tb_dio;
			exp_rec[DIO][hb_n][3] = 8'h00;
			exp_rec[DIO][hb_n][4] = hb_ts >> 24;
			exp_rec[DIO][hb_n][5] = hb_ts >> 16;
			exp_rec[DIO][hb_n][6] = hb_ts >> 8;
			exp_rec[DIO][hb_n][7] = hb_ts;
			exp_rec[DIO][hb_n][8] = 8'h00;
			exp_rec[DIO][hb_n][9] = 8'h1D;
			exp_t0[DIO][hb_n]     = t_por + hb_k * HB_NS;
			n_exp[DIO] = hb_n + 1;
			n_dio_hb = n_dio_hb + 1;
		end
	end

	integer sat_done = 0;
	initial begin : probe2_stim
		integer j;
		wait (por_seen);
		#(30_000);
		p2_burst(37, 10_000, 10_000, 5);                 // 37 edges, glitches in both levels
		for (j = 0; j < 12; j = j + 1) begin             // idle glitches
			#(20_000 + (prnd(0) % 180_000));
			p2_glitch(gw(0));
		end
		p2_burst(1000, 10_000, 10_000, 97);              // 1000 edges at 50 kHz (Pi: 1000 at 500 Hz)
		#(200_000);
		for (j = 0; j < 20; j = j + 1) begin             // contact bounce before each rising edge
			p2_glitch(300.0); #(300.0);
			p2_set(1'b1); #(5_000);
			p2_set(1'b0); #(5_000);
		end
		tb_bursts = tb_bursts + 1;
		p2_burst(200, 1_500, 1_500, 0);                  // 18-clock halves: accepted
		// Saturation: 300 glitches inside one heartbeat window.
		wait (n_load[CNT] >= 1);
		#(10_000);
		for (j = 0; j < 300; j = j + 1) begin p2_glitch(400.0); #(250.0); end
		sat_done = 1;
		// Level 1 across a boundary (the record's level bit), with glitches low.
		#(1_000_000);
		p2_set(1'b1);
		for (j = 0; j < 5; j = j + 1) begin #(25_000_000); p2_glitch(gw(0)); end
		#(10_000_000);
		p2_set(1'b0);
		p2_det_done = 1'b1;
		forever begin                                    // random bursts and gaps (mode 0 phases only)
			#(2_000_000 + (prnd(0) % 8_000_000));
			wait (p2_en);
			p2_busy = 1'b1;
			p2_burst(1 + (prnd(0) % 200), 2_000 + (prnd(0) % 18_000), 2_000 + (prnd(0) % 18_000),
			         (prnd(0) % 4 != 0) ? 0 : 25 + (prnd(0) % 40));
			if (prnd(0) % 2) begin #(50_000); p2_glitch(gw(0)); end
			p2_busy = 1'b0;
		end
	end

	// ------------------------------------------------------------------
	// MCU model
	// ------------------------------------------------------------------
	function [15:0] tb_crc(input [15:0] crc, input [7:0] b);
		integer i;
		reg fb;
		begin
			tb_crc = crc;
			for (i = 7; i >= 0; i = i - 1) begin
				fb     = tb_crc[15] ^ b[i];
				tb_crc = {tb_crc[14:0], 1'b0} ^ (fb ? 16'h1021 : 16'h0000);
			end
		end
	endfunction

	reg        synced   = 1'b0;
	reg [31:0] expected = 32'd0;
	reg [7:0]  mosi_buf [0:4095];
	reg [7:0]  miso_buf [0:4095];
	integer    bytes_clocked;
	reg        fr_ok;
	reg [31:0] fr_start, fr_dropped;
	reg [15:0] fr_count;
	reg [7:0]  fr_status;

	// Byte stream reassembly
	reg [7:0]  sbuf [0:65535];
	integer    s_head = 0, s_tail = 0;     // free-running; index & 16'hFFFF

	integer    drops_ok   = 0;
	integer    recs_ok    = 0;
	integer    recs_by    [0:NSRC-1];
	integer    gaps_total = 0;             // records missing in the global numbering
	integer    last_seq   = -1;
	integer    dup_samp   = 0;
	integer    split_recs = 0;             // records that straddled two frames
	integer    n_fe = 0, n_brk = 0;
	integer    cnt_sat = 0, cnt_glitch = 0, cnt_lvl1 = 0;   // delivered counter records: glitch=255, sum, level=1
	reg [31:0] cnt_rise = 0;
	initial for (k0 = 0; k0 < NSRC; k0 = k0 + 1) recs_by[k0] = 0;

	integer dump_fd = 0, rec_fd = 0, v3_fd = 0, dumping = 0;
	integer n_echo_ok = 0;

	// Applied config as the testbench expects it (C3 block, contract §3).
	reg  [7:0]  tb_gen  = 8'd0;
	reg  [1:0]  tb_mode = 2'd0;
	reg  [2:0]  tb_spi  = 3'd0;
	reg  [15:0] tb_div [0:3];
	initial begin tb_div[0] = 16'd1250; tb_div[1] = 16'd1250; tb_div[2] = 16'd1250; tb_div[3] = 16'd104; end
	// Requested config for the next send_cfg.
	reg  [7:0]  rq_gen = 8'd0, rq_mode = 8'd0, rq_spi = 8'd0;
	reg  [15:0] rq_div [0:3];
	integer     n_cfg [0:2];                     // C3 transactions: good / bad CRC / out of range
	integer     n_cfg_total = 0;
	integer     cfg_kind = 0;
	initial begin n_cfg[0] = 0; n_cfg[1] = 0; n_cfg[2] = 0; end
	reg [8*256-1:0] fname;

	task build_ctrl(input integer len, input corrupt);
		integer i;
		reg [15:0] c;
		begin
			for (i = 0; i < len; i = i + 1) mosi_buf[i] = 8'h00;
			mosi_buf[0] = 8'h5C;
			mosi_buf[1] = expected[7:0];
			mosi_buf[2] = expected[15:8];
			mosi_buf[3] = expected[23:16];
			mosi_buf[4] = expected[31:24];
			mosi_buf[5] = len % 256;
			mosi_buf[6] = len / 256;
			mosi_buf[7] = synced ? 8'h01 : 8'h00;
			c = 16'hFFFF;
			for (i = 0; i < 8; i = i + 1) c = tb_crc(c, mosi_buf[i]);
			mosi_buf[8] = c[15:8];
			mosi_buf[9] = c[7:0];
			if (corrupt) mosi_buf[3] = mosi_buf[3] ^ 8'h01;
		end
	endtask

	// C3 config block (contract §3): C3 gen mode spi_cfg div0..3 (u16 LE) crc16 BE.
	// kind 0 = valid, 1 = CRC corrupted, 2 = valid CRC but div0 = 15 (out of range).
	task build_cfg(input integer kind);
		integer i;
		reg [15:0] c, d0;
		begin
			d0 = (kind == 2) ? 16'd15 : rq_div[0];
			mosi_buf[0]  = 8'hC3;
			mosi_buf[1]  = rq_gen;
			mosi_buf[2]  = rq_mode;
			mosi_buf[3]  = rq_spi;
			mosi_buf[4]  = d0[7:0];        mosi_buf[5]  = d0[15:8];
			mosi_buf[6]  = rq_div[1][7:0]; mosi_buf[7]  = rq_div[1][15:8];
			mosi_buf[8]  = rq_div[2][7:0]; mosi_buf[9]  = rq_div[2][15:8];
			mosi_buf[10] = rq_div[3][7:0]; mosi_buf[11] = rq_div[3][15:8];
			c = 16'hFFFF;
			for (i = 0; i < 12; i = i + 1) c = tb_crc(c, mosi_buf[i]);
			mosi_buf[12] = c[15:8];
			mosi_buf[13] = c[7:0];
			if (kind == 1) mosi_buf[13] = mosi_buf[13] ^ 8'h40;
		end
	endtask

	// One C3 transaction (own CS), kept >= 10 us after / 400 us before a
	// heartbeat boundary so the mode never changes near one.  Checks that it
	// changed nothing in the stream state, and that the config was applied
	// (kind 0) or ignored (kinds 1, 2).
	reg [31:0] sn_rd, sn_hw, sn_dr, sn_wr;
	reg        sn_cg;
	task cfg_xfer(input integer kind);
		real ph;
		reg  ok;
		begin
			ph = hb_phase(0);
			if (ph > HB_NS - 400_000.0) #(HB_NS - ph + 10_000.0);
			else if (ph < 10_000.0)     #(10_000.0 - ph);
			sn_rd = dut.u_link.rd_off; sn_hw = dut.u_link.sent_hw;
			sn_dr = dut.u_link.dropped_reported; sn_cg = dut.u_link.ctrl_good;
			build_cfg(kind);
			xfer(14, -1);
			if (dut.u_link.rd_off !== sn_rd || dut.u_link.sent_hw !== sn_hw ||
			    dut.u_link.dropped_reported !== sn_dr || dut.u_link.ctrl_good !== sn_cg) begin
				$display("FAIL: C3 config transaction changed the stream state (rd %08x->%08x hw %08x->%08x)",
				         sn_rd, dut.u_link.rd_off, sn_hw, dut.u_link.sent_hw);
				errors = errors + 1;
			end
			if (kind == 0) begin
				tb_gen = rq_gen; tb_mode = rq_mode[1:0]; tb_spi = rq_spi[2:0];
				tb_div[0] = rq_div[0]; tb_div[1] = rq_div[1]; tb_div[2] = rq_div[2]; tb_div[3] = rq_div[3];
			end
			ok = (dut.cfg_gen === tb_gen) && (dut.cfg_mode === tb_mode) && (dut.cfg_spi === tb_spi) &&
			     (dut.cfg_div0 === tb_div[0]) && (dut.cfg_div1 === tb_div[1]) &&
			     (dut.cfg_div2 === tb_div[2]) && (dut.cfg_div3 === tb_div[3]);
			if (!ok) begin
				$display("FAIL: C3 config kind %0d: DUT gen %02x mode %0d spi %0d div %0d/%0d/%0d/%0d, expected gen %02x mode %0d spi %0d div %0d/%0d/%0d/%0d",
				         kind, dut.cfg_gen, dut.cfg_mode, dut.cfg_spi, dut.cfg_div0, dut.cfg_div1, dut.cfg_div2, dut.cfg_div3,
				         tb_gen, tb_mode, tb_spi, tb_div[0], tb_div[1], tb_div[2], tb_div[3]);
				errors = errors + 1;
			end
			n_cfg[kind] = n_cfg[kind] + 1;
			n_cfg_total = n_cfg_total + 1;
		end
	endtask

	task xfer(input integer len, input integer abort_at);
		integer i, b;
		reg [7:0] rx;
		begin
			spi_cs = 1'b0;
			#(2 * HALF);
			bytes_clocked = 0;
			for (i = 0; i < len && (abort_at < 0 || i < abort_at); i = i + 1) begin
				rx = 8'h00;
				for (b = 7; b >= 0; b = b - 1) begin
					spi_mosi = mosi_buf[i][b];
					#(HALF);
					spi_sck = 1'b1;
					rx[b]   = spi_miso;
					#(HALF);
					spi_sck = 1'b0;
				end
				miso_buf[i]   = rx;
				bytes_clocked = i + 1;
			end
			#(2 * HALF);
			spi_cs = 1'b1;
			#(20000);                                    // ESP inter-transaction gap
		end
	endtask

	// Check the record at the head of the stream (its length is known).
	task check_record(input integer src, input integer len, input [127:0] label);
		integer i, j, match, seq, gap, flagpos, tsb;
		reg [7:0] e, g;
		reg       flag;
		real      tsu;
		begin
			flagpos = (src == 0) ? 6 : 1;          // CAN byte 6 bit 7; every other record byte 1 bit 3
			tsb     = (src == SPI || src == DIO) ? 4 : 3;   // first timestamp byte (UART, I2C: 3)
			match = -1;
			for (j = n_rx[src]; j < n_load[src] && match < 0 && (j == n_rx[src] || drops_ok); j = j + 1) begin
				match = j;
				for (i = 0; i < len; i = i + 1) begin
					g = sbuf[(s_head + i) & 16'hFFFF];
					e = exp_rec[src][j][i];
					if (src == 0 && i >= 15 && i <= 18) e = exp_ts[0][j][8 * (18 - i) +: 8];
					if (src != 0 && src != CNT && i >= tsb && i <= tsb + 3) e = exp_ts[src][j][8 * (tsb + 3 - i) +: 8];
					if (i == flagpos) e = (src == 0) ? {g[7], e[6:0]} : {e[7:4], g[3], e[2:0]};
					if (g !== e) match = -1;
				end
			end
			if (match < 0) begin
				$display("FAIL [%0s]: source %0d record does not match its next expected (#%0d of %0d loaded):",
				         label, src, n_rx[src], n_load[src]);
				$write("   got ");
				for (i = 0; i < len; i = i + 1) $write("%02x ", sbuf[(s_head + i) & 16'hFFFF]);
				$write("\n   exp ");
				for (i = 0; i < len; i = i + 1) $write("%02x ", exp_rec[src][n_rx[src]][i]);
				if (src == CNT) $write("(predicted by the PROBE2 model)\n");   // (no string ternary:
				else            $write("(timestamp bytes from the DUT latch)\n");   //  iverilog prints a number)
				$write("   (t = %0.3f ms, expected t0 %0.3f ms)\n", $realtime / 1e6, exp_t0[src][n_rx[src]] / 1e6);
				errors = errors + 1;
				n_rx[src] = n_rx[src] + 1;
			end else begin
				seq  = exp_seq[src][match];
				gap  = seq - last_seq - 1;
				flag = (src == 0) ? sbuf[(s_head + 6) & 16'hFFFF][7] : sbuf[(s_head + 1) & 16'hFFFF][3];
				if (gap < 0) begin
					$display("FAIL [%0s]: arrival order broken (load #%0d delivered after #%0d)", label, seq, last_seq);
					errors = errors + 1;
				end else if (gap > 0 && !drops_ok) begin
					$display("FAIL [%0s]: %0d record(s) missing before load #%0d", label, gap, seq);
					errors = errors + 1;
				end
				if (flag !== (gap > 0)) begin
					$display("FAIL [%0s]: dropped flag %0d with %0d record(s) missing before it", label, flag, gap);
					errors = errors + 1;
				end
				// Independent timestamp check against elapsed time.
				// UART: microseconds, latched <= 250 ns after the edge -> within 2 us.
				// CAN: bit times; can_bit_timing restarts the bit at every hard
				// sync (SOF), so the counter may lose up to one bit per frame
				// against wall time (partner semantics, kept): bound by the SOFs.
				// The CAN counter runs at the nominal 1 us/bit while idle and tracks
				// the transmitter (CAN_PPM) during frames: allow +-|CAN_PPM| too.
				tsu = (exp_t0[src][match] - t_por) / 1000.0;
				if (src == 0) begin
					if (exp_ts[0][match] > tsu * (1.0 + ppm_abs) + 2.0 ||
					    exp_ts[0][match] < tsu * (1.0 - ppm_abs) - (match + 1) - 2.0) begin
						$display("FAIL [%0s]: CAN timestamp %0d bit times, elapsed %0.1f us", label, exp_ts[0][match], tsu);
						errors = errors + 1;
					end
				end else if (exp_ts[src][match] > tsu + 2.0 || exp_ts[src][match] + 2.0 < tsu) begin
					$display("FAIL [%0s]: source %0d timestamp %0d, elapsed %0.1f", label, src, exp_ts[src][match], tsu);
					errors = errors + 1;
				end
				if (src >= 1 && src <= 4) begin
					if (exp_rec[src][match][1][5]) n_fe  = n_fe + 1;
					if (exp_rec[src][match][1][4]) n_brk = n_brk + 1;
				end
				if (src == CNT) begin
					if (exp_ts[CNT][match] != {exp_rec[CNT][match][6], exp_rec[CNT][match][7], exp_rec[CNT][match][8], exp_rec[CNT][match][9]}) begin
						$display("FAIL [%0s]: counter record %0d timestamp %0d us", label, match, exp_ts[CNT][match]);
						errors = errors + 1;
					end
					if (exp_rec[CNT][match][10] == 8'd255) cnt_sat = cnt_sat + 1;
					if (exp_rec[CNT][match][1][7])         cnt_lvl1 = cnt_lvl1 + 1;
					cnt_glitch = cnt_glitch + exp_rec[CNT][match][10];
					cnt_rise   = {exp_rec[CNT][match][2], exp_rec[CNT][match][3], exp_rec[CNT][match][4], exp_rec[CNT][match][5]};
				end
				if (rec_fd != 0) begin
					$fwrite(rec_fd, "R %0d %0d", src, match);
					for (i = 0; i < len; i = i + 1) $fwrite(rec_fd, " %02x", sbuf[(s_head + i) & 16'hFFFF]);
					$fwrite(rec_fd, "\n");
				end
				if (v3_fd != 0) begin
					$fwrite(v3_fd, "R");
					for (i = 0; i < len; i = i + 1) $fwrite(v3_fd, " %02x", sbuf[(s_head + i) & 16'hFFFF]);
					$fwrite(v3_fd, "\n");
				end
				gaps_total   = gaps_total + gap;
				last_seq     = seq;
				recs_ok      = recs_ok + 1;
				recs_by[src] = recs_by[src] + 1;
				n_rx[src]    = match + 1;
			end
			s_head = s_head + len;
		end
	endtask

	task consume_stream(input [127:0] label);
		integer avail, len, src;
		reg [7:0] m;
		reg done;
		begin
			done = 0;
			while (!done) begin
				avail = s_tail - s_head;
				if (avail == 0) done = 1;
				else begin
					m = sbuf[s_head & 16'hFFFF];
					if (m == 8'hAA) begin len = 20; src = 0; end
					else if (m == 8'hE7) begin len = 12; src = CNT; end
					else if (m == 8'h1C) begin len = 8;  src = I2C; end
					else if (m == 8'h5B) begin len = 10; src = SPI; end
					else if (m == 8'hD1) begin len = 10; src = DIO; end
					else if (m == 8'hC5) begin
						len = 8;
						src = (avail >= 2) ? sbuf[(s_head + 1) & 16'hFFFF][7:6] + 1 : 1;
					end else begin
						$display("FAIL [%0s]: stream desynchronised at byte %02x", label, m);
						errors = errors + 1;
						s_head = s_head + 1;
						len = 0;
					end
					if (len != 0) begin
						if (avail < len) done = 1;            // rest arrives in the next frame
						else
							check_record(src, len, label);
					end
				end
			end
		end
	endtask

	task parse(input integer len, input [127:0] label);
		integer i, n, crc_at, behind, skip;
		reg [15:0] c, got;
		begin
			fr_ok = 1'b0;
			if (miso_buf[0] !== 8'hA5 || miso_buf[1] !== 8'h5A || miso_buf[2] !== 8'h01) begin
				$display("  [%0s] bad sync %02x %02x %02x", label, miso_buf[0], miso_buf[1], miso_buf[2]);
			end else begin
				fr_start   = {miso_buf[14], miso_buf[13], miso_buf[12], miso_buf[11]};
				fr_count   = {miso_buf[16], miso_buf[15]};
				fr_dropped = {miso_buf[20], miso_buf[19], miso_buf[18], miso_buf[17]};
				fr_status  = miso_buf[21];
				n          = fr_count;
				crc_at     = 22 + 2 * n;
				if (crc_at + 2 > len)
					$display("  [%0s] %0d samples do not fit %0d bytes", label, n, len);
				else begin
					c = 16'hFFFF;
					for (i = 0; i < crc_at; i = i + 1) c = tb_crc(c, miso_buf[i]);
					got = {miso_buf[crc_at], miso_buf[crc_at + 1]};
					if (c !== got)
						$display("  [%0s] CRC mismatch: computed %04x, frame says %04x", label, c, got);
					else
						fr_ok = 1'b1;
				end
			end

			if (fr_ok) begin
				// Config echo (protocol v3): byte 3 = gen, byte 4 = probe_mode
				// of the last applied C3 block; bytes 5..10 reserved 0.
				if (miso_buf[3] !== tb_gen || miso_buf[4] !== {6'd0, tb_mode}) begin
					$display("FAIL [%0s]: config echo %02x %02x, expected gen %02x mode %0d", label, miso_buf[3], miso_buf[4], tb_gen, tb_mode);
					errors = errors + 1;
				end else
					n_echo_ok = n_echo_ok + 1;
				for (i = 5; i <= 10; i = i + 1)
					if (miso_buf[i] !== 8'h00) begin
						$display("FAIL [%0s]: reserved byte %0d = %02x", label, i, miso_buf[i]);
						errors = errors + 1;
					end
				for (i = crc_at + 2; i < len; i = i + 1)
					if (miso_buf[i] !== 8'hFF) begin
						$display("FAIL [%0s]: padding byte %0d = %02x", label, i, miso_buf[i]);
						errors = errors + 1;
					end
				if (!synced || (fr_status & ST_ACK_REJECTED)) begin
					synced   = 1'b1;
					expected = fr_start;
				end
				behind = expected - fr_start;
				if (behind < 0) begin
					$display("FAIL [%0s]: frame starts beyond what we still need", label);
					errors = errors + 1;
					behind = 0;
				end
				skip = (behind > n) ? n : behind;
				dup_samp = dup_samp + skip;
				for (i = skip; i < n; i = i + 1) begin
					sbuf[s_tail & 16'hFFFF]       = miso_buf[22 + 2 * i];
					sbuf[(s_tail + 1) & 16'hFFFF] = miso_buf[22 + 2 * i + 1];
					s_tail = s_tail + 2;
				end
				if (esp_fd != 0 && n > skip) begin
					$fwrite(esp_fd, "P");
					for (i = skip; i < n; i = i + 1)
						$fwrite(esp_fd, " %02x %02x", miso_buf[22 + 2 * i], miso_buf[22 + 2 * i + 1]);
					$fwrite(esp_fd, "\n");
				end
				if (behind < n) expected = fr_start + n;
				consume_stream(label);
				if (s_tail != s_head) split_recs = split_recs + 1;
			end
		end
	endtask

	task dump(input integer len, input [8*16-1:0] kind);
		integer i;
		begin
			if (dump_fd != 0 && dumping) begin
				$fwrite(dump_fd, "T %0d %0s\nM", len, kind);
				for (i = 0; i < 10; i = i + 1) $fwrite(dump_fd, " %02x", mosi_buf[i]);
				$fwrite(dump_fd, "\nS");
				for (i = 0; i < bytes_clocked; i = i + 1) $fwrite(dump_fd, " %02x", miso_buf[i]);
				$fwrite(dump_fd, "\nE %08x %0d %0d\n", expected, synced, gaps_total);
			end
		end
	endtask

	task poll(input integer len, input [127:0] label);
		begin
			build_ctrl(len, 1'b0);
			xfer(len, -1);
			parse(len, label);
			if (!fr_ok) begin
				$display("FAIL [%0s]: a clean transfer produced a bad frame", label);
				errors = errors + 1;
			end
			dump(len, "clean");
		end
	endtask

	function integer all_done(input dummy);
		integer q;
		begin
			all_done = 1;
			// Everything sent has reached a holding slot; the rest is checked by
			// the buffer/stream conditions in drain().  (n_rx cannot be used:
			// a source's last records may legitimately be dropped.)
			for (q = 0; q < NSRC; q = q + 1)
				if (n_load[q] < n_exp[q]) all_done = 0;
		end
	endfunction

	// Counter records loaded after drain() last found the DUT empty and not
	// yet delivered: the heartbeat never stops, so one may be in flight when
	// a phase ends.  Only those are excused from the per-phase accounting.
	function integer in_flight(input integer q);   // (heartbeat sources: COUNTER, DIO)
		integer j;
		begin
			in_flight = 0;
			if (q == CNT || q == DIO)
				for (j = n_rx[q]; j < n_load[q]; j = j + 1)
					if (exp_tl[q][j] >= t_drain_chk) in_flight = in_flight + 1;
		end
	endfunction
	function integer cnt_in_flight(input dummy);
		cnt_in_flight = in_flight(CNT) + in_flight(DIO);
	endfunction

	task drain(input [127:0] label);
		integer guard, q;
		begin
			guard = 0;
			while ((!all_done(0) || dut.u_link.level != 0 || dut.valid != 0 || s_tail != s_head) && guard < 400) begin
				poll(RL, label);
				guard = guard + 1;
			end
			t_drain_chk = $realtime;
			poll(RL, label);
			for (q = 0; q < NSRC; q = q + 1)
				if (!drops_ok && n_rx[q] + ((q == CNT || q == DIO) ? in_flight(q) + (n_exp[q] - n_load[q]) : 0) != n_exp[q]) begin
					$display("FAIL [%0s]: source %0d: %0d of %0d records accounted for", label, q, n_rx[q], n_exp[q]);
					errors = errors + 1;
				end
		end
	endtask

	// Background poller, like the ESP task.
	reg polling = 1'b0;
	reg poller_idle = 1'b1;
	integer inject = 0;          // request: 1 corrupt ctrl, 2 corrupt MISO, 3 abort
	integer cur_inj = 0;
	integer n_inj [1:4];
	initial begin n_inj[1] = 0; n_inj[2] = 0; n_inj[3] = 0; n_inj[4] = 0; end
	always begin
		wait (polling);
		poller_idle = 1'b0;
		cur_inj = inject;            // take the request at the START of a transfer
		inject  = 0;
		if (cur_inj != 0) n_inj[cur_inj] = n_inj[cur_inj] + 1;
		case (cur_inj)
			1: begin
				build_ctrl(RL, 1'b1); xfer(RL, -1); parse(RL, "ctrl corrupt");
				if (!fr_ok || !(fr_status & ST_CTRL_BAD) || fr_count != 0) begin
					$display("FAIL: corrupt control block not answered with an empty CTRL_BAD frame");
					errors = errors + 1;
				end
				dump(RL, "ctrl_corrupt");
			end
			2: begin
				build_ctrl(RL, 1'b0); xfer(RL, -1);
				miso_buf[24] = miso_buf[24] ^ 8'h20;
				parse(RL, "miso corrupt");
				if (fr_ok) begin $display("FAIL: corrupted MISO frame accepted"); errors = errors + 1; end
				dump(RL, "miso_corrupt");
			end
			3: begin
				build_ctrl(RL, 1'b0); xfer(RL, 60);
				dump(RL, "abort");
			end
			4: cfg_xfer(cfg_kind);
			default: poll(RL, "stream");
		endcase
		poller_idle = 1'b1;
		#1;
	end

	task stop_poller;
		begin polling = 1'b0; wait (poller_idle); end
	endtask

	// ------------------------------------------------------------------
	// Protocol v3 probe stimulus.  Probe pins: PROBE1 = urx[3], PROBE2 = p2,
	// PROBE3..8 = p38[0..5].  Each model appends the records the DUT must
	// produce (content exact; timestamps are mirrored from the DUT when it
	// loads them and checked against exp_t0 within +-2 us on delivery).
	// ------------------------------------------------------------------
	integer   n_dio_gl = 0;
	reg [7:0] spi_pmo = 8'd0, spi_pmi = 8'd0;
	localparam real SPI_TCO = 100.0;     // SPI data change after the shift edge, ns
	function [7:0] dio_pins(input dummy);
		dio_pins = {p38, p2, urx[3]};
	endfunction

	task push_probe(input integer src, input [7:0] b1, input [7:0] b2, input [7:0] b3,
	                input [7:0] b8, input real t0);
		integer n;
		begin
			n = n_exp[src];
			exp_rec[src][n][1] = b1;
			exp_rec[src][n][2] = b2;
			if (src == I2C) begin
				exp_rec[src][n][0] = 8'h1C; exp_rec[src][n][7] = 8'hC1;
			end else begin
				exp_rec[src][n][0] = (src == SPI) ? 8'h5B : 8'hD1;
				exp_rec[src][n][3] = b3;
				exp_rec[src][n][8] = b8;
				exp_rec[src][n][9] = (src == SPI) ? 8'hB5 : 8'h1D;
			end
			exp_t0[src][n] = t0;
			n_exp[src] = n + 1;
		end
	endtask

	// --- DIO-8 -----------------------------------------------------------
	task dio_expect(input [7:0] v);
		begin
			push_probe(DIO, 8'h00, v, v ^ tb_dio, 8'h00, $realtime);
			tb_dio = v;
		end
	endtask
	task dio_apply(input [7:0] v);
		begin urx[3] = v[0]; p2 = v[1]; p38 = v[7:2]; end
	endtask
	// A real change (all bits at once), then gap ns of quiet.
	task dio_set(input [7:0] v, input real gap);
		begin
			p2_guard;
			if (v != tb_dio) dio_expect(v);
			dio_apply(v);
			#(gap);
		end
	endtask
	// Bits of `mask` toggle one after another, skew ns apart: still ONE record.
	task dio_skew(input [7:0] mask, input real skew, input real gap);
		integer i;
		reg [7:0] v;
		begin
			p2_guard;
			v = tb_dio;
			dio_expect(tb_dio ^ mask);
			for (i = 0; i < 8; i = i + 1)
				if (mask[i]) begin v[i] = ~v[i]; dio_apply(v); #(skew); end
			#(gap);
		end
	endtask
	// Excursion of w ns (< 0.85 us) on the bits of mask: must be filtered.
	task dio_glitch(input [7:0] mask, input real w);
		begin
			p2_guard;
			dio_apply(tb_dio ^ mask); #(w);
			dio_apply(tb_dio);        #(2_000.0);
			n_dio_gl = n_dio_gl + 1;
		end
	endtask

	// --- I2C (SCL = PROBE1, SDA = PROBE2) ---------------------------------
	integer IH = 625;                    // quarter SCL period: 625 ns = 400 kHz
	reg [1:0] i2c_sf  = 2'b00;           // {start, rstart} for the next record
	integer   i2c_pend = -1;             // last byte record: may still get stop / timeout
	reg       i2c_in  = 1'b0;
	real      i2c_le  = 0.0;             // last SCL/SDA edge
	task i2c_scl(input v); begin if (urx[3] !== v) i2c_le = $realtime; urx[3] = v; end endtask
	task i2c_sda(input v); begin if (p2 !== v) i2c_le = $realtime; p2 = v; end endtask

	// START from idle (gap = SDA fall -> SCL fall), or repeated START.
	task i2c_start(input integer gap);
		begin
			if (!i2c_in) begin
				i2c_sda(1'b1); i2c_scl(1'b1); #(2 * IH);
				i2c_sda(1'b0); #(gap);
				i2c_scl(1'b0); #(IH);
				i2c_sf = 2'b10;
			end else begin
				i2c_sda(1'b1); #(IH);
				i2c_scl(1'b1); #(IH);        // the DUT samples a tentative bit here and discards it
				i2c_sda(1'b0); #(gap);
				i2c_scl(1'b0); #(IH);
				i2c_pend = -1;               // the held byte went out at the repeated START, no flag
				i2c_sf = 2'b01;
			end
			i2c_in = 1'b1;
		end
	endtask
	task i2c_bit(input b);               // SCL low on entry and exit, 4 IH per bit
		begin
			i2c_sda(b);    #(IH);
			i2c_scl(1'b1); #(2 * IH);
			i2c_scl(1'b0); #(IH);
		end
	endtask
	task i2c_byte(input [7:0] d, input ack);
		integer i;
		real t9;
		begin
			for (i = 7; i >= 0; i = i - 1) i2c_bit(d[i]);
			i2c_pend = -1;                   // the previous byte went out at bit 7's commit
			i2c_sda(~ack); #(IH);            // ACK slot (driven by the addressed device)
			i2c_scl(1'b1); #(2 * IH);
			i2c_scl(1'b0); t9 = $realtime; #(IH);
			push_probe(I2C, {i2c_sf, 1'b0, 1'b1, 1'b0, ack, 2'b00}, d, 8'h00, 8'h00, t9);
			i2c_pend = n_exp[I2C] - 1;
			i2c_sf = 2'b00;
		end
	endtask
	task i2c_stop;
		begin
			i2c_sda(1'b0); #(IH);
			i2c_scl(1'b1); #(IH);            // tentative bit, discarded by the STOP
			i2c_sda(1'b1);
			if (i2c_pend >= 0) exp_rec[I2C][i2c_pend][1] = exp_rec[I2C][i2c_pend][1] | 8'h20;
			else push_probe(I2C, {i2c_sf, 6'b100000}, 8'h00, 8'h00, 8'h00, $realtime);
			i2c_in = 1'b0; i2c_pend = -1; i2c_sf = 2'b00;
			#(4 * IH);
		end
	endtask
	// n (1..8) bits of d (MSB first), then STOP: a partial record.
	task i2c_partial_stop(input [7:0] d, input integer n);
		integer i;
		reg [7:0] m;
		begin
			m = 8'h00;
			for (i = 7; i >= 8 - n; i = i - 1) begin i2c_bit(d[i]); m[i] = d[i]; end
			i2c_pend = -1;
			i2c_sda(1'b0); #(IH);
			i2c_scl(1'b1); #(IH);
			i2c_sda(1'b1);
			push_probe(I2C, {i2c_sf, 6'b110001}, m, 8'h00, 8'h00, $realtime);
			i2c_in = 1'b0; i2c_sf = 2'b00;
			#(4 * IH);
		end
	endtask
	// Bus left idle (SCL low) for 1.3 ms inside a transaction: timeout.
	task i2c_timeout;
		begin
			// expectation first: the DUT emits 1 ms after the last edge
			if (i2c_pend >= 0) exp_rec[I2C][i2c_pend][1] = exp_rec[I2C][i2c_pend][1] | 8'h02;
			else push_probe(I2C, {i2c_sf, 6'b000010}, 8'h00, 8'h00, 8'h00, i2c_le + 1.0e6);
			i2c_in = 1'b0; i2c_pend = -1; i2c_sf = 2'b00;
			#(1_300_000);
			// back to idle without a START: SCL up (SDA low), then SDA up = a STOP
			// outside any transaction, which the DUT ignores
			i2c_sda(1'b0); #(IH); i2c_scl(1'b1); #(IH); i2c_sda(1'b1); #(4 * IH);
		end
	endtask
	task i2c_write(input [7:0] addr, input integer n, input integer gap);
		integer k;
		begin
			i2c_start(gap);
			i2c_byte(addr, 1'b1);
			for (k = 0; k < n; k = k + 1) i2c_byte(prnd(0), (k != n - 1) || (prnd(0) % 3 != 0));
			i2c_stop;
		end
	endtask

	// --- SPI (SCK = PROBE1, MOSI = PROBE2, MISO = PROBE3, CS = PROBE4) ----
	// nb whole bytes, then pbits (0..7) more, at half-period half ns, in the
	// configured mode (tb_spi = {lsb_first, cpol, cpha}).
	task spi_txn(input integer nb, input integer pbits, input real half);
		integer k, i, nbits, bi, first_idx, last_idx;
		reg cpha, cpol, lsb;
		reg [7:0] mo, mi, amo, ami;
		real ts;
		begin
			cpha = tb_spi[0]; cpol = tb_spi[1]; lsb = tb_spi[2];
			urx[3] = cpol;
			#(half);
			p38[1] = 1'b0;                                   // CS low
			#(half);
			last_idx = -1; first_idx = n_exp[SPI];
			for (k = 0; k < nb + ((pbits != 0) ? 1 : 0); k = k + 1) begin
				nbits = (k < nb) ? 8 : pbits;
				mo = prnd(0); mi = prnd(0);
				amo = 8'h00; ami = 8'h00;
				for (i = 0; i < nbits; i = i + 1) begin
					bi = lsb ? i : 7 - i;
					amo[bi] = mo[bi]; ami[bi] = mi[bi];
					// Data changes TCO = 100 ns after the shift edge (more than one
					// 83 ns sample), as a real master/slave output does, so a
					// decoder that sampled on the shift edge would read the old bit.
					if (!cpha) begin
						if (k != 0 || i != 0) #(SPI_TCO);    // shift edge was the previous trailing edge
						p2 = mo[bi]; p38[0] = mi[bi]; #(half - ((k != 0 || i != 0) ? SPI_TCO : 0.0));
						urx[3] = ~cpol; ts = $realtime; #(half);  // leading edge: sample
						urx[3] = cpol;
					end else begin
						urx[3] = ~cpol; #(SPI_TCO);               // leading edge: shift
						p2 = mo[bi]; p38[0] = mi[bi]; #(half - SPI_TCO);
						urx[3] = cpol; ts = $realtime; #(half);   // trailing edge: sample
					end
				end
				if (nbits == 8) begin
					push_probe(SPI, {(k == 0), 1'b0, 1'b0, 1'b1, 4'd0}, mo, mi, 8'd8, ts);
					last_idx = n_exp[SPI] - 1;
				end else begin
					spi_pmo = amo; spi_pmi = ami;
				end
			end
			#(half);
			p38[1] = 1'b1;                                   // CS high
			if (pbits != 0)
				push_probe(SPI, {(nb == 0), 1'b1, 1'b1, 1'b1, 4'd0}, spi_pmo, spi_pmi, pbits, $realtime);
			else if (last_idx >= 0)
				exp_rec[SPI][last_idx][1] = exp_rec[SPI][last_idx][1] | 8'h40;
			else
				push_probe(SPI, 8'hC0, 8'h00, 8'h00, 8'h00, $realtime);   // CS pulse, no clocks
			#(4 * half);
		end
	endtask

	// Send a C3 block: through the poller if it is running, else directly.
	task send_cfg(input integer kind);
		integer c0;
		begin
			if (polling) begin
				c0 = n_cfg_total;
				cfg_kind = kind;
				inject = 4;
				wait (n_cfg_total != c0);
			end else
				cfg_xfer(kind);
		end
	endtask
	// Change probe mode / spi_cfg / divisors (rq_div holds the divisors).
	task set_cfg(input [1:0] mode, input [2:0] spi);
		begin
			rq_gen  = tb_gen + 8'd1;
			rq_mode = {6'd0, mode};
			rq_spi  = {5'd0, spi};
			send_cfg(0);
			if (v3_fd != 0)
				$fwrite(v3_fd, "# C3 applied: gen %0d probe_mode %0d spi_cfg %0d div %0d/%0d/%0d/%0d (records below may have been captured before it)\n",
				        tb_gen, tb_mode, tb_spi, tb_div[0], tb_div[1], tb_div[2], tb_div[3]);
		end
	endtask
	function [15:0] div_of(input integer baud);
		div_of = (12_000_000 + baud / 2) / baud;
	endfunction

	// Probe-mode change: reset the counter / DIO models as the DUT resets its
	// filters, and expect the DIO entry record for probes already high.
	reg [1:0] tb_mode_q = 2'd0;
	always @(posedge clk_12m)
		if (dut.mode_q !== tb_mode_q) begin
			tb_mode_q = dut.mode_q;
			tb_lvl = 1'b0;
			tb_glw = 0;
			tb_dio = 8'd0;
			if (tb_mode_q == 2'd1 && dio_pins(0) != 8'd0) dio_expect(dio_pins(0));
		end

	task v3_note(input [8*256-1:0] txt);
		begin
			if (v3_fd != 0) $fwrite(v3_fd, "# %0s\n", txt);
		end
	endtask

	// ------------------------------------------------------------------
	// Tests
	// ------------------------------------------------------------------
	integer i, k, gaps_before, sent_before, a_cnt_before, d_u3_before, d_cnt_before, c_ld4, c_ld5, c_rx4, c_rx5, infl;
	integer ph_before [0:8];
	integer sw, sp;
	reg     fast;
	reg [8*9-1:0] check_str;
	reg [15:0] c16;

	initial begin
		if ($test$plusargs("vcd")) begin
			$dumpfile("can_frame_link_tb.vcd");
			$dumpvars(0, can_frame_link_tb);
		end
		if ($value$plusargs("dump=%s", fname))    dump_fd = $fopen(fname, "w");
		if ($value$plusargs("records=%s", fname)) rec_fd  = $fopen(fname, "w");
		if ($value$plusargs("esp=%s", fname))     esp_fd  = $fopen(fname, "w");
		if ($value$plusargs("v3=%s", fname))      v3_fd   = $fopen(fname, "w");
		if (v3_fd != 0) begin
			$fwrite(v3_fd, "# SignalBench protocol v3 golden records (PROTOCOL_V3_CONTRACT.md section 6).\n");
			$fwrite(v3_fd, "# Written by can_frame_link_tb (fpga-code can_frame_link_board): every record the\n");
			$fwrite(v3_fd, "# simulated ESP received, in stream order, as 'R <hex bytes>'; '# ' lines mark scenarios.\n");
			$fwrite(v3_fd, "# Records: AA CAN (20 B), C5 UART (8), E7 counter (12), 1C I2C (8), 5B SPI (10), D1 DIO (10).\n");
			$fwrite(v3_fd, "# Config: BUF_AW=%0d (1 Ki-sample buffer when 10: phase C overflows it, so some\n", BUF_AW);
			$fwrite(v3_fd, "# records carry dropped=1 and the stream skips the dropped ones), read_len %0d,\n", READ_LEN);
			$fwrite(v3_fd, "# UART0..3 start at %0d/%0d/%0d/%0d baud.\n", U0_BAUD, U1_BAUD, U2_BAUD, U3_BAUD);
		end
		if (esp_fd != 0) begin
			$fwrite(esp_fd, "# can_frame_link_tb vectors for test_link_records.c replay_vectors().\n");
			$fwrite(esp_fd, "# C/U/E = records exactly as the DUT wrote them into the link buffer (push order),\n");
			$fwrite(esp_fd, "# captured at the SPRAM write port: C = 20-byte CAN (0xAA), U = 8-byte UART (0xC5,\n");
			$fwrite(esp_fd, "# chan 0-3), E = 12-byte PROBE2 edge-counter record (0xE7).  P = payload bytes of\n");
			$fwrite(esp_fd, "# each valid frame as the MCU accepts them (resent samples removed), in stream order.\n");
		end

		$display("=== can_frame_link testbench: BUF_AW=%0d, offsets from %08x, CAN %0d ppm, read_len %0d ===",
		         BUF_AW, INITIAL_OFFSET, CAN_PPM, READ_LEN);
		$display("    UART0 %0d baud (tx %+0d ppm), UART1 %0d baud (tx %+0d ppm), UART2 %0d baud (tx %+0d ppm), UART3 %0d baud (tx %+0d ppm)",
		         U0_BAUD, U0_PPM, U1_BAUD, U1_PPM, U2_BAUD, U2_PPM, U3_BAUD, U3_PPM);

		check_str = "123456789";
		c16 = 16'hFFFF;
		for (i = 8; i >= 0; i = i - 1) c16 = tb_crc(c16, check_str[8*i +: 8]);
		if (c16 !== 16'h29B1) begin $display("FAIL: tb CRC %04x", c16); errors = errors + 1; end

		wait (dut.por[7]);
		t_por = $realtime;
		por_seen = 1'b1;
		#(20 * BIT_NS);

		// Reset config must be gen 0, mode 0, spi 0, div 1250/1250/1250/104.
		if (dut.cfg_gen !== 8'd0 || dut.cfg_mode !== 2'd0 || dut.cfg_spi !== 3'd0 || dut.cfg_div0 !== 16'd1250 ||
		    dut.cfg_div1 !== 16'd1250 || dut.cfg_div2 !== 16'd1250 || dut.cfg_div3 !== 16'd104) begin
			$display("FAIL: config reset values"); errors = errors + 1;
		end
		// Bauds other than the reset defaults: one C3 block before any traffic.
		for (k = 0; k < 4; k = k + 1) rq_div[k] = div_of(u_baud[k]);
		if (rq_div[0] != 16'd1250 || rq_div[1] != 16'd1250 || rq_div[2] != 16'd1250 || rq_div[3] != 16'd104) begin
			set_cfg(2'd0, 3'd0);
			$display("  config: C3 gen %0d sets divisors %0d/%0d/%0d/%0d (UART %0d/%0d/%0d/%0d baud)",
			         tb_gen, tb_div[0], tb_div[1], tb_div[2], tb_div[3], u_baud[0], u_baud[1], u_baud[2], u_baud[3]);
		end
		v3_note("phase A: CAN + UART0..3 (probe UART, mode 0) + PROBE2 counter, with link faults injected");

		// -- A: CAN + four UART channels + PROBE2 concurrently, MCU polling --
		//    (runs past the first heartbeat so the phase-A vectors hold a
		//    counter record)
		dumping = 1;
		polling = 1'b1;
		fork
			begin
				send_mix(24, 9, 0);              // frame 9: bad CRC
				inject = 3;  wait (inject == 0);  send_mix(12, -1, 0);   // aborted transfer
				inject = 2;  wait (inject == 0);  send_mix(12, -1, 5);   // corrupted MISO frame
				inject = 1;  wait (inject == 0);  send_mix(12, -1, 0);   // corrupted control block
			end
			uart_script(0, 16, 5, -1, 9);        // framing error, genuine 0x00
			uart_script(1, 20, -1, 12, 3);       // break, genuine 0x00
			uart_script(2, 12, 8, 4, -1);        // break and framing error
			begin                                // UART3: Pi packets, then errors at 115200
				#(5_000_000);
				pi_packets(4, 20);
				#(5_000_000);
				uart_script(3, 12, 4, 8, 2);     // framing error, break, genuine 0x00
				pi_packets(2, 0);
			end
			wait (n_load[CNT] >= 1);
		join
		stop_poller;
		drain("A drain");
		dumping = 0;
		$display("  [t = %0.1f ms]", $realtime / 1e6);
		$display("  A: CAN %0d, UART0 %0d, UART1 %0d, UART2 %0d, UART3 %0d, COUNTER %0d records delivered (%0d framing errors, %0d breaks), %0d straddled frames, %0d dropped",
		         recs_by[0], recs_by[1], recs_by[2], recs_by[3], recs_by[4], recs_by[5], n_fe, n_brk, split_recs, gaps_total);
		if (gaps_total != 0 || dut.u_link.dropped != 0) begin
			$display("FAIL [A]: records dropped while the MCU was keeping up"); errors = errors + 1;
		end
		for (k = 0; k <= CNT; k = k + 1)
			if (recs_by[k] == 0) begin
				$display("FAIL [A]: source %0d delivered no records (empty compare)", k); errors = errors + 1;
			end
		if (n_inj[1] != 1 || n_inj[2] != 1 || n_inj[3] != 1) begin
			$display("FAIL [A]: fault injections ran %0d/%0d/%0d times, expected 1 each", n_inj[1], n_inj[2], n_inj[3]);
			errors = errors + 1;
		end else
			$display("  A: survived 1 corrupt control block, 1 corrupt MISO frame, 1 aborted transfer mid-stream");

		fast = $test$plusargs("fast");               // debug only: skip B, B2, C, D
		if (!fast) begin
		// -- B: SCK sweep, CAN + UART0 + UART3 -----------------------------
		v3_note("phase B: link SCK sweep 400 kHz .. 1.5 MHz, then read_len 54 (records straddle frames)");
		for (k = 0; k < 4; k = k + 1) begin
			case (k)
				0: HALF = 1250;   // 400 kHz
				1: HALF = 667;    // 750 kHz
				2: HALF = 417;    // 1.2 MHz
				3: HALF = 333;    // 1.5 MHz ceiling
			endcase
			polling = 1'b1;
			fork
				send_mix(8, -1, 2);
				uart_script(0, 4, -1, -1, -1);
				uart_script(3, 6, (k == 1) ? 1 : -1, (k == 2) ? 2 : -1, (k == 3) ? 0 : -1);
			join
			stop_poller;
			drain("B sweep");
		end
		HALF = 500;
		// -- B2: short reads (15 samples per frame) force records to straddle
		split_recs = 0;
		RL = 54;
		polling = 1'b1;
		fork
			send_mix(6, -1, 0);
			uart_script(2, 3, -1, -1, -1);
			pi_packets(1, 0);
		join
		stop_poller;
		drain("B2 straddle");
		RL = READ_LEN;
		if (split_recs == 0) begin
			$display("FAIL [B2]: no record straddled a frame boundary"); errors = errors + 1;
		end else
			$display("  B2: read_len 54: %0d frames ended mid-record, every record reassembled exactly", split_recs);
		$display("  [t = %0.1f ms]", $realtime / 1e6);
		$display("  B: SCK 400 kHz / 750 kHz / 1.2 MHz / 1.5 MHz: %0d records delivered so far, all exact", recs_ok);

		// -- C: overflow (small buffer only): the MCU stops reading ---------
		//    Held until a heartbeat has been loaded while stopped, so CAN,
		//    UART3 and COUNTER records are all among the dropped.
		if (BUF_AW <= 10) begin
			v3_note("phase C: overflow - the ESP stops reading; records are dropped whole and the next one carries dropped=1");
			gaps_before = gaps_total;
			sent_before = n_exp[0] + n_exp[1] + n_exp[2] + n_exp[3] + n_exp[4] + n_exp[5];
			c_ld4 = n_load[4]; c_ld5 = n_load[5]; c_rx4 = recs_by[4]; c_rx5 = recs_by[5];
			drops_ok = 1;
			fork
				send_mix(120, -1, 0);
				uart_script(0, 20, -1, -1, -1);
				uart_script(2, 10, -1, -1, -1);
				begin #(2_000_000); pi_packets(6, 0); end
				begin : c_hb
					integer want;
					want = n_load[CNT] + 1;
					wait (n_load[CNT] >= want);
				end
			join
			drain("C drain");
			infl = cnt_in_flight(0);
			// Missing = loaded but never delivered (includes drops after the
			// last delivered record, which no later record has flagged yet).
			if (gseq - infl - recs_ok == gaps_before) begin
				$display("FAIL [C]: buffer overflowed but nothing was dropped"); errors = errors + 1;
			end
			if (fr_dropped !== gseq - infl - recs_ok) begin
				$display("FAIL [C]: header reports %0d records dropped, %0d missing", fr_dropped, gseq - infl - recs_ok);
				errors = errors + 1;
			end
			$display("  [t = %0.1f ms]", $realtime / 1e6);
			$display("  C: overflow: %0d of %0d records dropped whole (header dropped = %0d records); UART3 %0d of %0d and COUNTER %0d of %0d loaded during C were dropped",
			         gseq - infl - recs_ok - gaps_before,
			         n_exp[0] + n_exp[1] + n_exp[2] + n_exp[3] + n_exp[4] + n_exp[5] - sent_before, fr_dropped,
			         (n_load[4] - c_ld4) - (recs_by[4] - c_rx4), n_load[4] - c_ld4,
			         (n_load[5] - c_ld5 - infl) - (recs_by[5] - c_rx5), n_load[5] - c_ld5 - infl);
			if ((n_load[4] - c_ld4) - (recs_by[4] - c_rx4) == 0 || (n_load[5] - c_ld5 - infl) - (recs_by[5] - c_rx5) == 0) begin
				$display("FAIL [C]: overflow did not drop at least one UART3 and one COUNTER record"); errors = errors + 1;
			end
			// Recovery: the first new record must carry dropped=1 if the drops
			// were trailing; drops stay allowed until it has been checked.
			polling = 1'b1;
			fork
				send_mix(10, -1, 0);
				uart_script(1, 4, -1, -1, -1);
			join
			stop_poller;
			drain("C after");
			// The DUT is empty now, so a record loaded but never delivered (and
			// not a counter record loaded after the drain check) was dropped; the
			// global numbering has already charged it to gaps_total.  Move each
			// source past its trailing drops, so later phases match exactly.
			for (k = 0; k < NSRC; k = k + 1)
				n_rx[k] = n_load[k] - in_flight(k);
			drops_ok = 0;
			if (gaps_total != fr_dropped) begin
				$display("FAIL [C]: %0d records missing in the delivered sequence, header says %0d", gaps_total, fr_dropped);
				errors = errors + 1;
			end else
				$display("  C: recovered; every missing record was reported, and flagged on the next delivered one");
		end else
			$display("  C: overflow skipped (buffer too large to fill in simulation)");

		// -- D: the home-run load: Pi packets back to back on UART3 with the
		//    PROBE2 bursts and heartbeats running, MCU polling; no drops.
		v3_note("phase D: Pi packets back to back on the probe UART with the PROBE2 counter running");
		gaps_before  = gaps_total;
		d_u3_before  = recs_by[4];
		d_cnt_before = recs_by[5];
		polling = 1'b1;
		fork
			pi_packets(PI_PACKETS, 0);
			wait (sat_done);
		join
		stop_poller;
		drain("D drain");
		$display("  [t = %0.1f ms]", $realtime / 1e6);
		$display("  D: %0d Pi packets back to back at %0d baud: %0d UART3 records, %0d COUNTER records delivered, %0d dropped",
		         PI_PACKETS, U3_BAUD, recs_by[4] - d_u3_before, recs_by[5] - d_cnt_before, gaps_total - gaps_before);
		if (gaps_total != gaps_before) begin
			$display("FAIL [D]: records dropped while the MCU was keeping up"); errors = errors + 1;
		end
		if (recs_by[4] - d_u3_before < 16 * PI_PACKETS || recs_by[5] == d_cnt_before) begin
			$display("FAIL [D]: expected >= %0d UART3 and > 0 COUNTER records", 16 * PI_PACKETS); errors = errors + 1;
		end
		end else
			$display("  +fast: phases B, B2, C, D skipped (debug run, not a PASS gate)");


		// ================================================================
		// Protocol v3: probe modes, C3 config, runtime baud.
		// ================================================================
		for (k = 0; k < NSRC; k = k + 1) ph_before[k] = recs_by[k];
		gaps_before = gaps_total;
		// Leave mode 0: stop the PROBE2 bursts first (the pin rests low).
		wait (p2_det_done);
		p2_en = 1'b0;
		wait (!p2_busy);
		p2 = 1'b0;

		// -- E: I2C (mode 3), switched while CAN + UART0 stream ------------
		v3_note("phase E: probe_mode 3 (I2C) set while CAN/UART stream; writes, read with repeated START, NACK, STOP-only, partial byte, timeouts, START sweep, 1 MHz");
		polling = 1'b1;
		fork
			send_mix(10, -1, 40);
			uart_script(0, 6, -1, -1, -1);
			begin
				#(300_000);
				for (k = 0; k < 4; k = k + 1) rq_div[k] = tb_div[k];
				set_cfg(2'd3, tb_spi);                       // -> I2C, mid-stream
				// SCL = PROBE1 is high (UART idle), SDA = PROBE2 low: raising SDA
				// is a STOP outside a transaction, which must give no record.
				i2c_sda(1'b1); #(20_000);
				i2c_write(8'h6C, 1, 300);                    // address 0x36 W + 1 byte
				i2c_start(300);                              // write then repeated START, read
				i2c_byte(8'h6C, 1'b1); i2c_byte(8'h0B, 1'b1);
				i2c_start(300);
				i2c_byte(8'h6D, 1'b1); i2c_byte(8'h93, 1'b1); i2c_byte(8'h5A, 1'b0);   // last read NACKed
				i2c_stop;
				i2c_start(300); i2c_stop;                    // START, STOP: one marker record
				i2c_start(300); i2c_byte(8'hA0, 1'b0);       // address NACKed
				i2c_partial_stop(8'hB7, 5);                  // 5 bits then STOP: partial
				i2c_start(300); i2c_partial_stop(8'h81, 3);  // partial as the first byte
				i2c_start(300); i2c_byte(8'h50, 1'b1); i2c_byte(8'h01, 1'b1);
				i2c_timeout;                                 // held byte gets `timeout`
				i2c_start(300); i2c_timeout;                 // timeout with nothing held
				i2c_write(8'h42, 3, 300);                    // next START is `start`, not rstart
				// START sweep: SDA-fall -> SCL-fall gap x phase against the clock
				for (sw = 0; sw < 4; sw = sw + 1)
					for (sp = 0; sp < 4; sp = sp + 1) begin
						#(sp * 21);
						i2c_write(8'h6C + 2 * sw, 1, (sw == 0) ? 200 : (sw == 1) ? 300 : (sw == 2) ? 450 : 700);
						#(10_000);
					end
				IH = 250;                                    // 1 MHz (Fm+)
				i2c_write(8'h3C, 6, 260);
				i2c_start(260); i2c_byte(8'h3D, 1'b1); i2c_start(260); i2c_byte(8'h3E, 1'b1); i2c_byte(8'h77, 1'b0); i2c_stop;
				IH = 625;
				// mid-byte mode switch: START, a byte, 4 bits, then -> SPI.  The
				// byte goes out (released by the next byte's first bit), the 4
				// bits are discarded with the I2C decoder's state.  The C3 block
				// is sent directly (poller paused) so it lands well inside the
				// 1 ms idle timeout, which would otherwise close the byte first.
				p38[1] = 1'b1;                               // SPI CS (PROBE4) idle high
				stop_poller;
				i2c_start(300); i2c_byte(8'hC4, 1'b1);
				i2c_bit(1'b1); i2c_pend = -1; i2c_bit(1'b0); i2c_bit(1'b1); i2c_bit(1'b1);
				i2c_in = 1'b0; i2c_sf = 2'b00;
				set_cfg(2'd2, 3'd0);                         // -> SPI mode 0
				polling = 1'b1;
			end
		join
		stop_poller;
		drain("E drain");
		$display("  [t = %0.1f ms]", $realtime / 1e6);
		$display("  E: I2C %0d records delivered (with CAN %0d, UART0 %0d), %0d dropped",
		         recs_by[I2C] - ph_before[I2C], recs_by[0] - ph_before[0], recs_by[1] - ph_before[1], gaps_total - gaps_before);

		// -- F: SPI (mode 2): modes 0..3, MSB/LSB first, partial, 1.5 MHz ---
		v3_note("phase F: probe_mode 2 (SPI, set at the end of E in the middle of an I2C byte); modes 0-3, MSB and LSB first, partial bytes, CS-only pulse, SCK 1.5 MHz, 2 MHz and 500 kHz");
		for (k = 0; k < NSRC; k = k + 1) ph_before[k] = recs_by[k];
		polling = 1'b1;
		fork
			send_mix(8, -1, 60);
			uart_script(1, 5, -1, -1, -1);
			begin
				#(5_000);                                    // (SPI mode 0 set at the end of E)
				for (sw = 0; sw < 8; sw = sw + 1) begin      // spi_cfg = {lsb, cpol, cpha}
					if (sw != 0) set_cfg(2'd2, sw[2:0]);
					urx[3] = tb_spi[1];                       // SCK idles at CPOL
					#(5_000);
					spi_txn(4, 0, 333.0);                     // 1.5 MHz
					spi_txn(3, 5, 333.0);                     // + partial byte (5 bits)
					spi_txn(0, 3, 333.0);                     // partial only
					spi_txn(1, 0, 1000.0);                    // 500 kHz
					spi_txn(3, 0, 250.0);                     // 2 MHz: margin above the 1.5 MHz requirement
					#(200_000);
				end
				set_cfg(2'd2, 3'd0);
				urx[3] = 1'b0;
				#(5_000);
				spi_txn(0, 0, 333.0);                         // CS pulse, no clocks: end-only record
				spi_txn(12, 0, 333.0);                        // 12-byte burst at 1.5 MHz
				// mid-transaction mode switch: CS low, 1 byte + 3 bits, -> DIO.
				// The byte goes out (released by bit 1 of the next byte); the 3
				// bits and the missing CS rise are discarded.
				#(333); p38[1] = 1'b0; #(333);
				for (sw = 7; sw >= 0; sw = sw - 1) begin
					p2 = sw[0]; p38[0] = ~sw[0]; #(333); urx[3] = 1'b1; #(333); urx[3] = 1'b0;
				end
				push_probe(SPI, 8'h90, 8'hAA, 8'h55, 8'd8, $realtime - 333.0);   // cs_start, data_valid; no cs_end
				for (sw = 0; sw < 3; sw = sw + 1) begin #(333); urx[3] = 1'b1; #(333); urx[3] = 1'b0; end
				#(5_000);
			end
		join
		stop_poller;
		drain("F drain");
		$display("  [t = %0.1f ms]", $realtime / 1e6);
		$display("  F: SPI %0d records delivered (with CAN %0d, UART1 %0d), %0d dropped",
		         recs_by[SPI] - ph_before[SPI], recs_by[0] - ph_before[0], recs_by[2] - ph_before[2], gaps_total - gaps_before);

		// -- G: DIO-8 (mode 1): changes, glitches, skewed edges, heartbeats -
		v3_note("phase G: probe_mode 1 (DIO-8) entered mid-SPI-transaction; changes, sub-us glitches (filtered), skewed multi-channel edges, 100 ms heartbeats");
		for (k = 0; k < NSRC; k = k + 1) ph_before[k] = recs_by[k];
		sp = n_dio_hb;
		polling = 1'b1;
		fork
			send_mix(6, -1, 80);
			uart_script(2, 5, -1, -1, -1);
			begin
				set_cfg(2'd1, 3'd0);                         // -> DIO (entry record for probes high)
				#(20_000);
				for (sw = 0; sw < 8; sw = sw + 1) dio_set(8'h01 << sw, 6_000);          // walking one
				dio_set(8'h00, 6_000);
				dio_set(8'hFF, 6_000);                                                 // all at once
				dio_set(8'h00, 6_000);
				for (sw = 0; sw < 40; sw = sw + 1) dio_set(prnd(0), 3_000 + (prnd(0) % 20_000));
				for (sw = 0; sw < 12; sw = sw + 1) begin                               // glitches < 1 us
					dio_glitch(8'h01 << (sw % 8), 100.0 + (prnd(0) % 751));
					dio_glitch(8'hA5, 300.0 + (prnd(0) % 500));
				end
				for (sw = 0; sw < 6; sw = sw + 1) dio_skew(8'h0F << (sw % 5), 50.0, 8_000);   // 3 x 50 ns skew
				for (sw = 0; sw < 10; sw = sw + 1) begin                                // glitch on one, edge on another
					p2_guard;
					dio_apply(tb_dio ^ 8'h80); #(400);
					dio_apply(tb_dio);
					dio_expect(tb_dio ^ 8'h02); dio_apply(tb_dio); #(8_000);
				end
				wait (n_dio_hb >= sp + 2);                   // two heartbeats while idle
				dio_set(8'h5A, 6_000);
				wait (n_dio_hb >= sp + 3);                   // one more with levels != 0
				// back to mode 0: probe UART idles high, PROBE2 low
				dio_set(8'h01, 20_000);
			end
		join
		stop_poller;
		drain("G drain");
		$display("  [t = %0.1f ms]", $realtime / 1e6);
		$display("  G: DIO %0d records delivered (%0d heartbeats, %0d glitch pulses filtered), %0d dropped",
		         recs_by[DIO] - ph_before[DIO], n_dio_hb - sp, n_dio_gl, gaps_total - gaps_before);

		// -- H: back to mode 0 with a baud change; rejected C3 blocks -------
		v3_note("phase H: C3 with bad CRC and with div0=15 (both ignored); probe_mode 0 + baud change (UART0 57600, probe UART 230400), traffic at the new bauds");
		for (k = 0; k < NSRC; k = k + 1) ph_before[k] = recs_by[k];
		polling = 1'b1;
		rq_gen = tb_gen + 8'd1; rq_mode = 8'd0; rq_spi = 8'd0;
		for (k = 0; k < 4; k = k + 1) rq_div[k] = tb_div[k];
		send_cfg(1);                                         // bad CRC: ignored
		send_cfg(2);                                         // div0 = 15: ignored
		rq_div[0] = div_of(57_600); rq_div[3] = div_of(230_400);
		set_cfg(2'd0, 3'd0);
		u_baud[0] = 57_600; u_baud[3] = 230_400;
		p2_en = 1'b1;
		fork
			send_mix(8, -1, 40);
			uart_script(0, 24, 7, 15, 3);                    // at 57600: framing error, break, 0x00
			begin
				#(1_000_000);
				uart_script(3, 16, 5, 11, 2);                // at 230400
				pi_packets(4, 0);
			end
			begin : h_hb                                     // the counter resumes in mode 0
				integer want;
				want = n_load[CNT] + 1;
				wait (n_load[CNT] >= want);
			end
		join
		stop_poller;
		drain("H drain");
		$display("  [t = %0.1f ms]", $realtime / 1e6);
		$display("  H: baud change: UART0 %0d records at 57600, UART3 %0d at 230400; config echo matched in %0d frames; C3 good/bad-CRC/out-of-range = %0d/%0d/%0d",
		         recs_by[1] - ph_before[1], recs_by[4] - ph_before[4], n_echo_ok, n_cfg[0], n_cfg[1], n_cfg[2]);
		if (gaps_total != gaps_before) begin
			$display("FAIL [E-H]: records dropped while the MCU was keeping up"); errors = errors + 1;
		end
		if (recs_by[I2C] == 0 || recs_by[SPI] == 0 || recs_by[DIO] == 0 || recs_by[1] == ph_before[1] || recs_by[4] == ph_before[4] ||
		    recs_by[CNT] == ph_before[CNT] ||
		    n_cfg[1] == 0 || n_cfg[2] == 0 || n_dio_hb == 0) begin
			$display("FAIL [E-H]: a v3 scenario produced no records (empty compare)"); errors = errors + 1;
		end

		// -- Totals --------------------------------------------------------
		infl = cnt_in_flight(0);
		for (k = 0; k < NSRC; k = k + 1) begin
			if (n_load[k] != n_exp[k] && !((k == CNT || k == DIO) && n_exp[k] - n_load[k] == 1)) begin
				$display("FAIL: source %0d: DUT loaded %0d records, testbench sent %0d", k, n_load[k], n_exp[k]);
				errors = errors + 1;
			end
			if (recs_by[k] == 0) begin
				$display("FAIL: source %0d delivered no records (empty compare)", k);
				errors = errors + 1;
			end
		end
		if (recs_ok + gaps_total + infl != gseq) begin
			$display("FAIL: %0d delivered + %0d dropped + %0d in flight != %0d records", recs_ok, gaps_total, infl, gseq);
			errors = errors + 1;
		end
		if (cnt_sat == 0) begin
			$display("FAIL: no counter record reported a saturated glitch count"); errors = errors + 1;
		end
		if (cnt_lvl1 == 0 || cnt_lvl1 == recs_by[CNT]) $display("  note: counter level bit seen only as %0d", cnt_lvl1 != 0);
		$display("  counter: %0d records (period 100 ms exact), rise %0d at the last delivered (tb drove %0d edges in %0d bursts), glitch sum %0d (tb injected %0d), %0d saturated at 255, %0d with level=1, %0d in flight at end",
		         recs_by[CNT], cnt_rise, tb_rise, tb_bursts, cnt_glitch, tb_gl_total, cnt_sat, cnt_lvl1, infl);
		$display("  totals: %0d records (%0d CAN, %0d/%0d/%0d/%0d UART0/1/2/3, %0d COUNTER, %0d I2C, %0d SPI, %0d DIO) delivered exactly once in arrival order, %0d dropped (all reported), %0d duplicate samples skipped",
		         recs_ok, recs_by[0], recs_by[1], recs_by[2], recs_by[3], recs_by[4], recs_by[5], recs_by[I2C], recs_by[SPI], recs_by[DIO], gaps_total, dup_samp);

		if (dump_fd != 0) $fclose(dump_fd);
		if (esp_fd  != 0) $fclose(esp_fd);
		if (rec_fd  != 0) $fclose(rec_fd);
		if (v3_fd   != 0) begin
			$fwrite(v3_fd, "# end: %0d records (%0d CAN, %0d/%0d/%0d/%0d UART0..3, %0d counter, %0d I2C, %0d SPI, %0d DIO), %0d dropped\n",
			        recs_ok, recs_by[0], recs_by[1], recs_by[2], recs_by[3], recs_by[4], recs_by[5], recs_by[I2C], recs_by[SPI], recs_by[DIO], gaps_total);
			$fclose(v3_fd);
		end
		if (errors == 0 && !fast) $display("PASS: can_frame_link end-to-end");
		else if (errors == 0)     $display("(+fast run: no errors, but B-D were skipped; not a PASS)");
		else             $display("FAIL: %0d error(s)", errors);
		$finish;
	end

	initial begin
		#3_000_000_000;
		$display("FAIL: watchdog timeout");
		$finish;
	end
endmodule
