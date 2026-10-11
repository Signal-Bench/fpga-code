`timescale 1ns/1ps

// ---------------------------------------------------------------------------
// Behavioral stand-ins for the iCE40 hard primitives.  Same convention as the
// other testbenches: there is no vendor simulation library in this repo.
//
// The design's SPI slave is plain fabric logic, so — unlike spi_hw_stream —
// nothing about the SPI behaviour is modelled here.  The only primitives
// are the oscillator and the SPRAM.
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

// Behaviour copied from yosys techlibs/ice40/cells_sim.v (the model the
// toolflow assumes): registered read with one cycle of latency, DATAOUT
// undefined after a write cycle, contents undefined at power-up.
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

// ---------------------------------------------------------------------------
// Testbench
//
// The testbench is the SPI master AND a model of the MCU's receive logic.
// Its CRC and frame parser are written independently of the RTL (bitwise
// CRC over data bits rather than the RTL's byte-XOR form; division instead
// of the RTL's bit slicing), so a pass is a cross-check, not a round-trip.
//
// The property under test: every sample the FPGA writes reaches the MCU
// exactly once, in order — through corrupted control blocks, corrupted
// frames, aborted transfers, bogus acks, an MCU reboot and different read
// lengths and SCK rates — and the only samples ever missing are the ones
// the FPGA itself reports as dropped because its buffer was full.
// ---------------------------------------------------------------------------
module spi_frame_stream_tb;

	parameter integer BUF_AW = 10;
	parameter [31:0]  INITIAL_OFFSET = 32'd0;

	localparam integer SAMPLE_RATE_HZ = 62_500;      // 16-bit samples = 1 Mbps, the target rate
	localparam integer SAMPLE_NS      = 1_000_000_000 / SAMPLE_RATE_HZ;
	localparam integer CAPACITY       = 1 << BUF_AW;

	// Protocol constants, restated independently of the RTL.
	localparam [7:0] ST_ACK_APPLIED  = 8'h01;
	localparam [7:0] ST_CTRL_BAD     = 8'h02;
	localparam [7:0] ST_ACK_REJECTED = 8'h04;
	localparam [7:0] ST_DROPPED      = 8'h08;

	reg  spi_sck  = 1'b0;
	reg  spi_cs   = 1'b1;
	reg  spi_mosi = 1'b0;
	wire spi_miso;
	wire spi_cs_flash;
	wire led_r, led_g, led_b;

	top #(
		.SAMPLE_RATE_HZ(SAMPLE_RATE_HZ),
		.BUF_AW(BUF_AW),
		.INITIAL_OFFSET(INITIAL_OFFSET)
	) dut (
		.spi_sck(spi_sck), .spi_cs(spi_cs), .spi_mosi(spi_mosi), .spi_miso(spi_miso),
		.spi_cs_flash(spi_cs_flash),
		.led_r(led_r), .led_g(led_g), .led_b(led_b)
	);

	integer errors = 0;
	integer HALF   = 250;              // ns, half an SCK period: 2 MHz

	// ------------------------------------------------------------------
	// CRC-16/CCITT-FALSE, bit at a time over the data bits.
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

	// ------------------------------------------------------------------
	// The "MCU" state the testbench keeps, mirroring the firmware.
	// ------------------------------------------------------------------
	reg        synced   = 1'b0;
	reg [31:0] expected = 32'd0;       // next sample offset needed

	// Value checker: the fake sensor is a counter, so every sample is the
	// previous one plus one, except where the FPGA dropped samples.
	reg        val_primed  = 1'b0;
	reg [15:0] val_next    = 16'd0;
	integer    total_gap   = 0;        // sum of all jumps seen in sample values
	integer    samples_new = 0;
	integer    samples_dup = 0;

	// Last parsed frame.
	reg        fr_ok;
	reg [31:0] fr_start;
	reg [15:0] fr_count;
	reg [31:0] fr_dropped;
	reg [7:0]  fr_status;

	reg [7:0] mosi_buf [0:4095];
	reg [7:0] miso_buf [0:4095];
	integer   bytes_clocked;

	// ------------------------------------------------------------------
	// Control block.  `ack_override` / `corrupt` let tests send a bogus ack
	// or a damaged block.
	// ------------------------------------------------------------------
	task build_ctrl(input integer len, input use_override, input [31:0] ack_override,
	                input corrupt);
		integer i;
		reg [15:0] c;
		reg [31:0] ack;
		begin
			for (i = 0; i < len; i = i + 1)
				mosi_buf[i] = 8'h00;
			ack = use_override ? ack_override : expected;
			mosi_buf[0] = 8'h5C;
			mosi_buf[1] = ack[7:0];
			mosi_buf[2] = ack[15:8];
			mosi_buf[3] = ack[23:16];
			mosi_buf[4] = ack[31:24];
			mosi_buf[5] = len % 256;
			mosi_buf[6] = len / 256;
			mosi_buf[7] = (use_override || synced) ? 8'h01 : 8'h00;
			c = 16'hFFFF;
			for (i = 0; i < 8; i = i + 1)
				c = tb_crc(c, mosi_buf[i]);
			mosi_buf[8] = c[15:8];
			mosi_buf[9] = c[7:0];
			if (corrupt)
				mosi_buf[2] = mosi_buf[2] ^ 8'h10;   // one bit, after the CRC
		end
	endtask

	// ------------------------------------------------------------------
	// One SPI transfer, mode 0, MSB first.  `abort_at` >= 0 raises CS after
	// that many bytes, the way a reset or a driver error would.
	// ------------------------------------------------------------------
	task xfer(input integer len, input integer abort_at);
		integer i, b;
		reg [7:0] rx;
		begin
			spi_cs = 1'b0;
			#(2 * HALF);                       // CS setup, as cs_ena_pretrans
			bytes_clocked = 0;
			for (i = 0; i < len && (abort_at < 0 || i < abort_at); i = i + 1) begin
				rx = 8'h00;
				for (b = 7; b >= 0; b = b - 1) begin
					spi_mosi = mosi_buf[i][b];
					#(HALF);
					spi_sck = 1'b1;
					rx[b]   = spi_miso;            // master samples on the rising edge
					#(HALF);
					spi_sck = 1'b0;
				end
				miso_buf[i]   = rx;
				bytes_clocked = i + 1;
			end
			#(2 * HALF);                       // CS hold, as cs_ena_posttrans
			spi_cs = 1'b1;
			#(2000);                           // gap before the next transfer
		end
	endtask

	// ------------------------------------------------------------------
	// Parse the frame in miso_buf and apply it, exactly as the MCU does.
	// Returns with fr_ok = 0 and the MCU state untouched if anything is
	// wrong with the frame.
	// ------------------------------------------------------------------
	task parse(input integer len, input [127:0] label);
		integer    i, n, skip, crc_at;
		integer    behind;
		reg [15:0] c, got, v, gap;
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
				if (crc_at + 2 > len) begin
					$display("  [%0s] frame of %0d samples does not fit %0d bytes", label, n, len);
				end else begin
					c = 16'hFFFF;
					for (i = 0; i < crc_at; i = i + 1)
						c = tb_crc(c, miso_buf[i]);
					got = {miso_buf[crc_at], miso_buf[crc_at + 1]};
					if (c !== got) begin
						$display("  [%0s] CRC mismatch: computed %04x, frame says %04x", label, c, got);
					end else
						fr_ok = 1'b1;
				end
			end

			if (fr_ok) begin
				for (i = 3; i <= 10; i = i + 1)
					if (miso_buf[i] !== 8'h00) begin
						$display("FAIL [%0s]: reserved byte %0d = %02x", label, i, miso_buf[i]);
						errors = errors + 1;
					end
				for (i = crc_at + 2; i < len; i = i + 1)
					if (miso_buf[i] !== 8'hFF) begin
						$display("FAIL [%0s]: padding byte %0d = %02x, expected FF", label, i, miso_buf[i]);
						errors = errors + 1;
					end

				// MCU rule: first frame, or the FPGA rejecting our ack
				// (it restarted), resynchronises to wherever it starts.
				if (!synced || (fr_status & ST_ACK_REJECTED)) begin
					synced     = 1'b1;
					expected   = fr_start;
					val_primed = 1'b0;
				end

				behind = expected - fr_start;
				if (behind < 0) begin
					$display("FAIL [%0s]: frame starts at %0d, beyond the %0d still needed — samples skipped",
					         label, fr_start, expected);
					errors = errors + 1;
					behind = 0;
				end
				skip = (behind > n) ? n : behind;
				samples_dup = samples_dup + skip;

				for (i = skip; i < n; i = i + 1) begin
					v = {miso_buf[22 + 2 * i + 1], miso_buf[22 + 2 * i]};
					// Before any drop, a sample's value IS its offset.
					if (fr_dropped == 0 && v !== ((fr_start + i) % 65536)) begin
						$display("FAIL [%0s]: sample at offset %0d = %0d, expected %0d",
						         label, fr_start + i, v, (fr_start + i) % 65536);
						errors = errors + 1;
					end
					if (val_primed) begin
						gap = v - val_next;
						total_gap = total_gap + gap;
					end
					val_next   = v + 1'b1;
					val_primed = 1'b1;
					samples_new = samples_new + 1;
				end

				if (behind < n)
					expected = fr_start + n;
			end
		end
	endtask

	// ------------------------------------------------------------------
	// Optional vector dump for the MCU host test (+dump=<file>).
	// ------------------------------------------------------------------
	integer dump_fd = 0;
	reg [8*256-1:0] dump_name;

	task dump(input integer len, input [8*16-1:0] kind);
		integer i;
		begin
			if (dump_fd != 0) begin
				$fwrite(dump_fd, "T %0d %0s\nM", len, kind);
				for (i = 0; i < 10; i = i + 1)
					$fwrite(dump_fd, " %02x", mosi_buf[i]);
				$fwrite(dump_fd, "\nS");
				for (i = 0; i < bytes_clocked; i = i + 1)
					$fwrite(dump_fd, " %02x", miso_buf[i]);
				$fwrite(dump_fd, "\nE %08x %0d %0d\n", expected, synced, total_gap);
			end
		end
	endtask

	// A normal poll: build the control block from MCU state, transfer, parse.
	task poll(input integer len, input [127:0] label);
		begin
			build_ctrl(len, 1'b0, 32'd0, 1'b0);
			xfer(len, -1);
			parse(len, label);
			if (!fr_ok) begin
				$display("FAIL [%0s]: a clean transfer produced a bad frame", label);
				errors = errors + 1;
			end
			dump(len, "clean");
		end
	endtask

	task expect_status(input [7:0] want, input [127:0] label);
		begin
			if (fr_status !== want) begin
				$display("FAIL [%0s]: status %02x, expected %02x", label, fr_status, want);
				errors = errors + 1;
			end
		end
	endtask

	// ------------------------------------------------------------------
	// Memory-safety monitors, checked on every core clock.  The value checks
	// in `parse` cannot catch a read of a not-yet-written slot if the sensor
	// happens to write it before the read lands — the data comes out right
	// by luck.  These catch the access itself.
	//   * every SPRAM read is of a sample already written and not yet freed
	//   * every SPRAM write lands outside the unacknowledged region
	//   * a frame never promises more samples than are buffered
	// ------------------------------------------------------------------
	integer mem_errors = 0;
	always @(posedge dut.clk) begin
		if (dut.ram_re && ((dut.fetch_off - dut.rd_off) >= (dut.wr_off - dut.rd_off))) begin
			if (mem_errors < 5)
				$display("FAIL: SPRAM read of offset %08x outside the buffered range [%08x, %08x)",
				         dut.fetch_off, dut.rd_off, dut.wr_off);
			mem_errors = mem_errors + 1;
		end
		if (dut.ram_we && ((dut.wr_off - dut.rd_off) >= CAPACITY)) begin
			if (mem_errors < 5)
				$display("FAIL: SPRAM write at %08x would overwrite unacknowledged data (rd %08x)",
				         dut.wr_off, dut.rd_off);
			mem_errors = mem_errors + 1;
		end
		if (dut.dec == dut.DEC_BUILD && {16'd0, dut.count_q} > dut.level) begin
			if (mem_errors < 5)
				$display("FAIL: frame of %0d samples built with only %0d buffered", dut.count_q, dut.level);
			mem_errors = mem_errors + 1;
		end
	end

	// ------------------------------------------------------------------
	// Stand-alone check of the four-bank SPRAM wrapper (AW = 16), so the
	// bank decode is exercised even in the small-buffer run.
	// ------------------------------------------------------------------
	reg         rt_clk = 1'b0;
	reg  [15:0] rt_addr = 0, rt_wdata = 0;
	reg         rt_we = 1'b0, rt_re = 1'b0;
	wire [15:0] rt_rdata;
	always #20 rt_clk = ~rt_clk;

	sample_ram #(.AW(16)) u_ram_test (
		.clk(rt_clk), .addr(rt_addr), .wdata(rt_wdata),
		.we(rt_we), .re(rt_re), .rdata(rt_rdata)
	);

	task rt_write(input [15:0] a, input [15:0] d);
		begin
			@(negedge rt_clk); rt_addr = a; rt_wdata = d; rt_we = 1'b1; rt_re = 1'b0;
			@(negedge rt_clk); rt_we = 1'b0;
		end
	endtask

	// Read, then immediately write somewhere else, then capture: proves the
	// capture-on-the-next-edge rule holds even when a write follows.
	task rt_read_check(input [15:0] a, input [15:0] want);
		begin
			@(negedge rt_clk); rt_addr = a; rt_re = 1'b1; rt_we = 1'b0;
			@(negedge rt_clk); rt_re = 1'b0;
			rt_addr = 16'h2222; rt_wdata = 16'hDEAD; rt_we = 1'b1;
			// rt_rdata now shows the read; the write lands on the next edge.
			if (rt_rdata !== want) begin
				$display("FAIL: SPRAM bank test addr %04x read %04x, expected %04x", a, rt_rdata, want);
				errors = errors + 1;
			end
			@(negedge rt_clk); rt_we = 1'b0;
		end
	endtask

	// ------------------------------------------------------------------
	// Tests
	// ------------------------------------------------------------------
	integer    i, k, n_before, dup_before, gap_before;
	reg [15:0] c16;
	reg [31:0] start_before, ack_sent, abort_start;
	reg [8*9-1:0] check_str;

	initial begin
		if ($test$plusargs("vcd")) begin
			$dumpfile("spi_frame_stream_tb.vcd");
			$dumpvars(0, spi_frame_stream_tb);
		end
		if ($value$plusargs("dump=%s", dump_name))
			dump_fd = $fopen(dump_name, "w");

		$display("=== spi_frame_stream testbench: BUF_AW=%0d (%0d samples), offsets from %08x ===",
		         BUF_AW, CAPACITY, INITIAL_OFFSET);

		// -- 0: CRC check value, both implementations --------------------
		check_str = "123456789";
		c16 = 16'hFFFF;
		for (i = 8; i >= 0; i = i - 1)
			c16 = tb_crc(c16, check_str[8*i +: 8]);
		if (c16 !== 16'h29B1) begin
			$display("FAIL: testbench CRC-16/CCITT-FALSE check value %04x, expected 29b1", c16);
			errors = errors + 1;
		end
		c16 = 16'hFFFF;
		for (i = 8; i >= 0; i = i - 1)
			c16 = dut.crc16_byte(c16, check_str[8*i +: 8]);
		if (c16 !== 16'h29B1) begin
			$display("FAIL: RTL CRC-16/CCITT-FALSE check value %04x, expected 29b1", c16);
			errors = errors + 1;
		end

		// -- 0b: SPRAM wrapper across all four banks ----------------------
		rt_write(16'h0000, 16'h1000);
		rt_write(16'h3FFF, 16'h13FF);
		rt_write(16'h4000, 16'h2000);
		rt_write(16'h8001, 16'h3001);
		rt_write(16'hC000, 16'h4000);
		rt_write(16'hFFFF, 16'h4FFF);
		rt_read_check(16'h0000, 16'h1000);
		rt_read_check(16'h3FFF, 16'h13FF);
		rt_read_check(16'h4000, 16'h2000);
		rt_read_check(16'h8001, 16'h3001);
		rt_read_check(16'hC000, 16'h4000);
		rt_read_check(16'hFFFF, 16'h4FFF);
		$display("  SPRAM: four-bank decode and next-edge capture OK");

		// Let some samples accumulate.
		#(25 * SAMPLE_NS);

		// -- 1: first contact: MCU not synced, nothing acknowledged ------
		build_ctrl(256, 1'b0, 32'd0, 1'b0);
		xfer(256, -1);
		parse(256, "first");
		dump(256, "clean");
		if (!fr_ok) begin
			$display("FAIL [first]: no valid frame"); errors = errors + 1;
		end
		expect_status(8'h00, "first");
		if (fr_start !== INITIAL_OFFSET || fr_count == 0) begin
			$display("FAIL [first]: start %08x count %0d, expected start %08x and some samples",
			         fr_start, fr_count, INITIAL_OFFSET);
			errors = errors + 1;
		end
		$display("  first frame: start %08x, %0d samples", fr_start, fr_count);
		if (led_g !== 1'b0) begin
			$display("FAIL: green LED off after a valid control block"); errors = errors + 1;
		end

		// -- 2: steady streaming -----------------------------------------
		for (k = 0; k < 10; k = k + 1) begin
			start_before = expected;
			poll(256, "steady");
			expect_status(ST_ACK_APPLIED, "steady");
			if (fr_start !== start_before) begin
				$display("FAIL [steady]: frame starts at %0d, MCU acked %0d", fr_start, start_before);
				errors = errors + 1;
			end
		end
		$display("  steady: %0d samples, %0d duplicates, total value gap %0d", samples_new, samples_dup, total_gap);

		// -- 2b: the target rate, sustained ---------------------------------
		// Reads back to back at 2 MHz with the source at exactly 1 Mbps.
		// Keeping up means: nothing dropped, and the backlog stays at about
		// two reads' worth (the frame in flight is only freed by the next
		// read's ack) instead of growing.  Delivered throughput lags the
		// source by that in-flight read, so it reads a little under 1 Mbps
		// over a finite window — the backlog bound is the real check.
		// The MCU uses 2 KB reads; the 1 Ki-sample test buffer is too small
		// to hold two of those in flight, so it uses 1 KB.
		begin : sustained
			integer s0, d0, rl, per_read, worst;
			realtime t0;
			rl       = (CAPACITY >= 4096) ? 2048 : 1024;
			per_read = (rl * 8 * 2 * HALF) / SAMPLE_NS + 2;   // samples produced per read
			s0 = samples_new;
			d0 = dut.dropped;
			t0 = $realtime;
			worst = 0;
			for (k = 0; k < 24; k = k + 1) begin
				poll(rl, "1 Mbps");
				if (dut.level > worst) worst = dut.level;
			end
			if (dut.dropped !== d0) begin
				$display("FAIL [1 Mbps]: %0d samples dropped at the target rate", dut.dropped - d0);
				errors = errors + 1;
			end
			if (worst > 2 * per_read + 16) begin
				$display("FAIL [1 Mbps]: backlog reached %0d samples (bound %0d) — the MCU is falling behind",
				         worst, 2 * per_read + 16);
				errors = errors + 1;
			end
			$display("  1 Mbps sustained, %0d-byte reads: %0d samples in %0.1f ms (%0.3f Mbps), 0 dropped, backlog <= %0d samples",
			         rl, samples_new - s0, ($realtime - t0) / 1e6,
			         (samples_new - s0) * 16.0 / (($realtime - t0) / 1e9) / 1e6, worst);
			$display("  link ceiling at 2 MHz with %0d-byte reads: %0.2f Mbps of payload",
			         rl, (rl - 24.0) * 8 / ((rl * 8.0 * 2 * HALF + 4000) / 1e9) / 1e6);
		end

		// -- 3: corrupted control block ------------------------------------
		// The FPGA must ignore it entirely: no ack applied (the stream does
		// not move), and since read_len cannot be trusted either, an empty
		// frame — which fits any read — reporting CTRL_BAD.
		start_before = fr_start;              // the FPGA's committed point
		n_before     = fr_count;              // what the next good read acks
		build_ctrl(256, 1'b0, 32'd0, 1'b1);
		xfer(256, -1);
		parse(256, "bad ctrl");
		dump(256, "ctrl_corrupt");
		if (!fr_ok) begin
			$display("FAIL [bad ctrl]: the reply to a corrupt control block did not parse"); errors = errors + 1;
		end
		if (!(fr_status & ST_CTRL_BAD)) begin
			$display("FAIL [bad ctrl]: status %02x, CTRL_BAD not set", fr_status); errors = errors + 1;
		end
		if (fr_start !== start_before || fr_count !== 0) begin
			$display("FAIL [bad ctrl]: frame start %0d count %0d, expected start %0d and an empty frame",
			         fr_start, fr_count, start_before);
			errors = errors + 1;
		end
		if (led_g !== 1'b1) begin
			$display("FAIL: green LED still on after a corrupted control block"); errors = errors + 1;
		end
		poll(256, "after bad ctrl");
		if (!(fr_status & ST_ACK_APPLIED) || fr_start !== start_before + n_before) begin
			$display("FAIL [after bad ctrl]: status %02x start %0d", fr_status, fr_start); errors = errors + 1;
		end
		$display("  corrupted control block: ignored, empty frame sent, stream resumed where it was");

		// -- 4: corrupted frame on MISO --------------------------------------
		// The MCU must reject it and not ack it; the FPGA must resend it.
		build_ctrl(256, 1'b0, 32'd0, 1'b0);
		xfer(256, -1);
		miso_buf[22] = miso_buf[22] ^ 8'h04;  // first payload byte, one bit
		start_before = expected;
		parse(256, "bad frame");
		dump(256, "miso_corrupt");
		if (fr_ok) begin
			$display("FAIL [bad frame]: corrupted frame was accepted"); errors = errors + 1;
		end
		abort_start = {miso_buf[14], miso_buf[13], miso_buf[12], miso_buf[11]};
		poll(256, "after bad frame");
		if (fr_start !== abort_start) begin
			$display("FAIL [bad frame]: resend starts at %0d, rejected frame started at %0d",
			         fr_start, abort_start);
			errors = errors + 1;
		end
		$display("  corrupted frame: rejected and resent from %0d", fr_start);

		// -- 5: transfer aborted mid-payload ---------------------------------
		build_ctrl(256, 1'b0, 32'd0, 1'b0);
		xfer(256, 40);
		dump(256, "abort");
		abort_start = {miso_buf[14], miso_buf[13], miso_buf[12], miso_buf[11]};
		poll(256, "after abort");
		if (fr_start !== abort_start) begin
			$display("FAIL [abort]: resend starts at %0d, aborted frame started at %0d",
			         fr_start, abort_start);
			errors = errors + 1;
		end
		$display("  aborted transfer: resent from %0d", fr_start);

		// -- 6: ack beyond anything sent -------------------------------------
		// Must be refused: honouring it would free samples nobody received.
		start_before = expected;
		build_ctrl(256, 1'b1, expected + 32'd100000, 1'b0);
		xfer(256, -1);
		parse(256, "bogus ack");
		dump(256, "badack");
		expect_status(ST_ACK_REJECTED, "bogus ack");
		if (fr_start > start_before) begin
			$display("FAIL [bogus ack]: FPGA freed samples up to %0d on a bogus ack", fr_start);
			errors = errors + 1;
		end
		poll(256, "after bogus ack");
		$display("  out-of-range ack: rejected, nothing freed");

		// -- 7: read length changes, frames follow ---------------------------
		poll(32, "len 32");
		if (fr_count > (32 - 24) / 2) begin
			$display("FAIL [len 32]: %0d samples in a 32-byte read", fr_count); errors = errors + 1;
		end
		poll(64, "len 64");
		#(60 * SAMPLE_NS);
		poll(1024, "len 1024");
		poll(1024, "len 1024");
		poll(256, "len 256");
		$display("  read lengths 32/64/1024/256: frames sized to fit, stream continuous");

		// -- 8: MCU reboot ---------------------------------------------------
		// The last ack the FPGA applied is the `expected` we sent in the last
		// poll.  After a reboot the MCU sends no ack; the FPGA must restart
		// from exactly that point — nothing past it may have been freed.
		build_ctrl(256, 1'b0, 32'd0, 1'b0);
		ack_sent = {mosi_buf[4], mosi_buf[3], mosi_buf[2], mosi_buf[1]};
		xfer(256, -1);
		parse(256, "pre-reboot");
		dump(256, "clean");
		synced     = 1'b0;                    // reboot: all MCU state lost
		val_primed = 1'b0;
		build_ctrl(256, 1'b0, 32'd0, 1'b0);
		xfer(256, -1);
		parse(256, "reboot");
		dump(256, "reboot");
		if (fr_start !== ack_sent) begin
			$display("FAIL [reboot]: restarted at %0d, last applied ack was %0d", fr_start, ack_sent);
			errors = errors + 1;
		end
		poll(256, "after reboot");
		$display("  MCU reboot: resumed from the last acknowledged sample (%0d)", ack_sent);

		gap_before = total_gap;
		if (dut.dropped !== 0) begin
			$display("FAIL: FPGA dropped %0d samples while the MCU was keeping up", dut.dropped);
			errors = errors + 1;
		end
		if (total_gap !== 0) begin
			$display("FAIL: %0d samples missing although the FPGA dropped none", total_gap);
			errors = errors + 1;
		end

		// -- 9: SCK sweep ------------------------------------------------------
		// Each rate puts the frame decision at a different point relative to
		// the byte loads, so sweep several — including odd ones — up to the
		// stated 3 MHz ceiling.  This checks frame integrity only: below
		// ~1 MHz the link cannot carry the 50 kHz test sensor, so the slow
		// rates drop samples.  Those drops are legitimate and are checked
		// by the exact accounting at the end of the overflow test.
		for (i = 0; i < 7; i = i + 1) begin
			case (i)
				0: HALF = 1250;                   // 400 kHz
				1: HALF = 500;                    // 1 MHz
				2: HALF = 417;                    // 1.2 MHz
				3: HALF = 333;                    // 1.5 MHz
				4: HALF = 250;                    // 2 MHz
				5: HALF = 200;                    // 2.5 MHz
				6: HALF = 167;                    // 3 MHz, the stated ceiling
			endcase
			for (k = 0; k < 3; k = k + 1) poll(256, "sck sweep");
		end
		HALF = 250;
		$display("  SCK sweep 400 kHz - 3 MHz: clean");

		// -- 10: buffer overflow (small buffer only) ---------------------------
		if (CAPACITY <= 1024) begin
			#((CAPACITY + 150) * SAMPLE_NS);   // MCU stops reading
			if (led_r !== 1'b0) begin
				$display("FAIL [overflow]: red LED not lit while dropping"); errors = errors + 1;
			end
			poll(1024, "overflow");
			if (!(fr_status & ST_DROPPED)) begin
				$display("FAIL [overflow]: DROPPED not reported (status %02x)", fr_status);
				errors = errors + 1;
			end
			if (fr_dropped == 0) begin
				$display("FAIL [overflow]: buffer was full but nothing reported dropped"); errors = errors + 1;
			end
			// Drain until the FPGA has stopped dropping for two frames in a
			// row, so every drop is behind a sample we have received.
			k = 0;
			n_before = -1;
			dup_before = 0;
			while (k < 20 && dup_before < 2) begin
				poll(1024, "drain");
				if (fr_dropped == n_before && !(fr_status & ST_DROPPED))
					dup_before = dup_before + 1;
				else
					dup_before = 0;
				n_before = fr_dropped;
				k = k + 1;
			end
			// Every missing value must be one the FPGA says it dropped.
			if (total_gap !== fr_dropped) begin
				$display("FAIL [overflow]: %0d samples missing but FPGA reports %0d dropped",
				         total_gap, fr_dropped);
				errors = errors + 1;
			end else
				$display("  overflow: %0d samples dropped, all accounted for, nothing else lost (%0d drain reads)",
				         fr_dropped, k);
		end else begin
			$display("  overflow: skipped (buffer too large to fill in simulation)");
			// Still close the books: drain whatever the slow sweep left
			// buffered, then every missing value must be a reported drop.
			for (k = 0; k < 8; k = k + 1) poll(1024, "final drain");
			if (total_gap !== fr_dropped) begin
				$display("FAIL [final]: %0d samples missing but FPGA reports %0d dropped",
				         total_gap, fr_dropped);
				errors = errors + 1;
			end
		end

		$display("  totals: %0d samples delivered, %0d duplicates skipped, %0d dropped by the FPGA; stream ended at offset %08x",
		         samples_new, samples_dup, total_gap, expected);

		if (dump_fd != 0)
			$fclose(dump_fd);

		if (mem_errors != 0) begin
			$display("FAIL: %0d memory-safety violation(s)", mem_errors);
			errors = errors + mem_errors;
		end else
			$display("  memory safety: no read outside buffered data, no overwrite of unacked data");

		if (errors == 0)
			$display("PASS: all spi_frame_stream tests completed successfully");
		else
			$display("FAIL: %0d error(s)", errors);
		$finish;
	end

	// Watchdog.
	initial begin
		#2_000_000_000;
		$display("FAIL: watchdog timeout");
		$finish;
	end

endmodule
