/** \file
 * Integration test for can_bit_timing + can_frame_fsm.
 *
 * The testbench builds real CAN frames from scratch -- it computes the CRC-15
 * and applies bit stuffing itself, independently of the DUT -- then drives the
 * resulting bit stream onto rx at 1 Mbit/s and checks what comes back out.
 *
 *   vvp can_frame_fsm_tb.out          run the suite
 *   vvp can_frame_fsm_tb.out +vcd     also dump can_frame_fsm_tb.vcd
 */
`timescale 1ns/1ps

module can_frame_fsm_tb;
	localparam real CLK_NS = 83.333333;   // 12 MHz
	localparam real BIT_NS = 1000.0;      // 1 Mbit/s

	localparam [2:0] ERR_NONE   = 3'd0;
	localparam [2:0] ERR_STUFF  = 3'd1;
	localparam [2:0] ERR_CRC    = 3'd2;
	localparam [2:0] ERR_FORM   = 3'd3;
	localparam [2:0] ERR_ACK    = 3'd4;
	localparam [2:0] ERR_UNATTR = 3'd5;
	localparam [2:0] ERR_DOM    = 3'd6;

	reg        clk = 1'b0;
	reg        rst = 1'b1;
	reg  [7:0] brp = 8'd0;
	reg        rx  = 1'b1;

	wire bus_idle, sample_tick, sample_bit, bit_tick, rx_sync, hard_sync, resync;
	wire        frame_strobe, frame_ide, frame_rtr, frame_crc_ok;
	wire        frame_ack_ok, frame_overload;
	wire [28:0] frame_id;
	wire [3:0]  frame_dlc;
	wire [63:0] frame_data;
	wire [2:0]  frame_err;

	can_bit_timing u_bt (
		.clk(clk), .rst(rst), .brp(brp), .rx_raw(rx), .bus_idle(bus_idle),
		.sample_tick(sample_tick), .sample_bit(sample_bit), .bit_tick(bit_tick),
		.rx_sync(rx_sync), .hard_sync(hard_sync), .resync(resync)
	);

	can_frame_fsm u_fsm (
		.clk(clk), .rst(rst),
		.sample_tick(sample_tick), .sample_bit(sample_bit),
		.bus_idle(bus_idle), .frame_strobe(frame_strobe),
		.frame_id(frame_id), .frame_ide(frame_ide), .frame_rtr(frame_rtr),
		.frame_dlc(frame_dlc), .frame_data(frame_data),
		.frame_crc_ok(frame_crc_ok), .frame_ack_ok(frame_ack_ok),
		.frame_overload(frame_overload), .frame_err(frame_err)
	);

	always #(CLK_NS/2.0) clk = ~clk;

	// ----------------------------------------------------------------
	// Capture the most recent record
	// ----------------------------------------------------------------
	integer    n_records = 0;
	reg [28:0] r_id;
	reg        r_ide, r_rtr, r_crc_ok, r_ack_ok, r_overload;
	reg [3:0]  r_dlc;
	reg [63:0] r_data;
	reg [2:0]  r_err;

	always @(posedge clk) if (frame_strobe) begin
		n_records  = n_records + 1;
		r_id       = frame_id;
		r_ide      = frame_ide;
		r_rtr      = frame_rtr;
		r_dlc      = frame_dlc;
		r_data     = frame_data;
		r_crc_ok   = frame_crc_ok;
		r_ack_ok   = frame_ack_ok;
		r_overload = frame_overload;
		r_err      = frame_err;
	end

	// ----------------------------------------------------------------
	// Frame construction: CRC-15 and bit stuffing done here, independently
	// ----------------------------------------------------------------
	reg     raw [0:255];
	reg     txb [0:383];
	integer raw_len, tx_len;
	integer i, run, nbytes;
	reg        last;
	reg [14:0] crc;
	reg        nxt;

	task push_raw(input b); begin raw[raw_len] = b; raw_len = raw_len + 1; end endtask
	task push_tx (input b); begin txb[tx_len]  = b; tx_len  = tx_len  + 1; end endtask

	// ack_dom: 1 = some node acknowledges (dominant ACK slot)
	// corrupt: 1 = flip a CRC bit so the frame fails its own check
	task build_frame(input [28:0] id, input ide, input rtr,
	                 input [3:0] dlc, input [63:0] data,
	                 input ack_dom, input corrupt);
		begin
			raw_len = 0;
			push_raw(1'b0);                                  // SOF
			if (!ide) begin
				for (i = 10; i >= 0; i = i - 1) push_raw(id[i]);
				push_raw(rtr);                               // RTR
				push_raw(1'b0);                              // IDE dominant
				push_raw(1'b0);                              // r0
			end else begin
				for (i = 28; i >= 18; i = i - 1) push_raw(id[i]);
				push_raw(1'b1);                              // SRR recessive
				push_raw(1'b1);                              // IDE recessive
				for (i = 17; i >= 0; i = i - 1) push_raw(id[i]);
				push_raw(rtr);                               // RTR
				push_raw(1'b0);                              // r1
				push_raw(1'b0);                              // r0
			end
			for (i = 3; i >= 0; i = i - 1) push_raw(dlc[i]);
			nbytes = (dlc > 8) ? 8 : dlc;
			if (!rtr)
				for (i = 0; i < nbytes * 8; i = i + 1) push_raw(data[63 - i]);

			// CRC-15 over the destuffed SOF..end-of-data stream
			crc = 15'd0;
			for (i = 0; i < raw_len; i = i + 1) begin
				nxt = raw[i] ^ crc[14];
				crc = {crc[13:0], 1'b0};
				if (nxt) crc = crc ^ 15'h4599;
			end
			if (corrupt) crc = crc ^ 15'h0001;
			for (i = 14; i >= 0; i = i - 1) push_raw(crc[i]);

			// Bit stuffing covers everything built so far
			tx_len = 0; run = 0; last = 1'bx;
			for (i = 0; i < raw_len; i = i + 1) begin
				if (raw[i] === last) run = run + 1;
				else begin run = 1; last = raw[i]; end
				push_tx(raw[i]);
				if (run == 5) begin
					push_tx(~last); last = ~last; run = 1;
				end
			end

			// Fixed-form tail, not stuffed
			push_tx(1'b1);                                   // CRC delimiter
			push_tx(ack_dom ? 1'b0 : 1'b1);                  // ACK slot
			push_tx(1'b1);                                   // ACK delimiter
			for (i = 0; i < 7; i = i + 1) push_tx(1'b1);     // EOF
			for (i = 0; i < 3; i = i + 1) push_tx(1'b1);     // intermission
		end
	endtask

	task drive_tx;
		begin
			for (i = 0; i < tx_len; i = i + 1) begin
				rx = txb[i];
				#(BIT_NS);
			end
			rx = 1'b1;
			#(BIT_NS * 6);
		end
	endtask

	// Drive all but the last drop_last bits, so a custom tail can be appended.
	// The built tail is 13 bits: CRC delim, ACK, ACK delim, 7 EOF, 3 IFS.
	task drive_partial(input integer drop_last);
		begin
			for (i = 0; i < tx_len - drop_last; i = i + 1) begin
				rx = txb[i];
				#(BIT_NS);
			end
		end
	endtask

	task drive_level(input b, input integer nbits);
		begin
			rx = b;
			#(BIT_NS * nbits);
			rx = 1'b1;
		end
	endtask

	// ----------------------------------------------------------------
	// Checks
	// ----------------------------------------------------------------
	integer fails = 0;
	integer expected_records = 0;

	task expect_frame(input [511:0] name, input [28:0] id, input ide,
	                  input rtr, input [3:0] dlc, input [63:0] data);
		begin
			expected_records = expected_records + 1;
			if (n_records != expected_records)
				report_fail(name, "no record emitted");
			else if (r_err !== ERR_NONE) report_fail(name, "unexpected error code");
			else if (r_id  !== id)       report_fail(name, "wrong id");
			else if (r_ide !== ide)      report_fail(name, "wrong IDE");
			else if (r_rtr !== rtr)      report_fail(name, "wrong RTR");
			else if (r_dlc !== dlc)      report_fail(name, "wrong DLC");
			else if (!rtr && dlc != 0 && r_data[63 -: 64] !== data)
				report_fail(name, "wrong data");
			else if (!r_crc_ok)          report_fail(name, "CRC not ok");
			else if (!r_ack_ok)          report_fail(name, "ACK not ok");
			else $display("  PASS  %0s", name);
		end
	endtask

	task expect_err(input [511:0] name, input [2:0] code);
		begin expect_err_n(name, code, 1); end
	endtask

	// n_new: how many records this stimulus is expected to produce; the check
	// is against the last one.
	task expect_err_n(input [511:0] name, input [2:0] code, input integer n_new);
		begin
			expected_records = expected_records + n_new;
			if (n_records != expected_records)
				report_fail(name, "no record emitted");
			else if (r_err !== code) begin
				$display("  FAIL  %0s : expected err %0d, got %0d", name, code, r_err);
				fails = fails + 1;
			end else
				$display("  PASS  %0s (err %0d)", name, code);
		end
	endtask

	task expect_overload(input [511:0] name, input flag);
		begin
			if (r_overload !== flag) begin
				$display("  FAIL  %0s : expected overload=%b, got %b", name, flag, r_overload);
				fails = fails + 1;
			end else
				$display("  PASS  %0s (overload=%b)", name, flag);
		end
	endtask

	task report_fail(input [511:0] name, input [511:0] why);
		begin
			$display("  FAIL  %0s : %0s", name, why);
			$display("        id=%h ide=%b rtr=%b dlc=%0d data=%h crc_ok=%b ack_ok=%b err=%0d records=%0d/%0d",
			         r_id, r_ide, r_rtr, r_dlc, r_data, r_crc_ok, r_ack_ok, r_err,
			         n_records, expected_records);
			fails = fails + 1;
			if (n_records != expected_records) expected_records = n_records;
		end
	endtask

	// ----------------------------------------------------------------
	initial begin
		if ($test$plusargs("vcd")) begin
			$dumpfile("can_frame_fsm_tb.vcd");
			$dumpvars(0, can_frame_fsm_tb);
		end

		repeat (4) @(posedge clk);
		rst = 1'b0;
		#(BIT_NS * 12);

		$display("CAN frame decoder tests");

		// GM6020 feedback frame: ID 0x205, DLC 8, plausible payload
		build_frame(29'h205, 1'b0, 1'b0, 4'd8, 64'h1FFF_0064_00C8_2A00, 1'b1, 1'b0);
		drive_tx;
		expect_frame("std 0x205 dlc8 (GM6020 feedback)", 29'h205, 1'b0, 1'b0,
		             4'd8, 64'h1FFF_0064_00C8_2A00);

		// GM6020 command frame
		build_frame(29'h1FF, 1'b0, 1'b0, 4'd8, 64'h0BB8_F448_0000_0000, 1'b1, 1'b0);
		drive_tx;
		expect_frame("std 0x1FF dlc8 (GM6020 command)", 29'h1FF, 1'b0, 1'b0,
		             4'd8, 64'h0BB8_F448_0000_0000);

		// Payload chosen to force many stuff bits (long runs of equal bits)
		build_frame(29'h700, 1'b0, 1'b0, 4'd8, 64'h0000_0000_FFFF_FFFF, 1'b1, 1'b0);
		drive_tx;
		expect_frame("std stuff-heavy payload", 29'h700, 1'b0, 1'b0,
		             4'd8, 64'h0000_0000_FFFF_FFFF);

		// Extended frame, full 29-bit identifier
		build_frame(29'h15AA_5533, 1'b1, 1'b0, 4'd4, 64'hDEAD_BEEF_0000_0000, 1'b1, 1'b0);
		drive_tx;
		expect_frame("ext 0x15AA5533 dlc4", 29'h15AA_5533, 1'b1, 1'b0,
		             4'd4, 64'hDEAD_BEEF_0000_0000);

		// DLC 0 -- no data field at all
		build_frame(29'h123, 1'b0, 1'b0, 4'd0, 64'h0, 1'b1, 1'b0);
		drive_tx;
		expect_frame("std dlc0", 29'h123, 1'b0, 1'b0, 4'd0, 64'h0);

		// Remote frame: RTR recessive, no data regardless of DLC
		build_frame(29'h321, 1'b0, 1'b1, 4'd8, 64'h0, 1'b1, 1'b0);
		drive_tx;
		expect_frame("std remote frame", 29'h321, 1'b0, 1'b1, 4'd8, 64'h0);

		// Extended remote frame
		build_frame(29'h1FFF_FFFF, 1'b1, 1'b1, 4'd2, 64'h0, 1'b1, 1'b0);
		drive_tx;
		expect_frame("ext remote frame", 29'h1FFF_FFFF, 1'b1, 1'b1, 4'd2, 64'h0);

		// --- error cases ---

		// Corrupted CRC
		build_frame(29'h205, 1'b0, 1'b0, 4'd8, 64'h1122_3344_5566_7788, 1'b1, 1'b1);
		drive_tx;
		expect_err("corrupted CRC", ERR_CRC);

		// Nobody acknowledges: recessive ACK slot.  This is what the bench
		// rig produces when the GM6020 is powered with the STM32 unplugged.
		build_frame(29'h205, 1'b0, 1'b0, 4'd8, 64'h1122_3344_5566_7788, 1'b0, 1'b0);
		drive_tx;
		expect_err("no acknowledge", ERR_ACK);

		// Stuff violation: 8 dominant bits in a row inside the stuffed region
		drive_level(1'b0, 8);
		#(BIT_NS * 16);
		expect_err("stuff error (8 dominant)", ERR_STUFF);

		// Bus stuck dominant well past any legal error flag.  From idle this
		// first looks like a SOF followed by a stuff violation, and only after
		// 13 dominant bits can it be called a stuck bus -- so two records.
		drive_level(1'b0, 24);
		#(BIT_NS * 16);
		expect_err_n("bus stuck dominant", ERR_DOM, 2);

		// Form error: dominant CRC delimiter, where the fixed form requires
		// recessive.  Driven as a 6-bit error flag right after the CRC.
		build_frame(29'h205, 1'b0, 1'b0, 4'd8, 64'hAA55_AA55_AA55_AA55, 1'b1, 1'b0);
		drive_partial(13);
		drive_level(1'b0, 6);
		#(BIT_NS * 16);
		expect_err("form error (dominant CRC delimiter)", ERR_FORM);

		// Unattributed: CRC and ACK both checked out, then the bus is jammed
		// during EOF.  Someone else rejected the frame for a reason invisible
		// from this point on the bus -- most likely a bit error, which cannot
		// be proven passively.
		build_frame(29'h206, 1'b0, 1'b0, 4'd2, 64'h1234_0000_0000_0000, 1'b1, 1'b0);
		drive_partial(10);
		drive_level(1'b0, 6);
		#(BIT_NS * 16);
		expect_err("unattributed (jammed during EOF)", ERR_UNATTR);

		// Overload frame between two data frames.  It is not an error and gets
		// no error code, but it must be parsed or frame sync is lost, and it
		// is flagged on the next record.
		build_frame(29'h207, 1'b0, 1'b0, 4'd1, 64'h5A00_0000_0000_0000, 1'b1, 1'b0);
		drive_partial(3);                 // stop before intermission
		expect_frame("frame before overload", 29'h207, 1'b0, 1'b0,
		             4'd1, 64'h5A00_0000_0000_0000);
		drive_level(1'b0, 6);             // overload flag
		drive_level(1'b1, 11);            // overload delimiter + intermission
		build_frame(29'h208, 1'b0, 1'b0, 4'd1, 64'h7700_0000_0000_0000, 1'b1, 1'b0);
		drive_tx;
		expect_frame("frame after overload", 29'h208, 1'b0, 1'b0,
		             4'd1, 64'h7700_0000_0000_0000);
		expect_overload("overload flagged on following record", 1'b1);

		#(BIT_NS * 20);
		$display("");
		if (fails == 0) $display("ALL TESTS PASSED (%0d records)", n_records);
		else            $display("%0d FAILURE(S)", fails);
		$finish;
	end
endmodule
