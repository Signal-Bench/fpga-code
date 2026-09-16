/** \file
 * End-to-end test for can_sniffer: CAN bit stream in, SPI records out.
 *
 * Builds real CAN frames (own CRC-15 and bit stuffing), drives them onto
 * can_rx at 1 Mbit/s, then decodes the SPI master's output the way a logic
 * analyzer would -- sampling MOSI on SCK rising while CS is low -- and checks
 * the resulting 20-byte records field by field.
 */
`timescale 1ns/1ps

module can_sniffer_tb;
	localparam real CLK_NS = 83.333333;
	localparam real BIT_NS = 1000.0;

	reg clk = 1'b0, rx = 1'b1;
	wire spi_sck, spi_mosi, spi_cs, spi_cs_flash, dbg_frame, led_r, led_g, led_b;

	can_sniffer uut (
		.clk_12(clk), .can_rx(rx),
		.spi_sck(spi_sck), .spi_mosi(spi_mosi), .spi_cs(spi_cs),
		.spi_cs_flash(spi_cs_flash), .dbg_frame(dbg_frame),
		.led_r(led_r), .led_g(led_g), .led_b(led_b)
	);

	always #(CLK_NS/2.0) clk = ~clk;

	// ----------------------------------------------------------------
	// SPI sink: sample MOSI on SCK rising while CS is low, MSB first
	// ----------------------------------------------------------------
	reg [7:0] rxb [0:255];
	integer   nbytes = 0, nbits = 0;
	reg [7:0] sr = 8'd0;

	always @(posedge spi_sck) if (!spi_cs) begin
		sr = {sr[6:0], spi_mosi};
		nbits = nbits + 1;
		if (nbits == 8) begin
			rxb[nbytes] = sr;
			nbytes = nbytes + 1;
			nbits  = 0;
		end
	end

	// ----------------------------------------------------------------
	// CAN frame construction (identical to can_frame_fsm_tb)
	// ----------------------------------------------------------------
	reg     raw [0:255];
	reg     txb [0:383];
	integer raw_len, tx_len, i, run, nbytes_d;
	reg        last;
	reg [14:0] crc;
	reg        nxt;

	task push_raw(input b); begin raw[raw_len] = b; raw_len = raw_len + 1; end endtask
	task push_tx (input b); begin txb[tx_len]  = b; tx_len  = tx_len  + 1; end endtask

	task build_frame(input [28:0] id, input ide, input rtr,
	                 input [3:0] dlc, input [63:0] data, input ack_dom);
		begin
			raw_len = 0;
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
			nbytes_d = (dlc > 8) ? 8 : dlc;
			if (!rtr) for (i = 0; i < nbytes_d * 8; i = i + 1) push_raw(data[63 - i]);
			crc = 15'd0;
			for (i = 0; i < raw_len; i = i + 1) begin
				nxt = raw[i] ^ crc[14];
				crc = {crc[13:0], 1'b0};
				if (nxt) crc = crc ^ 15'h4599;
			end
			for (i = 14; i >= 0; i = i - 1) push_raw(crc[i]);
			tx_len = 0; run = 0; last = 1'bx;
			for (i = 0; i < raw_len; i = i + 1) begin
				if (raw[i] === last) run = run + 1;
				else begin run = 1; last = raw[i]; end
				push_tx(raw[i]);
				if (run == 5) begin push_tx(~last); last = ~last; run = 1; end
			end
			push_tx(1'b1);
			push_tx(ack_dom ? 1'b0 : 1'b1);
			push_tx(1'b1);
			for (i = 0; i < 7; i = i + 1) push_tx(1'b1);
			for (i = 0; i < 3; i = i + 1) push_tx(1'b1);
		end
	endtask

	task drive_tx;
		begin
			for (i = 0; i < tx_len; i = i + 1) begin rx = txb[i]; #(BIT_NS); end
			rx = 1'b1;
		end
	endtask

	// ----------------------------------------------------------------
	// Checks
	// ----------------------------------------------------------------
	integer fails = 0, base = 0;
	reg [31:0] ts0, ts1;

	task chk(input [511:0] what, input [31:0] got, input [31:0] want);
		begin
			if (got !== want) begin
				$display("  FAIL  %0s: got %h want %h", what, got, want);
				fails = fails + 1;
			end
		end
	endtask

	task expect_record(input [511:0] name, input integer idx,
	                   input [28:0] id, input [7:0] flags,
	                   input [3:0] dlc, input [63:0] data);
		begin
			base = idx * 20;
			if (nbytes < base + 20) begin
				$display("  FAIL  %0s: only %0d bytes drained, need %0d", name, nbytes, base + 20);
				fails = fails + 1;
			end else begin
				chk("start marker", rxb[base+0],  8'hAA);
				chk("flags",        rxb[base+1],  flags);
				chk("id[28:24]",    rxb[base+2],  {3'b000, id[28:24]});
				chk("id[23:16]",    rxb[base+3],  id[23:16]);
				chk("id[15:8]",     rxb[base+4],  id[15:8]);
				chk("id[7:0]",      rxb[base+5],  id[7:0]);
				chk("dlc",          rxb[base+6],  {4'b0000, dlc});
				chk("data0",        rxb[base+7],  data[63:56]);
				chk("data1",        rxb[base+8],  data[55:48]);
				chk("data7",        rxb[base+14], data[7:0]);
				chk("end marker",   rxb[base+19], 8'h55);
				$display("  %0s  %0s", (fails == 0) ? "PASS " : "done ", name);
			end
		end
	endtask

	initial begin
		if ($test$plusargs("vcd")) begin
			$dumpfile("can_sniffer_tb.vcd");
			$dumpvars(0, can_sniffer_tb);
		end

		#(BIT_NS * 20);
		$display("CAN sniffer end-to-end (CAN in -> SPI records out)");

		// GM6020 feedback frame, acknowledged: clean decode
		build_frame(29'h205, 1'b0, 1'b0, 4'd8, 64'h1FFF_0064_00C8_2A00, 1'b1);
		drive_tx;
		#(BIT_NS * 260);          // let the 20-byte record drain at 1 MHz
		// flags = {ide,rtr,ovl,ack_ok,crc_ok,err} = 0,0,0,1,1,000
		expect_record("std 0x205 dlc8, acknowledged", 0,
		              29'h205, 8'b000_1_1_000, 4'd8, 64'h1FFF_0064_00C8_2A00);
		ts0 = {rxb[15], rxb[16], rxb[17], rxb[18]};

		// Same frame unacknowledged: decodes, but carries ERR_ACK (4) and ack_ok=0.
		// This is exactly the motor-alone bench case.
		build_frame(29'h205, 1'b0, 1'b0, 4'd8, 64'h1FFF_0064_00C8_2A00, 1'b0);
		drive_tx;
		#(BIT_NS * 260);
		// flags = 0,0,0,0,1,100 -- ack_ok clear, but crc_ok still set: the CRC
		// really was fine, only the acknowledgement was missing.
		expect_record("std 0x205 dlc8, no acknowledge", 1,
		              29'h205, 8'b000_0_1_100, 4'd8, 64'h1FFF_0064_00C8_2A00);

		// Extended frame
		build_frame(29'h15AA_5533, 1'b1, 1'b0, 4'd4, 64'hDEAD_BEEF_0000_0000, 1'b1);
		drive_tx;
		#(BIT_NS * 260);
		expect_record("ext 0x15AA5533 dlc4", 2,
		              29'h15AA_5533, 8'b100_1_1_000, 4'd4, 64'hDEAD_BEEF_0000_0000);

		ts1 = {rxb[55], rxb[56], rxb[57], rxb[58]};
		if (ts1 <= ts0) begin
			$display("  FAIL  timestamps not advancing: %0d then %0d", ts0, ts1);
			fails = fails + 1;
		end else
			$display("  PASS  timestamps advance (%0d -> %0d bit times)", ts0, ts1);

		$display("");
		if (fails == 0) $display("ALL TESTS PASSED (%0d bytes drained, %0d records)", nbytes, nbytes/20);
		else            $display("%0d FAILURE(S) (%0d bytes drained)", fails, nbytes);
		$finish;
	end
endmodule
