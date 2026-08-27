`timescale 1ns/1ps

module SB_HFOSC (
	input  wire CLKHFPU,
	input  wire CLKHFEN,
	output reg  CLKHF
);
	initial CLKHF = 1'b0;
	always begin
		#10.416667;
		if (CLKHFPU && CLKHFEN)
			CLKHF = ~CLKHF;
		else
			CLKHF = 1'b0;
	end
endmodule

module SB_GB (
	input  wire USER_SIGNAL_TO_GLOBAL_BUFFER,
	output wire GLOBAL_BUFFER_OUTPUT
);
	assign GLOBAL_BUFFER_OUTPUT = USER_SIGNAL_TO_GLOBAL_BUFFER;
endmodule

// ---------------------------------------------------------------------------
// Testbench for i2c_sniffer — ping-pong buffer variant
//
// Test pattern: START 0x36 W ACK 0x0B ACK START 0x36 R ACK 0x93 NACK STOP
//
// Expected packed SPI bytes (sentinel + raw SDA bitstream):
//
//   Phase 1 (write):
//     0x6C = address 0x36 with W=0 (bits 7-0)
//     0x05 = ACK(0) + 0x0B bits[7:1]   packed into one byte
//     0xA0 = 0x0B bit[0]=1, ACK=0, rST-setup-SCL=1  padded to MSB
//            (the i2c_repeated_start task raises SCL once with SDA=1 before
//             the actual START edge, which the sniffer captures as a bit)
//   0xFF  = repeated-START marker
//   Phase 2 (read):
//     0x6D = address 0x36 with R=1
//     0x49 = ACK(0) + 0x93 bits[7:1]
//     0xC0 = 0x93 bit[0]=1, NACK=1, STOP-setup-SCL=0  padded
//            (i2c_stop pulls SDA low then raises SCL before raising SDA)
//
// Buffer allocation (4 × 128-byte ping-pong, buf_sel[1:0] in addr[8:7]):
//   Test 1  → buf 0  (RAM[  0..  2])
//   Test 1b → buf 1  (RAM[128..130])
//   Test 2  → buf 2  (RAM[256..262])
//   Test 3  → buf 3  (RAM[384..510])
// ---------------------------------------------------------------------------
module i2c_sniffer_tb;
	reg scl_pin = 1'b1;
	reg sda_pin = 1'b1;

	wire spi_sck;
	wire spi_mosi;
	wire spi_cs;
	wire spi_cs_flash;
	wire led_r;
	wire led_g;
	wire led_b;

	localparam integer I2C_HALF_PERIOD_NS = 1250;

	i2c_sniffer uut (
		.scl_pin(scl_pin),
		.sda_pin(sda_pin),
		.spi_sck(spi_sck),
		.spi_mosi(spi_mosi),
		.spi_cs(spi_cs),
		.spi_cs_flash(spi_cs_flash),
		.led_r(led_r),
		.led_g(led_g),
		.led_b(led_b)
	);

	// -----------------------------------------------------------------------
	// Helpers
	// -----------------------------------------------------------------------
	task fail;
		input [1023:0] message;
		begin
			$display("FAIL: %0s", message);
			$finish(1);
		end
	endtask

	task expect_ram_byte;
		input [8:0]    addr;
		input [7:0]    expected;
		input [1023:0] message;
		begin
			if (uut.capture_ram[addr] !== expected) begin
				$display("RAM[%0d] = 0x%02x, expected 0x%02x", addr,
				         uut.capture_ram[addr], expected);
				fail(message);
			end
		end
	endtask

	task wait_clk_cycles;
		input integer count;
		integer idx;
		begin
			for (idx = 0; idx < count; idx = idx + 1)
				@(posedge uut.clk_48);
		end
	endtask

	task wait_for_spi_transfer_start;
		input integer max_cycles;
		integer count;
		begin
			if (spi_cs !== 1'b0) begin
				count = 0;
				while ((spi_cs !== 1'b0) && (count < max_cycles)) begin
					@(posedge uut.clk_48);
					count = count + 1;
				end
				if (spi_cs !== 1'b0) begin
					$display("SPI start timeout: capture_state=%0d spi_state=%0d cap_wr_ptr=%0d q_count=%0d cs=%b sck=%b",
					         uut.capture_state, uut.spi_state,
					         uut.cap_wr_ptr, uut.q_count,
					         spi_cs, spi_sck);
					fail("timed out waiting for SPI transfer start");
				end
			end
		end
	endtask

	task wait_for_spi_cs_high;
		input integer max_cycles;
		integer count;
		begin
			count = 0;
			while ((spi_cs !== 1'b1) && (count < max_cycles)) begin
				@(posedge uut.clk_48);
				count = count + 1;
			end
			if (spi_cs !== 1'b1)
				fail("timed out waiting for SPI chip-select to return high");
		end
	endtask

	// -----------------------------------------------------------------------
	// I2C stimulus tasks
	// -----------------------------------------------------------------------
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

	// Send last bit of a byte and then immediately issue a STOP.
	// stop_gap_ns controls the SDA-to-SCL hold time; use a small value
	// (e.g. 80 ns) to exercise the simultaneous stop_cond / scl_rising path.
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

	// Repeated START: SCL low → SDA high → SCL high → SDA low (START) → SCL low.
	// Note: the SCL rise with SDA=1 before the START edge is captured as one
	// extra data bit by the sniffer (this is expected/correct raw-stream behavior).
	task i2c_repeated_start;
		begin
			scl_pin = 1'b0;
			sda_pin = 1'b1;
			#(I2C_HALF_PERIOD_NS);
			scl_pin = 1'b1;          // captured as bit: SDA=1
			#(I2C_HALF_PERIOD_NS);
			sda_pin = 1'b0;          // START condition fires here
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

	// -----------------------------------------------------------------------
	// SPI capture tasks
	// -----------------------------------------------------------------------
	task spi_read_byte;
		output [7:0] data_value;
		integer bit_idx;
		begin
			data_value = 8'h00;
			for (bit_idx = 0; bit_idx < 8; bit_idx = bit_idx + 1) begin
				@(posedge spi_sck);
				#1;
				data_value = {data_value[6:0], spi_mosi};
			end
		end
	endtask

	task spi_expect_byte;
		input [7:0]    expected;
		input [1023:0] message;
		reg [7:0] observed;
		begin
			spi_read_byte(observed);
			if (observed !== expected) begin
				$display("SPI byte: observed 0x%02x, expected 0x%02x", observed, expected);
				fail(message);
			end
		end
	endtask

	// -----------------------------------------------------------------------
	// Timeout watchdog
	// -----------------------------------------------------------------------
	integer idx;

	initial begin
		#80_000_000;
		$display("TIMEOUT: capture_state=%0d spi_state=%0d cap_wr_ptr=%0d q_count=%0d spi_cs=%b spi_sck=%b",
		         uut.capture_state, uut.spi_state,
		         uut.cap_wr_ptr, uut.q_count, spi_cs, spi_sck);
		$finish(1);
	end

	// -----------------------------------------------------------------------
	// Test sequence
	// -----------------------------------------------------------------------
	initial begin
		if ($test$plusargs("vcd")) begin
			$dumpfile("i2c_sniffer_tb.vcd");
			$dumpvars(0, i2c_sniffer_tb);
		end

		wait_clk_cycles(20);
		if (spi_cs_flash !== 1'b1)
			fail("spi_cs_flash should stay high");

		// -------------------------------------------------------------------
		// Test 1: simple write transaction
		//   START 0x36 W ACK 0x0B ACK STOP
		//
		//   Expected SPI:  FF  6C  05  80
		//   Expected RAM (buf 0, base=0):  [0]=6C  [1]=05  [2]=80
		// -------------------------------------------------------------------
		i2c_start();
		i2c_send_byte(8'h6C);       // 0x36 W
		i2c_send_bit(1'b0);         // ACK
		i2c_send_byte(8'h0B);       // data
		i2c_send_bit(1'b0);         // ACK
		i2c_stop();

		wait_for_spi_transfer_start(20000);
		spi_expect_byte(8'hFF, "T1: sentinel mismatch");
		spi_expect_byte(8'h6C, "T1: byte 0 (addr) mismatch");
		spi_expect_byte(8'h05, "T1: byte 1 (ACK+data[7:1]) mismatch");
		spi_expect_byte(8'h80, "T1: byte 2 (partial stop) mismatch");
		wait_for_spi_cs_high(20000);

		expect_ram_byte(9'd0, 8'h6C, "T1 RAM[0]");
		expect_ram_byte(9'd1, 8'h05, "T1 RAM[1]");
		expect_ram_byte(9'd2, 8'h80, "T1 RAM[2]");
		wait_clk_cycles(8);

		// -------------------------------------------------------------------
		// Test 1b: same transaction but ACK and STOP are jammed together
		//   (exercises the simultaneous stop_cond / scl_rising corner case)
		//
		//   Expected SPI:  FF  6C  05  80
		//   Expected RAM (buf 1, base=128):  [128]=6C  [129]=05  [130]=80
		// -------------------------------------------------------------------
		i2c_start();
		i2c_send_byte(8'h6C);
		i2c_send_bit(1'b0);
		i2c_send_byte(8'h0B);
		i2c_send_bit_then_stop(1'b0, 80);

		wait_for_spi_transfer_start(20000);
		spi_expect_byte(8'hFF, "T1b: sentinel mismatch");
		spi_expect_byte(8'h6C, "T1b: byte 0 mismatch");
		spi_expect_byte(8'h05, "T1b: byte 1 mismatch");
		spi_expect_byte(8'h80, "T1b: byte 2 mismatch");
		wait_for_spi_cs_high(20000);

		expect_ram_byte(9'd128, 8'h6C, "T1b RAM[128]");
		expect_ram_byte(9'd129, 8'h05, "T1b RAM[129]");
		expect_ram_byte(9'd130, 8'h80, "T1b RAM[130]");
		wait_clk_cycles(8);

		// -------------------------------------------------------------------
		// Test 2: repeated-START transaction (the main regression)
		//   START 0x36 W ACK 0x0B ACK  START  0x36 R ACK 0x93 NACK STOP
		//
		//   The ACK before the repeated START plus the setup SCL edge inside
		//   i2c_repeated_start produce a 3-bit partial byte (0xA0) that must
		//   be stored before the 0xFF marker.
		//
		//   Expected SPI:  FF  6C  05  A0  FF  6D  49  C0
		//   Expected RAM (buf 2, base=256):
		//     [256]=6C  [257]=05  [258]=A0  [259]=FF
		//     [260]=6D  [261]=49  [262]=C0
		// -------------------------------------------------------------------
		i2c_start();
		i2c_send_byte(8'h6C);       // 0x36 W
		i2c_send_bit(1'b0);         // ACK
		i2c_send_byte(8'h0B);
		i2c_send_bit(1'b0);         // ACK  ← these bits end up in partial byte 0xA0
		i2c_repeated_start();       // flushes partial, writes 0xFF marker
		i2c_send_byte(8'h6D);       // 0x36 R
		i2c_send_bit(1'b0);         // ACK
		i2c_send_byte(8'h93);
		i2c_send_bit(1'b1);         // NACK
		i2c_stop();

		wait_for_spi_transfer_start(20000);
		spi_expect_byte(8'hFF, "T2: sentinel mismatch");
		spi_expect_byte(8'h6C, "T2: write addr byte mismatch");
		spi_expect_byte(8'h05, "T2: write data byte 1 mismatch");
		spi_expect_byte(8'hA0, "T2: partial byte before rSTART mismatch");
		spi_expect_byte(8'hFF, "T2: repeated-START marker mismatch");
		spi_expect_byte(8'h6D, "T2: read addr byte mismatch");
		spi_expect_byte(8'h49, "T2: read data byte 1 mismatch");
		spi_expect_byte(8'hC0, "T2: partial stop byte mismatch");
		wait_for_spi_cs_high(20000);

		expect_ram_byte(9'd256, 8'h6C, "T2 RAM[256] write addr");
		expect_ram_byte(9'd257, 8'h05, "T2 RAM[257] packed write");
		expect_ram_byte(9'd258, 8'hA0, "T2 RAM[258] partial before rSTART");
		expect_ram_byte(9'd259, 8'hFF, "T2 RAM[259] rSTART marker");
		expect_ram_byte(9'd260, 8'h6D, "T2 RAM[260] read addr");
		expect_ram_byte(9'd261, 8'h49, "T2 RAM[261] packed read");
		expect_ram_byte(9'd262, 8'hC0, "T2 RAM[262] partial stop");
		wait_clk_cycles(8);

		// -------------------------------------------------------------------
		// Test 3: overflow — buffer limit is 128 bytes per capture slot.
		//   Send 130 raw bytes to trigger the overflow flag, then verify
		//   the SPI drains exactly 127 bytes (the last write that would
		//   reach cap_wr_ptr==127 sets the overflow flag instead of writing).
		//
		//   Expected SPI:  FF  AA × 127
		//   Expected RAM (buf 3, base=384):  [384]=AA  [510]=AA
		// -------------------------------------------------------------------
		i2c_start();
		for (idx = 0; idx < 130; idx = idx + 1)
			i2c_send_byte(8'hAA);

		wait_clk_cycles(8);
		if (uut.buf_overflow !== 1'b1)
			fail("T3: buf_overflow should assert once per-buffer limit is reached");

		i2c_stop();
		wait_for_spi_transfer_start(500000);
		spi_expect_byte(8'hFF, "T3: sentinel mismatch");
		for (idx = 0; idx < 127; idx = idx + 1)
			spi_expect_byte(8'hAA, "T3: overflow payload byte mismatch");
		wait_for_spi_cs_high(500000);

		expect_ram_byte(9'd384, 8'hAA, "T3 RAM[384] first byte");
		expect_ram_byte(9'd510, 8'hAA, "T3 RAM[510] last stored byte");

		$display("PASS: all i2c_sniffer tests completed successfully");
		$finish(0);
	end
endmodule
