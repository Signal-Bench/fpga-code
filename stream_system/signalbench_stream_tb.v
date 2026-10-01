/** \file
 * Testbench for the SignalBench STREAM top.
 *
 * Talks to the DUT exactly as the ESP32 firmware does: ready check, mode
 * select with `53 42 4D xx`, then 16-byte read bursts with MOSI = 0x00.
 * ACKs and data are FOUND within a burst rather than expected at a fixed
 * position, as spi_counter_stream_tb does -- SB_SPI's TXDR pipelining makes
 * the exact offset depend on timing.
 *
 * The I2C stimulus tasks are copied VERBATIM from
 * ../i2c_sniffer/i2c_sniffer_tb.v, and the expected byte streams are that
 * testbench's own expectations.  So the I2C checks are a direct cross-check
 * against the hardware-tested sniffer, not a round trip against new code.
 *
 * The UART stimulus is tools/send_pattern.c's pattern at 9600 baud.
 *
 * PRIMITIVE MODELS at the bottom.  The SB_SPI model is the one from
 * ../final_system/signalbench_tb.v (originally spi_counter_stream_tb.v on
 * main) and is NOT silicon-accurate -- passing here says nothing about the
 * hard IP's real timing.
 */
`timescale 1ns/1ps

module signalbench_stream_tb;
	reg clk_12 = 1'b0;
	always #41.667 clk_12 = ~clk_12;

	// i2c_sniffer_tb.v uses scl_pin / sda_pin; ch0 / ch1 here.
	reg scl_pin = 1'b1, sda_pin = 1'b1;
	reg  uart_a = 1'b1, uart_b = 1'b1;
	reg  mode_uart = 1'b0;           // which rig is "plugged in" to ch0/ch1
	wire ch0 = mode_uart ? uart_a : scl_pin;
	wire ch1 = mode_uart ? uart_b : sda_pin;

	reg  p_sck = 1'b0, p_mosi = 1'b0, p_miso = 1'b0, p_cs = 1'b1;
	reg  mcu_sck = 1'b0, mcu_cs = 1'b1, mcu_mosi = 1'b0;
	wire mcu_miso, spi_cs_flash, led_r, led_g, led_b;

	signalbench_stream dut (
		.clk_12(clk_12), .ch0(ch0), .ch1(ch1),
		.p_sck(p_sck), .p_mosi(p_mosi), .p_miso(p_miso), .p_cs(p_cs),
		.mcu_sck(mcu_sck), .mcu_cs(mcu_cs), .mcu_mosi(mcu_mosi),
		.mcu_miso(mcu_miso), .spi_cs_flash(spi_cs_flash),
		.led_r(led_r), .led_g(led_g), .led_b(led_b)
	);

	integer errors = 0;

	// ----------------------------------------------------------------
	// MCU-side master: mode 0, 2 MHz -- the ESP32 firmware's default.
	// ----------------------------------------------------------------
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

	reg [7:0] junk;
	task send_cmd(input [7:0] b0, input [7:0] b1, input [7:0] b2, input [7:0] b3);
		begin
			mcu_cs = 1'b0; #SCK_HALF;
			spi_xfer(b0, junk); spi_xfer(b1, junk);
			spi_xfer(b2, junk); spi_xfer(b3, junk);
			#SCK_HALF mcu_cs = 1'b1;
			#5000;                      // turnaround, as the firmware does
		end
	endtask

	reg [7:0] burst [0:31];
	integer   blen;
	task read_burst(input integer n);
		integer i;
		reg [7:0] b;
		begin
			mcu_cs = 1'b0; #SCK_HALF;
			for (i = 0; i < n; i = i + 1) begin
				spi_xfer(8'h00, b);
				burst[i] = b;
			end
			#SCK_HALF mcu_cs = 1'b1;
			blen = n;
			#5000;
		end
	endtask

	task dump(input [255:0] label);
		integer i;
		begin
			$write("  %0s:", label);
			for (i = 0; i < blen; i = i + 1) $write(" %02x", burst[i]);
			$write("\n");
		end
	endtask

	// Find exp[0..n-1] as a contiguous run in the burst; -1 if absent.
	reg [7:0] exp [0:15];
	function integer find_seq(input integer n);
		integer i, j, ok;
		begin
			find_seq = -1;
			for (i = 0; i + n <= blen && find_seq < 0; i = i + 1) begin
				ok = 1;
				for (j = 0; j < n; j = j + 1)
					if (burst[i+j] !== exp[j]) ok = 0;
				if (ok) find_seq = i;
			end
		end
	endfunction

	task expect_seq(input integer n, input [255:0] label);
		integer at;
		begin
			at = find_seq(n);
			if (at < 0) begin
				$display("  FAIL  %0s: sequence not found", label);
				errors = errors + 1;
			end else
				$display("  PASS  %0s (at byte %0d)", label, at);
		end
	endtask

	task set_exp4(input [7:0] a, input [7:0] b, input [7:0] c, input [7:0] d);
		begin exp[0]=a; exp[1]=b; exp[2]=c; exp[3]=d; end
	endtask

	// ----------------------------------------------------------------
	// I2C stimulus -- VERBATIM from ../i2c_sniffer/i2c_sniffer_tb.v
	// ----------------------------------------------------------------
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

	// ----------------------------------------------------------------
	// UART stimulus, 8-N-1 LSB first.  Drives line A, line B, or both
	// in lockstep (to force the same-cycle collision case).
	// ----------------------------------------------------------------
	localparam real BIT_NS = 1.0e9 / 9600.0;

	task uart_send(input [7:0] d, input on_a, input on_b);
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
			uart_a = 1'b1; uart_b = 1'b1;
			#BIT_NS;
		end
	endtask

	integer k;
	reg [7:0] pattern [0:7];

	initial begin
		if ($test$plusargs("vcd")) begin
			$dumpfile("signalbench_stream_tb.vcd");
			$dumpvars(0, signalbench_stream_tb);
		end
		pattern[0]=8'hAA; pattern[1]=8'h12; pattern[2]=8'h34; pattern[3]=8'hFF;
		pattern[4]=8'hFF; pattern[5]=8'h9A; pattern[6]=8'hBC; pattern[7]=8'hDE;

		#30000;

		$display("== ready check ==");
		send_cmd(8'h53, 8'h42, 8'hA5, 8'h5A);
		read_burst(16); dump("burst");
		set_exp4(8'h53, 8'h42, 8'h4F, 8'h4B); expect_seq(4, "ready ACK 53 42 4F 4B");

		$display("== select I2C: 53 42 4D 00 ==");
		send_cmd(8'h53, 8'h42, 8'h4D, 8'h00);
		read_burst(16); dump("burst");
		set_exp4(8'h53, 8'h42, 8'h41, 8'h00); expect_seq(4, "mode ACK 53 42 41 00");

		$display("== I2C T1: START 6C ACK 0B ACK STOP  (i2c_sniffer_tb: FF 6C 05 80) ==");
		i2c_start();
		i2c_send_byte(8'h6C); i2c_send_bit(1'b0);
		i2c_send_byte(8'h0B); i2c_send_bit(1'b0);
		i2c_stop();
		#20000;
		read_burst(16); dump("burst");
		set_exp4(8'hFF, 8'h6C, 8'h05, 8'h80); expect_seq(4, "T1 stream");

		$display("== I2C T1b: ACK jammed against STOP, 80 ns  (expect FF 6C 05 80) ==");
		i2c_start();
		i2c_send_byte(8'h6C); i2c_send_bit(1'b0);
		i2c_send_byte(8'h0B);
		i2c_send_bit_then_stop(1'b0, 80);
		#20000;
		read_burst(16); dump("burst");
		set_exp4(8'hFF, 8'h6C, 8'h05, 8'h80); expect_seq(4, "T1b stream");

		$display("== I2C T2: write, repeated START, read, NACK, STOP ==");
		$display("   (i2c_sniffer_tb: FF 6C 05 A0 FF 6D 49 C0)");
		i2c_start();
		i2c_send_byte(8'h6C); i2c_send_bit(1'b0);
		i2c_send_byte(8'h0B); i2c_send_bit(1'b0);
		i2c_repeated_start();
		i2c_send_byte(8'h6D); i2c_send_bit(1'b0);
		i2c_send_byte(8'h93); i2c_send_bit(1'b1);
		i2c_stop();
		#20000;
		read_burst(16); dump("burst");
		exp[0]=8'hFF; exp[1]=8'h6C; exp[2]=8'h05; exp[3]=8'hA0;
		exp[4]=8'hFF; exp[5]=8'h6D; exp[6]=8'h49; exp[7]=8'hC0;
		expect_seq(8, "T2 stream");

		$display("== select UART: 53 42 4D 04 ==");
		send_cmd(8'h53, 8'h42, 8'h4D, 8'h04);
		read_burst(16); dump("burst");
		set_exp4(8'h53, 8'h42, 8'h41, 8'h04); expect_seq(4, "mode ACK 53 42 41 04");
		mode_uart = 1'b1;
		#20000;

		$display("== UART: send_pattern.c bytes on line A at 9600 ==");
		for (k = 0; k < 8; k = k + 1) uart_send(pattern[k], 1'b1, 1'b0);
		#20000;
		read_burst(16); dump("burst");
		for (k = 0; k < 8; k = k + 1) exp[k] = pattern[k];
		expect_seq(8, "AA 12 34 FF FF 9A BC DE");

		$display("== UART: same byte on BOTH lines in lockstep (collision) ==");
		uart_send(8'h5C, 1'b1, 1'b1);
		#20000;
		read_burst(16); dump("burst");
		exp[0]=8'h5C; exp[1]=8'h5C; expect_seq(2, "both bytes kept: 5C 5C");

		$display("== invalid mode 53 42 4D 07: no ACK, protocol unchanged ==");
		send_cmd(8'h53, 8'h42, 8'h4D, 8'h07);
		read_burst(16); dump("burst");
		set_exp4(8'h53, 8'h42, 8'h41, 8'h07);
		if (find_seq(4) >= 0) begin
			$display("  FAIL  invalid mode was ACKed"); errors = errors + 1;
		end else $display("  PASS  invalid mode not ACKed");
		if (dut.proto !== 3'd4) begin
			$display("  FAIL  proto changed to %0d", dut.proto); errors = errors + 1;
		end else $display("  PASS  proto still UART");

		$display("== switch flushes the FIFO ==");
		// Leave the whole 8-byte pattern unread.  The 4-byte command
		// transaction clocks some of it out (and the MCU discards those),
		// so more than 4 must be queued for the flush to be exercised.
		for (k = 0; k < 8; k = k + 1) uart_send(pattern[k], 1'b1, 1'b0);
		#20000;
		$display("   FIFO holds %0d bytes before the switch",
		         dut.u_link.wr_ptr - dut.u_link.rd_ptr);
		mode_uart = 1'b0;
		send_cmd(8'h53, 8'h42, 8'h4D, 8'h00);
		$display("   FIFO holds %0d bytes after the switch",
		         dut.u_link.wr_ptr - dut.u_link.rd_ptr);
		read_burst(16); dump("burst");
		set_exp4(8'h53, 8'h42, 8'h41, 8'h00); expect_seq(4, "mode ACK 53 42 41 00");
		begin : no_stale
			integer i, stale;
			stale = 0;
			for (i = 0; i < blen; i = i + 1)
				if (burst[i] === 8'hAA || burst[i] === 8'h12 || burst[i] === 8'h34 ||
				    burst[i] === 8'h9A || burst[i] === 8'hBC || burst[i] === 8'hDE)
					stale = 1;
			if (stale) begin
				$display("  FAIL  stale UART bytes survived the switch"); errors = errors + 1;
			end else $display("  PASS  no stale UART bytes after the switch");
		end

		$display("");
		if (errors == 0) $display("ALL TESTS PASSED");
		else             $display("%0d FAILURE(S)", errors);
		$finish;
	end

	initial begin
		#200_000_000;
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
