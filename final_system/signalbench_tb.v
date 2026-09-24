/** \file
 * End-to-end testbench for the SignalBench unified top.
 *
 * Drives the MCU SPI link exactly as the ESP32-H2 would: selects a
 * protocol, starts a capture, drives traffic on the corresponding probed
 * bus, then drains the capture RAM back over SPI and checks the records.
 *
 * PRIMITIVE MODELS.  There is no vendor simulation library in this repo, so
 * SB_PLL40_CORE, SB_GB_IO, SB_IO and SB_SPI are modelled behaviourally at
 * the bottom of this file.  The SB_SPI model is the one from
 * ../spi_counter_stream/spi_counter_stream_tb.v and carries that file's
 * caveats -- in particular it has NO LEADING DUMMY BYTE, whereas real
 * silicon requires one (FPGA-TN-02011 Table 12.8).  A passing run here is
 * therefore NOT evidence that the link works on hardware; it checks the
 * decoders, the central state machine, the packer and the drain ordering.
 */
`timescale 1ns/1ps

module signalbench_tb;
	localparam real CORE_NS = 1000.0/24.0;

	reg clk_12 = 1'b0;
	always #41.667 clk_12 = ~clk_12;      // 12 MHz

	// probed buses
	reg can_rx = 1'b1;
	reg scl = 1'b1, sda = 1'b1;
	reg uart_tx = 1'b1, uart_rx_l = 1'b1;
	reg p_sck = 1'b0, p_mosi = 1'b0, p_miso = 1'b0, p_cs = 1'b1;

	// MCU link
	reg  mcu_sck = 1'b0, mcu_cs = 1'b1, mcu_mosi = 1'b0;
	wire mcu_miso;
	wire mcu_irq, spi_cs_flash, led_r, led_g, led_b;

	integer errors = 0;

	signalbench dut (
		.clk_12(clk_12),
		.can_rx(can_rx), .scl_pin(scl), .sda_pin(sda),
		.uart_tx_pin(uart_tx), .uart_rx_pin(uart_rx_l),
		.p_sck(p_sck), .p_mosi(p_mosi), .p_miso(p_miso), .p_cs(p_cs),
		.mcu_sck(mcu_sck), .mcu_cs(mcu_cs), .mcu_mosi(mcu_mosi),
		.mcu_miso(mcu_miso), .mcu_irq(mcu_irq),
		.spi_cs_flash(spi_cs_flash),
		.led_r(led_r), .led_g(led_g), .led_b(led_b)
	);

	// ----------------------------------------------------------------
	// MCU-side SPI master, mode 0, ~2 MHz
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

	task mcu_write(input [7:0] op, input [7:0] dat);
		reg [7:0] junk;
		begin
			mcu_cs = 1'b0; #SCK_HALF;
			spi_xfer(op, junk);
			spi_xfer(dat, junk);
			#SCK_HALF mcu_cs = 1'b1; #(SCK_HALF*4);
		end
	endtask

	task mcu_cmd(input [7:0] op);
		reg [7:0] junk;
		begin
			mcu_cs = 1'b0; #SCK_HALF;
			spi_xfer(op, junk);
			#SCK_HALF mcu_cs = 1'b1; #(SCK_HALF*4);
		end
	endtask

	// Read n bytes after an opcode.  The leading byte is discarded as the
	// protocol dummy, which is what real silicon forces.
	task mcu_read(input [7:0] op, input integer n, output [7:0] first);
		reg [7:0] b;
		integer i;
		begin
			mcu_cs = 1'b0; #SCK_HALF;
			spi_xfer(op, b);
			spi_xfer(8'h00, b);          // dummy
			first = 8'hxx;
			for (i = 0; i < n; i = i + 1) begin
				spi_xfer(8'h00, b);
				if (i == 0) first = b;
			end
			#SCK_HALF mcu_cs = 1'b1; #(SCK_HALF*4);
		end
	endtask

	// ----------------------------------------------------------------
	// I2C driver, ~100 kHz
	// ----------------------------------------------------------------
	localparam real I2C_Q = 2500.0;

	task i2c_start; begin sda=1; scl=1; #I2C_Q; sda=0; #I2C_Q; scl=0; #I2C_Q; end endtask
	task i2c_stop;  begin sda=0; scl=0; #I2C_Q; scl=1; #I2C_Q; sda=1; #I2C_Q; end endtask

	task i2c_byte(input [7:0] d, input ack);
		integer i;
		begin
			for (i = 7; i >= 0; i = i - 1) begin
				sda = d[i]; #I2C_Q;
				scl = 1;    #(I2C_Q*2);
				scl = 0;    #I2C_Q;
			end
			sda = ~ack; #I2C_Q;         // ACK slot: low = acked
			scl = 1;     #(I2C_Q*2);
			scl = 0;     #I2C_Q;
			sda = 1;
		end
	endtask

	task uart_send(input [7:0] d);
		integer i;
		begin
			uart_tx = 1'b0; #1000;             // start bit, 1 Mbaud
			for (i = 0; i < 8; i = i + 1) begin
				uart_tx = d[i];  #1000;        // LSB first
			end
			uart_tx = 1'b1; #2000;             // stop bit + idle
		end
	endtask

	task check(input [7:0] got, input [7:0] want, input [255:0] name);
		begin
			if (got !== want) begin
				$display("  FAIL  %0s: got 0x%02x want 0x%02x", name, got, want);
				errors = errors + 1;
			end else begin
				$display("  PASS  %0s = 0x%02x", name, got);
			end
		end
	endtask

	reg [7:0] b0;
	integer k;

	initial begin
		if ($test$plusargs("vcd")) begin
			$dumpfile("signalbench_tb.vcd");
			$dumpvars(0, signalbench_tb);
		end

		#20000;                          // let the PLL model lock and POR clear

		$display("-- configure: protocol = I2C, untriggered record --");
		mcu_write(8'h01, 8'h00);         // SET_PROTOCOL = I2C
		mcu_write(8'h06, 8'h00);         // SET_TRIG_LEN = 0  -> record now
		mcu_cmd  (8'h10);                // ARM_RECORD

		$display("-- drive one I2C transaction: addr 0x36 W, data 0xA5 --");
		i2c_start;
		i2c_byte(8'h6C, 1'b1);           // 0x36 << 1 | W, acked
		i2c_byte(8'hA5, 1'b1);
		i2c_stop;
		#200000;

		$display("-- read status --");
		mcu_read(8'h20, 1, b0);
		if (b0[1:0] !== 2'd0) begin
			$display("  FAIL  status protocol field = %0d, want 0", b0[1:0]);
			errors = errors + 1;
		end else $display("  PASS  status protocol field = 0 (I2C)");

		$display("-- drain first record --");
		mcu_cs = 1'b0; #SCK_HALF;
		spi_xfer(8'h21, b0);             // DRAIN opcode
		spi_xfer(8'h00, b0);             // dummy
		spi_xfer(8'h00, b0); check(b0, 8'hAA, "record[0] start marker");
		spi_xfer(8'h00, b0); check(b0, 8'h28, "record[1] proto=I2C flags=start+ack");
		for (k = 2; k <= 6; k = k + 1) spi_xfer(8'h00, b0);
		spi_xfer(8'h00, b0); check(b0, 8'h6C, "record[7] payload = addr byte");
		#SCK_HALF mcu_cs = 1'b1;


		$display("");
		$display("== test 2: switch protocol to UART, untriggered ==");
		mcu_cmd  (8'h12);                // STOP
		mcu_write(8'h01, 8'h02);         // SET_PROTOCOL = UART
		mcu_write(8'h02, 8'd5);          // baud divisor low = 5 -> 1 Mbaud
		mcu_write(8'h03, 8'd0);          // baud divisor high
		mcu_write(8'h06, 8'h00);         // trig_len = 0
		mcu_cmd  (8'h10);                // ARM_RECORD
		#20000;
		uart_send(8'h5A);
		#50000;

		if (dut.u_cap.wr_rec == 0) begin
			$display("  FAIL  UART produced no record");
			errors = errors + 1;
		end else $display("  PASS  UART produced %0d record(s)", dut.u_cap.wr_rec);

		mcu_cs = 1'b0; #SCK_HALF;
		spi_xfer(8'h21, b0);
		spi_xfer(8'h00, b0);
		spi_xfer(8'h00, b0); check(b0, 8'hAA, "uart record[0] marker");
		spi_xfer(8'h00, b0); check(b0, 8'h80, "uart record[1] proto=UART, dir=TX");
		for (k = 2; k <= 6; k = k + 1) spi_xfer(8'h00, b0);
		spi_xfer(8'h00, b0); check(b0, 8'h5A, "uart record[7] payload");
		#SCK_HALF mcu_cs = 1'b1; #(SCK_HALF*4);

		$display("");
		$display("== test 3: trigger on a pattern, earlier bytes ignored ==");
		mcu_cmd  (8'h12);                // STOP
		mcu_write(8'h01, 8'h00);         // SET_PROTOCOL = I2C
		mcu_write(8'h06, 8'h01);         // trig_len = 1
		mcu_write(8'h08, 8'hC4);         // trigger on byte 0xC4
		mcu_cmd  (8'h10);                // ARM_RECORD
		#20000;

		i2c_start;
		i2c_byte(8'h10, 1'b1);           // before the trigger -- must be ignored
		i2c_byte(8'h22, 1'b1);           // ditto
		i2c_byte(8'hC4, 1'b1);           // <-- trigger fires here
		i2c_byte(8'h77, 1'b1);           // after -- must be captured
		i2c_stop;
		#200000;

		if (!dut.u_cap.recording && dut.u_cap.armed) begin
			$display("  FAIL  trigger never fired");
			errors = errors + 1;
		end else $display("  PASS  trigger fired");

		mcu_cs = 1'b0; #SCK_HALF;
		spi_xfer(8'h21, b0);
		spi_xfer(8'h00, b0);
		spi_xfer(8'h00, b0); check(b0, 8'hAA, "trig record[0] marker");
		for (k = 1; k <= 6; k = k + 1) spi_xfer(8'h00, b0);
		spi_xfer(8'h00, b0); check(b0, 8'hC4, "trig record[7] = the matching byte");
		for (k = 8; k <= 31; k = k + 1) spi_xfer(8'h00, b0);
		spi_xfer(8'h00, b0); check(b0, 8'hAA, "next record[0] marker");
		for (k = 1; k <= 6; k = k + 1) spi_xfer(8'h00, b0);
		spi_xfer(8'h00, b0); check(b0, 8'h77, "next record[7] = byte after trigger");
		#SCK_HALF mcu_cs = 1'b1;

		$display("");
		if (errors == 0) $display("ALL TESTS PASSED");
		else             $display("%0d FAILURE(S)", errors);
		$finish;
	end

	initial begin
		#50_000_000;
		$display("TIMEOUT");
		$finish;
	end
endmodule

// ====================================================================
// Behavioural primitive models -- see the file header.
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
	// DIVR=0, DIVF=63, DIVQ=5 with SIMPLE feedback is exactly x2.
	reg out = 1'b0;
	always @(posedge REFERENCECLK) out <= 1'b1;
	reg clk2 = 1'b0;
	always #20.8333 clk2 = ~clk2;        // 24 MHz
	reg locked = 1'b0;
	initial begin #5000; locked = 1'b1; end
	assign PLLOUTCORE   = clk2;
	assign PLLOUTGLOBAL = clk2;
	assign LOCK         = locked;
endmodule

module SB_GB_IO (input PACKAGE_PIN, output GLOBAL_BUFFER_OUTPUT, output D_IN_0);
	parameter PIN_TYPE = 6'b0000_01;
	parameter PULLUP = 1'b0;
	assign GLOBAL_BUFFER_OUTPUT = PACKAGE_PIN;
	assign D_IN_0 = PACKAGE_PIN;
endmodule

module SB_IO (inout PACKAGE_PIN, input OUTPUT_ENABLE, input D_OUT_0, output D_IN_0);
	parameter PIN_TYPE = 6'b0000_01;
	parameter PULLUP = 1'b0;
	assign PACKAGE_PIN = OUTPUT_ENABLE ? D_OUT_0 : 1'bz;
	assign D_IN_0 = PACKAGE_PIN;
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
