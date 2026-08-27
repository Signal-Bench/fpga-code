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

module i2c_sniffer_start_sweep_tb;
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
	localparam integer MAX_LOG_BYTES = 8;

	integer phase_ns;
	integer start_gap_ns;

	reg [7:0] write_log [0:MAX_LOG_BYTES-1];
	integer write_count;
	integer idx;

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

	task wait_clk_cycles;
		input integer count;
		integer step;
		begin
			for (step = 0; step < count; step = step + 1)
				@(posedge uut.clk_48);
		end
	endtask

	task i2c_start_gap;
		input integer sda_to_scl_fall_ns;
		begin
			sda_pin = 1'b1;
			scl_pin = 1'b1;
			#(I2C_HALF_PERIOD_NS);
			sda_pin = 1'b0;
			#(sda_to_scl_fall_ns);
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

	task i2c_repeated_start_gap;
		input integer sda_to_scl_fall_ns;
		begin
			scl_pin = 1'b0;
			sda_pin = 1'b1;
			#(I2C_HALF_PERIOD_NS);
			scl_pin = 1'b1;
			#(I2C_HALF_PERIOD_NS);
			sda_pin = 1'b0;
			#(sda_to_scl_fall_ns);
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

	task print_result;
		begin
			$write("gap=%0dns phase=%0dns -> ", start_gap_ns, phase_ns);
			if ((write_count == 6) &&
			    (write_log[0] == 8'h6C) &&
			    (write_log[1] == 8'h05) &&
			    (write_log[2] == 8'hFF) &&
			    (write_log[3] == 8'hAD) &&
			    (write_log[4] == 8'hA9) &&
			    (write_log[5] == 8'h38)) begin
				$display("full_capture 6C 05 FF AD A9 38");
			end else if ((write_count == 3) &&
			             (write_log[0] == 8'h6D) &&
			             (write_log[1] == 8'h49) &&
			             (write_log[2] == 8'hC0)) begin
				$display("read_only 6D 49 C0");
			end else if (write_count == 0) begin
				$display("no_capture");
			end else begin
				$write("other");
				for (idx = 0; idx < write_count; idx = idx + 1)
					$write(" %02x", write_log[idx]);
				$display("");
			end
		end
	endtask

	always @(posedge uut.clk_48) begin
		if (uut.capture_ram_wr_en && (write_count < MAX_LOG_BYTES)) begin
			write_log[write_count] = uut.capture_ram_wr_data;
			write_count = write_count + 1;
		end
	end

	initial begin
		#10000000;
		$display("gap=%0dns phase=%0dns -> timeout", start_gap_ns, phase_ns);
		$finish(1);
	end

	initial begin
		phase_ns = 0;
		start_gap_ns = I2C_HALF_PERIOD_NS;
		if (!$value$plusargs("phase_ns=%d", phase_ns))
			phase_ns = 0;
		if (!$value$plusargs("start_gap_ns=%d", start_gap_ns))
			start_gap_ns = I2C_HALF_PERIOD_NS;
		write_count = 0;

		wait_clk_cycles(20);
		@(posedge uut.clk_48);
		#(phase_ns);

		i2c_start_gap(start_gap_ns);
		i2c_send_byte(8'h6C);
		i2c_send_bit(1'b0);
		i2c_send_byte(8'h0B);
		i2c_send_bit(1'b0);
		i2c_repeated_start_gap(start_gap_ns);
		i2c_send_byte(8'h6D);
		i2c_send_bit(1'b0);
		i2c_send_byte(8'h93);
		i2c_send_bit(1'b1);
		i2c_stop();

		wait_clk_cycles(2000);
		print_result();
		$finish(0);
	end
endmodule
