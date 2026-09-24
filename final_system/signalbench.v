/** \file
 * SignalBench unified top -- all four sniffers under one central state
 * machine, driven by the ESP32-H2 over SPI.
 *
 * See ../unified_top.md for the design rationale, the measured resource and
 * timing budget, and the reasoning behind every clocking decision here.
 * See CHANGES.md in this directory for what was modified in each existing
 * sniffer to get here.
 *
 * CLOCKING
 *   12 MHz board crystal (pin 20, jumper R16) -> SB_PLL40_CORE -> 24 MHz.
 *   The PLL multiplies exactly, so the crystal's ~50 ppm accuracy is
 *   preserved and CAN's ISO 11898-1 budget of df < 0.98% still clears by
 *   ~200x.  SB_HFOSC is NOT usable: +/-10% against a 0.98% budget.
 *
 *   CAN runs at brp = 1, so tq = 2 core clocks = 83.3 ns and 12 tq/bit gives
 *   1 Mbit/s -- numerically identical to the 12 MHz / brp = 0 configuration
 *   verified on hardware against the GM6020.
 *
 * CLOCK DOMAINS -- three, and only three:
 *   1. clk, 24 MHz     everything except the two below
 *   2. probed SCK      sb_spi's shift registers only (~40 flops)
 *   3. SB_SPI internal the hard IP's own shift register; crossing is in silicon
 *
 *   The capture RAM is in domain 1 for ALL protocols, including SPI.  It is
 *   shared, so it cannot be clocked by a signal only one decoder has.
 *
 * DECODER SELECTION
 *   All four decoders are instantiated permanently and in parallel.  The
 *   `proto` register selects which one drives the shared back end; the rest
 *   are held in synchronous reset.  Clock gating would be the wrong tool --
 *   it needs a global buffer per gated clock (there are 8), and manufactures
 *   clock domain crossings that need not exist.
 *
 * PIN 35 IS RESERVED.  Instantiating the PLL consumes that I/O site; any
 * assignment to it makes place-and-route fail outright.
 */
`include "../common/util.v"
`include "../common/uart.v"

module signalbench (
	input  wire clk_12,          // pin 20, GBIN -- 12 MHz crystal via R16

	// probed buses
	input  wire can_rx,          // via transceiver (VP230, RS on 10k, TXD to 3V3)
	input  wire scl_pin,
	input  wire sda_pin,
	input  wire uart_tx_pin,     // target's TX
	input  wire uart_rx_pin,     // target's RX
	input  wire p_sck,           // pin 37, GBIN -- probed SPI clock
	input  wire p_mosi,
	input  wire p_miso,
	input  wire p_cs,

	// MCU link: FPGA is the SPI slave
	input  wire mcu_sck,
	input  wire mcu_cs,
	input  wire mcu_mosi,
	inout  wire mcu_miso,
	output wire mcu_irq,

	output wire spi_cs_flash,    // hold the onboard flash deselected
	output wire led_r,
	output wire led_g,
	output wire led_b
);
	assign spi_cs_flash = 1'b1;

	localparam [1:0] P_I2C = 2'd0, P_CAN = 2'd1, P_UART = 2'd2, P_SPI = 2'd3;

	// ----------------------------------------------------------------
	// Clocking: 12 -> 24 MHz.  Divider values from `icepll -i 12 -o 24`.
	// VCO lands at 768 MHz, inside the 533-1066 MHz window of
	// FPGA-DS-02008 Table 4.22.
	// ----------------------------------------------------------------
	wire clk, pll_lock;
	SB_PLL40_CORE #(
		.FEEDBACK_PATH("SIMPLE"),
		.DIVR(4'd0), .DIVF(7'd63), .DIVQ(3'd5),
		.FILTER_RANGE(3'b001)
	) u_pll (
		.REFERENCECLK(clk_12), .PLLOUTCORE(clk), .LOCK(pll_lock),
		.RESETB(1'b1), .BYPASS(1'b0)
	);

	// Reset is held until the PLL locks; a decoder running on an unlocked
	// clock would produce garbage records rather than no records.
	reg [7:0] por = 8'd0;
	always @(posedge clk) if (!por[7]) por <= por + 8'd1;
	wire rst = ~por[7] | ~pll_lock;

	reg [31:0] timestamp = 32'd0;
	always @(posedge clk) timestamp <= timestamp + 32'd1;

	// ----------------------------------------------------------------
	// MCU link and configuration registers
	// ----------------------------------------------------------------
	wire [1:0]  proto;
	wire [15:0] baud_div;
	wire [7:0]  can_brp;
	wire [2:0]  spi_mode;
	wire [7:0]  trig0, trig1, trig2, trig3;
	wire [2:0]  trig_len;
	wire        arm_record, start_stream, stop_cmd, drain_advance;
	wire        armed, recording, streaming, done, dropped;
	wire [15:0] fill_level;
	wire [7:0]  drain_data;
	wire        miso_d, miso_oe;

	// Point-to-point link, but the IP's output enable is honoured so a
	// second slave could share the bus later without a rework.
	SB_IO #(.PIN_TYPE(6'b1010_01)) u_miso_io (
		.PACKAGE_PIN(mcu_miso), .OUTPUT_ENABLE(miso_oe), .D_OUT_0(miso_d)
	);

	sb_mcu_link u_mcu (
		.clk(clk), .rst(rst),
		.mcu_sck(mcu_sck), .mcu_cs(mcu_cs), .mcu_mosi(mcu_mosi),
		.mcu_miso(miso_d), .mcu_miso_oe(miso_oe),
		.proto(proto), .baud_div(baud_div), .can_brp(can_brp),
		.spi_mode(spi_mode),
		.trig0(trig0), .trig1(trig1), .trig2(trig2), .trig3(trig3),
		.trig_len(trig_len),
		.arm_record(arm_record), .start_stream(start_stream),
		.stop(stop_cmd), .drain_advance(drain_advance),
		.armed(armed), .recording(recording), .streaming(streaming),
		.done(done), .dropped(dropped), .fill_level(fill_level),
		.drain_data(drain_data)
	);

	// A decoder that is not selected is held in reset.  This also means a
	// protocol switch always restarts the decoder from a known state.
	wire rst_i2c  = rst | (proto != P_I2C);
	wire rst_can  = rst | (proto != P_CAN);
	wire rst_uart = rst | (proto != P_UART);
	wire rst_spi  = rst | (proto != P_SPI);

	// ----------------------------------------------------------------
	// I2C
	// ----------------------------------------------------------------
	wire        i2c_stb;
	wire [5:0]  i2c_flags;
	wire [31:0] i2c_aux;
	wire [3:0]  i2c_len;
	wire [63:0] i2c_payload;
	sb_i2c u_i2c (
		.clk(clk), .rst(rst_i2c), .scl_pin(scl_pin), .sda_pin(sda_pin),
		.ev_strobe(i2c_stb), .ev_flags(i2c_flags), .ev_aux(i2c_aux),
		.ev_len(i2c_len), .ev_payload(i2c_payload)
	);

	// ----------------------------------------------------------------
	// UART -- both probed lines
	// ----------------------------------------------------------------
	wire        u_stb;
	wire [5:0]  u_flags;
	wire [31:0] u_aux;
	wire [3:0]  u_len;
	wire [63:0] u_payload;
	sb_uart u_uart (
		.clk(clk), .rst(rst_uart), .baud_div(baud_div),
		.tx_pin(uart_tx_pin), .rx_pin(uart_rx_pin),
		.ev_strobe(u_stb), .ev_flags(u_flags), .ev_aux(u_aux),
		.ev_len(u_len), .ev_payload(u_payload)
	);

	// ----------------------------------------------------------------
	// SPI -- probed SCK onto a global buffer, then its own clock domain.
	// A dedicated GBIN pin is used rather than letting nextpnr promote an
	// ordinary pin, because pad->fabric->global delay costs maximum
	// sniffable clock rate on exactly this signal.
	// ----------------------------------------------------------------
	wire sck_g;
	SB_GB_IO #(.PIN_TYPE(6'b0000_01)) u_sck_gb (
		.PACKAGE_PIN(p_sck), .GLOBAL_BUFFER_OUTPUT(sck_g)
	);

	wire        spi_stb;
	wire [5:0]  spi_flags;
	wire [31:0] spi_aux;
	wire [3:0]  spi_len;
	wire [63:0] spi_payload;
	sb_spi u_spi_dec (
		.clk(clk), .rst(rst_spi), .sck_g(sck_g),
		.mosi(p_mosi), .miso(p_miso), .cs(p_cs), .spi_mode(spi_mode),
		.ev_strobe(spi_stb), .ev_flags(spi_flags), .ev_aux(spi_aux),
		.ev_len(spi_len), .ev_payload(spi_payload)
	);

	// ----------------------------------------------------------------
	// CAN -- reused unmodified from can_sniffer/, except that the bit
	// timing is built with PIPELINE_RESYNC=1 so 24 MHz closes.  That
	// requires brp >= 1, which sb_mcu_link clamps in hardware.
	// ----------------------------------------------------------------
	wire can_smp_tick, can_smp_bit, can_idle, can_stb;
	wire [28:0] can_id;
	wire [3:0]  can_dlc;
	wire [63:0] can_data;
	wire can_ide, can_rtr, can_crc_ok, can_ack_ok, can_ovl;
	wire [2:0] can_err;

	can_bit_timing #(
		.PROP_SEG(4), .PHASE_SEG1(4), .PHASE_SEG2(3), .SJW(3),
		.PIPELINE_RESYNC(1)
	) u_can_bt (
		.clk(clk), .rst(rst_can), .brp(can_brp), .rx_raw(can_rx),
		.bus_idle(can_idle),
		.sample_tick(can_smp_tick), .sample_bit(can_smp_bit),
		.bit_tick(), .rx_sync(), .hard_sync(), .resync()
	);

	can_frame_fsm u_can_fsm (
		.clk(clk), .rst(rst_can),
		.sample_tick(can_smp_tick), .sample_bit(can_smp_bit),
		.bus_idle(can_idle),
		.frame_strobe(can_stb), .frame_id(can_id), .frame_ide(can_ide),
		.frame_rtr(can_rtr), .frame_dlc(can_dlc), .frame_data(can_data),
		.frame_crc_ok(can_crc_ok), .frame_ack_ok(can_ack_ok),
		.frame_overload(can_ovl), .frame_err(can_err)
	);

	// ----------------------------------------------------------------
	// Event mux.  Everything downstream of here is protocol-agnostic.
	// ----------------------------------------------------------------
	reg        ev_strobe;
	reg [5:0]  ev_flags;
	reg [31:0] ev_aux;
	reg [3:0]  ev_len;
	reg [63:0] ev_payload;

	always @* begin
		case (proto)
			P_CAN: begin
				ev_strobe  = can_stb;
				ev_flags   = {can_ide, can_rtr, can_crc_ok, can_ack_ok, can_ovl, 1'b0};
				ev_aux     = {can_err, can_id};
				ev_len     = can_dlc;
				ev_payload = can_data;
			end
			P_UART: begin
				ev_strobe  = u_stb;   ev_flags = u_flags;
				ev_aux     = u_aux;   ev_len   = u_len;   ev_payload = u_payload;
			end
			P_SPI: begin
				ev_strobe  = spi_stb; ev_flags = spi_flags;
				ev_aux     = spi_aux; ev_len   = spi_len; ev_payload = spi_payload;
			end
			default: begin
				ev_strobe  = i2c_stb; ev_flags = i2c_flags;
				ev_aux     = i2c_aux; ev_len   = i2c_len; ev_payload = i2c_payload;
			end
		endcase
	end

	// ----------------------------------------------------------------
	// Shared capture back end
	// ----------------------------------------------------------------
	sb_capture #(.N_RECORDS(64), .ADDR_W(11)) u_cap (
		.clk(clk), .rst(rst),
		.ev_strobe(ev_strobe), .ev_proto(proto), .ev_flags(ev_flags),
		.ev_aux(ev_aux), .ev_len(ev_len), .ev_payload(ev_payload),
		.timestamp(timestamp),
		.arm_record(arm_record), .start_stream(start_stream), .stop(stop_cmd),
		.trig0(trig0), .trig1(trig1), .trig2(trig2), .trig3(trig3),
		.trig_len(trig_len),
		.drain_advance(drain_advance), .drain_data(drain_data),
		.armed(armed), .recording(recording), .streaming(streaming),
		.done(done), .dropped(dropped), .fill_level(fill_level)
	);

	// IRQ: tells the MCU a triggered capture finished, so it does not have
	// to poll the status register continuously.
	assign mcu_irq = done;

	// ----------------------------------------------------------------
	// LEDs, active low.  Priority-encoded so each state is one colour --
	// the same convention can_bringup.v uses.
	// ----------------------------------------------------------------
	localparam integer STRETCH = 24_000_000 / 20;     // 50 ms
	reg [21:0] ev_stretch = 22'd0;
	always @(posedge clk) begin
		if (ev_strobe)          ev_stretch <= STRETCH[21:0];
		else if (|ev_stretch)   ev_stretch <= ev_stretch - 22'd1;
	end

	wire blink = |ev_stretch;
	assign led_r = ~(dropped);                       // red:   data was lost
	assign led_g = ~(blink & ~dropped);              // green: events arriving
	assign led_b = ~(recording | armed | streaming); // blue:  capture active
endmodule
