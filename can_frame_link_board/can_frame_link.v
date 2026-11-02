/*
 * can_frame_link.v — SignalBench PCBA (U7 iCE40UP5K-SG48) top: every board
 * receiver -> records -> framed SPI link -> ESP32-H2.
 *
 *   CAN    pin 27 (U15 TCAN330)       -> can_bit_timing -> can_frame_fsm -> 20-byte record
 *   UART 0 pin 23 (U13 RS-232 ROUT1)  -> uart_rx_sniff  ->  8-byte record
 *   UART 1 pin 25 (U13 RS-232 ROUT2)  -> uart_rx_sniff  ->  8-byte record
 *   UART 2 pin 26 (U14 RS-485 RO)     -> uart_rx_sniff  ->  8-byte record
 *   UART 3 pin 32 (U19 <- P1.9 PROBE1) -> uart_rx_sniff  ->  8-byte record
 *   PROBE2 pin 35 (U19 <- P1.10)       -> sync + 1 us glitch filter + edge
 *                                         counter -> 12-byte counter record /100 ms
 *        -> one holding slot per source -> oldest-first arbiter (never splits a
 *           record) -> spi_frame_link (atomic variable-length push, 4 x SPRAM
 *           ring, logic SPI slave, A5 5A 01 frames, CRC-16, MCU ack) -> MISO (46)
 *
 * Contract: validation/battery_can_chain/CONTRACT.md §1, §4, §5, §6, §7.1
 * (authoritative).
 *
 * Partner sources: can_bit_timing.v, can_crc15.v, can_frame_fsm.v unchanged from
 * Signal-Bench/fpga-code main 2eca834; the CAN timestamp and record layout are
 * taken from can_sniffer.v (same commit); spi_frame_link.v is derived from
 * spi_frame_stream.v (feat/spi-frame-stream 58a12d6).  See SOURCE_COMMIT.
 * uart_rx_sniff.v and this file are new.
 *
 * Clock: one domain, the 12 MHz board oscillator on pin 44 (G6, Y1 -> U4,
 * +/-20 ppm).  CAN needs df < 0.98% (clock_choice.md); SB_HFOSC is +/-10% and is
 * not used.  CAN: BRP=0 -> 12 tq/bit -> 1 Mbit/s; bit rate = 1 MHz/(BRP+1).
 * UART: 16x NCO per channel from the same clock.  Consequence of 12 MHz for the
 * logic SPI slave: SCK ceiling 1.5 MHz (was 3 MHz at 24 MHz); recommended 1 MHz.
 *
 * CAN record (20 bytes = 10 samples):
 *   [0] 0xAA  [1] {ide, rtr, overload, ack_ok, crc_ok, err[2:0]}
 *   [2..5] identifier, 29 bits right-justified, big-endian
 *   [6] {dropped, 3'b000, dlc[3:0]}  [7..14] data[0..7] left-justified
 *   [15..18] timestamp, CAN bit times since configuration, big-endian, at SOF
 *   [19] 0x55
 * UART record (8 bytes = 4 samples), chan 0..2 = UART0..2, chan 3 = PROBE1:
 *   [0] 0xC5  [1] {chan[1:0], framing_err, brk, dropped, 3'b000}  [2] data
 *   [3..6] timestamp, microseconds since configuration, big-endian, at the
 *          start-bit edge   [7] 0x5A
 * Counter record (12 bytes = 6 samples), PROBE2, every HB_PERIOD_US (100 ms):
 *   [0] 0xE7  [1] {level, 3'b000, dropped, 3'b000}  (level = filtered level now)
 *   [2..5] rise, filtered rising edges since configuration, u32 big-endian
 *   [6..9] timestamp, microseconds since configuration (us_ctr), big-endian
 *   [10] glitch, rejected excursions since the previous record, saturating u8
 *   [11] 0x7E
 * Sample k = record[2k] | record[2k+1] << 8; the link sends samples low byte
 * first, so the record bytes appear on MISO in their original order.
 *
 * `dropped` (all record types): at least one record of ANY type was lost
 * since the previous record that reached the buffer.  The frame header's
 * `dropped` field counts RECORDS of any type (see spi_frame_link.v change 3).
 *
 * Arbitration: each source has one holding slot (6: CAN, UART0..3, COUNTER).
 * When the link is idle, the OLDEST occupied slot is granted (ties in the same
 * clock: CAN, UART0, UART1, UART2, UART3, COUNTER), its record is copied to the
 * output buffer and the slot frees at once.  The link writes it in 10 (CAN),
 * 4 (UART) or 6 (COUNTER) clocks.  Worst case, all six slots full drain in
 * 10 + 4*4 + 6 = 32 clocks.  A record waits in its slot at most for the record
 * being written plus every older slot: 22 clocks (e.g. COUNTER being written,
 * then four UART records), 26 for COUNTER behind CAN + 4 UART.  Refill times:
 * UART one frame (>= 160 clocks at 750 kbaud, 1042 at UART3's 115200), COUNTER
 * 1.2 M clocks, CAN one bit (12 clocks, BRP=0) only for two decoder records one
 * bit apart (error/EOF corner), so a CAN slot can be overrun only if that
 * corner coincides with four UART records and a counter record inside the
 * same ~2 us.  If any slot is ever overrun, the NEW record is dropped and
 * counted (drop_add), never split or corrupted.
 *
 * Pins driven: spi_miso only, and only while spi_cs is low (tri-stated when CS
 * is high, per FPGA_PINOUT_ICE40UP5K-SG48.md).  The CAN/RS-232/RS-485 receiver
 * inputs have the pad pull-up on (idle = recessive / mark during power ramp);
 * the two probe inputs (32, 35) have it OFF, because U19 (SN74LVC8T245, A->B
 * hard-wired) drives them push-pull (CONTRACT §7.1).  Pin 16 (FT_SSn), pin 18
 * (RT_MOSI), probes 3-8 and RGB0-2 are untouched.
 *
 * PROBE2 edge counter (CONTRACT §7.1): 2-flop synchroniser, then a filter
 * that accepts a new level only after the synchronised input has differed
 * from the filtered level for GLITCH_CLKS (12) consecutive clocks = 1 us.
 * rise (u32) counts filtered rising edges and is never reset.  glitch counts
 * rejected excursions: each time the synchronised input leaves the filtered
 * level and returns before it is accepted, +1 (so a 2-transition glitch pulse
 * counts once, and contact bounce before a real edge counts once per bounce).
 * It saturates at 255 and restarts from 0 when a counter record is loaded into
 * its holding slot (an overrun record does not reset it, so nothing is lost
 * there; a record later dropped by a full link buffer loses its glitch count,
 * and the next record says `dropped`).  Timestamp and contents are taken on
 * the same clock, the first clock of each HB_PERIOD_US period of us_ctr, so the
 * k-th record's timestamp is exactly k * HB_PERIOD_US (100000 k).
 */
`default_nettype none

module top #(
	parameter [7:0]   BRP               = 8'd0,      // CAN: tq = (BRP+1) clk; 0 -> 1 Mbit/s
	parameter integer UART0_BAUD        = 9_600,     // RS-232 P1.7 RX_IN  (CONTRACT §6)
	parameter integer UART1_BAUD        = 9_600,     // RS-232 P1.8 TX_IN
	parameter integer UART2_BAUD        = 9_600,     // RS-485 P1.3/P1.4
	parameter integer UART3_BAUD        = 115_200,   // PROBE1 P1.9 <- Pi GPIO4 (CONTRACT §7.1)
	parameter integer HB_PERIOD_US      = 100_000,   // counter record / heartbeat period (§7.1)
	parameter integer GLITCH_CLKS       = 12,        // PROBE2 filter: stable clocks to accept (1 us)
	parameter integer BUF_AW            = 16,        // 64 Ki samples
	parameter integer MAX_FRAME_SAMPLES = 2036,
	parameter [31:0]  INITIAL_OFFSET    = 32'd0      // test hook only
) (
	input  wire clk_12m,       // 44  IOB_3b_G6  <- Y1 12 MHz via U4 2Y
	input  wire can_rx_pin,    // 27  IOT_38b    <- U15 TCAN330 RXD
	input  wire uart0_pin,     // 23  IOT_37a    <- U13 ROUT1 (FPGA_RS232_RX_OUT, P1.7)
	input  wire uart1_pin,     // 25  IOT_36b    <- U13 ROUT2 (FPGA_RS232_TX_OUT, P1.8)
	input  wire uart2_pin,     // 26  IOT_39a    <- U14 RO    (FPGA_RS485_RX, P1.3/4)
	input  wire probe1_pin,    // 32  IOT_43a    <- U19 B (IDC1_BUF <- P1.9 PROBE1)
	input  wire probe2_pin,    // 35  IOT_46b_G0 <- U19 B (IDC2_BUF <- P1.10 PROBE2)
	input  wire spi_sck,       // 20  RT_SCK     <- ESP32 GPIO10 (shared with LCD SCK)
	input  wire spi_cs,        // 47  RT_CSn     <- ESP32 GPIO27, active low
	input  wire spi_mosi,      // 48  FPGA_IRQn  <- ESP32 GPIO22 (defect F1 rework)
	output wire spi_miso       // 46  RT_MISO    -> ESP32 GPIO11
);
	localparam integer CLK_HZ = 12_000_000;
	wire clk = clk_12m;

	// ----------------------------------------------------------------
	// Pads
	// ----------------------------------------------------------------
	wire can_rx, uart0_rx, uart1_rx, uart2_rx, uart3_rx, probe2_raw;
	SB_IO #(.PIN_TYPE(6'b0000_01), .PULLUP(1'b1)) u_can_io   (.PACKAGE_PIN(can_rx_pin), .D_IN_0(can_rx));
	SB_IO #(.PIN_TYPE(6'b0000_01), .PULLUP(1'b1)) u_uart0_io (.PACKAGE_PIN(uart0_pin),  .D_IN_0(uart0_rx));
	SB_IO #(.PIN_TYPE(6'b0000_01), .PULLUP(1'b1)) u_uart1_io (.PACKAGE_PIN(uart1_pin),  .D_IN_0(uart1_rx));
	SB_IO #(.PIN_TYPE(6'b0000_01), .PULLUP(1'b1)) u_uart2_io (.PACKAGE_PIN(uart2_pin),  .D_IN_0(uart2_rx));
	// Probe inputs: pull-up OFF, U19 drives push-pull (CONTRACT §7.1).
	SB_IO #(.PIN_TYPE(6'b0000_01), .PULLUP(1'b0)) u_probe1_io (.PACKAGE_PIN(probe1_pin), .D_IN_0(uart3_rx));
	SB_IO #(.PIN_TYPE(6'b0000_01), .PULLUP(1'b0)) u_probe2_io (.PACKAGE_PIN(probe2_pin), .D_IN_0(probe2_raw));

	// MISO: output enable straight from the CS pad (not the synchroniser), so
	// the first bit (0xA5 MSB) is on the wire as soon as CS falls.
	wire miso_d;
	SB_IO #(.PIN_TYPE(6'b1010_01), .PULLUP(1'b0)) u_miso_io (
		.PACKAGE_PIN(spi_miso), .OUTPUT_ENABLE(~spi_cs), .D_OUT_0(miso_d));

	// ----------------------------------------------------------------
	// Power-on reset (as can_sniffer.v) and the microsecond timebase
	// ----------------------------------------------------------------
	reg [7:0] por = 8'd0;
	wire      rst = ~por[7];
	always @(posedge clk) if (!por[7]) por <= por + 8'd1;

	reg [3:0]  us_div = 4'd0;
	reg [31:0] us_ctr = 32'd0;
	wire       us_tick = (us_div == (CLK_HZ / 1_000_000) - 1);
	always @(posedge clk) begin
		if (rst) begin
			us_div <= 4'd0;
			us_ctr <= 32'd0;
		end else if (us_tick) begin
			us_div <= 4'd0;
			us_ctr <= us_ctr + 32'd1;
		end else
			us_div <= us_div + 4'd1;
	end

	// ----------------------------------------------------------------
	// CAN decoder chain (partner modules, unchanged)
	// ----------------------------------------------------------------
	wire bus_idle, sample_tick, sample_bit, bit_tick, rx_sync, hard_sync, resync;
	wire        frame_strobe, frame_ide, frame_rtr, frame_crc_ok;
	wire        frame_ack_ok, frame_overload;
	wire [28:0] frame_id;
	wire [3:0]  frame_dlc;
	wire [63:0] frame_data;
	wire [2:0]  frame_err;

	can_bit_timing u_bt (
		.clk(clk), .rst(rst), .brp(BRP),
		.rx_raw(can_rx), .bus_idle(bus_idle),
		.sample_tick(sample_tick), .sample_bit(sample_bit),
		.bit_tick(bit_tick), .rx_sync(rx_sync),
		.hard_sync(hard_sync), .resync(resync)
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

	// CAN timestamp in bit times, latched at SOF (as can_sniffer.v).
	reg [31:0] ts_ctr    = 32'd0;
	reg [31:0] ts_latch  = 32'd0;
	reg        idle_prev = 1'b1;
	always @(posedge clk) begin
		if (bit_tick) ts_ctr <= ts_ctr + 32'd1;
		idle_prev <= bus_idle;
		if (idle_prev & ~bus_idle) ts_latch <= ts_ctr;
	end

	// ----------------------------------------------------------------
	// UART receivers
	// ----------------------------------------------------------------
	wire [3:0]  u_stb, u_fe, u_brk, u_busy;
	wire [7:0]  u_data [0:3];
	wire [31:0] u_ts   [0:3];

	uart_rx_sniff #(.CLK_HZ(CLK_HZ), .BAUD(UART0_BAUD)) u_uart0 (
		.clk(clk), .rst(rst), .rx(uart0_rx), .us_now(us_ctr),
		.strobe(u_stb[0]), .data(u_data[0]), .fe(u_fe[0]), .brk(u_brk[0]), .ts(u_ts[0]), .busy(u_busy[0]));
	uart_rx_sniff #(.CLK_HZ(CLK_HZ), .BAUD(UART1_BAUD)) u_uart1 (
		.clk(clk), .rst(rst), .rx(uart1_rx), .us_now(us_ctr),
		.strobe(u_stb[1]), .data(u_data[1]), .fe(u_fe[1]), .brk(u_brk[1]), .ts(u_ts[1]), .busy(u_busy[1]));
	uart_rx_sniff #(.CLK_HZ(CLK_HZ), .BAUD(UART2_BAUD)) u_uart2 (
		.clk(clk), .rst(rst), .rx(uart2_rx), .us_now(us_ctr),
		.strobe(u_stb[2]), .data(u_data[2]), .fe(u_fe[2]), .brk(u_brk[2]), .ts(u_ts[2]), .busy(u_busy[2]));
	uart_rx_sniff #(.CLK_HZ(CLK_HZ), .BAUD(UART3_BAUD)) u_uart3 (
		.clk(clk), .rst(rst), .rx(uart3_rx), .us_now(us_ctr),
		.strobe(u_stb[3]), .data(u_data[3]), .fe(u_fe[3]), .brk(u_brk[3]), .ts(u_ts[3]), .busy(u_busy[3]));

	// ----------------------------------------------------------------
	// PROBE2 edge counter (CONTRACT §7.1)
	// ----------------------------------------------------------------
	reg  [1:0]  p2_q     = 2'b00;            // synchroniser (line rests low)
	wire        p2_s     = p2_q[1];
	reg         p2_lvl   = 1'b0;             // filtered level
	reg  [3:0]  p2_stab  = 4'd0;             // clocks p2_s has differed from p2_lvl
	reg  [31:0] p2_rise  = 32'd0;
	reg  [7:0]  p2_glitch = 8'd0;
	reg  [16:0] hb_us    = 17'd0;            // microseconds into the heartbeat period
	reg         hb_stb   = 1'b0;             // first clock of each period: us_ctr = k * HB_PERIOD_US
	wire        p2_accept = (p2_s != p2_lvl) && (p2_stab == GLITCH_CLKS - 1);
	wire        p2_reject = (p2_s == p2_lvl) && (p2_stab != 4'd0);   // excursion ended early
	wire        cnt_load;                    // the counter slot loads (defined with the slots)

	// synthesis translate_off
	initial if (GLITCH_CLKS < 2 || GLITCH_CLKS > 15 || HB_PERIOD_US < 2 || HB_PERIOD_US > 131071) begin
		$display("top: GLITCH_CLKS=%0d / HB_PERIOD_US=%0d out of range", GLITCH_CLKS, HB_PERIOD_US);
		$finish;
	end
	// synthesis translate_on

	always @(posedge clk) begin
		p2_q <= {p2_q[0], probe2_raw};
		if (rst) begin
			p2_lvl  <= 1'b0;
			p2_stab <= 4'd0;
			hb_us   <= 17'd0;
		end else begin
			if (p2_s == p2_lvl)
				p2_stab <= 4'd0;
			else if (p2_accept) begin
				p2_stab <= 4'd0;
				p2_lvl  <= p2_s;
			end else
				p2_stab <= p2_stab + 4'd1;
			if (us_tick) hb_us <= (hb_us == HB_PERIOD_US - 1) ? 17'd0 : hb_us + 17'd1;
		end
		hb_stb <= !rst && us_tick && (hb_us == HB_PERIOD_US - 1);
		if (!rst && p2_accept && p2_s) p2_rise <= p2_rise + 32'd1;
		// glitch: counted from the clock after a load; a reject in the load
		// clock itself belongs to the next record.
		if (cnt_load)
			p2_glitch <= {7'd0, !rst && p2_reject};
		else if (!rst && p2_reject && p2_glitch != 8'hFF)
			p2_glitch <= p2_glitch + 8'd1;
	end

	// ----------------------------------------------------------------
	// Records as sample vectors (sample k in bits [16k+15:16k]).  The
	// `dropped` bit is left 0 here and inserted at grant time.
	// ----------------------------------------------------------------
	function [159:0] can_vec(input [28:0] id, input [63:0] d, input [31:0] ts,
	                         input [3:0] dlc, input [2:0] err,
	                         input ide, input rtr, input ovl, input ack, input crc);
		begin
			can_vec = {
				8'h55,     ts[7:0],          // s9
				ts[15:8],  ts[23:16],        // s8
				ts[31:24], d[7:0],           // s7
				d[15:8],   d[23:16],         // s6
				d[31:24],  d[39:32],         // s5
				d[47:40],  d[55:48],         // s4
				d[63:56],  {4'b0000, dlc},   // s3  (byte 6 bit 7 = dropped)
				id[7:0],   id[15:8],         // s2
				id[23:16], {3'b000, id[28:24]},  // s1
				{ide, rtr, ovl, ack, crc, err}, 8'hAA   // s0
			};
		end
	endfunction

	function [63:0] uart_vec(input [1:0] chan, input fe, input brk,
	                         input [7:0] d, input [31:0] ts);
		begin
			uart_vec = {
				8'h5A,     ts[7:0],          // s3
				ts[15:8],  ts[23:16],        // s2
				ts[31:24], d,                // s1
				{chan, fe, brk, 1'b0, 3'b000}, 8'hC5   // s0 (byte 1 bit 3 = dropped)
			};
		end
	endfunction

	function [95:0] cnt_vec(input lvl, input [31:0] rise, input [31:0] ts, input [7:0] glitch);
		begin
			cnt_vec = {
				8'h7E,     glitch,           // s5
				ts[7:0],   ts[15:8],         // s4
				ts[23:16], ts[31:24],        // s3
				rise[7:0], rise[15:8],       // s2
				rise[23:16], rise[31:24],    // s1
				{lvl, 3'b000, 1'b0, 3'b000}, 8'hE7   // s0 (byte 1 bit 3 = dropped)
			};
		end
	endfunction

	localparam integer CAN_DROP_BIT  = 3 * 16 + 7;   // byte 6 bit 7
	localparam integer UART_DROP_BIT = 8 + 3;        // byte 1 bit 3 (UART and COUNTER)

	// ----------------------------------------------------------------
	// Holding slots: 0 = CAN, 1..4 = UART 0..3, 5 = COUNTER
	// ----------------------------------------------------------------
	localparam integer NS = 6;
	wire [NS-1:0] arrive = {hb_stb, u_stb[3], u_stb[2], u_stb[1], u_stb[0], frame_strobe} & {NS{~rst}};

	reg  [NS-1:0] valid = {NS{1'b0}};
	reg  [159:0]  slot_can = 160'd0;
	reg  [63:0]   slot_u [1:4];
	reg  [95:0]   slot_cnt = 96'd0;
	reg  [NS-1:0] ahead [0:NS-1];   // ahead[i][j]: slot j's record is older than slot i's
	initial begin : init_slots
		integer n;
		for (n = 1; n <= 4; n = n + 1) slot_u[n] = 64'd0;
		for (n = 0; n < NS; n = n + 1) ahead[n] = {NS{1'b0}};
	end

	// Grant: the oldest occupied slot, when the link can take a record.
	wire          rec_busy, rec_accept, rec_dropped;
	wire [3:0]    rec_idx;
	reg  [NS-1:0] eligible;
	reg  [NS-1:0] grant;
	always @(*) begin : arb
		integer e;
		for (e = 0; e < NS; e = e + 1)
			eligible[e] = valid[e] && ((valid & ahead[e]) == {NS{1'b0}});
		grant = {NS{1'b0}};
		if (!rec_busy && !rst) begin
			if      (eligible[0]) grant = 6'b000001;
			else if (eligible[1]) grant = 6'b000010;
			else if (eligible[2]) grant = 6'b000100;
			else if (eligible[3]) grant = 6'b001000;
			else if (eligible[4]) grant = 6'b010000;
			else if (eligible[5]) grant = 6'b100000;
		end
	end

	wire [NS-1:0] load    = arrive & (~valid | grant);   // slot free (or freeing now)
	wire [NS-1:0] overrun = arrive & valid & ~grant;     // slot still full: drop the new one
	wire [2:0]    drop_add = {2'b00, overrun[0]} + {2'b00, overrun[1]} + {2'b00, overrun[2]} +
	                         {2'b00, overrun[3]} + {2'b00, overrun[4]} + {2'b00, overrun[5]};
	assign cnt_load = load[5];

	always @(posedge clk) begin : slots
		integer ii, jj;
		valid <= (valid & ~grant) | load;

		if (load[0])
			slot_can <= can_vec(frame_id, frame_data, ts_latch, frame_dlc, frame_err,
			                    frame_ide, frame_rtr, frame_overload, frame_ack_ok, frame_crc_ok);
		if (load[1]) slot_u[1] <= uart_vec(2'd0, u_fe[0], u_brk[0], u_data[0], u_ts[0]);
		if (load[2]) slot_u[2] <= uart_vec(2'd1, u_fe[1], u_brk[1], u_data[1], u_ts[1]);
		if (load[3]) slot_u[3] <= uart_vec(2'd2, u_fe[2], u_brk[2], u_data[2], u_ts[2]);
		if (load[4]) slot_u[4] <= uart_vec(2'd3, u_fe[3], u_brk[3], u_data[3], u_ts[3]);
		if (load[5]) slot_cnt  <= cnt_vec(p2_lvl, p2_rise, us_ctr, p2_glitch);

		// Age matrix.  A slot loading now is behind every slot that stays
		// occupied, and behind higher-priority slots loading in the same clock.
		for (ii = 0; ii < NS; ii = ii + 1)
			for (jj = 0; jj < NS; jj = jj + 1)
				if (ii == jj)
					ahead[ii][jj] <= 1'b0;
				else if (load[ii])
					ahead[ii][jj] <= (valid[jj] & ~grant[jj]) | (load[jj] & (jj < ii));
				else if (load[jj])
					ahead[ii][jj] <= 1'b0;
	end

	// ----------------------------------------------------------------
	// Output buffer: the granted record, with the dropped flag inserted.
	// ----------------------------------------------------------------
	reg  [159:0] out_vec = 160'd0;
	reg          drop_stky = 1'b0;    // a record was lost since the last one buffered

	wire        rec_req = (grant != {NS{1'b0}});
	wire [3:0]  rec_len = grant[0] ? 4'd10 : grant[5] ? 4'd6 : 4'd4;

	always @(posedge clk) begin
		if (grant[0]) begin
			out_vec <= slot_can;
			out_vec[CAN_DROP_BIT] <= drop_stky;
		end else if (grant[5]) begin
			out_vec <= {64'd0, slot_cnt};
			out_vec[UART_DROP_BIT] <= drop_stky;
		end else if (rec_req) begin
			out_vec <= {96'd0, grant[1] ? slot_u[1] : grant[2] ? slot_u[2] :
			                   grant[3] ? slot_u[3] : slot_u[4]};
			out_vec[UART_DROP_BIT] <= drop_stky;
		end
		// Set wins: a drop in the same clock as an accept belongs after it.
		if (rec_dropped || overrun != {NS{1'b0}})
			drop_stky <= 1'b1;
		else if (rec_accept)
			drop_stky <= 1'b0;
	end

	wire [15:0] rec_sample = out_vec[{rec_idx, 4'b0000} +: 16];

	// ----------------------------------------------------------------
	// Framed link
	// ----------------------------------------------------------------
	wire ctrl_good, frame_sent;
	spi_frame_link #(
		.BUF_AW(BUF_AW),
		.MAX_FRAME_SAMPLES(MAX_FRAME_SAMPLES),
		.MAX_REC_SAMPLES(10),
		.INITIAL_OFFSET(INITIAL_OFFSET)
	) u_link (
		.clk(clk),
		.spi_sck(spi_sck), .spi_cs(spi_cs), .spi_mosi(spi_mosi), .spi_miso(miso_d),
		.rec_req(rec_req), .rec_len(rec_len), .rec_idx(rec_idx), .rec_sample(rec_sample),
		.rec_busy(rec_busy), .rec_accept(rec_accept), .rec_dropped(rec_dropped),
		.drop_add(drop_add),
		.ctrl_good(ctrl_good), .frame_sent(frame_sent)
	);

endmodule
