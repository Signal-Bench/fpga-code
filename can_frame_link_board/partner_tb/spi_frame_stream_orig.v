/*
 * spi_frame_stream.v — framed, acknowledged sample stream to the ESP32-H2
 *
 * A fake sensor writes 16-bit samples into a large SPRAM ring buffer.  The
 * MCU (SPI master) drains it one frame per CS assertion.  Nothing is removed
 * from the buffer until the MCU confirms it received that data intact, so a
 * corrupted, aborted or missed read costs a retransmission, never data.  The
 * only way a sample is lost is the buffer filling up, and every such drop is
 * counted and reported in the frame header.
 *
 * Wire protocol (full spec in README.md), SPI mode 0, MSB first:
 *
 *   MOSI, every transfer:  5C | ack u32 | read_len u16 | flags | crc16
 *     ack      = offset of the next sample the MCU needs; everything before
 *                it has been received intact and may be freed
 *     read_len = length of this SPI transfer, so the frame always fits
 *     flags[0] = ACK_VALID (clear while the MCU has not synced yet)
 *
 *   MISO, every transfer:  A5 5A 01 00x8 | start u32 | count u16 |
 *                          dropped u32 | status | samples... | crc16 | FF...
 *     Bytes 0-10 are fixed: they go out while the control block is still
 *     arriving.  The header proper starts at byte 11, after the control
 *     block has been checked and its ack applied, so every frame starts at
 *     the oldest unacknowledged sample.
 *
 * SPI slave: implemented in fabric, NOT the SB_SPI hard IP.  The hard IP
 * holds one byte ahead of the shifter and its behaviour across CS boundaries
 * is unverified on this silicon (see spi_hw_stream's +drop_preloaded_on_cs
 * test).  Here the FPGA starts every frame from byte 0 at CS fall and knows
 * exactly which bit is on the wire.
 *
 * SCK, CS and MOSI are oversampled at the 24 MHz core clock through 2-flop
 * synchronizers.  MISO changes 3 core cycles (<= 125 ns) after the SCK edge
 * that requests it, which leaves >= 125 ns of setup at 2 MHz and ~42 ns at
 * 3 MHz.  Treat 3 MHz as the ceiling.  Because MISO changes late, data stays
 * valid for >= 83 ns past each falling edge, so it also tolerates a master
 * that samples half a cycle late (the ESP32-H2 sample-point question).
 *
 * Timing structure: nothing on the SPI path is computed in the cycle it is
 * needed.  The frame decision runs as a short pipeline (DEC_*) with one
 * carry chain per stage, and the next byte plus its CRC update are
 * precomputed in a three-stage prep pipeline (P1-P3) that settles in three
 * cycles — a byte time is >= 64 core cycles at the 3 MHz ceiling.  A load
 * that would use an unsettled value poisons the frame's CRC instead, so a
 * too-fast master gets a retransmission, never wrong data.
 *
 * Buffer: 4 x SB_SPRAM256KA = 64 Ki samples = 128 KB at BUF_AW = 16.
 * SPRAM is single-port; the sample writer has priority and needs one cycle
 * per SAMPLE_RATE_HZ tick, so the reader is never meaningfully delayed.
 *
 * Debug LEDs (active low on the UPduino):
 *   RED:   pulses when a sample is dropped (buffer full: MCU not keeping up)
 *   GREEN: on while the last control block from the MCU was valid
 *   BLUE:  pulses when a frame carrying samples is sent
 */

module top #(
	parameter integer SAMPLE_RATE_HZ    = 62_500,   // x 16 bits = 1 Mbps of payload
	parameter integer BUF_AW            = 16,     // 2^16 samples, all four SPRAMs
	parameter integer MAX_FRAME_SAMPLES = 2036,   // (4096 - 24) / 2
	// Test hook: where the stream offsets start.  Leave at 0 in hardware.
	// The testbench starts just below 2^32 to push the stream through the
	// 32-bit offset wrap and across the SPRAM bank boundaries.
	parameter [31:0]  INITIAL_OFFSET    = 32'd0
) (
	input  wire spi_sck,       // gpio_11 — SCK in from master
	input  wire spi_cs,        // gpio_19 — CS in from master (active low)
	input  wire spi_mosi,      // gpio_21 — control block in from master
	output wire spi_miso,      // gpio_13 — frames out to master
	output wire spi_cs_flash,  // pin 16  — hold the onboard flash deselected
	output wire led_r,
	output wire led_g,
	output wire led_b
);
	assign spi_cs_flash = 1'b1;

	// ----------------------------------------------------------------
	// 48 MHz oscillator -> 24 MHz core clock.  Same divide-by-2 as the
	// other designs.  Oscillator accuracy does not matter here: SPI carries
	// its own clock, and the sample rate is a test parameter.
	// ----------------------------------------------------------------
	wire clk_48;
	reg  clk_core_r = 0;
	wire clk        = clk_core_r;

	SB_HFOSC u_hfosc (.CLKHFPU(1'b1), .CLKHFEN(1'b1), .CLKHF(clk_48));
	always @(posedge clk_48) clk_core_r <= ~clk_core_r;

	localparam integer CORE_HZ    = 24_000_000;
	localparam integer TICK_DIV   = CORE_HZ / SAMPLE_RATE_HZ;   // must be >= 4, see writer
	localparam integer TICK_W     = (TICK_DIV <= 2) ? 1 : $clog2(TICK_DIV);
	localparam integer LED_CYCLES = CORE_HZ / 10;             // ~100 ms

	localparam [31:0] CAPACITY = 32'd1 << BUF_AW;

	// Protocol constants — keep in sync with README.md and the MCU parser.
	localparam [7:0]  VERSION    = 8'h01;
	localparam [7:0]  CTRL_MAGIC = 8'h5C;
	localparam [15:0] HDR_LEN    = 16'd22;   // bytes before the first sample
	localparam [15:0] OVERHEAD   = 16'd24;   // header + CRC

	localparam [7:0] ST_ACK_APPLIED  = 8'h01;
	localparam [7:0] ST_CTRL_BAD     = 8'h02;
	localparam [7:0] ST_ACK_REJECTED = 8'h04;
	localparam [7:0] ST_DROPPED      = 8'h08;   // samples dropped since the previous frame

	// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, MSB first, no final XOR.
	// Check value: "123456789" -> 0x29B1.
	function [15:0] crc16_byte(input [15:0] crc, input [7:0] data);
		integer i;
		reg [15:0] c;
		begin
			c = crc ^ {data, 8'h00};
			for (i = 0; i < 8; i = i + 1)
				c = c[15] ? ({c[14:0], 1'b0} ^ 16'h1021) : {c[14:0], 1'b0};
			crc16_byte = c;
		end
	endfunction

	// ----------------------------------------------------------------
	// Fake sensor: a 16-bit counter sampled at SAMPLE_RATE_HZ.  It advances
	// every tick whether or not the buffer has room, like a real sensor, so
	// a dropped sample shows up as a jump in value as well as in `dropped`.
	// ----------------------------------------------------------------
	reg [TICK_W-1:0] tick_ctr = 0;
	wire tick = (tick_ctr == TICK_DIV - 1);

	always @(posedge clk)
		tick_ctr <= tick ? {TICK_W{1'b0}} : tick_ctr + 1'b1;

	// Stream offsets count samples since configuration and wrap at 2^32.
	// The buffer address is the low BUF_AW bits.  Every comparison below is
	// done on differences, so the wrap is harmless.
	reg  [31:0] wr_off  = INITIAL_OFFSET;   // next offset to write
	reg  [31:0] rd_off  = INITIAL_OFFSET;   // oldest offset not yet acknowledged
	reg  [31:0] sent_hw = INITIAL_OFFSET;   // highest offset ever sent; caps a valid ack
	reg  [31:0] dropped = 0;                // samples dropped because the buffer was full
	reg  [15:0] sensor  = INITIAL_OFFSET[15:0];

	// Unacknowledged samples, for the testbench's memory-safety monitors.
	// The logic below uses registered copies to keep carry chains short.
	wire [31:0] level = wr_off - rd_off;

	// ----------------------------------------------------------------
	// Sample writer.  The full flag is registered two cycles behind wr_off
	// and rd_off (one carry chain per cycle).  Stale in the safe direction
	// only: an ack can only free space, so a stale flag at worst drops — and
	// counts — a sample that would have fit; and a write retires one cycle
	// after its tick, so by the next tick (TICK_DIV >= 4) the flag already
	// includes it.  It can never let the writer overwrite unacked data.
	// ----------------------------------------------------------------
	reg        wr_pending = 1'b0;
	reg [15:0] wr_data    = 0;
	reg [31:0] level_w    = 0;
	reg        full_w     = 1'b0;
	wire       sample_dropped = tick && full_w;

	always @(posedge clk) begin
		level_w <= wr_off - rd_off;
		full_w  <= (level_w >= CAPACITY);

		// The pending write is performed by the RAM on this edge (ram_we
		// below), so retire it first; a tick on the same edge re-arms it.
		if (wr_pending) begin
			wr_pending <= 1'b0;
			wr_off     <= wr_off + 1'b1;
		end

		if (tick) begin
			sensor <= sensor + 1'b1;
			if (sample_dropped)
				dropped <= dropped + 1'b1;
			else begin
				wr_pending <= 1'b1;
				wr_data    <= sensor;
			end
		end
	end

	// ----------------------------------------------------------------
	// Sample RAM.  Single port: the writer wins, the frame reader waits a
	// cycle.  Read data is registered (one cycle of latency) and a write
	// cycle turns DATAOUT to X, so the reader captures it on the very next
	// edge — see the fetch logic.
	// ----------------------------------------------------------------
	reg         fetch_req      = 1'b0;
	reg         fetch_inflight = 1'b0;
	reg  [31:0] fetch_off      = 0;
	reg  [15:0] fetch_left     = 0;

	wire              ram_we   = wr_pending;
	wire              ram_re   = !wr_pending && fetch_req;
	wire [BUF_AW-1:0] ram_addr = wr_pending ? wr_off[BUF_AW-1:0] : fetch_off[BUF_AW-1:0];
	wire [15:0]       ram_rdata;

	sample_ram #(.AW(BUF_AW)) u_ram (
		.clk(clk), .addr(ram_addr), .wdata(wr_data),
		.we(ram_we), .re(ram_re), .rdata(ram_rdata)
	);

	// ----------------------------------------------------------------
	// SPI pin synchronizers.  sck_q[1] and mosi_q[1] are both two flops
	// from the pin, so MOSI is sampled at the same instant the rising edge
	// is seen.
	// ----------------------------------------------------------------
	reg [2:0] sck_q  = 3'b000;
	reg [2:0] cs_q   = 3'b111;
	reg [1:0] mosi_q = 2'b00;

	always @(posedge clk) begin
		sck_q  <= {sck_q[1:0], spi_sck};
		cs_q   <= {cs_q[1:0], spi_cs};
		mosi_q <= {mosi_q[0], spi_mosi};
	end

	wire sck_rise =  sck_q[1] & ~sck_q[2];
	wire sck_fall = ~sck_q[1] &  sck_q[2];
	wire cs_low   = ~cs_q[1];
	wire cs_begin = ~cs_q[1] &  cs_q[2];
	wire mosi_bit =  mosi_q[1];

	// ----------------------------------------------------------------
	// Per-transfer state
	// ----------------------------------------------------------------
	// Transmit.  tx_shift idles at 0xA5 so its MSB is already on MISO when
	// CS falls, however short the master's CS-to-SCK setup is.
	reg [7:0]  tx_shift = 8'hA5;
	reg [2:0]  tx_bit   = 0;
	reg [15:0] next_pos = 16'd1;      // frame index of the NEXT byte to load
	reg [15:0] crc_tx   = 16'hFFFF;
	reg        poison   = 1'b0;       // a byte went out wrong: corrupt the CRC

	assign spi_miso = tx_shift[7];

	wire load = cs_low && sck_fall && (tx_bit == 3'd7);

	// Receive: the control block.  A finished byte is registered first and
	// processed on the next cycle, so the CRC update never sits behind the
	// MOSI synchronizer in the same cycle.
	reg [7:0]  rx_shift  = 0;
	reg [2:0]  rx_bit    = 0;
	reg [3:0]  rx_pos    = 0;         // saturates at 15
	reg [7:0]  rx_byte_q = 0;
	reg [3:0]  rx_pos_q  = 0;
	reg        rx_done   = 1'b0;
	reg [15:0] crc_rx    = 16'hFFFF;
	reg        magic_ok  = 1'b0;
	reg [31:0] c_ack     = 0;
	reg [15:0] c_len     = 0;
	reg [7:0]  c_flags   = 0;
	reg [15:0] c_crc     = 0;
	wire [7:0] rx_byte   = {rx_shift[6:0], mosi_bit};

	// Frame decision pipeline.
	localparam [2:0] DEC_IDLE  = 3'd0,
	                 DEC_CHECK = 3'd1,   // CRC compare; ack - rd and hw - rd
	                 DEC_APPLY = 3'd2,   // range check, free acked samples
	                 DEC_LEVEL = 3'd3,   // buffered samples; read_len budget
	                 DEC_CAP   = 3'd4,   // clamp budget to MAX_FRAME_SAMPLES
	                 DEC_COUNT = 3'd5,   // count = min(level, budget)
	                 DEC_BUILD = 3'd6;   // latch header, start the fetch
	reg [2:0]  dec       = DEC_IDLE;
	reg        hdr_ready = 1'b0;
	reg        ctrl_ok   = 1'b0;
	reg [31:0] ack_diff  = 0;
	reg [31:0] hw_diff   = 0;
	reg [31:0] level_d   = 0;
	reg [15:0] by_len    = 0;
	reg [15:0] cap_len   = 0;
	reg [15:0] count_q   = 0;
	reg [31:0] f_start   = 0;
	reg [15:0] f_count   = 0;
	reg [31:0] f_dropped = 0;
	reg [7:0]  f_status  = 0;
	reg [15:0] crc_pos   = HDR_LEN;   // first CRC byte: HDR_LEN + 2 * f_count
	reg [15:0] crc_pos1  = HDR_LEN + 16'd1;
	reg        ctrl_good = 1'b0;      // for the green LED
	reg [31:0] dropped_reported = 0;  // `dropped` as of the previous frame
	reg        frame_sent = 1'b0;     // one-cycle pulse for the blue LED

	// Payload prefetch: pf_word is the next sample to send, cur_hi the high
	// byte of the sample whose low byte is on the wire.
	reg [15:0] pf_word  = 0;
	reg        pf_valid = 1'b0;
	reg [7:0]  cur_hi   = 0;

	// ----------------------------------------------------------------
	// Prep pipeline: the byte for next_pos and the CRC after it.
	//   P1: classify next_pos (one compare chain per flag)
	//   P2: select the byte
	//   P3: CRC update over that byte
	// prep_age counts cycles since an input of the PENDING byte changed; a
	// load with prep_age < 3 would use stale values, so it poisons the
	// frame instead.  It is reset only by changes the pending byte depends
	// on: a load (new position, CRC, cur_hi), a prefetch landing while a
	// payload byte is pending (pf_word), and the header being latched while
	// a header-or-later byte is pending.  Bytes 1-10 are constants, so the
	// header and the first prefetch — both of which land around byte 10 at
	// some SCK rates — must NOT reset the age while one of those is
	// pending, or a good frame gets poisoned.
	// ----------------------------------------------------------------
	reg       p1_fixed = 1'b0, p1_hdr = 1'b0, p1_pay = 1'b0, p1_crc_hi = 1'b0, p1_crc_lo = 1'b0;
	reg       p1_incl  = 1'b0;   // byte is covered by the CRC
	reg       p1_odd   = 1'b0;
	reg [4:0] p1_sel   = 0;
	reg [7:0] p2_byte  = 8'hFF;
	reg       p2_poison = 1'b0, p2_lo = 1'b0, p2_incl = 1'b0;
	reg [15:0] p3_crc  = 16'hFFFF;
	reg       p3_incl  = 1'b0;
	reg [1:0] prep_age = 2'd0;

	always @(posedge clk) begin
		// P1
		p1_fixed  <= (next_pos <= 16'd10);
		p1_hdr    <= (next_pos >= 16'd11) && (next_pos < HDR_LEN);
		p1_pay    <= (next_pos >= HDR_LEN) && (next_pos < crc_pos);
		p1_crc_hi <= (next_pos == crc_pos);
		p1_crc_lo <= (next_pos == crc_pos1);
		p1_incl   <= (next_pos < crc_pos);
		p1_odd    <= next_pos[0];
		p1_sel    <= next_pos[4:0];

		// P2
		p2_byte   <= 8'hFF;
		p2_poison <= 1'b0;
		p2_lo     <= 1'b0;
		p2_incl   <= p1_incl;
		if (p1_fixed) begin
			case (p1_sel)
				5'd1:    p2_byte <= 8'h5A;
				5'd2:    p2_byte <= VERSION;
				default: p2_byte <= 8'h00;
			endcase
		end else if (!hdr_ready) begin
			p2_poison <= 1'b1;            // control block missing or SCK too fast
		end else if (p1_hdr) begin
			case (p1_sel)
				5'd11:   p2_byte <= f_start[7:0];
				5'd12:   p2_byte <= f_start[15:8];
				5'd13:   p2_byte <= f_start[23:16];
				5'd14:   p2_byte <= f_start[31:24];
				5'd15:   p2_byte <= f_count[7:0];
				5'd16:   p2_byte <= f_count[15:8];
				5'd17:   p2_byte <= f_dropped[7:0];
				5'd18:   p2_byte <= f_dropped[15:8];
				5'd19:   p2_byte <= f_dropped[23:16];
				5'd20:   p2_byte <= f_dropped[31:24];
				default: p2_byte <= f_status;     // 21
			endcase
		end else if (p1_pay) begin
			if (!p1_odd) begin                    // HDR_LEN is even: even = low byte
				p2_byte <= pf_word[7:0];
				p2_lo   <= 1'b1;
				if (!pf_valid)
					p2_poison <= 1'b1;            // prefetch missed its deadline
			end else
				p2_byte <= cur_hi;
		end else if (p1_crc_hi)
			p2_byte <= crc_tx[15:8] ^ {8{poison}};
		else if (p1_crc_lo)
			p2_byte <= crc_tx[7:0] ^ {8{poison}};

		// P3
		p3_crc  <= crc16_byte(crc_tx, p2_byte);
		p3_incl <= p2_incl;
	end

	// ----------------------------------------------------------------
	// SPI engine, frame decision and payload fetch
	// ----------------------------------------------------------------
	always @(posedge clk) begin
		frame_sent <= 1'b0;
		if (prep_age != 2'd3)
			prep_age <= prep_age + 1'b1;

		// -- RAM read: the read was issued on the previous edge --
		if (fetch_inflight) begin
			pf_word        <= ram_rdata;
			pf_valid       <= 1'b1;
			fetch_inflight <= 1'b0;
			if (next_pos >= HDR_LEN)    // only a payload byte reads pf_word
				prep_age <= 2'd0;
		end
		if (ram_re) begin
			fetch_inflight <= 1'b1;
			fetch_req      <= 1'b0;
			fetch_off      <= fetch_off + 1'b1;
			fetch_left     <= fetch_left - 1'b1;
		end

		// -- control block bytes, one cycle after they arrive --
		rx_done <= 1'b0;
		if (rx_done) begin
			if (rx_pos_q <= 4'd7)
				crc_rx <= crc16_byte(crc_rx, rx_byte_q);
			case (rx_pos_q)
				4'd0: magic_ok     <= (rx_byte_q == CTRL_MAGIC);
				4'd1: c_ack[7:0]   <= rx_byte_q;
				4'd2: c_ack[15:8]  <= rx_byte_q;
				4'd3: c_ack[23:16] <= rx_byte_q;
				4'd4: c_ack[31:24] <= rx_byte_q;
				4'd5: c_len[7:0]   <= rx_byte_q;
				4'd6: c_len[15:8]  <= rx_byte_q;
				4'd7: c_flags      <= rx_byte_q;
				4'd8: c_crc[15:8]  <= rx_byte_q;
				4'd9: begin
					c_crc[7:0] <= rx_byte_q;
					dec        <= DEC_CHECK;
				end
				default: ;
			endcase
		end

		// -- decision: once per transfer, after control byte 9 --
		case (dec)
			DEC_CHECK: begin
				ctrl_ok  <= magic_ok && (c_crc == crc_rx);
				ack_diff <= c_ack - rd_off;
				hw_diff  <= sent_hw - rd_off;
				dec      <= DEC_APPLY;
			end

			DEC_APPLY: begin
				f_status <= 8'h00;
				if (ctrl_ok) begin
					ctrl_good <= 1'b1;
					if (c_flags[0]) begin
						// Accept an ack anywhere between what is already
						// freed and what has actually been sent.
						if (ack_diff <= hw_diff) begin
							rd_off   <= c_ack;
							f_status <= ST_ACK_APPLIED;
						end else
							f_status <= ST_ACK_REJECTED;
					end
				end else begin
					ctrl_good <= 1'b0;
					f_status  <= ST_CTRL_BAD;
				end
				dec <= DEC_LEVEL;
			end

			DEC_LEVEL: begin
				// A corrupt control block means read_len cannot be trusted:
				// send an empty frame, which fits any read.  It still reports
				// CTRL_BAD and where the stream stands.
				level_d <= wr_off - rd_off;
				by_len  <= (ctrl_ok && c_len >= OVERHEAD) ? ((c_len - OVERHEAD) >> 1) : 16'd0;
				hw_diff <= sent_hw - rd_off;
				dec     <= DEC_CAP;
			end

			DEC_CAP: begin
				cap_len <= (by_len < MAX_FRAME_SAMPLES) ? by_len : MAX_FRAME_SAMPLES[15:0];
				dec     <= DEC_COUNT;
			end

			DEC_COUNT: begin
				// level_d may be a few samples stale: it only under-counts,
				// and those samples go in the next frame.
				count_q <= (level_d < {16'd0, cap_len}) ? level_d[15:0] : cap_len;
				dec     <= DEC_BUILD;
			end

			DEC_BUILD: begin
				f_start   <= rd_off;
				f_count   <= count_q;
				f_dropped <= dropped;
				dropped_reported <= dropped;
				if (dropped != dropped_reported)
					f_status <= f_status | ST_DROPPED;
				crc_pos   <= HDR_LEN + {count_q[14:0], 1'b0};
				crc_pos1  <= HDR_LEN + {count_q[14:0], 1'b1};
				if ({16'd0, count_q} > hw_diff)
					sent_hw <= rd_off + count_q;
				fetch_off  <= rd_off;
				fetch_left <= count_q;
				fetch_req  <= (count_q != 16'd0);
				pf_valid   <= 1'b0;
				hdr_ready  <= 1'b1;
				if (next_pos >= 16'd11)
					prep_age <= 2'd0;
				frame_sent <= (count_q != 16'd0);
				dec        <= DEC_IDLE;
			end

			default: ;
		endcase

		// -- the SPI transfer itself --
		if (cs_begin) begin
			tx_shift   <= 8'hA5;
			tx_bit     <= 3'd0;
			next_pos   <= 16'd1;
			crc_tx     <= crc16_byte(16'hFFFF, 8'hA5);
			poison     <= 1'b0;
			rx_bit     <= 3'd0;
			rx_pos     <= 4'd0;
			crc_rx     <= 16'hFFFF;
			magic_ok   <= 1'b0;
			dec        <= DEC_IDLE;
			hdr_ready  <= 1'b0;
			crc_pos    <= HDR_LEN;
			crc_pos1   <= HDR_LEN + 16'd1;
			pf_valid   <= 1'b0;
			fetch_req  <= 1'b0;
			fetch_left <= 16'd0;
			prep_age   <= 2'd0;
		end else if (cs_low) begin
			if (sck_rise) begin
				rx_shift <= rx_byte;
				rx_bit   <= rx_bit + 1'b1;
				if (rx_bit == 3'd7) begin
					rx_byte_q <= rx_byte;
					rx_pos_q  <= rx_pos;
					rx_done   <= 1'b1;
					if (rx_pos != 4'd15)
						rx_pos <= rx_pos + 1'b1;
				end
			end

			if (sck_fall) begin
				if (load) begin
					tx_bit   <= 3'd0;
					tx_shift <= p2_byte;
					if (next_pos != 16'hFFFF)
						next_pos <= next_pos + 1'b1;
					if (p3_incl)
						crc_tx <= p3_crc;
					if (p2_poison || prep_age != 2'd3)
						poison <= 1'b1;
					if (p2_lo) begin
						cur_hi   <= pf_word[15:8];
						pf_valid <= 1'b0;
						if (fetch_left != 16'd0)
							fetch_req <= 1'b1;
					end
					prep_age <= 2'd0;
				end else begin
					tx_bit   <= tx_bit + 1'b1;
					tx_shift <= {tx_shift[6:0], 1'b1};
				end
			end
		end else begin
			// CS high: park on the first frame byte, stop fetching.
			tx_shift  <= 8'hA5;
			fetch_req <= 1'b0;
		end
	end

	// ----------------------------------------------------------------
	// LEDs
	// ----------------------------------------------------------------
	reg [21:0] drop_led_ctr  = 0;
	reg [21:0] frame_led_ctr = 0;

	always @(posedge clk) begin
		if (sample_dropped)
			drop_led_ctr <= LED_CYCLES[21:0];
		else
			drop_led_ctr <= drop_led_ctr - (drop_led_ctr != 0);

		if (frame_sent)
			frame_led_ctr <= LED_CYCLES[21:0];
		else
			frame_led_ctr <= frame_led_ctr - (frame_led_ctr != 0);
	end

	assign led_r = ~(drop_led_ctr != 0);
	assign led_g = ~ctrl_good;
	assign led_b = ~(frame_led_ctr != 0);

endmodule

// --------------------------------------------------------------------------
// Sample RAM: up to four SB_SPRAM256KA (16K x 16 each), one port.
// AW <= 14 uses one block; AW = 16 uses all four.  Read data appears on the
// edge after `re` (registered) — capture it on the next edge, because a
// write cycle makes DATAOUT undefined (yosys ice40 cells_sim.v).
// --------------------------------------------------------------------------
module sample_ram #(
	parameter integer AW = 16
) (
	input  wire          clk,
	input  wire [AW-1:0] addr,
	input  wire [15:0]   wdata,
	input  wire          we,
	input  wire          re,
	output wire [15:0]   rdata
);
	localparam integer NBANK = (AW > 14) ? (1 << (AW - 14)) : 1;

	wire [13:0] row;
	wire [1:0]  bank;

	generate
		if (AW >= 14) begin : g_row
			assign row = addr[13:0];
		end else begin : g_row_small
			assign row = {{(14 - AW){1'b0}}, addr};
		end
		if (AW > 14) begin : g_bank
			assign bank = addr[AW-1:14];
		end else begin : g_bank_one
			assign bank = 2'd0;
		end
	endgenerate

	wire [16*NBANK-1:0] dout;
	reg  [1:0]          bank_q = 2'd0;

	genvar g;
	generate
		for (g = 0; g < NBANK; g = g + 1) begin : g_spram
			SB_SPRAM256KA u_spram (
				.ADDRESS(row),
				.DATAIN(wdata),
				.MASKWREN(4'b1111),
				.WREN(we),
				.CHIPSELECT((we | re) && (bank == g)),
				.CLOCK(clk),
				.STANDBY(1'b0),
				.SLEEP(1'b0),
				.POWEROFF(1'b1),
				.DATAOUT(dout[16*g +: 16])
			);
		end
	endgenerate

	always @(posedge clk)
		if (re)
			bank_q <= bank;

	assign rdata = dout[16*bank_q +: 16];
endmodule
