/*
 * spi_frame_link.v — framed, acknowledged record stream to the ESP32-H2
 *
 * DERIVED FROM Signal-Bench/fpga-code spi_frame_stream/spi_frame_stream.v
 * (branch feat/spi-frame-stream, commit 58a12d6e75dd92ee8cd29762cb47b56a811b4a9f).
 * The original header follows unchanged; the changes for the SignalBench
 * PCBA CAN chain are listed after it.
 *
 * ---------------------------------------------------------------- original
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
 * ------------------------------------------------------------ end original
 *
 * CHANGES (SignalBench can_frame_link_board, 2026-10-11).  The SPI engine,
 * control-block receiver, prep pipeline and frame format are unchanged.
 *
 *  1. Module boundary.  The 48 MHz HFOSC / 2 clock, the fake sensor, the LEDs
 *     and the pin-16 flash-CS output are gone.  `clk` is an input; on the
 *     SignalBench PCBA it is the 12 MHz board oscillator (pin 44), so every
 *     cycle count above doubles in time: MISO changes 2-3 cycles = 167-250 ns
 *     after the falling SCK edge.  SCK ceiling is therefore 1.5 MHz (83 ns of
 *     setup), recommended 1 MHz (>= 250 ns).  A byte at 1.5 MHz is 64 core
 *     cycles, the same ratio the original was designed for at 3 MHz / 24 MHz.
 *
 *  2. Atomic variable-length record writer.  The producer offers a record
 *     of rec_len (1..15) 16-bit samples with a one-cycle `rec_req`; the link
 *     reads them back through `rec_idx` / `rec_sample` and writes them on
 *     consecutive cycles (the reader waits; SPRAM is single-port).  The
 *     record is accepted only if all rec_len slots are free, and wr_off
 *     advances by the whole record in one step after the last write, so a
 *     frame can never include a partial record that is still being written.
 *     A record that does not fit (or that arrives while the writer is busy;
 *     the top's arbiter never does that) is dropped whole.
 *     The free-space test is combinational from the registered offsets
 *     (one subtract + compare, trivial at 12 MHz), so there is no stale-flag
 *     window and `rec_busy` lasts exactly rec_len cycles.
 *
 *  3. `dropped` counts RECORDS, not samples.  With mixed 10-sample (CAN) and
 *     4-sample (UART) records a sample count of dropped data cannot be turned
 *     back into a record count, and the offsets never skip for dropped data
 *     anyway (dropped data never enters the buffer), so the field is only a
 *     report.  This is the one change in field MEANING; the wire format is
 *     unchanged.  ST_DROPPED behaves as before.
 *
 *  4. Frames end on a record boundary whenever `count` is limited by the
 *     buffered level (wr_off is always a boundary).  When read_len or
 *     MAX_FRAME_SAMPLES limits it, the last record may continue in the next
 *     frame; the MCU reassembles the byte stream, which is exactly-once and
 *     in order, so this is lossless.  (Rounding to whole records is not
 *     possible without storing record boundaries, since lengths differ.)
 *
 *  5. Status outputs (ctrl_good, frame_sent) replace the LEDs.
 *
 * CHANGES (SignalBench protocol v3, 2026-11-01, docs/PROTOCOL_V3_CONTRACT.md §3).
 *
 *  6. C3 config block.  A CS transaction whose first MOSI byte is 0xC3 is a
 *     config write, not a stream read:
 *       C3 | gen | probe_mode | spi_cfg | div0 u16 LE | div1 | div2 | div3 | crc16 BE
 *     14 bytes, CRC-16/CCITT-FALSE (this file's crc16_byte) over bytes 0..11,
 *     sent big-endian in bytes 12..13 (same CRC and byte order as the 5C
 *     block).  Bytes 1..9 are captured in the 5C block's byte registers
 *     (c_ack/c_len/c_flags/c_crc, which store bytes 1..9 whatever byte 0
 *     is), bytes 10..11 in two of their own and byte 12 is compared on
 *     arrival with the CRC high byte; when byte 13 arrives the fields are
 *     copied into the sticky config registers
 *     (outputs cfg_*) only if the CRC matches AND probe_mode <= 3, spi_cfg
 *     bits 7..3 are 0 and every divN >= 16.  Otherwise nothing changes.
 *     Bytes after 13 are ignored; a transaction that ends before byte 13
 *     changes nothing.
 *     A C3 transaction never reaches the frame decision (DEC_*): no ack is
 *     applied, nothing is fetched, sent_hw / rd_off / `dropped_reported` /
 *     ctrl_good / the ST_DROPPED edge are untouched, so it cannot consume,
 *     ack or advance stream data.  Its MISO is A5 5A 01, the echo bytes, then
 *     poisoned filler: don't-care, the ESP discards it.
 *     Reset (configuration) values: gen 0, probe_mode 0, spi_cfg 0, div
 *     1250 / 1250 / 1250 / 104 (9600 / 9600 / 9600 / 115200 baud), the
 *     DIVn_RST parameters.
 *
 *  7. Echo.  MISO bytes 3 and 4 of every frame, formerly fixed 0x00, carry
 *     the applied config: byte 3 = gen, byte 4 = probe_mode.  Bytes 5..10
 *     stay 0x00.  Bytes 1..10 are, and always were, covered by the frame CRC
 *     (the CRC runs over bytes 0 .. 22+2*count-1), so the CRC is computed over
 *     the echo bytes actually sent; the prep pipeline needs no change because
 *     the config registers only change inside a C3 transaction, never during
 *     a 5C read, so bytes 3..4 are constant for the whole transfer.
 */
`default_nettype none

module spi_frame_link #(
	parameter integer BUF_AW            = 16,     // 2^16 samples, all four SPRAMs
	parameter integer MAX_FRAME_SAMPLES = 2036,   // (4096 - 24) / 2
	parameter integer MAX_REC_SAMPLES   = 10,     // longest record offered (<= 15)
	// Test hook: where the stream offsets start.  Leave at 0 in hardware.
	parameter [31:0]  INITIAL_OFFSET    = 32'd0,
	// C3 config reset values (contract §3): UART clocks per bit
	parameter [15:0]  DIV0_RST          = 16'd1250,   // RS-232 RX   9600
	parameter [15:0]  DIV1_RST          = 16'd1250,   // RS-232 TX   9600
	parameter [15:0]  DIV2_RST          = 16'd1250,   // RS-485      9600
	parameter [15:0]  DIV3_RST          = 16'd104     // probe UART  115200
) (
	input  wire        clk,
	// SPI slave pins (already through the pad; synchronised here)
	input  wire        spi_sck,
	input  wire        spi_cs,       // active low
	input  wire        spi_mosi,
	output wire        spi_miso,
	// Record producer interface
	input  wire        rec_req,      // 1-clk: a record is ready at rec_sample[rec_idx]
	input  wire [3:0]  rec_len,      // its length in samples, 1..MAX_REC_SAMPLES, with rec_req
	output wire [3:0]  rec_idx,      // which sample of the record the link is reading
	input  wire [15:0] rec_sample,   // producer's sample mux, combinational from rec_idx
	output wire        rec_busy,     // a record is being written; rec_req now = drop
	output wire        rec_accept,   // 1-clk, same cycle as rec_req: record will be written
	output wire        rec_dropped,  // 1-clk, same cycle as rec_req: record dropped
	input  wire [2:0]  drop_add,     // records the producer lost upstream this cycle (counted too)
	// Status
	output reg         ctrl_good,    // last control block was valid
	output reg         frame_sent,   // 1-clk: a frame carrying samples was decided
	// Applied C3 config (sticky; change only on a valid C3 block)
	output reg  [7:0]  cfg_gen,
	output reg  [1:0]  cfg_mode,     // probe_mode 0..3
	output reg  [2:0]  cfg_spi,      // {lsb_first, cpol, cpha}
	output reg  [15:0] cfg_div0,
	output reg  [15:0] cfg_div1,
	output reg  [15:0] cfg_div2,
	output reg  [15:0] cfg_div3,
	output reg         cfg_stb       // 1-clk: a config block was applied
);
	localparam [31:0] CAPACITY = 32'd1 << BUF_AW;

	// Protocol constants — keep in sync with README.md and the MCU parser.
	localparam [7:0]  VERSION    = 8'h01;
	localparam [7:0]  CTRL_MAGIC = 8'h5C;
	localparam [7:0]  CFG_MAGIC  = 8'hC3;
	localparam [15:0] HDR_LEN    = 16'd22;   // bytes before the first sample
	localparam [15:0] OVERHEAD   = 16'd24;   // header + CRC

	localparam [7:0] ST_ACK_APPLIED  = 8'h01;
	localparam [7:0] ST_CTRL_BAD     = 8'h02;
	localparam [7:0] ST_ACK_REJECTED = 8'h04;
	localparam [7:0] ST_DROPPED      = 8'h08;   // samples dropped since the previous frame

	// synthesis translate_off
	initial begin
		if (MAX_REC_SAMPLES < 1 || MAX_REC_SAMPLES > 15 || CAPACITY < 2 * MAX_REC_SAMPLES) begin
			$display("spi_frame_link: bad MAX_REC_SAMPLES=%0d for BUF_AW=%0d", MAX_REC_SAMPLES, BUF_AW);
			$finish;
		end
	end
	always @(posedge clk)
		if (rec_req && (rec_len == 4'd0 || rec_len > MAX_REC_SAMPLES)) begin
			$display("spi_frame_link: rec_len %0d out of range", rec_len);
			$finish;
		end
	// synthesis translate_on

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

	// Stream offsets count samples since configuration and wrap at 2^32.
	// The buffer address is the low BUF_AW bits.  Every comparison below is
	// done on differences, so the wrap is harmless.
	reg  [31:0] wr_off  = INITIAL_OFFSET;   // next offset to commit (whole records only)
	reg  [31:0] wr_ptr  = INITIAL_OFFSET;   // offset being written inside a record
	reg  [31:0] rd_off  = INITIAL_OFFSET;   // oldest offset not yet acknowledged
	reg  [31:0] sent_hw = INITIAL_OFFSET;   // highest offset ever sent; caps a valid ack
	reg  [31:0] dropped = 0;                // RECORDS dropped (buffer full / writer busy)

	// Committed, unacknowledged samples.
	wire [31:0] level = wr_off - rd_off;

	// ----------------------------------------------------------------
	// Atomic record writer.
	// Free-space test: level is combinational from two registers.  rd_off
	// only ever moves forward (an ack is accepted only if rd <= ack <=
	// sent_hw), so space can only grow behind the writer's back; wr_off is
	// only changed by this writer, on the edge it returns to idle.  So the
	// test made in the idle cycle is exact and cannot go stale before the
	// writes it guards.
	// ----------------------------------------------------------------
	reg        w_active = 1'b0;
	reg  [3:0] w_idx    = 4'd0;
	reg  [3:0] w_len    = 4'd1;
	wire       room     = (level <= (CAPACITY - {28'd0, rec_len}));

	assign rec_busy    = w_active;
	assign rec_accept  = rec_req && !w_active && room;
	assign rec_dropped = rec_req && !rec_accept;
	assign rec_idx     = w_idx;

	always @(posedge clk) begin
		if (rec_dropped || drop_add != 3'd0)
			dropped <= dropped + {31'd0, rec_dropped} + {29'd0, drop_add};

		if (rec_accept) begin
			w_active <= 1'b1;
			w_idx    <= 4'd0;
			w_len    <= rec_len;
			wr_ptr   <= wr_off;
		end else if (w_active) begin
			// This edge performs the SPRAM write of sample w_idx (ram_we).
			wr_ptr <= wr_ptr + 1'b1;
			w_idx  <= w_idx + 1'b1;
			if (w_idx == w_len - 4'd1) begin
				w_active <= 1'b0;
				wr_off   <= wr_off + {28'd0, w_len};   // publish the whole record at once
			end
		end
	end

	// ----------------------------------------------------------------
	// Sample RAM.  Single port: the writer wins, the frame reader waits
	// (at most MAX_REC_SAMPLES cycles).  Read data is registered (one cycle
	// of latency) and a write cycle turns DATAOUT to X, so the reader
	// captures it on the very next edge — see the fetch logic.
	// ----------------------------------------------------------------
	reg         fetch_req      = 1'b0;
	reg         fetch_inflight = 1'b0;
	reg  [31:0] fetch_off      = 0;
	reg  [15:0] fetch_left     = 0;

	wire              ram_we   = w_active;
	wire              ram_re   = !ram_we && fetch_req;
	wire [BUF_AW-1:0] ram_addr = ram_we ? wr_ptr[BUF_AW-1:0] : fetch_off[BUF_AW-1:0];
	wire [15:0]       ram_rdata;

	sample_ram #(.AW(BUF_AW)) u_ram (
		.clk(clk), .addr(ram_addr), .wdata(rec_sample),
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

	// C3 config block (change 6).  Bytes 1..9 land in the 5C block's own
	// registers (the byte-position case below stores them whatever byte 0
	// was): c_ack = bytes 1..4, c_len = 5..6, c_flags = 7, c_crc = 8..9.
	// Only bytes 10..12 need registers of their own.
	reg        cfg_txn    = 1'b0;     // this transaction started with 0xC3
	reg [15:0] cfg_b1011  = 0;        // bytes 10, 11 (div3, little-endian)
	reg        cfg_crc_hi = 1'b0;     // byte 12 matched the CRC high byte
	wire [7:0]  sh_gen  = c_ack[7:0];
	wire [7:0]  sh_mode = c_ack[15:8];
	wire [7:0]  sh_spi  = c_ack[23:16];
	wire [15:0] sh_div0 = {c_len[7:0], c_ack[31:24]};       // little-endian fields
	wire [15:0] sh_div1 = {c_flags, c_len[15:8]};
	wire [15:0] sh_div2 = {c_crc[7:0], c_crc[15:8]};
	wire [15:0] sh_div3 = cfg_b1011;
	// Evaluated in the clock byte 13 is processed (rx_byte_q = CRC low byte).
	wire cfg_valid = cfg_crc_hi && (rx_byte_q == crc_rx[7:0]) &&
	                 (sh_mode[7:2] == 6'd0) && (sh_spi[7:3] == 5'd0) &&
	                 (sh_div0[15:4] != 12'd0) && (sh_div1[15:4] != 12'd0) &&
	                 (sh_div2[15:4] != 12'd0) && (sh_div3[15:4] != 12'd0);
	initial begin
		cfg_gen = 8'd0; cfg_mode = 2'd0; cfg_spi = 3'd0; cfg_stb = 1'b0;
		cfg_div0 = DIV0_RST; cfg_div1 = DIV1_RST; cfg_div2 = DIV2_RST; cfg_div3 = DIV3_RST;
	end

	// Frame decision pipeline.
	localparam [3:0] DEC_IDLE  = 4'd0,
	                 DEC_CHECK = 4'd1,   // CRC compare; ack - rd and hw - rd
	                 DEC_APPLY = 4'd2,   // range check, free acked samples
	                 DEC_LEVEL = 4'd3,   // buffered samples; read_len budget
	                 DEC_CAP   = 4'd4,   // clamp budget to MAX_FRAME_SAMPLES
	                 DEC_COUNT = 4'd5,   // count = min(level, budget)
	                 DEC_BUILD = 4'd6;   // latch header, start the fetch
	reg [3:0]  dec       = DEC_IDLE;
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
	reg [31:0] dropped_reported = 0;  // `dropped` as of the previous frame

	initial ctrl_good  = 1'b0;
	initial frame_sent = 1'b0;

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
				5'd3:    p2_byte <= cfg_gen;                // echo (change 7)
				5'd4:    p2_byte <= {6'd0, cfg_mode};
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
		cfg_stb <= 1'b0;
		if (rx_done) begin
			if (rx_pos_q <= 4'd7 || (cfg_txn && rx_pos_q <= 4'd11))
				crc_rx <= crc16_byte(crc_rx, rx_byte_q);
			// C3 config block (change 6)
			if (cfg_txn) begin
				if (rx_pos_q == 4'd10)
					cfg_b1011[7:0] <= rx_byte_q;
				if (rx_pos_q == 4'd11)
					cfg_b1011[15:8] <= rx_byte_q;
				if (rx_pos_q == 4'd12)
					cfg_crc_hi <= (rx_byte_q == crc_rx[15:8]);
				if (rx_pos_q == 4'd13 && cfg_valid) begin
					cfg_gen  <= sh_gen;
					cfg_mode <= sh_mode[1:0];
					cfg_spi  <= sh_spi[2:0];
					cfg_div0 <= sh_div0;
					cfg_div1 <= sh_div1;
					cfg_div2 <= sh_div2;
					cfg_div3 <= sh_div3;
					cfg_stb  <= 1'b1;
				end
			end
			case (rx_pos_q)
				4'd0: begin
					magic_ok <= (rx_byte_q == CTRL_MAGIC);
					cfg_txn  <= (rx_byte_q == CFG_MAGIC);
				end
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
					if (!cfg_txn)              // a C3 transaction never decides a frame
						dec <= DEC_CHECK;
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
			cfg_txn    <= 1'b0;
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

endmodule

// --------------------------------------------------------------------------
// Sample RAM (unchanged from spi_frame_stream.v): up to four SB_SPRAM256KA
// (16K x 16 each), one port.  AW <= 14 uses one block; AW = 16 uses all four.
// Read data appears on the edge after `re` (registered) — capture it on the
// next edge, because a write cycle makes DATAOUT undefined (yosys ice40
// cells_sim.v).
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
