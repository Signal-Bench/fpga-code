/*
 * probe_decoders.v — I2C and SPI sniffers on the logic probes, protocol v3
 * (docs/PROTOCOL_V3_CONTRACT.md §1/§2), SignalBench PCBA, 12 MHz core clock.
 *
 * Both decoders OVERSAMPLE the probe lines at the 12 MHz core clock: there is
 * no SCK/SCL clock domain (all 8 SB_GB of the UP5K are already in use, and a
 * probed clock would need one).  Inputs arrive already synchronised (2 flops,
 * `can_frame_link.v` probe bank) together with their previous sample, so an
 * edge is "now != before".  Every line goes through the same synchroniser,
 * so their relative order is preserved to within one 83 ns sample.
 *
 * Each decoder emits one event per record (ev_stb, 1 clock) with the record's
 * flag byte and fields; can_frame_link.v packs it.  Bit 3 of the flag byte
 * (`dropped`) is always 0 here: it is inserted at arbiter grant.
 * Timestamps: only one probe decoder runs at a time, so they share ONE 32-bit
 * timestamp register in can_frame_link.v; a decoder pulses ts_ld
 * (combinational) in the clock whose us_ctr the record must carry, and the
 * register then holds it until the record has been offered to its slot.
 * The ev_* output registers double as the hold register for a completed byte
 * that waits for its follow-up event.
 *
 * rst holds a decoder idle (can_frame_link.v asserts it whenever the probe
 * mode is not this decoder's, and on a mode / SPI-config change), so no half
 * decoded byte survives a mode change.  `held` tells the top a completed byte
 * is waiting in ev_*; if the decoder is reset with it there, the top counts it
 * as a dropped record.
 *
 * Deglitch (contract §2, ruled 2026-11-01): every line a decoder uses passes a
 * 2-sample stability filter: the filtered level takes a new value only when
 * two consecutive synchronised samples (~167 ns) agree on it.  A crossing that
 * lasts a single 83 ns sample (ringing on a slow edge, a double crossing) is
 * ignored.  All lines of one decoder get the same filter, so their relative
 * timing is unchanged (each clean edge is delayed by exactly one clock).  SCK
 * / SCL high and low times must be >= ~170 ns: SPI is specified to 1.5 MHz.
 */
`default_nettype none

// ===========================================================================
// I2C, record 1C: flags data ts.  SCL = PROBE1, SDA = PROBE2.
//
// Bus conditions (as partner sb_i2c in final_system/sb_decoders.v, from
// i2c_sniffer.v): START = SDA falls while SCL is high, STOP = SDA rises while
// SCL is high, both judged on the same synchronised sample.  If SDA and SCL
// change in the same 83 ns sample, SCL is taken as already low (no START /
// STOP), which is right for data changes with tHD;DAT = 0.  A real START has
// tHD;STA >= 260 ns (Fm+), 600 ns (Fm), so it is always seen.
//
// Bits: SDA is sampled at the SCL rising edge but only COMMITTED at the
// following SCL falling edge.  The SCL rise that precedes every STOP and
// every repeated START is therefore never mistaken for a data bit: the
// condition arrives while SCL is still high and discards the tentative bit.
// (sb_i2c commits at the rise and so records one stray bit before each
// STOP / rSTART.)  Bits are counted only inside a transaction (after START).
//
// Records (one per completed byte, address byte included): a byte completes
// at the commit of its 9th (ACK) bit, where ts is latched.  It is then held
// until the next bus event so that `stop` / `timeout` can be set on it:
// emitted at the commit of the next byte's first bit (no flag), at a
// repeated START (no flag; the next byte gets `rstart`), at STOP (`stop`) or
// at the idle timeout (`timeout`).  A STOP / timeout with no byte pending
// emits a record with data_valid = 0 (and `partial` + the bits if a byte
// was cut short).
//
// flags: b7 start, b6 rstart, b5 stop, b4 data_valid, b3 dropped (0 here),
//        b2 ack (SDA low on the 9th clock), b1 timeout, b0 partial.
// partial: the transaction ended (STOP, timeout or repeated START) after
//   1..8 committed bits but before the ACK clock; data = the bits received,
//   MSB-aligned (first bit in bit 7, unreceived bits 0), data_valid = 0,
//   ack = 0 (contract §2 ruling 2026-11-01).
// timeout: inside a transaction, the bus idle - SCL AND SDA both high - for
//   TIMEOUT_US (1 ms); the transaction is then closed (the next START reports
//   `start`).  SCL held low (clock stretching, any length) never times out.
// start / rstart: set on the first record after START / repeated START
//   (whatever its kind, so a START immediately followed by STOP is one
//   record: start=1, stop=1, data_valid=0).
// A STOP outside a transaction is ignored (no record).
// ===========================================================================
module probe_i2c #(
	parameter integer TIMEOUT_US = 1000
) (
	input  wire        clk,
	input  wire        rst,
	input  wire        us_tick,        // 1 clock per microsecond
	input  wire        scl_s, scl_sd,  // synchronised sample now / previous
	input  wire        sda_s, sda_sd,
	output reg         ev_stb,         // 1 clock: the record in ev_flags/ev_data is complete
	output reg  [7:0]  ev_flags,
	output reg  [7:0]  ev_data,
	output wire        ts_ld,          // capture us_ctr into the shared probe timestamp now
	output wire        held            // a completed byte waits in ev_*
);
	localparam [7:0] F_STOP = 8'h20, F_DV = 8'h10, F_TMO = 8'h02, F_PART = 8'h01;

	// synthesis translate_off
	initial if (TIMEOUT_US < 2 || TIMEOUT_US > 1023) begin
		$display("probe_i2c: TIMEOUT_US=%0d out of range 2..1023 (10-bit counter)", TIMEOUT_US);
		$finish;
	end
	// synthesis translate_on

	// 2-sample deglitch, same on both lines: scl/sda = filtered now, *_d = before.
	reg scl = 1'b1, scl_d = 1'b1, sda = 1'b1, sda_d = 1'b1;
	always @(posedge clk) begin
		if (rst) begin
			scl <= scl_s; scl_d <= scl_s; sda <= sda_s; sda_d <= sda_s;
		end else begin
			if (scl_s == scl_sd) scl <= scl_s;
			if (sda_s == sda_sd) sda <= sda_s;
			scl_d <= scl;
			sda_d <= sda;
		end
	end

	wire scl_rise =  scl & ~scl_d;
	wire scl_fall = ~scl &  scl_d;
	wire start_c  = ~sda &  sda_d & scl;
	wire stop_c   =  sda & ~sda_d & scl;
	wire bus_free =  scl & sda;        // released: the only state that can time out

	reg        in_txn = 1'b0;      // between START and STOP / timeout
	reg  [1:0] s_flag = 2'b00;     // {start, rstart} for the next record
	reg        tbit   = 1'b0;      // tentative bit (sampled at SCL rise)
	reg        tbit_v = 1'b0;
	reg  [3:0] cnt    = 4'd0;      // committed bits of the current byte (0..8)
	reg  [7:0] sh     = 8'd0;      // bit k written at 7-k (MSB first)
	reg        pend_v = 1'b0;      // ev_* hold a completed byte awaiting its follow-up
	reg  [9:0] idle_us = 10'd0;       // microseconds with SCL and SDA both high
	assign held = pend_v;

	// Decisions for this clock (priority: START, STOP, timeout, bits).
	wire tmo      = in_txn && us_tick && (idle_us == TIMEOUT_US - 1);
	wire commit   = in_txn && scl_fall && tbit_v && !start_c && !stop_c && !tmo;
	wire ack_done = commit && (cnt == 4'd8);                    // byte complete
	wire rel_byte  = commit && (cnt == 4'd0) && pend_v;          // next byte begins
	wire close    = in_txn && (start_c || stop_c || tmo);       // segment ends here
	wire close_new = close && !pend_v && (cnt != 4'd0 || !start_c);  // a new record (partial / marker)
	assign ts_ld  = !rst && (ack_done || close_new);

	wire [7:0] why = start_c ? 8'h00 : stop_c ? F_STOP : F_TMO;

	initial begin ev_stb = 1'b0; ev_flags = 8'd0; ev_data = 8'd0; end

	always @(posedge clk) begin
		ev_stb <= 1'b0;
		if (rst) begin
			in_txn <= 1'b0; s_flag <= 2'b00; tbit_v <= 1'b0; cnt <= 4'd0; sh <= 8'd0;
			pend_v <= 1'b0; idle_us <= 10'd0;
		end else begin
			if (!bus_free)    idle_us <= 10'd0;
			else if (us_tick) idle_us <= (idle_us == 10'd1023) ? idle_us : idle_us + 10'd1;

			if (start_c || stop_c || tmo) begin
				if (close) begin
					if (pend_v) begin
						ev_stb   <= 1'b1;                       // the held byte, plus stop / timeout
						ev_flags <= ev_flags | why;
					end else if (close_new) begin
						ev_stb   <= 1'b1;                       // cut-short byte or a bare marker
						ev_flags <= {s_flag, 6'd0} | why | ((cnt != 4'd0) ? F_PART : 8'h00);   // partial: data_valid 0
						ev_data  <= sh;
					end
				end
				s_flag <= start_c ? (in_txn ? 2'b01 : 2'b10) : 2'b00;
				in_txn <= start_c;
				tbit_v <= 1'b0; cnt <= 4'd0; sh <= 8'd0; pend_v <= 1'b0;
			end else if (in_txn) begin
				if (scl_rise) begin
					tbit   <= sda;
					tbit_v <= 1'b1;
				end
				if (commit) begin
					tbit_v <= 1'b0;
					if (ack_done) begin
						ev_flags <= {s_flag, 1'b0, 1'b1, 1'b0, ~tbit, 2'b00};
						ev_data  <= sh;
						pend_v   <= 1'b1;
						s_flag   <= 2'b00;
						cnt      <= 4'd0;
						sh       <= 8'd0;
					end else begin
						if (rel_byte) begin
							ev_stb <= 1'b1;
							pend_v <= 1'b0;
						end
						sh[3'd7 - cnt[2:0]] <= tbit;
						cnt <= cnt + 4'd1;
					end
				end
			end
		end
	end
endmodule

// ===========================================================================
// SPI, record 5B: flags mosi miso ts nbits.  SCK = PROBE1, MOSI = PROBE2,
// MISO = PROBE3, CS = PROBE4 (active low).  cfg = {lsb_first, cpol, cpha}.
//
// Sampling edge: rising when CPOL == CPHA (modes 0 and 3), falling otherwise
// (modes 1 and 2).  SCK, CS, MOSI and MISO all pass the same 2-sample filter.
// MOSI and MISO are taken from the same filtered sample in which the SCK edge
// is first seen; they are stable there for half an SCK period minus the
// device's output delay.  SCK high and low must each last >= ~170 ns (two
// samples): supported to 1.5 MHz (contract), 2 MHz tested.
//
// Framing: after reset the decoder waits until CS has been seen high
// (`armed`), so a mode change in the middle of a transaction does not decode
// its tail.  CS falling starts a segment; a sampling edge seen in the same
// clock as CS falling is bit 0 (the first SCK edge may follow CS immediately).
// After that, edges are counted while CS was low in the previous sample (an
// edge in the same sample as CS rising still counts: some masters raise CS
// right after the last edge).  Bit k of a byte
// is written at index 7-k (MSB first) or k (LSB first), so a cut-short byte is
// already MSB-aligned / LSB-aligned as the contract asks.
//
// Records: a byte completes at its 8th sampling edge (ts latched) and is held
// until the next event so that `cs_end` can be set on it: emitted at the next
// byte's first sampling edge, or at CS rise (cs_end=1).  At CS rise with no
// byte pending: a cut-short byte (1..7 bits) gives partial=1, nbits=bits;
// nothing at all gives an end-only record (data_valid=0, nbits=0, mosi = miso
// = 0).  (Degenerate case: a first bit of a new byte in the same sample as CS
// rising, with a byte pending — that single bit is discarded and the pending
// byte is reported with cs_end=1.)
//
// flags: b7 cs_start (first record after CS fell), b6 cs_end (CS rose after
//        this byte), b5 partial (nbits < 8), b4 data_valid, b3 dropped (0
//        here), b2..0 = 0.
// ===========================================================================
module probe_spi (
	input  wire        clk,
	input  wire        rst,
	input  wire [2:0]  cfg,            // {lsb_first, cpol, cpha}
	input  wire        sck_s, sck_sd,  // synchronised sample now / previous
	input  wire        mosi_s,
	input  wire        miso_s,
	input  wire        cs_s, cs_sd,
	output reg         ev_stb,         // 1 clock: the record in ev_* is complete
	output reg  [7:0]  ev_flags,
	output reg  [7:0]  ev_mosi,
	output reg  [7:0]  ev_miso,
	output reg  [3:0]  ev_nbits,
	output wire        ts_ld,          // capture us_ctr into the shared probe timestamp now
	output wire        held            // a completed byte waits in ev_*
);
	wire cpha = cfg[0], cpol = cfg[1], lsb = cfg[2];

	// 2-sample deglitch, same on all four lines (sck/cs: filtered now, *_d before).
	reg sck = 1'b0, sck_d = 1'b0, cs = 1'b1, cs_d = 1'b1, mosi = 1'b0, miso = 1'b0;
	reg mosi_p = 1'b0, miso_p = 1'b0;          // previous samples of MOSI / MISO
	always @(posedge clk) begin
		mosi_p <= mosi_s;
		miso_p <= miso_s;
		if (rst) begin
			sck <= sck_s; sck_d <= sck_s; cs <= cs_s; cs_d <= cs_s; mosi <= mosi_s; miso <= miso_s;
		end else begin
			if (sck_s  == sck_sd) sck  <= sck_s;
			if (cs_s   == cs_sd)  cs   <= cs_s;
			if (mosi_s == mosi_p) mosi <= mosi_s;
			if (miso_s == miso_p) miso <= miso_s;
			sck_d <= sck;
			cs_d  <= cs;
		end
	end

	reg        armed  = 1'b0;      // CS seen high since reset
	reg        in_cs  = 1'b0;
	reg        first  = 1'b0;      // no record yet in this CS segment
	reg  [2:0] cnt    = 3'd0;      // bits of the current byte
	reg  [7:0] mo     = 8'd0, mi = 8'd0;
	reg        pend_v = 1'b0;      // ev_* hold a completed byte awaiting its follow-up
	assign held = pend_v;

	wire edge_s  = (cpol ^ cpha) ? (~sck & sck_d) : (sck & ~sck_d);
	wire cs_fall = armed && !in_cs && !cs && cs_d;
	wire samp    = edge_s && ((in_cs && !cs_d) || cs_fall);   // incl. an edge in the CS-fall clock
	wire cs_rise = in_cs && cs && !cs_d;
	wire [2:0] idx = lsb ? cnt : (3'd7 - cnt);

	// Decisions for this clock.
	wire byte_done = samp && (cnt == 3'd7);                     // a byte completes
	wire rel       = !byte_done && pend_v && (cs_rise || samp); // release the held byte
	wire cut       = !byte_done && !pend_v && cs_rise;          // partial or end-only
	assign ts_ld   = !rst && (byte_done || cut);

	// The current byte with this sample's bits written in.
	reg [7:0] mo_w, mi_w;
	always @(*) begin
		mo_w = mo; mi_w = mi;
		mo_w[idx] = mosi;
		mi_w[idx] = miso;
	end

	initial begin ev_stb = 1'b0; ev_flags = 8'd0; ev_mosi = 8'd0; ev_miso = 8'd0; ev_nbits = 4'd0; end

	always @(posedge clk) begin
		ev_stb <= 1'b0;
		if (rst) begin
			armed <= 1'b0; in_cs <= 1'b0; first <= 1'b0; cnt <= 3'd0; mo <= 8'd0; mi <= 8'd0; pend_v <= 1'b0;
		end else begin
			if (cs) armed <= 1'b1;
			if (cs_fall) begin
				// cnt = 0 and mo = mi = 0 here (cleared at the last CS rise / reset),
				// so mo_w / mi_w hold just bit 0 when an edge came with CS.
				in_cs <= 1'b1; first <= 1'b1; pend_v <= 1'b0;
				cnt <= samp ? 3'd1 : 3'd0;
				mo  <= samp ? mo_w : 8'd0;
				mi  <= samp ? mi_w : 8'd0;
			end else if (in_cs) begin
				if (byte_done) begin
					ev_flags <= {first, cs_rise, 1'b0, 1'b1, 4'd0};
					ev_mosi  <= mo_w;
					ev_miso  <= mi_w;
					ev_nbits <= 4'd8;
					ev_stb   <= cs_rise;                    // CS rose with it: out now
					pend_v   <= !cs_rise;                   // else hold it for cs_end
					first    <= 1'b0;
					cnt <= 3'd0; mo <= 8'd0; mi <= 8'd0;
				end else if (rel) begin
					ev_stb      <= 1'b1;
					ev_flags[6] <= cs_rise;
					pend_v      <= 1'b0;
					if (!cs_rise) begin
						mo <= mo_w; mi <= mi_w; cnt <= cnt + 3'd1;
					end
				end else if (cut) begin
					ev_stb   <= 1'b1;
					first    <= 1'b0;
					if (samp || cnt != 3'd0) begin
						ev_flags <= {first, 1'b1, 1'b1, 1'b1, 4'd0};
						ev_mosi  <= samp ? mo_w : mo;
						ev_miso  <= samp ? mi_w : mi;
						ev_nbits <= {1'b0, cnt} + {3'd0, samp};
					end else begin
						ev_flags <= {first, 1'b1, 1'b0, 1'b0, 4'd0};
						ev_mosi  <= 8'd0; ev_miso <= 8'd0; ev_nbits <= 4'd0;
					end
				end else if (samp) begin
					mo <= mo_w; mi <= mi_w; cnt <= cnt + 3'd1;
				end
				if (cs_rise) begin
					in_cs <= 1'b0; pend_v <= 1'b0; cnt <= 3'd0; mo <= 8'd0; mi <= 8'd0;
				end
			end
		end
	end
endmodule
