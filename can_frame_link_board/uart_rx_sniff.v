/*
 * uart_rx_sniff.v — passive 8-N-1 UART receiver for the SignalBench sniffer.
 *
 * New for can_frame_link_board (2026-10-11), CONTRACT.md §4/§5.
 * Protocol v3 (2026-11-01, PROTOCOL_V3_CONTRACT.md §3): the bit period is a
 * RUNTIME input `div` = round(CLK_HZ / baud) clocks, set by the ESP's C3
 * config block (16..65535 = 750 000 .. 184 baud).  It was a BAUD parameter.
 *
 *  - Oversampling: an NCO makes a tick at exactly 16 / div per clock on
 *    average (acc += 16, wrap at div), i.e. 16 ticks per div-clock bit, so the
 *    average rate is exact for the configured divisor; only the phase is
 *    quantised to one clock.  The only rate error is the rounding of div
 *    (<= 0.5 clock per bit: exact at 9600 / div 1250, +0.16% at 115200 /
 *    div 104).  At div 1250: a tick every 78.125 clocks (78 or 79), jitter
 *    83 ns = 0.08% of a bit.  At div 104: a tick every 6.5 clocks, jitter 1%
 *    of a bit.  Break at 9600 = >= 1.04 ms low.
 *  - A divisor change takes effect at the next tick; a byte in flight when it
 *    changes may be received wrongly (the ESP changes baud between bytes).
 *    div < 16 is never applied (the config block is rejected); if it ever
 *    were, ticks would come every clock (16x at 750 kbaud is the ceiling).
 *  - Start: the falling edge is detected at the CLOCK rate while armed (not
 *    on a tick: at 9600 a tick is 6.5 us, which would quantise the timestamp).
 *    The timestamp (us since reset) is latched on that clock, 2-3 clocks
 *    (<= 250 ns) after the pin edge, and the NCO phase is reset there, so the
 *    oversampling grid is aligned to the edge: tick k falls k/16 bit after it.
 *  - Mid-bit majority: the samples at ticks 16b+7, 16b+8, 16b+9 are voted for
 *    bit b (0 start, 1..8 data LSB first, 9 stop): centred on 16b+8, the
 *    exact middle of the bit.
 *  - Start validation: a start bit that does not vote low is a glitch; the
 *    receiver re-arms without emitting anything.
 *  - Stop bit votes low: framing error, or BREAK if all eight data bits were
 *    also low (>= one full frame of zeros).  A break is reported as brk=1,
 *    fe=0, data=0x00 (the classified condition; not counted twice).  After
 *    either, the receiver waits for the line to vote high before re-arming,
 *    so a long break produces exactly one record.
 *  - Tolerance: the stop-bit centre is 9.5 bit times after the edge; the
 *    receiver tolerates roughly +-4.5% baud mismatch (tested at +-2%).
 *
 * rx is asynchronous; it is synchronised here (2 flops).
 */
`default_nettype none

module uart_rx_sniff #(
	parameter integer CLK_HZ = 12_000_000     // documentation only: div is in clocks
) (
	input  wire        clk,
	input  wire        rst,
	input  wire [15:0] div,         // clocks per bit, 16..65535 (runtime, from the C3 config)
	input  wire        rx,          // idle high
	input  wire [31:0] us_now,      // free-running microsecond counter
	output reg         strobe,      // 1-clk: data/fe/brk/ts valid
	output reg  [7:0]  data,
	output reg         fe,
	output reg         brk,
	output reg  [31:0] ts,
	output wire        busy         // receiving (for testbench mirrors)
);
	initial begin strobe = 1'b0; data = 8'd0; fe = 1'b0; brk = 1'b0; ts = 32'd0; end

	// -- synchroniser ----------------------------------------------------
	reg [1:0] rx_q = 2'b11;
	always @(posedge clk) rx_q <= {rx_q[0], rx};
	wire rx_s = rx_q[1];

	// -- 16x NCO: 16 ticks per div clocks ------------------------------------
	reg  [16:0] acc  = 17'd0;
	wire [16:0] nxt  = acc + 17'd16;
	wire        tick = (nxt >= {1'b0, div});
	wire        start_edge;          // defined below: realigns the NCO
	always @(posedge clk) acc <= start_edge ? 17'd0 : tick ? (nxt - {1'b0, div}) : nxt;

	// -- receiver ------------------------------------------------------------
	localparam [1:0] S_IDLE = 2'd0, S_RX = 2'd1, S_WAITHI = 2'd2;
	reg [1:0] st  = S_IDLE;
	reg [7:0] cnt = 8'd0;          // ticks since the start was detected
	reg [2:0] smp = 3'b111;        // last three tick samples
	reg [7:0] sh  = 8'd0;

	wire [2:0] smp_n = {smp[1:0], rx_s};
	wire       vote  = (smp_n[0] & smp_n[1]) | (smp_n[0] & smp_n[2]) | (smp_n[1] & smp_n[2]);
	wire [3:0] bitn  = cnt[7:4];

	// After reset (POR, or the probe UART entering mode 0) a start bit is only
	// accepted once the line has been seen high (idle mark): a line that is
	// low at that moment is not a start bit, and would otherwise be reported
	// as a framing error or break.
	reg armed = 1'b0;
	always @(posedge clk) armed <= !rst && (armed || rx_s);

	assign busy = (st == S_RX);
	assign start_edge = !rst && armed && (st == S_IDLE) && !rx_s;

	always @(posedge clk) begin
		strobe <= 1'b0;
		if (rst) begin
			st  <= S_IDLE;
			smp <= 3'b111;
		end else if (start_edge) begin
			st  <= S_RX;
			cnt <= 8'd0;             // counts ticks since the edge, minus one
			ts  <= us_now;
		end else if (tick) begin
			smp <= smp_n;
			case (st)
				S_IDLE: ;
				S_RX: begin
					cnt <= cnt + 8'd1;
					// cnt == 16b+8 on tick 16b+9: smp_n = ticks 16b+7..16b+9
					if (cnt[3:0] == 4'd8) begin
						if (bitn == 4'd0) begin
							if (vote) st <= S_IDLE;          // glitch, not a start bit
						end else if (bitn <= 4'd8) begin
							sh <= {vote, sh[7:1]};           // LSB first
						end else begin                       // stop bit
							strobe <= 1'b1;
							data   <= sh;
							brk    <= !vote && (sh == 8'd0);
							fe     <= !vote && (sh != 8'd0);
							st     <= vote ? S_IDLE : S_WAITHI;
						end
					end
				end
				S_WAITHI: if (vote) st <= S_IDLE;
				default: st <= S_IDLE;
			endcase
		end
	end
endmodule
