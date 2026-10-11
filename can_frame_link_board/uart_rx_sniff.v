/*
 * uart_rx_sniff.v — passive 8-N-1 UART receiver for the SignalBench sniffer.
 *
 * New for can_frame_link_board (2026-10-11), CONTRACT.md §4/§5.
 *
 *  - Oversampling: an NCO makes a tick at exactly 16 x BAUD on average from
 *    the 12 MHz board clock (acc += 16*BAUD, wrap at CLK_HZ), so the average
 *    rate is exact for any BAUD; only the phase is quantised to one clock.
 *    At the 9600 default (CONTRACT §6): 1250 clocks/bit, a tick every 78.125
 *    clocks (78 or 79), jitter 83 ns = 0.08% of a bit.  At 115200: a tick
 *    every 6.51 clocks, jitter 1% of a bit.  Break at 9600 = >= 1.04 ms low.
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
	parameter integer CLK_HZ = 12_000_000,
	parameter integer BAUD   = 9_600
) (
	input  wire        clk,
	input  wire        rst,
	input  wire        rx,          // idle high
	input  wire [31:0] us_now,      // free-running microsecond counter
	output reg         strobe,      // 1-clk: data/fe/brk/ts valid
	output reg  [7:0]  data,
	output reg         fe,
	output reg         brk,
	output reg  [31:0] ts,
	output wire        busy         // receiving (for testbench mirrors)
);
	localparam integer INC = 16 * BAUD;

	// synthesis translate_off
	initial if (INC > CLK_HZ || BAUD < 300) begin
		$display("uart_rx_sniff: BAUD=%0d not supported at CLK_HZ=%0d (need 16*BAUD <= CLK_HZ)", BAUD, CLK_HZ);
		$finish;
	end
	// synthesis translate_on

	initial begin strobe = 1'b0; data = 8'd0; fe = 1'b0; brk = 1'b0; ts = 32'd0; end

	// -- synchroniser ----------------------------------------------------
	reg [1:0] rx_q = 2'b11;
	always @(posedge clk) rx_q <= {rx_q[0], rx};
	wire rx_s = rx_q[1];

	// -- 16x NCO -----------------------------------------------------------
	reg  [24:0] acc  = 25'd0;
	wire [24:0] nxt  = acc + INC[24:0];
	wire        tick = (nxt >= CLK_HZ[24:0]);
	wire        start_edge;          // defined below: realigns the NCO
	always @(posedge clk) acc <= start_edge ? 25'd0 : tick ? (nxt - CLK_HZ[24:0]) : nxt;

	// -- receiver ------------------------------------------------------------
	localparam [1:0] S_IDLE = 2'd0, S_RX = 2'd1, S_WAITHI = 2'd2;
	reg [1:0] st  = S_IDLE;
	reg [7:0] cnt = 8'd0;          // ticks since the start was detected
	reg [2:0] smp = 3'b111;        // last three tick samples
	reg [7:0] sh  = 8'd0;

	wire [2:0] smp_n = {smp[1:0], rx_s};
	wire       vote  = (smp_n[0] & smp_n[1]) | (smp_n[0] & smp_n[2]) | (smp_n[1] & smp_n[2]);
	wire [3:0] bitn  = cnt[7:4];

	assign busy = (st == S_RX);
	assign start_edge = !rst && (st == S_IDLE) && !rx_s;

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
