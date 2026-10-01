/** \file
 * SignalBench stream framer: builds frames in the FIFO and publishes them
 * whole.  See MCU_FRAME_FORMAT.md for the wire format.
 *
 *   53 42 4D | flags/protocol | length | payload[length]
 *
 *   flags/protocol:  bit7 DROPPED   data was lost since the previous frame
 *                    bit6 CONT      this frame continues the previous one
 *                    bits5:4 0      reserved
 *                    bits3:0        protocol id, same numbering as the
 *                                   `53 42 4D xx` select command
 *
 * Decoders drive it with two operations:
 *
 *   BYTE(b)     append b to the payload, opening a frame if none is open
 *   CLOSE(c)    finish the frame; c = 1 marks the NEXT frame as CONT
 *
 * ATOMIC PUBLISH.  The payload is written into the FIFO memory as it
 * arrives, but past the commit pointer, where the reader cannot see it.
 * The header -- which needs the final length -- is written last, into five
 * bytes reserved at the start, and only then does the commit pointer move.
 * So the MCU can never read a header whose payload has not arrived yet,
 * which would make it read 0xFF idle bytes as payload.
 *
 * LOSS.  A frame that will not fit in the free space is abandoned whole
 * (never a truncated frame), and the next frame that IS published carries
 * DROPPED.  The same happens if the op queue overflows.
 *
 * ORDERING.  Ops pass through an 8-entry queue, so a decoder can issue one
 * op per clock even while a CLOSE is spending ~6 clocks writing a header.
 */
module sbs_framer #(
	parameter integer AW   = 12,      // FIFO = 2^AW bytes
	parameter integer MAXP = 255      // largest payload, <= 255
)(
	input  wire       clk,
	input  wire       rst,
	input  wire       flush,          // protocol switch: discard everything
	input  wire [2:0] proto,

	input  wire       op_stb,
	input  wire [1:0] op_kind,
	input  wire [7:0] op_data,

	output wire [7:0] head,           // FIFO read port, registered
	output wire       empty,
	input  wire       pop,

	output reg        dropped_pulse
);
	localparam [1:0] K_BYTE = 2'd0, K_CLOSE = 2'd2;   // 2'd1 unused
	localparam integer DEPTH = 1 << AW;
	localparam integer HDR   = 5;

	// ----------------------------------------------------------------
	// Op queue
	// ----------------------------------------------------------------
	reg [9:0] q [0:7];
	reg [3:0] qw = 4'd0, qr = 4'd0;
	wire      q_empty = (qw == qr);
	wire      q_full  = ((qw - qr) == 4'd8);
	wire [9:0] qh     = q[qr[2:0]];
	wire [1:0] k      = qh[9:8];
	wire [7:0] d      = qh[7:0];

	// ----------------------------------------------------------------
	// FIFO memory and pointers
	// ----------------------------------------------------------------
	reg [7:0]  mem [0:DEPTH-1];
	reg [AW:0] commit = 0, rd = 0;
	reg [7:0]  head_r = 8'd0;
	assign head  = head_r;
	assign empty = (commit == rd);

	// ----------------------------------------------------------------
	// Frame-in-progress state
	//
	// Addresses are kept as registers that step forward, never recomputed
	// as base + offset each clock: chaining fstart + 5 + wlen - rd into a
	// compare in one cycle was a four-adder path that capped this design
	// at ~17 MHz.
	// ----------------------------------------------------------------
	reg        open = 1'b0;        // a frame is being built
	reg        discard = 1'b0;     // dropping ops until the next CLOSE
	reg [AW:0] fstart = 0;         // the frame's first header byte
	reg [AW:0] wptr = 0;           // next payload address while open
	reg [AW:0] open_ptr = HDR;     // where payload[0] goes for the NEXT frame
	reg [8:0]  wlen = 9'd0;        // payload bytes so far
	reg [2:0]  step = 3'd0;
	reg        drop_pending = 1'b0;
	reg        cont_pending = 1'b0;

	wire [8:0]  cur_w = open ? wlen : 9'd0;
	wire [AW:0] a_nxt = open ? wptr : open_ptr;    // next payload address

	// Space check, registered and therefore one clock stale.  It can only
	// be stale in the safe direction by the reader (rd only moves forward);
	// in the unsafe direction by at most one op, which writes at most 1
	// byte.  The 8-byte margin covers that, at the cost of 8 bytes of FIFO.
	//
	// The difference MUST be computed in an (AW+1)-bit wire.  Written inline
	// as ((a_nxt - rd) < (DEPTH - 8)), Verilog sizes the subtraction to the
	// 32-bit integer on the right, so once the write pointer wraps past zero
	// and the read pointer hasn't, the result goes negative, reads as huge,
	// and every frame is dropped until the reader wraps too.  Found by the
	// overflow test with a 128-byte FIFO; at 4 KB it would have appeared as
	// intermittent drops every 8 KB of traffic.
	wire [AW:0] used_nxt = a_nxt - rd;
	reg space_ok = 1'b1;
	always @(posedge clk) space_ok <= (used_nxt < (DEPTH - 8));

	// ----------------------------------------------------------------
	// Decide this clock's action and its (single) memory write
	// ----------------------------------------------------------------
	localparam [3:0] A_NONE = 4'd0, A_SKIP = 4'd1, A_OVF = 4'd2, A_BYTE = 4'd3,
	                 A_CSTEP = 4'd6, A_COMMIT = 4'd7, A_CDISC = 4'd8;

	reg  [3:0]  act;
	reg         we;
	reg  [AW:0] waddr;
	reg  [7:0]  wdata;

	always @* begin
		act = A_NONE; we = 1'b0; waddr = a_nxt; wdata = 8'h00;
		if (!q_empty) begin
			case (k)
			K_BYTE: begin
				if (discard)                                  act = A_SKIP;
				else if (cur_w + 9'd1 > MAXP || !space_ok) act = A_OVF;
				else begin act = A_BYTE; we = 1'b1; waddr = a_nxt; wdata = d; end
			end
			default: begin   // K_CLOSE
				if (discard)    act = A_CDISC;
				else if (!open) act = A_SKIP;          // nothing to close
				else case (step)
					3'd0: begin act = A_CSTEP; we = 1'b1; waddr = fstart;        wdata = 8'h53; end
					3'd1: begin act = A_CSTEP; we = 1'b1; waddr = fstart + 3'd1; wdata = 8'h42; end
					3'd2: begin act = A_CSTEP; we = 1'b1; waddr = fstart + 3'd2; wdata = 8'h4D; end
					3'd3: begin act = A_CSTEP; we = 1'b1; waddr = fstart + 3'd4; wdata = wlen[7:0]; end
					3'd4: begin act = A_CSTEP; we = 1'b1; waddr = fstart + 3'd3;
					            wdata = {drop_pending, cont_pending, 3'b000, proto}; end
					default:    act = A_COMMIT;
				endcase
			end
			endcase
		end
	end

	always @(posedge clk) begin
		if (we) mem[waddr[AW-1:0]] <= wdata;
		head_r <= mem[rd[AW-1:0]];
	end

	// ----------------------------------------------------------------
	// State update
	// ----------------------------------------------------------------
	wire q_push   = op_stb & ~q_full;
	wire q_lost   = op_stb &  q_full;
	wire q_pop    = (act == A_SKIP) | (act == A_OVF) | (act == A_BYTE) |
	                (act == A_COMMIT) | (act == A_CDISC);

	always @(posedge clk) begin
		dropped_pulse <= 1'b0;

		if (rst || flush) begin
			qw <= 4'd0; qr <= 4'd0;
			commit <= 0; rd <= 0; open_ptr <= HDR;
			open <= 1'b0; discard <= 1'b0; step <= 3'd0;
			drop_pending <= 1'b0; cont_pending <= 1'b0;
		end else begin
			if (q_push) begin q[qw[2:0]] <= {op_kind, op_data}; qw <= qw + 4'd1; end
			if (q_pop)  qr <= qr + 4'd1;
			if (pop && !empty) rd <= rd + 1'b1;

			case (act)
			A_OVF: begin
				// Abandon the whole frame; it was never published, so the
				// commit pointer simply doesn't move.
				open <= 1'b0; discard <= 1'b1; step <= 3'd0;
			end
			A_BYTE: begin
				if (!open) fstart <= commit;
				open <= 1'b1; wptr <= a_nxt + 1'b1; wlen <= cur_w + 9'd1;
			end
			A_CSTEP: begin
				step <= step + 3'd1;
				if (step == 3'd4) drop_pending <= 1'b0;   // now in the header
			end
			A_COMMIT: begin
				commit   <= wptr;            // one past the last payload byte
				open_ptr <= wptr + HDR;
				open <= 1'b0; step <= 3'd0;
				cont_pending <= d[0];
			end
			A_CDISC: begin
				discard <= 1'b0; open <= 1'b0; step <= 3'd0;
				cont_pending <= d[0];
			end
			default: ;
			endcase

			// Loss is recorded last so it wins over the clear in A_CSTEP.
			if (act == A_OVF || q_lost) begin
				drop_pending  <= 1'b1;
				dropped_pulse <= 1'b1;
			end
			// A lost op corrupts whichever frame it belonged to -- the open
			// one or the one about to open -- so drop up to the next CLOSE.
			if (q_lost) discard <= 1'b1;
		end
	end
endmodule
