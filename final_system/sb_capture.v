/** \file
 * SignalBench shared capture back end: trigger, record packer, capture RAM.
 *
 * Built ONCE and shared by all four decoders.  This is why merging the
 * sniffers costs almost nothing: i2c_sniffer.v and can_sniffer.v each carry
 * their own packer, RAM and drain, and those are the expensive parts.
 *
 * Record: 24 bytes used, 32-byte stride.  The stride is a power of two so
 * addressing is a concatenation rather than a multiply -- the same trick
 * can_sniffer.v uses for its 20-in-32 records.
 *
 *   [0]       0xAA        start marker
 *   [1]       {proto[1:0], flags[5:0]}
 *   [2..5]    aux, 32-bit big-endian   (CAN: identifier; others: 0)
 *   [6]       {dropped, 3'b0, len[3:0]}
 *   [7..14]   payload[0..7], MSB first (len says how many are meaningful)
 *   [15..18]  timestamp, 32-bit big-endian, core clocks
 *   [19..22]  reserved, 0x00
 *   [23]      0x55        end marker
 *
 * Markers are a resync aid, not a length field: a host that loses sync hunts
 * for 0xAA and checks that byte 23 is 0x55.  Note this is deliberately NOT
 * spi_sniffer.md's length-prefixed format -- that document is right that a
 * 0xFF sentinel is unusable on an SPI bus, but 0xAA/0x55 at a fixed 32-byte
 * stride does not have that problem, and a fixed stride keeps the shared
 * packer simple.  Revisit if SPI transactions need to be one record each.
 *
 * "dropped" means at least one record was lost to a full buffer before this
 * one -- same convention as can_sniffer.v.
 */
module sb_capture #(
	parameter integer N_RECORDS = 64,           // 64 x 32 B = 2048 B = 4 EBR
	parameter integer ADDR_W    = 11            // log2(N_RECORDS * 32)
)(
	input  wire clk,
	input  wire rst,

	// selected decoder's event
	input  wire        ev_strobe,
	input  wire [1:0]  ev_proto,
	input  wire [5:0]  ev_flags,
	input  wire [31:0] ev_aux,
	input  wire [3:0]  ev_len,
	input  wire [63:0] ev_payload,
	input  wire [31:0] timestamp,

	// control from the central FSM
	input  wire        arm_record,              // pulse: arm, wait for trigger
	input  wire        start_stream,            // pulse: free-run, no trigger
	input  wire        stop,                    // pulse: back to idle
	input  wire [7:0]  trig0, trig1, trig2, trig3,
	input  wire [2:0]  trig_len,                // 0 = fire immediately

	// drain
	input  wire        drain_advance,           // pulse: step the read pointer
	output reg  [7:0]  drain_data,

	output reg         armed,
	output reg         recording,
	output reg         streaming,
	output reg         done,
	output reg         dropped,
	output wire [15:0] fill_level                // bytes written
);
	// ----------------------------------------------------------------
	// Rolling trigger comparator
	//
	// A window of the last four event bytes is compared against the
	// configured pattern.  trig_len = 0 means "record now" -- the
	// zero-length-pattern option from state_management.md Example 3.
	// ----------------------------------------------------------------
	// ----------------------------------------------------------------
	// Event pipeline register.
	//
	// The muxed event arrives combinationally from `proto` through the
	// protocol mux, and the trigger comparator plus the mode FSM sit
	// behind it.  proto -> mux -> compare -> FSM is the longest path in
	// the design and it does not close at 24 MHz.  Registering the event
	// here splits it in two.
	//
	// Cost is one core clock of latency -- 42 ns, and the timestamp is
	// captured in the same stage so records stay self-consistent.
	// ----------------------------------------------------------------
	reg        e_strobe  = 1'b0;
	reg [1:0]  e_proto   = 2'd0;
	reg [5:0]  e_flags   = 6'd0;
	reg [31:0] e_aux     = 32'd0;
	reg [3:0]  e_len     = 4'd0;
	reg [63:0] e_payload = 64'd0;
	reg [31:0] e_ts      = 32'd0;

	always @(posedge clk) begin
		if (rst) e_strobe <= 1'b0;
		else     e_strobe <= ev_strobe;
		e_proto   <= ev_proto;
		e_flags   <= ev_flags;
		e_aux     <= ev_aux;
		e_len     <= ev_len;
		e_payload <= ev_payload;
		e_ts      <= timestamp;
	end

	reg [31:0] win = 32'd0;

	// The comparison must include the byte arriving THIS cycle.  `win` is
	// updated by a non-blocking assignment, so comparing against it alone
	// tests the previous N bytes and fires one event late -- the matching
	// byte itself would then land outside the capture.
	wire [7:0]  cur     = e_payload[63:56];
	wire [31:0] win_now = {win[23:0], cur};

	always @(posedge clk) begin
		if (rst) win <= 32'd0;
		else if (e_strobe) win <= win_now;
	end

	// Byte order follows state_management.md: trig0 is the FIRST byte on
	// the wire, trig<len-1> the last, so the newest byte compares against
	// the highest-numbered configured register.
	//
	// Only the comparison against the byte arriving now is live.  Every
	// term that involves solely `win` and the pattern registers is
	// precomputed, because chaining four 8-bit compares behind the event
	// register is what stops 24 MHz closing.
	//
	// Precondition: `win` must be stable for one clock before an event.
	// It only changes on e_strobe, and the packer already refuses events
	// closer together than 24 core clocks, so this is strictly weaker
	// than a constraint the design has anyway.
	reg p2 = 1'b0, p3 = 1'b0, p4 = 1'b0;
	always @(posedge clk) begin
		if (rst) begin p2 <= 0; p3 <= 0; p4 <= 0; end
		else begin
			p2 <=  (win[7:0]   == trig0);
			p3 <=  (win[15:8]  == trig0) & (win[7:0] == trig1);
			p4 <=  (win[23:16] == trig0) & (win[15:8] == trig1)
			     & (win[7:0]   == trig2);
		end
	end

	wire h1 = (cur == trig0);
	wire h2 = p2 & (cur == trig1);
	wire h3 = p3 & (cur == trig2);
	wire h4 = p4 & (cur == trig3);

	wire trig_hit = (trig_len == 3'd0) ? 1'b1 :      // 0 = record now
	                (trig_len == 3'd1) ? h1 :
	                (trig_len == 3'd2) ? h2 :
	                (trig_len == 3'd3) ? h3 : h4;

	// ----------------------------------------------------------------
	// Capture RAM.  Inferred; yosys maps this to EBR.
	// ----------------------------------------------------------------
	reg [7:0] mem [0:(N_RECORDS*32)-1];

	reg [ADDR_W-1:0] wr_rec  = 0;     // record index being written
	reg [4:0]        wr_off  = 0;     // byte offset within the record
	reg [ADDR_W-1:0] rd_ptr  = 0;

	wire [ADDR_W-1:0] wr_addr = {wr_rec[ADDR_W-6:0], wr_off};
	assign fill_level = {{(16-ADDR_W){1'b0}}, wr_rec} << 5;

	// ----------------------------------------------------------------
	// Record packer.  A strobe latches the event, then 24 cycles clock it
	// into RAM one byte at a time.  Events arriving mid-pack are dropped
	// and flagged rather than corrupting the record in flight.
	// ----------------------------------------------------------------
	reg        packing = 1'b0;
	reg [4:0]  pk      = 5'd0;
	reg [1:0]  l_proto;
	reg [5:0]  l_flags;
	reg [31:0] l_aux;
	reg [3:0]  l_len;
	reg [63:0] l_payload;
	reg [31:0] l_ts;
	reg        l_dropped;

	reg [7:0] wdat;
	always @* begin
		case (pk)
			5'd0:  wdat = 8'hAA;
			5'd1:  wdat = {l_proto, l_flags};
			5'd2:  wdat = l_aux[31:24];
			5'd3:  wdat = l_aux[23:16];
			5'd4:  wdat = l_aux[15:8];
			5'd5:  wdat = l_aux[7:0];
			5'd6:  wdat = {l_dropped, 3'b000, l_len};
			5'd7:  wdat = l_payload[63:56];
			5'd8:  wdat = l_payload[55:48];
			5'd9:  wdat = l_payload[47:40];
			5'd10: wdat = l_payload[39:32];
			5'd11: wdat = l_payload[31:24];
			5'd12: wdat = l_payload[23:16];
			5'd13: wdat = l_payload[15:8];
			5'd14: wdat = l_payload[7:0];
			5'd15: wdat = l_ts[31:24];
			5'd16: wdat = l_ts[23:16];
			5'd17: wdat = l_ts[15:8];
			5'd18: wdat = l_ts[7:0];
			5'd23: wdat = 8'h55;
			default: wdat = 8'h00;        // [19..22] reserved
		endcase
	end

	// The event that fires the trigger must itself be recorded -- for a
	// triggered capture it is the single most interesting event in the
	// buffer.  `recording` is set by a non-blocking assignment, so it is
	// still 0 during the cycle the trigger matches; including trigger_now
	// here is what stops that event being dropped.
	wire trigger_now = armed & e_strobe & trig_hit;
	wire capturing   = recording | streaming | trigger_now;
	wire buf_full    = (wr_rec >= N_RECORDS[ADDR_W-1:0]);

	always @(posedge clk) begin
		if (rst) begin
			armed <= 0; recording <= 0; streaming <= 0; done <= 0;
			dropped <= 0; packing <= 0; pk <= 0;
			wr_rec <= 0; wr_off <= 0; rd_ptr <= 0; l_dropped <= 0;
		end else begin
			// -- mode control ---------------------------------------
			if (stop) begin
				armed <= 0; recording <= 0; streaming <= 0;
			end else if (arm_record) begin
				armed <= 1; recording <= 0; streaming <= 0;
				done  <= 0; dropped <= 0;
				wr_rec <= 0; wr_off <= 0; rd_ptr <= 0;
			end else if (start_stream) begin
				armed <= 0; recording <= 0; streaming <= 1;
				done  <= 0; dropped <= 0;
				wr_rec <= 0; wr_off <= 0; rd_ptr <= 0;
			end else if (trigger_now) begin
				// The triggering event is recorded too -- see trigger_now.
				armed <= 0; recording <= 1;
			end

			// -- accept an event ------------------------------------
			if (e_strobe && capturing && !buf_full) begin
				if (packing) begin
					dropped   <= 1'b1;         // lost: packer still busy
				end else begin
					packing   <= 1'b1;
					pk        <= 5'd0;
					l_proto   <= e_proto;
					l_flags   <= e_flags;
					l_aux     <= e_aux;
					l_len     <= e_len;
					l_payload <= e_payload;
					l_ts      <= e_ts;
					l_dropped <= dropped;
					wr_off    <= 5'd0;
				end
			end else if (e_strobe && capturing && buf_full) begin
				dropped <= 1'b1;
			end

			// -- clock the record out into RAM -----------------------
			if (packing) begin
				mem[wr_addr] <= wdat;
				wr_off <= wr_off + 5'd1;
				if (pk == 5'd23) begin
					packing <= 1'b0;
					wr_rec  <= wr_rec + 1'b1;
					if (wr_rec + 1'b1 >= N_RECORDS[ADDR_W-1:0]) begin
						// Recording stops when the buffer fills; streaming
						// keeps running and simply reports drops.
						if (recording) begin recording <= 0; done <= 1'b1; end
					end
				end else begin
					pk <= pk + 5'd1;
				end
			end

			// -- drain ----------------------------------------------
			if (drain_advance) rd_ptr <= rd_ptr + 1'b1;
			drain_data <= mem[rd_ptr];
		end
	end
endmodule
