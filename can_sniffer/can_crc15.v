/** \file
 * CAN CRC-15 generator.
 *
 * Implements the shift register given verbatim in CAN 2.0B Part A p13 /
 * ISO 11898-1:2015, generator polynomial
 *
 *   x^15 + x^14 + x^10 + x^8 + x^7 + x^4 + x^3 + 1   ==  0x4599
 *
 *   CRC_RG = 0
 *   REPEAT
 *     CRCNXT      = NXTBIT EXOR CRC_RG(14)
 *     CRC_RG(14:1) = CRC_RG(13:0)
 *     CRC_RG(0)    = 0
 *     IF CRCNXT THEN CRC_RG = CRC_RG EXOR 0x4599
 *
 * Fed with the DESTUFFED bit stream from SOF through the end of the DATA
 * field.  It does not cover the CRC sequence itself -- note this is a
 * different span from the one bit stuffing covers (SOF through the end of the
 * CRC sequence), which is easy to conflate.
 *
 * One bit per bit time, so at 12 core clocks per bit this is doing nothing
 * for 11 of every 12 cycles.  There is no hard CRC block on the UP5K and no
 * reason to want one.
 */
module can_crc15 (
	input  wire        clk,
	input  wire        clear,    // synchronous reset of the register
	input  wire        en,       // consume bit_in this cycle
	input  wire        bit_in,
	output reg  [14:0] crc
);
	localparam [14:0] POLY = 15'h4599;

	always @(posedge clk) begin
		if (clear)
			crc <= 15'd0;
		else if (en) begin
			if (bit_in ^ crc[14]) crc <= {crc[13:0], 1'b0} ^ POLY;
			else                  crc <= {crc[13:0], 1'b0};
		end
	end
endmodule
