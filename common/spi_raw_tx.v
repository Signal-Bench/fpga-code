// Mode-0, MSB-first raw transmitter. SCK must be <= clk / 12; CS setup,
// hold and high time must each be >= 4 core clocks. MOSI is not interpreted.
module spi_raw_tx (
    input wire clk,
    input wire spi_sck,
    input wire spi_cs,
    input wire [7:0] data,
    input wire empty,
    output wire pop,
    output wire spi_miso
);
    // Two synchronizer stages plus edge history; all FIFO logic stays in clk.
    (* async_reg = "true" *) reg [2:0] sck_sync = 3'b000;
    (* async_reg = "true" *) reg [2:0] cs_sync = 3'b111;
    wire sck_rise = (sck_sync[2:1] == 2'b01);
    wire sck_fall = (sck_sync[2:1] == 2'b10);
    wire cs_start = (cs_sync[2:1] == 2'b10);
    wire selected = !cs_sync[1];

    reg [7:0] shift = 8'hFF;
    reg [2:0] bit_count = 0;
    reg byte_valid = 0;
    reg byte_complete = 0;

    // Commit at the final sampling edge, never when a byte is preloaded.
    assign pop = selected && !cs_start && sck_rise &&
                 (bit_count == 7) && byte_valid;
    assign spi_miso = spi_cs ? 1'b1 : shift[7];

    always @(posedge clk) begin
        sck_sync <= {sck_sync[1:0], spi_sck};
        cs_sync <= {cs_sync[1:0], spi_cs};

        if (!selected) begin
            shift <= 8'hFF;
            bit_count <= 0;
            byte_valid <= 0;
            byte_complete <= 0;
        end else if (cs_start) begin
            shift <= empty ? 8'hFF : data;
            bit_count <= 0;
            byte_valid <= !empty;
            byte_complete <= 0;
        end else if (sck_rise) begin
            bit_count <= bit_count + 1'b1;
            if (bit_count == 7)
                byte_complete <= 1;
        end else if (sck_fall) begin
            if (byte_complete) begin
                // New data after underflow waits here; it cannot splice into FF.
                shift <= empty ? 8'hFF : data;
                byte_valid <= !empty;
                byte_complete <= 0;
            end else begin
                shift <= {shift[6:0], 1'b1};
            end
        end
    end
endmodule
