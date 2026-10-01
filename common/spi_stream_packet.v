`ifndef SIGNALBENCH_SPI_STREAM_PACKET_V
`define SIGNALBENCH_SPI_STREAM_PACKET_V

// Snapshot a queued batch on 53 42 52 <limit>; return 53 42 44 <count> + data.
// Idle TX bytes never consume the source FIFO.
module spi_stream_packet #(
    parameter integer ENABLE_MODE_SELECT = 0
) (
    input wire clk,
    input wire rx_valid,
    input wire [7:0] rx_byte,
    input wire tx_accept,
    output wire [7:0] tx_byte,
    input wire [6:0] fifo_count,
    input wire [7:0] fifo_head,
    output reg fifo_pop = 0,
    output reg mode_switch = 0,
    output reg [7:0] mode = 8'h01
);
    localparam [7:0] MAX_PAYLOAD = 8'd58;

    reg [1:0] command_pos = 0;
    reg [7:0] command_opcode = 0;
    reg [31:0] header = 0;
    reg [2:0] header_remaining = 0;
    reg [7:0] payload_remaining = 0;

    wire [7:0] requested_count = rx_byte < MAX_PAYLOAD ? rx_byte : MAX_PAYLOAD;
    wire [7:0] available_count = {1'b0, fifo_count};
    wire [7:0] batch_count = requested_count < available_count
                           ? requested_count : available_count;

    assign tx_byte = header_remaining != 0 ? header[31:24]
                   : payload_remaining != 0 ? fifo_head : 8'hFF;

    always @(posedge clk) begin
        fifo_pop <= 0;
        mode_switch <= 0;

        if (tx_accept) begin
            if (header_remaining != 0) begin
                header <= {header[23:0], 8'h00};
                header_remaining <= header_remaining - 1'b1;
            end else if (payload_remaining != 0) begin
                fifo_pop <= 1;
                payload_remaining <= payload_remaining - 1'b1;
            end
        end

        if (rx_valid) begin
            case (command_pos)
                0: if (rx_byte == 8'h53) command_pos <= 1;
                1: command_pos <= rx_byte == 8'h42 ? 2 : rx_byte == 8'h53 ? 1 : 0;
                2: begin
                    if (rx_byte == 8'h52 || rx_byte == 8'hA5 || rx_byte == 8'h4D) begin
                        command_opcode <= rx_byte;
                        command_pos <= 3;
                    end else begin
                        command_pos <= rx_byte == 8'h53 ? 1 : 0;
                    end
                end
                3: begin
                    command_pos <= 0;
                    if (command_opcode == 8'h52) begin
                        header <= {8'h53, 8'h42, 8'h44, batch_count};
                        header_remaining <= 4;
                        payload_remaining <= batch_count;
                    end else if (command_opcode == 8'hA5 && rx_byte == 8'h5A) begin
                        header <= 32'h53424F4B;
                        header_remaining <= 4;
                        payload_remaining <= 0;
                    end else if (command_opcode == 8'h4D && ENABLE_MODE_SELECT &&
                                 (rx_byte == 8'h01 || rx_byte == 8'h02)) begin
                        mode <= rx_byte;
                        mode_switch <= 1;
                        header <= {8'h53, 8'h42, 8'h41, rx_byte};
                        header_remaining <= 4;
                        payload_remaining <= 0;
                    end
                end
            endcase
        end
    end
endmodule

`endif
