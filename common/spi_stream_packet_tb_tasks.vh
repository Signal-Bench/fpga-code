// Shared host transactions and packet checks for both stream-image testbenches.
integer packet_len;

task spi_command(input [7:0] opcode, input [7:0] argument);
    integer i, b;
    reg [7:0] tx;
    begin
        spi_cs = 1'b0;
        #(HALF);
        for (i = 0; i < 4; i = i + 1) begin
            case (i)
                0: tx = 8'h53;
                1: tx = 8'h42;
                2: tx = opcode;
                default: tx = argument;
            endcase
            for (b = 7; b >= 0; b = b - 1) begin
                spi_mosi = tx[b];
                spi_sck = 1'b1;
                #(HALF);
                spi_sck = 1'b0;
                #(HALF);
            end
        end
        spi_cs = 1'b1;
        spi_mosi = 1'b0;
        #(HALF);
    end
endtask

task spi_packet_read(input [7:0] requested);
    integer i, start;
    begin
        spi_command(8'h52, requested);
        #20000;
        spi_wire_read(64);
        start = 0;
        while (start < 2 && burst[start] === 8'hFF)
            start = start + 1;
        if (burst[start] !== 8'h53 || burst[start+1] !== 8'h42 ||
            burst[start+2] !== 8'h44)
            $fatal(1, "invalid packet header: %02x %02x %02x %02x %02x %02x",
                   burst[0], burst[1], burst[2], burst[3], burst[4], burst[5]);
        packet_len = burst[start+3];
        if (packet_len > requested || packet_len > 58)
            $fatal(1, "invalid count %0d for request %0d", packet_len, requested);
        for (i = start + 4 + packet_len; i < 64; i = i + 1)
            if (burst[i] !== 8'hFF)
                $fatal(1, "non-idle trailing byte at %0d: %02x", i, burst[i]);
        for (i = 0; i < packet_len; i = i + 1)
            burst[i] = burst[start+4+i];
    end
endtask

task spi_burst(input integer n);
    begin
        spi_packet_read(n);
        if (packet_len != n)
            $fatal(1, "expected %0d queued bytes, received %0d", n, packet_len);
    end
endtask

task check_packet_edges;
    integer i, start;
    reg [7:0] binary_data [0:7];
    begin
        // Stop only the synthetic producer and seed a known binary capture.
        force dut.tick = 1'b0;
        repeat (4) @(negedge dut.clk_core);
        binary_data[0] = 8'h00;
        binary_data[1] = 8'hFF;
        binary_data[2] = 8'h2E;
        binary_data[3] = 8'h53;
        binary_data[4] = 8'h42;
        binary_data[5] = 8'h44;
        binary_data[6] = 8'hFE;
        binary_data[7] = 8'h0A;
        dut.fifo_rd = 0;
        dut.fifo_wr = 8;
        dut.fifo_count = 8;
        for (i = 0; i < 8; i = i + 1)
            dut.fifo_mem[i] = binary_data[i];

        spi_command(8'hA5, 8'h5A);
        #20000;
        spi_wire_read(16);
        start = 0;
        while (start < 2 && burst[start] === 8'hFF)
            start = start + 1;
        if (burst[start] !== 8'h53 || burst[start+1] !== 8'h42 ||
            burst[start+2] !== 8'h4F || burst[start+3] !== 8'h4B)
            $fatal(1, "missing ready ACK");
        if (dut.fifo_count !== 7'd8)
            $fatal(1, "ready command consumed queued data");

        spi_packet_read(58);
        if (packet_len != 8 || dut.fifo_count !== 7'd0)
            $fatal(1, "short queue was not drained exactly");
        for (i = 0; i < 8; i = i + 1)
            if (burst[i] !== binary_data[i])
                $fatal(1, "binary byte %0d changed: %02x != %02x", i, burst[i], binary_data[i]);
        spi_packet_read(58);
        if (packet_len != 0 || dut.fifo_count !== 7'd0)
            $fatal(1, "empty poll consumed data or returned nonzero count");

        @(negedge dut.clk_core);
        dut.fifo_rd = 0;
        dut.fifo_wr = 0;
        dut.fifo_count = 64;
        for (i = 0; i < 64; i = i + 1)
            dut.fifo_mem[i] = i;
        spi_packet_read(0);
        if (packet_len != 0 || dut.fifo_count !== 7'd64)
            $fatal(1, "zero request consumed data");
        spi_packet_read(255);
        if (packet_len != 58 || dut.fifo_count !== 7'd6)
            $fatal(1, "maximum request did not cap at 58");
        for (i = 0; i < 58; i = i + 1)
            if (burst[i] !== i)
                $fatal(1, "full FIFO byte %0d changed", i);
        spi_packet_read(58);
        if (packet_len != 6 || dut.fifo_count !== 7'd0)
            $fatal(1, "FIFO remainder count incorrect");
        for (i = 0; i < 6; i = i + 1)
            if (burst[i] !== i + 58)
                $fatal(1, "FIFO remainder byte %0d changed", i);
        spi_packet_read(58);
        if (packet_len != 0)
            $fatal(1, "expected empty FIFO after draining");

        // An arrival after the command must wait for the next batch.
        @(negedge dut.clk_core);
        dut.fifo_rd = 0;
        dut.fifo_wr = 1;
        dut.fifo_count = 1;
        dut.fifo_mem[0] = 8'hA6;
        spi_command(8'h52, 8'd58);
        #2000;
        @(negedge dut.clk_core);
        force dut.tick = 1'b1;
        @(negedge dut.clk_core);
        force dut.tick = 1'b0;
        #18000;
        spi_wire_read(64);
        start = 0;
        while (start < 2 && burst[start] === 8'hFF)
            start = start + 1;
        if (burst[start] !== 8'h53 || burst[start+1] !== 8'h42 ||
            burst[start+2] !== 8'h44 || burst[start+3] !== 8'h01 ||
            burst[start+4] !== 8'hA6 || dut.fifo_count !== 7'd1)
            $fatal(1, "packet snapshot invalid: header=%02x %02x %02x count=%0d data=%02x queued=%0d",
                   burst[start], burst[start+1], burst[start+2], burst[start+3],
                   burst[start+4], dut.fifo_count);
        for (i = start + 5; i < 64; i = i + 1)
            if (burst[i] !== 8'hFF)
                $fatal(1, "new arrival leaked into reserved payload");
        release dut.tick;
        $display("  PASS: empty/short/full packets, snapshot, request limits, ACK isolation, binary bytes");
    end
endtask
