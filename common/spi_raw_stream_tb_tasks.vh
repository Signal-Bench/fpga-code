// Shared raw host transfers: no command transaction, header, or byte filtering.
reg [7:0] burst [0:63];
reg commands_on_mosi = 0;
localparam [95:0] OLD_COMMANDS = 96'h5342523A5342A55A53424D02;

task spi_burst(input integer n);
    integer i, b;
    reg [7:0] rx, tx;
    begin
        spi_cs = 1'b0;
        #(HALF);
        for (i = 0; i < n; i = i + 1) begin
            rx = 0;
            tx = commands_on_mosi ? OLD_COMMANDS[8*(11-(i % 12)) +: 8] : 8'h00;
            for (b = 7; b >= 0; b = b - 1) begin
                spi_mosi = tx[b];
                spi_sck = 1'b1;
                #1;
                rx[b] = spi_miso;
                #(HALF - 1);
                spi_sck = 1'b0;
                #(HALF);
            end
            burst[i] = rx;
        end
        spi_cs = 1'b1;
        spi_mosi = 1'b0;
        #(HALF);
    end
endtask

task check_empty_stream(input integer n);
    integer i;
    begin
        for (i = 0; i < n; i = i + 1)
            if (burst[i] !== 8'hFF)
                $fatal(1, "empty raw stream byte %0d = %02x, expected FF", i, burst[i]);
    end
endtask

task check_binary_stream;
    integer i;
    reg [7:0] binary_data [0:7];
    begin
        force dut.tick = 1'b0;
        #1000;
        spi_burst(64);
        spi_burst(64);
        check_empty_stream(64);

        binary_data[0] = 8'h00;
        binary_data[1] = 8'hFF;
        binary_data[2] = 8'h2E;
        binary_data[3] = 8'h53;
        binary_data[4] = 8'h42;
        binary_data[5] = 8'h44;
        binary_data[6] = 8'hFE;
        binary_data[7] = 8'h0A;
        @(negedge dut.clk_core);
        dut.fifo_rd = 0;
        dut.fifo_wr = 8;
        dut.fifo_count = 8;
        for (i = 0; i < 8; i = i + 1)
            dut.fifo_mem[i] = binary_data[i];
        #1000;
        commands_on_mosi = 1;
        spi_burst(3);
        for (i = 0; i < 3; i = i + 1)
            if (burst[i] !== binary_data[i])
                $fatal(1, "raw binary byte %0d changed: %02x != %02x", i, burst[i], binary_data[i]);
        spi_burst(5);
        for (i = 0; i < 5; i = i + 1)
            if (burst[i] !== binary_data[i+3])
                $fatal(1, "raw binary byte %0d changed across CS", i+3);
        spi_burst(64);
        check_empty_stream(64);
        if (dut.fifo_count !== 0)
            $fatal(1, "raw FIFO did not drain exactly");
        release dut.tick;
        $display("  PASS: raw binary bytes, CS continuity, idle clocks, MOSI commands ignored");
    end
endtask
