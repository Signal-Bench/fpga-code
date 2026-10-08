// Shared raw host transfers: no command transaction, header, or byte filtering.
reg [7:0] burst [0:63];
reg commands_on_mosi = 0;
localparam [95:0] OLD_COMMANDS = 96'h5342523A5342A55A53424D02;
integer half_period = HALF;
integer phase_delay = 0;
initial begin
    if ($value$plusargs("half=%d", half_period)) begin end
    if ($value$plusargs("phase=%d", phase_delay)) begin end
end

task spi_burst(input integer n);
    integer i, b;
    reg [7:0] rx, tx;
    begin
        #(phase_delay);
        spi_cs = 1'b0;
        #(2 * half_period);
        for (i = 0; i < n; i = i + 1) begin
            rx = 0;
            tx = commands_on_mosi ? OLD_COMMANDS[8*(11-(i % 12)) +: 8] : 8'h00;
            for (b = 7; b >= 0; b = b - 1) begin
                spi_mosi = tx[b];
                spi_sck = 1'b1;
                #1;
                rx[b] = spi_miso;
                #(half_period - 1);
                spi_sck = 1'b0;
                #(half_period);
            end
            burst[i] = rx;
        end
        spi_cs = 1'b1;
        spi_mosi = 1'b0;
        #(2 * half_period);
    end
endtask

task inject_byte(input [7:0] value);
    begin
        @(negedge dut.clk_core);
        dut.fifo_rd = 0;
        dut.fifo_wr = 1;
        dut.fifo_mem[0] = value;
        dut.fifo_count = 1;
    end
endtask

task check_poll_stream(input counter_pattern);
    integer poll, i, received, msg_pos;
    reg [7:0] expected;
    reg [95:0] message;
    begin
        force dut.tick = 0;
        #1000;
        spi_burst(64);
        spi_burst(64);
        check_empty_stream(64);
        reset_pattern;
        expected = 8'h20;
        msg_pos = 0;
        message = "Hello World!";
        received = 0;
        release dut.tick;
        for (poll = 0; poll < 16; poll = poll + 1) begin
            #10_000_000;
            spi_burst(64);
            for (i = 0; i < 64; i = i + 1) begin
                if (burst[i] !== 8'hFF) begin
                    if (!counter_pattern)
                        expected = message[8*(11-msg_pos) +: 8];
                    if (burst[i] !== expected)
                        $fatal(1, "poll %0d byte %0d: %02x != %02x", poll, i, burst[i], expected);
                    if (counter_pattern)
                        expected = (expected == 8'h7E) ? 8'h0A :
                                   (expected == 8'h0A) ? 8'h20 : expected + 1'b1;
                    msg_pos = (msg_pos == 11) ? 0 : msg_pos + 1;
                    received = received + 1;
                end
            end
        end
        if (received < 450)
            $fatal(1, "polling lost source data: only %0d bytes", received);
        $display("  PASS: 64-byte polling, idle FF and %0d consecutive source bytes", received);
    end
endtask

task check_byte_boundaries;
    integer bits, b, i, block;
    reg [7:0] rx;
    begin
        force dut.tick = 1'b0;
        #1000;
        spi_burst(64);
        spi_burst(64);
        check_empty_stream(64);

        // Every binary value must survive, including FF as real queued data.
        for (block = 0; block < 4; block = block + 1) begin
            @(negedge dut.clk_core);
            dut.fifo_rd = 0;
            dut.fifo_wr = 0;
            dut.fifo_count = 64;
            for (i = 0; i < 64; i = i + 1)
                dut.fifo_mem[i] = block * 64 + i;
            spi_burst(64);
            for (i = 0; i < 64; i = i + 1)
                if (burst[i] !== (block * 64 + i))
                    $fatal(1, "binary value %0d changed to %02x", block * 64 + i, burst[i]);
            if (dut.fifo_count !== 0)
                $fatal(1, "binary FIFO did not drain exactly");
        end

        // CS after 1..7 sample edges must leave the partially sent byte queued.
        // CS after edge 8, before its trailing falling edge, commits exactly once.
        for (bits = 1; bits <= 8; bits = bits + 1) begin
            inject_byte(8'hA5);
            spi_cs = 0;
            #(2 * half_period);
            for (b = 0; b < bits; b = b + 1) begin
                spi_sck = 1;
                #(half_period);
                if (b != bits - 1) begin
                    spi_sck = 0;
                    #(half_period);
                end
            end
            spi_cs = 1;
            #(2 * half_period);
            spi_sck = 0;
            #(2 * half_period);
            if (dut.fifo_count !== ((bits == 8) ? 0 : 1))
                $fatal(1, "CS abort after %0d bits changed FIFO incorrectly", bits);
            spi_burst(1);
            if (burst[0] !== ((bits == 8) ? 8'hFF : 8'hA5))
                $fatal(1, "CS abort after %0d bits did not restart the right byte", bits);
        end

        // Data arriving during idle FF cannot enter a byte already in progress.
        for (bits = 1; bits <= 7; bits = bits + 1) begin
            spi_cs = 0;
            #(2 * half_period);
            rx = 0;
            for (b = 7; b >= 0; b = b - 1) begin
                spi_sck = 1;
                #1;
                rx[b] = spi_miso;
                #(half_period - 1);
                spi_sck = 0;
                #(half_period);
                if (b == 8 - bits)
                    inject_byte(8'h96);
            end
            if (rx !== 8'hFF)
                $fatal(1, "late data spliced into idle byte after bit %0d: %02x", bits, rx);
            rx = 0;
            for (b = 7; b >= 0; b = b - 1) begin
                spi_sck = 1;
                #1;
                rx[b] = spi_miso;
                #(half_period - 1);
                spi_sck = 0;
                #(half_period);
            end
            spi_cs = 1;
            #(2 * half_period);
            if (rx !== 8'h96 || dut.fifo_count !== 0)
                $fatal(1, "late data not sent exactly once at next boundary: %02x", rx);
        end

        // Repeated one-byte transactions may not consume a prefetched next byte.
        @(negedge dut.clk_core);
        dut.fifo_rd = 0;
        dut.fifo_wr = 0;
        dut.fifo_count = 64;
        for (i = 0; i < 64; i = i + 1)
            dut.fifo_mem[i] = i;
        for (i = 0; i < 64; i = i + 1) begin
            spi_burst(1);
            if (burst[0] !== i)
                $fatal(1, "one-byte transaction skipped byte %0d: %02x", i, burst[0]);
        end
        spi_burst(64);
        check_empty_stream(64);
        release dut.tick;
        $display("  PASS: all binary values, partial CS aborts, late data, one-byte CS boundaries");
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
