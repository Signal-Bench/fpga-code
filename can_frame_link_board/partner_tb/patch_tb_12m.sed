# patch_tb_12m.sed -- adapt the partner's spi_frame_stream_tb.v (unmodified copy:
# spi_frame_stream_tb_orig.v) to the 12 MHz SignalBench clocking and to the
# shipped spi_frame_link hierarchy.  Every change is a mechanical rescale:
#   * the link core is 12 MHz, so every SCK rate is halved (3 MHz ceiling -> 1.5 MHz)
#     and the fake sensor rate is halved (1 Mbps -> 0.5 Mbps) to keep the same
#     link-capacity / source-rate ratio the partner tested;
#   * internal probes move from dut.* to dut.u_link.*;
#   * the write-side memory monitor checks wr_ptr (the slot actually being written,
#     since wr_off now only advances by whole records);
#   * RECORD_SAMPLES is passed through so the same suite runs with atomic records.
s/localparam integer SAMPLE_RATE_HZ = 62_500;.*/localparam integer SAMPLE_RATE_HZ = 31_250;      \/\/ 12 MHz core: half the partner's 1 Mbps/
s/parameter \[31:0\]  INITIAL_OFFSET = 32'd0;/parameter [31:0]  INITIAL_OFFSET = 32'd0;\n\tparameter integer RECORD_SAMPLES = 1;/
s/\.INITIAL_OFFSET(INITIAL_OFFSET)$/.INITIAL_OFFSET(INITIAL_OFFSET),\n\t\t.RECORD_SAMPLES(RECORD_SAMPLES)/
s/integer HALF   = 250;.*/integer HALF   = 500;              \/\/ ns, half an SCK period: 1 MHz (12 MHz core)/
s/0: HALF = 1250;.*/0: HALF = 1250;                   \/\/ 400 kHz/
s/1: HALF = 500; .*/1: HALF = 833;                    \/\/ 600 kHz/
s/2: HALF = 417; .*/2: HALF = 667;                    \/\/ 750 kHz/
s/3: HALF = 333; .*/3: HALF = 500;                    \/\/ 1 MHz (recommended)/
s/4: HALF = 250; .*/4: HALF = 417;                    \/\/ 1.2 MHz/
s/5: HALF = 200; .*/5: HALF = 357;                    \/\/ 1.4 MHz/
s/6: HALF = 167; .*/6: HALF = 333;                    \/\/ 1.5 MHz, the 12 MHz ceiling/
s/^\t\tHALF = 250;$/\t\tHALF = 500;/
s/SCK sweep 400 kHz - 3 MHz/SCK sweep 400 kHz - 1.5 MHz/
s/link ceiling at 2 MHz/link ceiling at 1 MHz/
s/"1 Mbps"/"0.5 Mbps"/g
s/  1 Mbps sustained/  0.5 Mbps sustained/
s/dut\.wr_off, dut\.rd_off);/dut.u_link.wr_ptr, dut.u_link.rd_off);/
s/if (dut\.ram_we \&\& ((dut\.wr_off - dut\.rd_off) >= CAPACITY))/if (dut.u_link.ram_we \&\& ((dut.u_link.wr_ptr - dut.u_link.rd_off) >= CAPACITY))/
s/dut\.\(clk\|ram_re\|ram_we\|fetch_off\|rd_off\|wr_off\|dec\|DEC_BUILD\|count_q\|level\|dropped\|crc16_byte\)\b/dut.u_link.\1/g
