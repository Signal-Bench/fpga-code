# can_frame_link_board: CAN + 4 × UART + probe bank (UART/counter, DIO-8, SPI, I2C) → framed SPI link (SignalBench PCBA)

This is the iCE40UP5K-SG48 (U7) bitstream for the SignalBench PCBA. It decodes every receiver on the board and the
eight logic probes, and streams the decoded records to the ESP32-H2 over the partners' framed, acked, CRC-16 SPI link.
Sources of truth: `validation/battery_can_chain/CONTRACT.md` (§1 pins and the CAN record, §4/§5 UART, §6 baud, §7.1
PROBE1 UART3 + PROBE2 edge counter) and, for **protocol v3** (probe modes, I2C/SPI/DIO records, the C3 config block,
runtime baud), [`../docs/PROTOCOL_V3_CONTRACT.md`](../docs/PROTOCOL_V3_CONTRACT.md) §1–§3. See the
[Protocol v3](#protocol-v3) section.

```
can_rx   27 ─ can_bit_timing ─ can_frame_fsm ─ 20-byte CAN record ─┐
uart0    23 ─ uart_rx_sniff (RS-232 P1.7)     ─  8-byte UART record ┤  one holding slot per source
uart1    25 ─ uart_rx_sniff (RS-232 P1.8)     ─  8-byte UART record ┤  oldest-first arbiter
uart2    26 ─ uart_rx_sniff (RS-485 P1.3/4)   ─  8-byte UART record ┤  (never splits a record)
PROBE1..8 (32 35 31 37 34 43 36 42), one decoder chosen by probe_mode:
  mode 0: P1 uart_rx_sniff (chan 3)           ─  8-byte C5 record  ┤  PROBE slot
          P2 1 µs filter + edge counter /100 ms ─ 12-byte E7 record  ┤  COUNTER slot
  mode 1: P1..8 1 µs filters, change + 100 ms ─ 10-byte D1 record  ┤  PROBE slot
  mode 2: probe_spi  (SCK MOSI MISO CS = P1..4) ─ 10-byte 5B record  ┤  PROBE slot
  mode 3: probe_i2c  (SCL SDA = P1 P2)         ─  8-byte 1C record  ┘  PROBE slot
        → spi_frame_link: atomic variable-length push into 4 × SPRAM (64 Ki samples),
          logic SPI slave, A5 5A 01 frames, CRC-16/CCITT-FALSE, MCU ack → spi_miso 46
          + C3 config block (gen, probe_mode, spi_cfg, 4 UART divisors), echoed in MISO bytes 3-4
```

**Do not flash from here.** Charles flashes the board. `make build` only writes `can_frame_link_board.bin`.

## Files

| File | What |
|---|---|
| `can_bit_timing.v`, `can_crc15.v`, `can_frame_fsm.v` | partner CAN decoder, **unmodified** (see `SOURCE_COMMIT`) |
| `spi_frame_link.v` | partner `spi_frame_stream.v` link. Original header kept; the changes are listed in its header (6-7: C3 config block, echo) |
| `uart_rx_sniff.v` | new: passive 8-N-1 receiver, 16× NCO, majority vote, framing/break detection (4 instances); bit period = runtime divisor (v3) |
| `probe_decoders.v` | new (v3): `probe_i2c` and `probe_spi`, oversampled at 12 MHz |
| `can_frame_link.v` | new top (`module top`): pads, POR, µs timebase, probe bank (mode mux, 8 glitch filters, PROBE2 edge counter + heartbeat, DIO-8), record packing, 6-slot arbiter, link |
| `can_frame_link_board.pcf` | board pins exactly per contract; only `spi_miso` is an output |
| `can_frame_link_tb.v` | end-to-end testbench (CAN + 4 UARTs incl. Pi packets on UART3 + PROBE2 edge/glitch source with its own counter model + I2C master, SPI master, DIO source + ESP model incl. C3 config blocks) |
| `v3_records.txt` | protocol v3 golden records (contract §6): every record the `small` run delivered, in stream order, `R <hex bytes>`, `# ` scenario lines. Also at `validation/vectors/v3_records.txt` |
| `ice40_sim_models.v` | sim-only `SB_IO` / `SB_SPRAM256KA` stand-ins (see the Verification section) |
| `partner_tb/` | partner testbenches (unmodified copies), the 24 MHz original for baseline, the 12 MHz wrapper and sed rescale |
| `frames_vectors.txt` | real MISO frames from the top-level sim (partner `frames.txt` format) for the ESP host test |
| `frames_vectors_records.txt` | the records the vectors run delivers (all phases), in order: `R <src> <index> <bytes…>`. **src: 0 CAN, 1 UART0 (P1.7), 2 UART1 (P1.8), 3 UART2 (RS-485), 4 UART3 (PROBE1, P1.9), 5 COUNTER (PROBE2, P1.10)**; index = that source's record number |
| `frames_vectors_esp.txt` | ESP `test_link_records` replay format (unchanged line format): `P` = payload bytes as the MCU accepts them; `C`/`U`/`E`/`I`/`S`/`D` = records as the DUT pushed them, captured at the SPRAM write port. `C` = 20-byte CAN, `U` = 8-byte UART (incl. chan 3), `E` = 12-byte edge-counter record, **`I` = 8-byte I2C, `S` = 10-byte SPI, `D` = 10-byte DIO (v3 letters)**. `frames_vectors_records.txt` src numbers add 6 I2C, 7 SPI, 8 DIO |
| `can_frame_link_board.bin` | the bitstream (git-ignored, `make build`): reset config UART0-2 9600, UART3 115200, probe mode 0; CAN 1 Mbit/s, counter record every 100 ms |

```
source ~/opt/oss-cad-suite/environment
make build        # yosys + nextpnr (12 MHz) + icepack + icetime -> out/, can_frame_link_board.bin
make sim-partner  # partner TBs: bit timing (+ tolerance sweep), frame FSM, spi_frame_stream baseline + 12 MHz x3
make sim-top      # end-to-end, three configurations (make -j3 sim-top: in parallel); fails unless each prints PASS
make vectors      # frames_vectors*.txt, plus v3_records.txt from the small run (V3_DEST=<path> copies it too)
```

## Pins (CONTRACT §1/§4/§5/§7.1)

| Port | Pin | Net | Dir |
|---|---|---|---|
| clk_12m | 44 | IOB_3b_G6 ← Y1 12 MHz via U4 | in |
| can_rx_pin | 27 | FPGA_CAN_RX ← U15 RXD | in, pull-up |
| uart0_pin | 23 | FPGA_RS232_RX_OUT ← U13 ROUT1 (P1.7) | in, pull-up |
| uart1_pin | 25 | FPGA_RS232_TX_OUT ← U13 ROUT2 (P1.8) | in, pull-up |
| uart2_pin | 26 | FPGA_RS485_RX ← U14 RO (P1.3/4) | in, pull-up |
| probe1_pin | 32 | IOT_43a, IDC1_BUF ← U19 ← P1.9 PROBE1 (Pi GPIO4, UART2 TX) | in, **no pull**: m0 UART3, m1 DIO0, m2 SCK, m3 SCL |
| probe2_pin | 35 | IOT_46b_G0, IDC2_BUF ← U19 ← P1.10 PROBE2 (Pi GPIO5) | in, **no pull**: m0 edge counter, m1 DIO1, m2 MOSI, m3 SDA |
| probe3_pin | 31 | IOT_42b, IDC3_BUF ← U19 ← P1.11 PROBE3 | in, **no pull**: m1 DIO2, m2 MISO |
| probe4_pin | 37 | IOT_45a_G1, IDC4_BUF ← U19 ← P1.12 PROBE4 | in, **no pull**: m1 DIO3, m2 CS (active low) |
| probe5_pin | 34 | IOT_44b, IDC5_BUF ← U19 ← P1.13 PROBE5 | in, **no pull**: m1 DIO4 |
| probe6_pin | 43 | IOT_49a, IDC6_BUF ← U19 ← P1.14 PROBE6 | in, **no pull**: m1 DIO5 |
| probe7_pin | 36 | IOT_48b, IDC7_BUF ← U19 ← P1.15 PROBE7 | in, **no pull**: m1 DIO6 |
| probe8_pin | 42 | IOT_51a, IDC8_BUF ← U19 ← P1.16 PROBE8 | in, **no pull**: m1 DIO7 |
| spi_sck | 20 | RT_SCK ← ESP GPIO10 | in |
| spi_cs | 47 | RT_CSn ← ESP GPIO27 | in |
| spi_mosi | 48 | FPGA_IRQn ← ESP GPIO22 (F1 rework) | in |
| spi_miso | 46 | RT_MISO → ESP GPIO11 | **out, only while CS is low** |

No other pin is used. The design does not touch pin 16 (FT_SSn), pin 18 (RT_MOSI), flash WP#/HOLD# or RGB0–2.
The input pull-ups hold each receiver line at its idle level (recessive / mark) while the transceivers power up.
The eight probe pins have the pad pull-up **off**: U19 (SN74LVC8T245, A→B, hard-wired) drives them push-pull, and
the probe path has its own 10 k pull-down on the A side (CONTRACT §7). Pins 35 and 37 are global-capable pads (G0,
G1), used here as plain inputs; nextpnr does not promote them.
MISO's output enable comes straight from the CS pad, so the first bit is valid as soon as CS falls, and the pin is
tri-stated whenever CS is high, as `FPGA_PINOUT_ICE40UP5K-SG48.md` asks.

## Decision 1: clocking — one 12 MHz domain from pin 44

**Choice:** everything (CAN, UART, SPI link) runs on the 12 MHz board oscillator, with no PLL, no HFOSC and no CDC.

* **CAN needs it.** The ISO 11898-1 budget for this 12 tq/bit, SJW=3 configuration is df < 0.98 %
  (`clock_choice.md`). SB_HFOSC is ±10 % and fails it about 10×. Y1 is ±20 ppm. With BRP = 0 (a `top`
  parameter), tq = 1 clock and the bus runs at 1 Mbit/s; rate = 1 MHz/(BRP+1). The partner decoder was designed and
  bench-tested at exactly this clock, so it is reused unchanged.
* **The SPI link at 12 MHz.** `spi_frame_stream` samples SCK/CS/MOSI through 2-flop synchronisers, so MISO changes
  2–3 core clocks after the falling SCK edge, which is **167–250 ns at 12 MHz**. That sets the SCK ceiling at
  **1.5 MHz** (83 ns setup), half the original 3 MHz at 24 MHz. A byte at 1.5 MHz/12 MHz is 64 clocks, the same
  ratio the partner pipeline was designed for at 3 MHz/24 MHz. Every pipeline argument in the original header still
  holds; the partner's own testbench passes at 12 MHz with the rates halved (see the Verification section).
* **Rejected: PLL 12 → 24 MHz.** On the SG48 package, SB_PLL40_PAD can only take its reference from pin 35, so pin 44 (G6)
  would need SB_PLL40_CORE fed from the global net. That is legal in nextpnr, but it adds a second clock relationship.
  CAN would then have to run at BRP = 1 off the PLL output, which ties CAN timing to the PLL's lock and jitter for no gain.
  The only benefit would be 2–3 MHz SCK, and the capacity numbers below show it is not needed.
* **Rejected: CDC FIFO** between a 12 MHz decoder domain and a 24 MHz link domain. It is the most logic and the most
  verification for the same benefit as the PLL.
* **Simplest correct wins:** one domain, and every reused module runs at a clock it has been verified at.

**Recommended ESP setting: SCK = 1 MHz** (≥ 240 ns MISO setup after the 250 ns worst case and pad delay).
1.5 MHz is the absolute ceiling and passes in simulation. On hardware, ESP input delay and pad delay are unmeasured, so do
not use it [HARDWARE-VERIFY]. **The ESP firmware default today is `CONFIG_SIGNALBENCH_FPGA_SPI_CLOCK_HZ = 2000000`,
which is over the ceiling for this bitstream and must be lowered to 1000000.** Over-speed is safe but useless: the
prep pipeline poisons the CRC and the MCU gets resends, never wrong data, but at 2 MHz the MISO setup itself goes negative.

## Decision 2: throughput

Link capacity, with back-to-back reads and 24 bytes of overhead per frame:

| SCK | read_len | samples/frame | transfer | samples/s | CAN records/s (10 samples) | UART records/s (4 samples) |
|---|---|---|---|---|---|---|
| 1 MHz | 1024 | 500 | 8.19 ms | 61.0 k | 6,100 | 15,200 |
| **1 MHz** | **2048** | **1012** | **16.4 ms** | **61.7 k** | **6,170** | **15,400** |
| 1.5 MHz | 2048 | 1012 | 10.9 ms | 92.6 k | 9,260 | 23,100 |

(The ESP's inter-transaction gap takes a few % off these. Any `poll period` sleep between reads takes more; see below.)

Offered load:

| Source | Records/s | Samples/s |
|---|---|---|
| Partner bench bus (GM6020), ~2,000 frames/s | 2,000 | 20,000 |
| 3 × UART at 9600 8-N-1, all three saturated (960 B/s each) | 2,880 | 11,520 |
| **Bench worst case** | **4,880** | **31,500** = 51 % of 1 MHz capacity |
| Home run (§7): UART3 Pi packets back to back at 115200 (11,520 B/s) + counter 10/s | 11,530 | 46,140 = 75 % of 1 MHz / 2048 capacity |
| Saturated 1 Mbit/s CAN, 8-byte std frames (111 bits min, ~135 with worst stuffing) | 7,400–9,000 | 74–90 k |
| Saturated 1 Mbit/s CAN, DLC 0 std frames (47–55 bits) | 18,000–21,000 | 180–210 k |

At **SCK 1 MHz, read_len 2048** the link carries the bench worst case with about 2× margin. A fully saturated bus
exceeds any SCK this link supports. The buffer then absorbs bursts: 65,536 samples is 6,553 CAN records, about 2.3 s
at a saturated 8-byte bus draining at 6.2 k/s. Past that, records are dropped whole and counted. BLE downstream of the
ESP is very likely the tighter limit; that is not measured here.

**ESP read policy:** keep `read_len = 2048` (the current default). Read again immediately when a frame came back full
(`count` == (read_len−24)/2), and sleep the poll period only when it did not. With an unconditional 10 ms sleep after a
16.4 ms read, the capacity drops to about 38 k samples/s, which still covers the 31.5 k bench worst case, but with
little margin.

## Record stream: what the ESP has to know

* **Records:** CAN 20 B (0xAA…0x55), UART 8 B (0xC5…0x5A), edge-counter 12 B (0xE7…0x7E) and, protocol v3,
  I2C 8 B (0x1C…0xC1), SPI 10 B (0x5B…0xB5) and DIO 10 B (0xD1…0x1D), interleaved in **arrival order**. The
  frame's sample bytes, taken in order (each sample LE), are the record bytes in order. Dispatch on the first byte:
  0xAA = 20 bytes, 0xC5 / 0x1C = 8, 0x5B / 0xD1 = 10, 0xE7 = 12. Every record except CAN carries `dropped` in byte 1
  bit 3 (CAN: byte 6 bit 7). The v3 layouts are in the [Protocol v3](#protocol-v3) section. UART byte 1 =
  `{chan[1:0], framing_err, brk, dropped, 3'b000}`, chan 0–2 = RS-232 RX / RS-232 TX / RS-485, **chan 3 = PROBE1
  (P1.9)**.
* **Counter record (PROBE2, CONTRACT §7.1), every 100 ms:**
  `[0] 0xE7 [1] {level, 3'b000, dropped, 3'b000} [2..5] rise u32 BE [6..9] ts_us u32 BE [10] glitch u8 [11] 0x7E`.
  `ts_us` is exactly k × 100000 for the k-th record (same `us_ctr` as UART), so it doubles as the FPGA heartbeat; a
  gap of more than 100 000 µs between consecutive counter records means counter records were lost (the `dropped` bit
  on the next record of any type says so too).
* **Records may straddle frames.** `count` ends on a record boundary whenever it is limited by what is buffered (wr_off
  is always a boundary, because records are published whole). When `read_len` limits it, the last record continues in
  the next frame. The link is exactly-once and in order, so the ESP keeps the partial record's bytes and continues.
  Rounding `count` down to whole records is impossible without storing record boundaries, because the lengths differ.
  The marker scan is needed only after the ESP itself loses state (reboot): the FPGA then resumes at the last acked
  offset, which may be mid-record. After ACK_REJECTED (FPGA restarted), `start` is 0, a record boundary.
* **`dropped` in the frame header counts RECORDS (any type), not samples.** This is the one change of field
  meaning in the link protocol. With mixed record lengths a sample count could not be turned back into records, and
  dropped data never consumed offsets anyway. The wire format and every other rule in `spi_frame_stream/README.md`
  are unchanged. The per-record `dropped` bit means "at least one record of any type was lost before this one".
* **Timestamps:**
  * UART timestamps are microseconds since configuration, latched within 250 ns of the start-bit edge.
  * I2C, SPI and DIO timestamps are the same microsecond counter (see Protocol v3 for the instant each one marks).
  * CAN timestamps are the partner's **CAN bit times** since configuration, latched at SOF. That is 1 µs per bit at
    1 Mbit/s, but `can_bit_timing` restarts the bit at every hard sync, so it can lose up to one bit per frame against
    wall time. Over a long run, CAN and UART timestamps drift apart by up to the number of CAN frames seen, in µs.
  * If cross-type correlation matters, latching `us_ctr` for CAN as well is a one-line change in `can_frame_link.v`,
    but it changes the contract's definition, so it is not done here.

## UART receiver (CONTRACT §4/§5/§6)

* **Protocol v3: the baud is set at run time.** Each receiver takes its bit period `divN` = round(12 MHz / baud)
  clocks from the C3 config block (contract §3). Reset values: 1250 / 1250 / 1250 / 104 = 9600 / 9600 / 9600 /
  115200 baud (UART0..2, UART3 = PROBE1). Valid 16..65535 (750 000..184 baud); a block with any divN < 16 is ignored.
  (Until v3 these were `UARTn_BAUD` build parameters.)
* An NCO adds 16 per clock and wraps at divN, so ticks come at exactly 16 per bit on average for the configured
  divisor. The only rate error is the rounding of divN: 0 at 9600 (1250), +0.16 % at 115200 (104) and 230400 (52),
  +0.16 % at 57600 (208). At 9600 a tick comes every 78.125 clocks (78/79), a jitter of 0.08 % of a bit.
* A divisor change takes effect at the next tick; a byte in flight at that moment may be received wrongly.
* The start edge is detected at the clock rate, not the tick rate (a 9600 tick is 6.5 µs). The timestamp is latched
  and the NCO phase reset at that edge, so tick k is k/16 bit after it.
* Each bit is decided by a majority vote of ticks 16b+7, 16b+8 and 16b+9, centred on mid-bit. A start bit that does
  not vote low is a glitch and is ignored.
* A stop bit that votes low with nonzero data is a framing error. All-zero data with a low stop bit is a break: brk=1,
  framing_err=0, data 0x00. A break is classified once and not also counted as a framing error.
* After either, the receiver re-arms only when the line votes high again, so a break of any length (≥ 1.04 ms at
  9600) is exactly one record.
* The stop-bit centre falls 9.5 bits after the edge, which tolerates about ±4.5 % baud error. ±2 % is tested.

## PROBE2 edge counter (CONTRACT §7.1)

(Protocol v3: probe_mode 0 only. The filter below is channel 2 of the 8-channel probe filter bank; it is reset on
every mode change, and `rise` keeps counting across modes but only edges seen in mode 0.)

* Pin 35 → 2-flop synchroniser → glitch filter: the filtered level changes only after the synchronised input has
  differed from it for **12 consecutive clocks (1 µs)** (`GLITCH_CLKS`). A pulse shorter than about 11 clocks
  (≈ 0.9 µs at the pin) is always rejected. One of 11–12 clocks may go either way, depending on synchroniser phase.
* `rise` (u32) counts filtered rising edges since configuration and is never reset.
* `glitch` (u8) counts **rejected excursions**: each time the synchronised input leaves the filtered level and comes
  back before being accepted, that counts +1. So a glitch pulse (two raw transitions) counts once, and bounce before a
  real edge counts once per bounce. It saturates at 255 and restarts from 0 when a counter record is loaded into its
  holding slot. If the link buffer later drops that record, its glitch count is lost; the next record of any type
  carries `dropped`.
* The record is taken every `HB_PERIOD_US` = 100 000 µs of `us_ctr` (the same µs counter as UART), on the first clock
  of the new period, so `ts_us` = k × 100 000 exactly. Level, rise and glitch are sampled on that same clock.
* The pad pull-up is off (U19 drives the pin). After configuration (and on every mode change) the filter starts at the
  pin's current level, so a pin that is already high shows level 1 with `rise` = 0 (protocol v3; the 2026-10-11 build
  started at 0 and counted that as a rise).

## Arbiter

* Each source (CAN, UART0, UART1, UART2, PROBE, COUNTER) has one holding slot. The PROBE slot (formerly UART3) takes
  the record of whichever probe decoder probe_mode selects (UART3 8 B, DIO 10 B, SPI 10 B, I2C 8 B); its length is
  stored with the record, so a record loaded just before a mode change still goes out whole. COUNTER is used in
  mode 0 only. When the link is idle, the **oldest** occupied slot is granted. Age is tracked in a 6×6 matrix; ties
  within one clock go CAN, UART0, UART1, UART2, PROBE, COUNTER.
* The granted record is copied to an output buffer, so its slot frees immediately. The link then writes it in
  10 (CAN), 4 (UART, I2C), 5 (SPI, DIO) or 6 (COUNTER) clocks. Worst case, all six slots are served in
  10 + 3×4 + 5 + 6 = 33 clocks.
* A record waits in its slot for the record being written plus every older slot: at most the other five records,
  ≤ 28 clocks.
* UART needs a whole frame to refill (≥ 160 clocks at 750 kbaud, 1042 at UART3's 115200), SPI one byte (64 clocks at
  1.5 MHz SCK), I2C one byte (≥ 9 SCL periods), DIO ≥ 15 clocks (1 µs filter + 3-clock window; a probe toggling
  faster than ~1 MHz can overrun the PROBE slot, and the record is then dropped and counted), and COUNTER 1.2 M clocks.
  CAN can refill in one bit time (12 clocks at BRP=0), but only for two decoder records one bit apart (EOF bit 5,
  then bit 6). So a CAN slot can be overrun only if that corner case coincides with ≥ 1 older UART/COUNTER record
  waiting. This was already true of the 4-slot version (16-clock worst wait). It is not a realistic coincidence on
  this bench.
* If a slot is ever overrun, the new record is dropped and counted (`drop_add`, up to 6 per clock), never split. The
  testbench asserts this path never fires.

## Resources and timing (`make build`, nextpnr seed 1, protocol v3)

| Resource | Used | Device | % |
|---|---|---|---|
| Logic cells | 4351 | 5280 | 82 % (flashed 2026-10-11 build: 3675 = 69 %) |
| SPRAM (16 K × 16) | 4 | 4 | 100 % (the 64 Ki-sample ring) |
| EBR | 0 | 30 | 0 % |
| SB_IO | 17 | 39 | |
| Global buffers | 8 | 8 | 100 % (clock + auto-promoted enables/resets; no second clock) |
| PLL / DSP / HFOSC / SB_SPI | 0 | | |

Timing at the 12 MHz constraint: **nextpnr Fmax 17.91 MHz, icetime 18.11 MHz (55.21 ns path vs 83.3
ns), PASS with 1.49× margin.** Seed sweep after the review fixes: seeds 1-6 all place, 17.6-18.5 MHz.

**Synthesis option `-dffe_min_ce_use 4` (new for v3, in the Makefile).** Without it the v3 netlist has 230 control
sets (clock-enable / reset pairs; an iCE40 tile of 8 logic cells shares one), 144 of them with fewer than 8 FFs, and
nextpnr's placer failed to legalise on 11 of 12 seeds (HeAP, SA and static placers alike) at 82 % LC. With it yosys
builds small-fanout enables from LUTs instead: 156 control sets, and every seed tried places (before the review
fixes: 1-5 at both 4 and 8, 17.9-19.0 MHz; after: 1-6, 17.6-18.5 MHz). If a later change makes placement fail again, look at control sets before features.
The contract's fallback order for cutting features (SPI LSB-first, then the DIO glitch filter) was not needed.

## Protocol v3

Contract: [`../docs/PROTOCOL_V3_CONTRACT.md`](../docs/PROTOCOL_V3_CONTRACT.md) (copy of
`validation/PROTOCOL_V3_CONTRACT.md`, frozen interface; §1 probe modes, §2 records, §3 config block).

### Probe modes

| probe_mode | Probes | Records (slot) |
|---|---|---|
| 0 (reset) | PROBE1 = UART ch3 RX (div3), PROBE2 = edge counter | C5 chan 3 (PROBE), E7 every 100 ms (COUNTER) |
| 1 DIO-8 | PROBE1..8 = DIO bit 0..7 | D1 (PROBE) |
| 2 SPI | PROBE1 SCK, PROBE2 MOSI, PROBE3 MISO, PROBE4 CS (active low) | 5B (PROBE) |
| 3 I2C | PROBE1 SCL, PROBE2 SDA | 1C (PROBE) |

CAN, RS-232 ×2 and RS-485 are decoded in every mode. All probe pads share one 2-flop synchroniser plus a
previous-sample register (edges = "now ≠ before"); UART3 keeps its own synchroniser inside `uart_rx_sniff`.
Every decoder oversamples at the 12 MHz core clock; there is no probed-clock domain (all 8 SB_GB are in use).
**Mode change:** each decoder is held in reset whenever probe_mode is not its own, so a change always starts the new
decoder from idle; a half-received byte of the old mode is discarded. A **completed** record that the change
discards (an I2C/SPI byte still held for its follow-up event, an event the old decoder produced in the clock of the
change, a DIO change inside its coalescing window, or an SPI byte held when spi_cfg changes) is counted as a
dropped record: frame header `dropped` + 1 and `dropped` on the next record (`lost_rec` in `can_frame_link.v`).
The 8 glitch filters restart on every mode change **at the current pin level**, so re-entering mode 0 with PROBE2
high gives level 1 and no phantom rising edge; the counter's `rise` total is never reset (it counts filtered rising
edges seen in mode 0), its `glitch` count is cleared. `probe_spi` is also reset when spi_cfg changes. The probe UART
(like every `uart_rx_sniff`) accepts a start bit only after its line has been seen high once since reset, so
entering mode 0 with PROBE1 low gives no break / framing-error record.

**Deglitch (contract §2):** I2C SCL and SDA, and SPI SCK, CS, MOSI and MISO, each pass a 2-sample stability filter
(the level changes only when two consecutive samples, ~167 ns, agree). All lines of a decoder get the same filter,
so their timing relative to each other is unchanged. Single-sample crossings (ringing, a 60 ns / 40 ns double
crossing on an SCL edge) are ignored; SCK / SCL high and low times must be ≥ ~170 ns.

### Records (contract §2; byte 1 bit 3 = `dropped`, inserted at arbiter grant)

* **I2C `1C flags data ts[4 BE] C1`.** flags = {start, rstart, stop, data_valid, dropped, ack, timeout, partial}.
  START / STOP = SDA falling / rising while SCL is high (same synchronised sample). SDA is sampled at SCL rise but a
  bit is only *committed* at the following SCL fall, so the SCL rise that precedes every STOP and every repeated
  START is never taken as a data bit (the partner `sb_i2c` records one stray bit there). One record per byte
  (address byte included), ts = µs at the commit of the 9th (ACK) bit, ack = SDA low on it. The byte is held until
  the next bus event so `stop` / `timeout` can be set on it: it goes out at the next byte's first bit (no flag), at a
  repeated START (no flag; the next byte gets `rstart`), at STOP (`stop`) or at the timeout (`timeout`). A STOP or
  timeout with nothing held gives a record with data_valid = 0 (start/rstart set if it is the first event after a
  START). **Timeout:** inside a transaction, the bus released (SCL **and** SDA high) for 1 ms (`I2C_TIMEOUT_US`,
  2..1023); the transaction is closed, the next START reports `start`. SCL held low (clock stretching, any length)
  never times out. **Partial** (contract ruling): the transaction ended (STOP, timeout or repeated START) after 1..8
  committed bits but before the ACK clock: partial = 1, **data_valid = 0, ack = 0**, data = the bits received
  MSB-aligned. A STOP outside a transaction gives no record. Tested at 400 kHz and 1 MHz, START hold 200-700 ns,
  a 5 ms clock stretch and SCL double crossings.
* **SPI `5B flags mosi miso ts[4 BE] nbits B5`.** flags = {cs_start, cs_end, partial, data_valid, dropped, 0, 0, 0}.
  spi_cfg = {lsb_first, cpol, cpha}; the sampling edge is rising for modes 0/3 and falling for 1/2. MOSI/MISO are
  taken from the same filtered sample in which the SCK edge is first seen, where they are stable for half an SCK
  period minus the device's output delay: **supported to 1.5 MHz (contract), tested in all 4 modes, MSB and LSB
  first, at 500 kHz, 1.5 MHz and 2 MHz** (SCK high and low ≥ ~170 ns; 4 MHz or a 100 ns high pulse do not decode).
  The first sampling edge may follow CS fall immediately: an edge seen in the same clock as CS falling is bit 0
  (tested with CS 20, 60 and 100 ns before the first edge). After reset the decoder waits for CS high, so a mode
  change in mid-transaction does not decode its tail. A byte completes at its 8th sampling edge (ts) and is held
  until the next event: next byte's first edge (no flag) or CS rise (cs_end). CS rise with no byte held: a cut-short
  byte gives partial = 1, nbits = 1..7, bits MSB-aligned (MSB first) or LSB-aligned (LSB first); no clocks at all gives
  an end-only record (data_valid = 0, nbits = 0, cs_start = cs_end = 1). A sampling edge in the same 83 ns sample as
  CS rising still counts.
* **DIO `D1 flags levels changed ts[4 BE] 00 1D`.** flags = {0000, dropped, 00, heartbeat}. Each channel has the
  PROBE2 filter (12 stable clocks = 1 µs; pulses < ~0.9 µs never pass). When the filtered levels differ from the last
  record offered, a 3-clock window opens so channels that change together (seen up to ~250 ns apart) give **one**
  record; levels = filtered levels, changed = levels XOR the last record, ts = µs at the first accepted change minus
  1 (the filter delay), i.e. the microsecond of the pin edge to within ±1 µs. Heartbeat on each 100 ms boundary of
  `us_ctr`: heartbeat = 1, changed = 0, ts = k × 100000 when the PROBE slot is free (else as soon as it is). On entering
  mode 1 the filters start at the pin levels and the last-record levels at 0, so probes already high give an entry
  record at once with those bits in `changed`.
  `changed` is relative to the last record offered to the slot, even if the link then dropped it (the next record
  carries `dropped`).

### Throughput of the probe decoders

SPI is the one probe source that can outrun the link: every byte is a 5-sample record, so continuous traffic at
1.5 MHz SCK offers ~940 k samples/s against the link's 61.7 k samples/s at 1 MHz (Decision 2). The 64 Ki-sample
buffer then absorbs ~13 000 bytes (~70 ms of back-to-back bytes at 1.5 MHz); after that whole records are dropped,
counted in the frame header and flagged on the next record. Bursty SPI (sensor reads, flash commands) fits.
I2C at 400 kHz (4 samples per ~23 µs byte, ~175 k samples/s while a transfer runs) and DIO (5 samples per change) are
bursty in the same way. BLE downstream of the ESP is slower still (plan: ~16 kB/s measured).

### C3 config block (contract §3; `spi_frame_link.v` changes 6-7)

`C3 gen probe_mode spi_cfg div0 div1 div2 div3 crc16` — 14 bytes in its own CS transaction, u16 fields LE,
CRC-16/CCITT-FALSE over bytes 0..11 sent big-endian. Applied when byte 13 arrives, only if the CRC matches and
probe_mode ≤ 3, spi_cfg bits 7..3 = 0, every divN ≥ 16; otherwise nothing changes. A C3 transaction never reaches the
frame decision: no ack, no fetch, `sent_hw` / `rd_off` / `dropped_reported` / `ctrl_good` untouched (the testbench
asserts all four around every C3 transaction). Its MISO is don't-care. **Echo:** MISO byte 3 = gen, byte 4 =
probe_mode in every later frame; bytes 5..10 stay 0. Bytes 1..10 have always been inside the frame CRC, so the CRC
covers the echo bytes as sent; an ESP parser that ignored bytes 3..10 but checked the CRC over the whole header is
unaffected. Reset: gen 0, probe_mode 0, spi_cfg 0, div 1250/1250/1250/104.

## Verification, protocol v3 (actual results, 2026-11-01)

All from a clean `out/`: `make -j6 build sim-partner sim-top vectors` (exit 0, no FAIL line anywhere in the log),
rerun after the review fixes (deglitch, I2C timeout rule, SPI CS lead, partial ruling, counted discards).

| Run | Result |
|---|---|
| `make sim-partner` | unchanged suite: bit timing 11 × `sampled 120/120 PASS` (incl. ±5000/±9000 ppm), `ALL TESTS PASSED (16 records)`, 4 × `PASS: all spi_frame_stream tests completed successfully` (24 MHz baseline + 12 MHz zero/small/full) |
| `sim-top-small` (1 Ki buffer, overflow) | `PASS: can_frame_link end-to-end`: 2182 records (217 CAN, 66/29/23/1564 UART0..3, 6 COUNTER, 67 I2C, 136 SPI, 74 DIO), 107 dropped in phase C, all reported |
| `sim-top-full` (4 SPRAM, offsets through 2³²) | `PASS: can_frame_link end-to-end`: 2027 records (130 CAN, 62/25/20/1508, 5 COUNTER, 67 I2C, 136 SPI, 74 DIO), 0 missing |
| `sim-top-baud` (UART 115200/230400/57600/115200 set by a C3 block at start, CAN +3000 ppm, 2 KB reads) | `PASS: can_frame_link end-to-end`: 2030 records, 0 missing |
| `make vectors` (BUF_AW 16) | `PASS: can_frame_link end-to-end`; frames_vectors*.txt regenerated, v3_records.txt copied from the small run |

The v3 phases in every configuration (E-H, after the unchanged A-D):

* **E, I2C (mode 3, set while CAN/UART stream):** 67 records: writes, write + repeated START + read with the last
  byte NACKed, START+STOP marker, NACKed address, partial bytes (as a later and as the first byte; data_valid 0),
  timeout (bus released 1 ms) on a held byte and with nothing held, the next START after a timeout is `start`, a
  5 ms clock stretch (no timeout), SCL double crossings (60 ns high / 40 ns low) on three bits, START sweep (SDA→SCL
  hold 200/300/450/700 ns × 4 phases against the clock), 1 MHz Fm+, and a switch to SPI in the middle of a byte (the
  earlier byte goes out, the 4 bits do not).
* **F, SPI (mode 2):** 136 records: all 8 spi_cfg values (modes 0-3 × MSB/LSB first), each at 1.5 MHz (4 bytes; 3
  bytes + 5-bit partial; 3-bit partial only), 500 kHz and 2 MHz; a CS pulse with no clocks (end-only record); a
  12-byte burst; CS falling 20 / 60 / 100 ns before the first sampling edge; a switch to DIO with CS still low after
  2 whole bytes (byte 1 goes out; byte 2, still held, is discarded and must show up as `dropped` on the next record
  and in the header count). The stimulus changes MOSI/MISO 100 ns after the shift edge, as a real device does, so
  sampling on the wrong edge fails.
* **G, DIO-8 (mode 1):** 73 records incl. the entry record (PROBE3 high from the SPI phase), walking one, all eight
  at once, 40 random changes, 24 glitch pulses of 100-850 ns on one or four channels (no record), 6 changes of three
  or four channels 50 ns apart (one record each), glitch on one channel with a real edge on another, and 3
  heartbeats (ts exactly k × 100000, loaded within 1 µs of the boundary).
* **H:** C3 blocks with a bad CRC low byte, a bad CRC high byte, div0 = 15, probe_mode 4, spi_cfg bit 3 set, and
  only 13 bytes clocked are all ignored (config and echo unchanged); 16 bytes clocked (valid, 2 extra bytes) and
  div2 = 16 exactly are applied. Mode 0 is then entered with PROBE1 low (no UART3 break record) and PROBE2 high (the
  counter shows level 1, no phantom rise), with UART0 at 57600 (div 208) and the probe UART at 230400 (div 52):
  24 + 80 records incl. framing error, break, genuine 0x00 and Pi packets; the counter resumes.
* Throughout: every frame's MISO bytes 3-4 must equal the applied gen / probe_mode (89-133 frames per config);
  every C3 transaction must leave `rd_off`, `sent_hw`, `dropped_reported` and `ctrl_good` unchanged; I2C/SPI/DIO
  records are byte-exact against the stimulus models (timestamps mirrored from the DUT and checked against
  elapsed time within ±2 µs); no phase E-H may drop a record; each v3 scenario must deliver records.

**Mutation check (2026-11-01, `+fast` runs, `full` config):** each of these was caught with FAIL lines: I2C ack
polarity inverted (69), SPI LSB-first ignored (55), DIO glitch filter 2 clocks instead of 12 (171), C3 CRC not
checked (3), C3 transaction allowed to run the frame decision (13), echo byte 4 stuck at 0 (48), UART0 divisor
ignored (17, plus watchdog), DIO coalescing window removed (60), SPI CPHA ignored (62; this one first passed, which
showed that the SPI stimulus changed data on the shift edge itself; the stimulus was fixed to change data 100 ns
later and the mutation then failed). After the review fixes, each of these also fails: C3 probe_mode / spi_cfg
range checks removed (5), filters restarting at 0 (phantom counter edge, 4), UART start accepted before the line
was seen high (198), I2C idle timer reset by any edge instead of counting only while the bus is released (49), I2C
SCL/SDA filter removed (102), SPI edge in the CS-fall clock not counted (15), discarded records not counted (3).

**Unit level (the reviewer's benches, ports renamed):** I2C: clean 400 kHz write; a 60 ns / 40 ns SCL double
crossing decodes 0x55 with no spurious record; a 5 ms clock stretch gives both bytes and the STOP; 1 MHz; SDA 30 ns
ahead of SCL fall. SPI mode 0, 960 bytes per case: 0 wrong at 1.0, 1.5, 2 and 3 MHz 50 % duty, with 70 ns data
delay, and with CS 0 / 20 / 40 / 60 / 100 / 150 ns before the first edge; 4 MHz and a 100 ns SCK high pulse fail,
as specified (high / low ≥ ~170 ns). UART: div 16 / 17 / 104 / 1250 / 65535 at ±2 % (±3.5 % at 104), 0 bad.

**Cross-layer:** `make -C firmware/v3-wt/main/fpga_spi_live/tests REC_VECTORS=validation/vectors/v3_records.txt`
(the ESP firmware's own parser, ASan/UBSan) replays `v3_records.txt`: `decoded 217/1682/6/67/136/74; 0 mismatches,
0 bad record lines, 0 resyncs`; make exits 0 with each suite's PASS line.

**Not verified:** anything on hardware; the probe input path (470 Ω + 10 k pull-down + TVS, U19) at real edge rates;
the I2C level on weak-pull-up buses (contract §1 hardware note); SPI above 1.5 MHz is outside the contract;
SPRAM behaviour comes from the yosys model.

## Verification (actual results, 2026-10-11, rerun after §7.1)

Partner testbenches on the reused modules, at this design's clocking:

| Run | Result |
|---|---|
| `can_bit_timing_tb` (12 MHz, native) | PASS 120/120 bits sampled; tolerance sweep: see `make sim-partner` |
| `can_frame_fsm_tb` (12 MHz, native) | PASS, 16/16 records (std/ext/RTR, CRC/ACK/stuff/form/unattributed/stuck errors, overload) |
| `spi_frame_stream_tb` on the **unmodified partner RTL at 24 MHz** (baseline) | PASS (the README says the suite had not been rerun since the timing-closure rework; it passes) |
| `spi_frame_stream_tb` on the **shipped `spi_frame_link` at 12 MHz**, 1-sample records (= original sensor), zero / small (2³² wrap, overflow) / full 4-SPRAM | PASS × 3. Rates halved: sensor 0.5 Mbps, SCK sweep 400 kHz–1.5 MHz, default 1 MHz. Overflow: 449 dropped, all accounted for |

End-to-end `can_frame_link_tb` (CAN + UART0/1/2/3 + PROBE2 concurrently, ESP model at 1 MHz unless swept; UART3 at
115200 everywhere):

| Config | Result |
|---|---|
| `small`: 1 Ki-sample buffer, UART0-2 9600 (tx +2 %/−2 %/0), UART3 tx +1.5 % | **PASS**: 1752 records (185 CAN, 36/24/18/1484 UART0/1/2/3, 5 COUNTER) delivered exactly once in arrival order. Overflow: 107 of 247 records dropped whole, including 40 of 96 UART3 and 1 of 1 COUNTER loaded during the overflow; header `dropped` = 107, flag on the next record |
| `full`: 4-SPRAM buffer, offsets through 2³², UART3 tx +1.5 % | **PASS**: 1597 records (98 CAN, 32/20/15/1428, 4 COUNTER) |
| `baud`: UART 115200/230400/57600/115200 (tx +2 %/−2 %/−2 %/−2 %), CAN +3000 ppm, 2 KB reads | **PASS**: 1599 records (98 CAN, 32/20/15/1428, 6 COUNTER) |

All three configurations also survive the injected faults (1 corrupt control block, 1 corrupt MISO frame and 1
transfer aborted mid-stream), and include 6–7 frames that end mid-record (read_len 54 phase), all reassembled.
The SCK sweep covers 400 kHz, 750 kHz, 1.2 MHz and 1.5 MHz. Phase D runs 80 Pi packets back to back on UART3 (1280
records) with the counter running, and drops nothing.

Each counter record is predicted independently by the testbench (level, rise, glitch, `ts_us` = k × 100000), and must
load within 1 µs of t_por + k × 100 ms. PROBE2 stimulus: bursts of 37, 1000 (50 kHz), 20 with bounce before each
edge, and 200 with 1.5 µs halves (accepted), plus random bursts. Glitches of 100–850 ns are injected in both levels,
in idle and inside bursts, plus 300 inside one window (saturates at 255). Level 1 is held across two boundaries.
Mutation check (`full` config, 2026-10-11): each of these was caught with FAIL: glitch not reset on load, filter of 10
clocks instead of 12, UART3 tagged as chan 2, and counter timestamp off by one.

ESP host replay (`firmware/integration/main/fpga_spi_live/tests`, `test_link_records frames_vectors_esp.txt`):
`38 P lines; expected CAN 98, UART 1495, counter 4; decoded 98 + 1495 + 4 (UART ch0/1/2/3 = 32/20/15/1428);
0 mismatches, 0 bad record lines, 0 resyncs`.

Each configuration checks:

* every record is byte-exact against what the testbench transmitted (counter records: against the testbench's own
  PROBE2 model);
* every source delivers > 0 records, and phases A and D must deliver UART3 and COUNTER records (an empty compare
  fails);
* each source's records arrive exactly once and in order;
* global arrival order holds: delivered records follow the DUT's slot-load sequence with no reordering;
* the dropped flag and the header count are consistent, and the overflow phase must drop ≥ 1 UART3 and ≥ 1 COUNTER
  record;
* counter records arrive every 100 ms (each loads within 1 µs of its boundary, `ts_us` = k × 100000);
* corrupt control blocks, corrupt MISO frames and aborted transfers are all survived;
* frames are whole or straddle cleanly;
* SPRAM memory safety holds on every clock;
* the slot-overrun and busy-offer paths never fire;
* timestamps fall within ±2 µs (UART) of elapsed time, and inside the documented bound for CAN.

Traffic covered:

* CAN: standard, extended, DLC 0–8, DLC 12, RTR and stuff-heavy frames, plus one bad-CRC frame (err=2, crc_ok=0).
* UART: random bytes with back-to-back bursts, framing errors, breaks and genuine 0x00 bytes, transmitted at +2 %,
  −2 % and exact baud. UART3: the same, plus the Pi's 16-byte `PB` packets (seq, t_ms, xorshift32, CRC-16) back to back.
* PROBE2: known-count bursts, sub-µs glitches, bounce, saturation, level 1 across a boundary.
* Config `small`: 1 Ki-sample buffer, overflow test, all UARTs at 9600.
* Config `full`: real buffer, offsets through 2³².
* Config `baud`: UARTs at 115200, 230400 and 57600 (tx +2 %, −2 %, −2 %), CAN at +3000 ppm, 2 KB reads.

Notes from verification:

* The yosys `cells_sim.v` `SB_IO` model computes its outputs in `always @*` blocks that iverilog does not evaluate at
  time 0, so a pin that never toggles reads X. CAN idle-high decoded as "stuck dominant" and MISO's first bit was X.
  `ice40_sim_models.v` replaces it with assign-based models for the two PIN_TYPEs used; SPRAM is the same model the
  partner TB uses.
* iverilog prints a ternary of string literals in `$fwrite` as a number; the first ESP vector file had
  "67" in place of "C" and the ESP replay decoded nothing (a vacuous pass). Fixed.
* A flat `#41.6666667` clock rounds to 41.667 ns at 1 ps resolution (8 ppm slow, 2 µs by 250 ms). The timestamp
  checks caught it, and the TB clock now alternates half-periods so it is exact.

**Not verified:**

* Anything on hardware.
* The 1.5 MHz ceiling: zero-delay simulation; real pad and ESP input delays are unmeasured.
* The ESP32-H2 SPI clock source: if it is the 32 MHz XTAL, a 1.5 MHz request rounds up to about 1.52 MHz, over the
  ceiling. 1 MHz is exact from either 32 or 48 MHz.
* SPRAM behaviour comes from the yosys model, not silicon.
* BLE throughput.

## Flashed image

**The protocol v3 build is NOT flashed** (2026-11-01). `make build` of branch `feat/protocol-v3` gives sha256
`5ebab71f7b91bd57c445e526b146e5e682fbafb2efe19391c1bf44b3c93a34b2` (yosys / nextpnr from the same OSS CAD Suite,
seed 1, `-dffe_min_ce_use 4`). Its reset config (probe_mode 0, 9600/9600/9600/115200) decodes the same buses
with the same records as the image below (UART3's reset divisor 104 is 0.16 % fast of 115200, where the old build's
NCO was exact; well inside the ±4.5 % tolerance), so it can replace it for the battery chain; the ESP firmware must send C3 blocks for
anything else, and must accept the gen/mode echo in MISO bytes 3-4 (until the first C3 block they are 00 00, as
before). The text below describes the image that is on board 2 now.

The bitstream on SignalBench board 2 is the **§7.1 build** (CAN + UART0-3 + PROBE2 counter), flashed
2026-10-11 ~21:10 with `iceprog` (VERIFY OK, CDONE high), sha256
`c4bfe62deea858ef98817c80a3fa8a87fd23b2befe73ebba0792f6c6c6409d9b` - identical to `make build` here
(yosys/nextpnr from the same OSS CAD Suite, seed 1). Confirmed live: UART3 and counter records decoded by the ESP.
The previous (pre-§7.1, CAN + UART0-2) image was sha256 `79f62b7b769cf4cc715e39c100d8bea92bc4dc383f0b7e2fd484989d8e564268`. The build is deterministic
(yosys + nextpnr from OSS CAD Suite), so `*.bin` stays git-ignored like the rest of this repo:
rebuild and compare the hash.
