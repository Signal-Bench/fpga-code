# can_frame_link_board: CAN + 3 × UART sniffer → framed SPI link (SignalBench PCBA)

This is the iCE40UP5K-SG48 (U7) bitstream for the battery-life + CAN chain. It decodes every receiver on the board and streams the
decoded records to the ESP32-H2 over the partners' framed, acked, CRC-16 SPI link.
`validation/battery_can_chain/CONTRACT.md` is the source of truth for this design (§1 pins and the CAN record, §4/§5
UART, §6 baud).

```
can_rx   27 ─ can_bit_timing ─ can_frame_fsm ─ 20-byte CAN record ─┐
uart0    23 ─ uart_rx_sniff (RS-232 P1.7)     ─  8-byte UART record ┤  one holding slot per source
uart1    25 ─ uart_rx_sniff (RS-232 P1.8)     ─  8-byte UART record ┤  oldest-first arbiter
uart2    26 ─ uart_rx_sniff (RS-485 P1.3/4)   ─  8-byte UART record ┘  (never splits a record)
        → spi_frame_link: atomic variable-length push into 4 × SPRAM (64 Ki samples),
          logic SPI slave, A5 5A 01 frames, CRC-16/CCITT-FALSE, MCU ack → spi_miso 46
```

**Do not flash from here.** Charles flashes the board. `make build` only writes `can_frame_link_board.bin`.

## Files

| File | What |
|---|---|
| `can_bit_timing.v`, `can_crc15.v`, `can_frame_fsm.v` | partner CAN decoder, **unmodified** (see `SOURCE_COMMIT`) |
| `spi_frame_link.v` | partner `spi_frame_stream.v` link. Original header kept; the changes are listed in its header |
| `uart_rx_sniff.v` | new: passive 8-N-1 receiver, 16× NCO, majority vote, framing/break detection |
| `can_frame_link.v` | new top (`module top`): pads, POR, µs timebase, record packing, arbiter, link |
| `can_frame_link_board.pcf` | board pins exactly per contract; only `spi_miso` is an output |
| `can_frame_link_tb.v` | end-to-end testbench (CAN + 3 UARTs + ESP model) |
| `ice40_sim_models.v` | sim-only `SB_IO` / `SB_SPRAM256KA` stand-ins (see the Verification section) |
| `partner_tb/` | partner testbenches (unmodified copies), the 24 MHz original for baseline, the 12 MHz wrapper and sed rescale |
| `frames_vectors.txt` | real MISO frames from the top-level sim (partner `frames.txt` format) for the ESP host test |
| `frames_vectors_records.txt` | the records those frames deliver, in order: `R <src> <index> <bytes…>` (src 0 CAN, 1–3 UART0–2) |
| `frames_vectors_esp.txt` | ESP `test_link_records` replay format: `P` = payload bytes as the MCU accepts them; `C`/`U` = records as the DUT pushed them, captured at the SPRAM write port |
| `can_frame_link_board.bin` | the bitstream (UART 9600/9600/9600, CAN 1 Mbit/s) |

```
source ~/opt/oss-cad-suite/environment
make build        # yosys + nextpnr (12 MHz) + icepack + icetime -> out/, can_frame_link_board.bin
make sim-partner  # partner TBs: bit timing (+ tolerance sweep), frame FSM, spi_frame_stream baseline + 12 MHz x3
make sim-top      # end-to-end, three configurations
make vectors      # frames_vectors.txt, frames_vectors_records.txt, frames_vectors_esp.txt
```

## Pins (CONTRACT §1/§4/§5)

| Port | Pin | Net | Dir |
|---|---|---|---|
| clk_12m | 44 | IOB_3b_G6 ← Y1 12 MHz via U4 | in |
| can_rx_pin | 27 | FPGA_CAN_RX ← U15 RXD | in, pull-up |
| uart0_pin | 23 | FPGA_RS232_RX_OUT ← U13 ROUT1 (P1.7) | in, pull-up |
| uart1_pin | 25 | FPGA_RS232_TX_OUT ← U13 ROUT2 (P1.8) | in, pull-up |
| uart2_pin | 26 | FPGA_RS485_RX ← U14 RO (P1.3/4) | in, pull-up |
| spi_sck | 20 | RT_SCK ← ESP GPIO10 | in |
| spi_cs | 47 | RT_CSn ← ESP GPIO27 | in |
| spi_mosi | 48 | FPGA_IRQn ← ESP GPIO22 (F1 rework) | in |
| spi_miso | 46 | RT_MISO → ESP GPIO11 | **out, only while CS is low** |

No other pin is used. The design does not touch pin 16 (FT_SSn), pin 18 (RT_MOSI), flash WP#/HOLD#, the probes or RGB0–2.
The input pull-ups hold each receiver line at its idle level (recessive / mark) while the transceivers power up.
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

* **Records:** CAN 20 B (0xAA…0x55) and UART 8 B (0xC5…0x5A), interleaved in **arrival order**. The frame's sample
  bytes, taken in order (each sample LE), are the record bytes in order. Dispatch on the first byte: 0xAA = 20 bytes,
  0xC5 = 8 bytes. UART byte 1 = `{chan[1:0], framing_err, brk, dropped, 3'b000}` (chan 3 reserved).
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
  * CAN timestamps are the partner's **CAN bit times** since configuration, latched at SOF. That is 1 µs per bit at
    1 Mbit/s, but `can_bit_timing` restarts the bit at every hard sync, so it can lose up to one bit per frame against
    wall time. Over a long run, CAN and UART timestamps drift apart by up to the number of CAN frames seen, in µs.
  * If cross-type correlation matters, latching `us_ctr` for CAN as well is a one-line change in `can_frame_link.v`,
    but it changes the contract's definition, so it is not done here.

## UART receiver (CONTRACT §4/§5/§6)

* `UART0_BAUD`/`UART1_BAUD`/`UART2_BAUD` are `top` parameters. They default to **9600** on all three (the built
  `.bin`).
* An NCO accumulates 16·BAUD per clock and wraps at 12,000,000, so the average rate is exact for any baud. At 9600 a
  bit is 1250 clocks and a tick comes every 78.125 clocks (78/79), a jitter of 0.08 % of a bit. Bauds up to
  750 kbaud are supported (16·BAUD ≤ 12 MHz).
* The start edge is detected at the clock rate, not the tick rate (a 9600 tick is 6.5 µs). The timestamp is latched
  and the NCO phase reset at that edge, so tick k is k/16 bit after it.
* Each bit is decided by a majority vote of ticks 16b+7, 16b+8 and 16b+9, centred on mid-bit. A start bit that does
  not vote low is a glitch and is ignored.
* A stop bit that votes low with nonzero data is a framing error. All-zero data with a low stop bit is a break: brk=1,
  framing_err=0, data 0x00. A break is classified once and not also counted as a framing error.
* After either, the receiver re-arms only when the line votes high again, so a break of any length (≥ 1.04 ms at
  9600) is exactly one record.
* The stop-bit centre falls 9.5 bits after the edge, which tolerates about ±4.5 % baud error. ±2 % is tested.

## Arbiter

* Each source (CAN, UART0, UART1, UART2) has one holding slot. When the link is idle, the **oldest** occupied slot is
  granted. Age is tracked in a 4×4 matrix; ties within one clock go CAN, UART0, UART1, UART2.
* The granted record is copied to an output buffer, so its slot frees immediately. The link then writes it in
  10 or 4 clocks. Worst case, all four slots are served in 22 clocks.
* No source can refill its slot that fast: CAN needs one bit time = 12 clocks between records at BRP=0 (EOF bit 5 then
  bit 6), and UART needs one frame ≥ 160 clocks.
* If a slot is ever overrun anyway, the new record is dropped and counted, never split. The testbench asserts this
  path never fires.

## Resources and timing (`make build`, nextpnr seed 1)

| Resource | Used | Device | % |
|---|---|---|---|
| Logic cells | 3194 | 5280 | 60 % |
| SPRAM (16 K × 16) | 4 | 4 | 100 % (the 64 Ki-sample ring) |
| EBR | 0 | 30 | 0 % |
| SB_IO | 9 | 39 | |
| Global buffers | 8 | 8 | 100 % (auto-promoted enables; no second clock) |
| PLL / DSP / HFOSC / SB_SPI | 0 | | |

Timing at the 12 MHz constraint: **nextpnr Fmax 20.77 MHz, icetime 20.53 MHz (48.7 ns path vs 83.3 ns), PASS with
1.7× margin.** The worst path is in the link's decision pipeline (`rd_off` → `level_d` carry chain).

## Verification (actual results, 2026-10-11)

Partner testbenches on the reused modules, at this design's clocking:

| Run | Result |
|---|---|
| `can_bit_timing_tb` (12 MHz, native) | PASS 120/120 bits sampled; tolerance sweep: see `make sim-partner` |
| `can_frame_fsm_tb` (12 MHz, native) | PASS, 16/16 records (std/ext/RTR, CRC/ACK/stuff/form/unattributed/stuck errors, overload) |
| `spi_frame_stream_tb` on the **unmodified partner RTL at 24 MHz** (baseline) | PASS (the README says the suite had not been rerun since the timing-closure rework; it passes) |
| `spi_frame_stream_tb` on the **shipped `spi_frame_link` at 12 MHz**, 1-sample records (= original sensor), zero / small (2³² wrap, overflow) / full 4-SPRAM | PASS × 3. Rates halved: sensor 0.5 Mbps, SCK sweep 400 kHz–1.5 MHz, default 1 MHz. Overflow: 449 dropped, all accounted for |

End-to-end `can_frame_link_tb` (CAN + UART0/1/2 concurrently, ESP model at 1 MHz unless swept):

| Config | Result |
|---|---|
| `small`: 1 Ki-sample buffer, UART 9600 ×3 (tx +2 %/−2 %/0) | **PASS**: 288 records (206 CAN, 38/24/20 UART0/1/2) delivered exactly once in arrival order. Overflow: 41 of 150 records dropped whole, header `dropped` = 41, flag on the next record |
| `full`: 4-SPRAM buffer, offsets through 2³² | **PASS**: 165 records (98 CAN, 32/20/15) |
| `baud`: UART 115200/230400/57600 (tx +2 %/−2 %/−2 %), CAN +3000 ppm, 2 KB reads | **PASS**: 165 records |

All three configurations also survive the injected faults (1 corrupt control block, 1 corrupt MISO frame and 1
transfer aborted mid-stream), and include 2–4 frames that end mid-record (read_len 54 phase), all reassembled.
The SCK sweep covers 400 kHz, 750 kHz, 1.2 MHz and 1.5 MHz.

ESP host replay (`firmware/integration/main/fpga_spi_live/tests`, `test_link_records frames_vectors_esp.txt`):
`vectors: 98 CAN + 67 UART expected, 98 + 67 decoded, 0 mismatches, 0 resyncs`.

Each configuration checks:

* every record is byte-exact against what the testbench transmitted;
* each source's records arrive exactly once and in order;
* global arrival order holds: delivered records follow the DUT's slot-load sequence with no reordering;
* the dropped flag and the header count are consistent;
* corrupt control blocks, corrupt MISO frames and aborted transfers are all survived;
* frames are whole or straddle cleanly;
* SPRAM memory safety holds on every clock;
* the slot-overrun and busy-offer paths never fire;
* timestamps fall within ±2 µs (UART) of elapsed time, and inside the documented bound for CAN.

Traffic covered:

* CAN: standard, extended, DLC 0–8, DLC 12, RTR and stuff-heavy frames, plus one bad-CRC frame (err=2, crc_ok=0).
* UART: random bytes with back-to-back bursts, framing errors, breaks and genuine 0x00 bytes, transmitted at +2 %,
  −2 % and exact baud.
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

The bitstream on SignalBench board 2 (flashed 2026-10-11 with `iceprog`, VERIFY OK) is
`make build` output with sha256 `79f62b7b769cf4cc715e39c100d8bea92bc4dc383f0b7e2fd484989d8e564268`. The build is deterministic
(yosys + nextpnr from OSS CAD Suite), so `*.bin` stays git-ignored like the rest of this repo:
rebuild and compare the hash.
