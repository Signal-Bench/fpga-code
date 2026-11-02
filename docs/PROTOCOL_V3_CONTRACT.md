# SignalBench protocol v3 — interface contract (2026-11-01)

Single source of truth for the FPGA gateware, ESP32-H2 firmware, Android decoder and Windows
visualizer. Every implementation agent works from this file; changes go through Charles / the
orchestrating session, never edited unilaterally by one layer.

Conventions: "LE" = little-endian, "BE" = big-endian. Bit 0 = least significant. µs timestamps are the
FPGA's free-running 32-bit microsecond counter (`us_ctr`, wraps every ~71.6 min); receivers unwrap.

---

## 1. Probe pins and modes

8 probes reach the FPGA: PROBE1..8 = header P1.9..P1.16 = FPGA pins 32, 35, 31, 37, 34, 43, 36, 42.
Always captured regardless of mode: CAN (pin 27), RS-232 RX ch0 (pin 23), RS-232 TX ch1 (pin 25),
RS-485 ch2 (pin 26).

| probe_mode | Name | Probe use | Records emitted from probes |
|---|---|---|---|
| 0 (reset default) | UART-probe | PROBE1 = UART ch3 RX; PROBE2 = edge counter (today's behaviour) | C5 (ch3), E7 |
| 1 | DIO-8 | PROBE1..8 = DIO bit 0..7 | D1 |
| 2 | SPI | PROBE1 SCK, PROBE2 MOSI, PROBE3 MISO, PROBE4 CS (active low) | 5B |
| 3 | I2C | PROBE1 SCL, PROBE2 SDA | 1C |

On any mode change the FPGA resets the probe decoders' state (no half-decoded byte carries over).

Hardware note: each probe has 470 Ω series + 10 kΩ pull-down to GND on the probe side (R68/R69… on
IDC1/IDC2). On weak-pull-up I2C buses this lowers the high level; bench check decides whether R68/R69
are removed. Firmware does not depend on it.

---

## 2. FPGA → ESP records (inside the framed SPI link, unchanged transport)

Fixed length, even (link carries u16 samples, bytes in order), unique start AND end byte.
**Uniform drop bit: byte 1 bit 3 = `dropped`** for every 8/10/12-byte record (inserted at arbiter grant,
as today for UART). CAN keeps its existing drop bit (byte 6 bit 7).

| Start | Len | End | Record | Layout |
|---|---|---|---|---|
| `AA` | 20 | `55` | CAN (unchanged) | see `validation/battery_can_chain/CONTRACT.md` §1 |
| `C5` | 8 | `5A` | UART (unchanged) | `C5 {chan[1:0]<<6, ferr<<5, brk<<4, dropped<<3} data ts[4 BE] 5A` |
| `E7` | 12 | `7E` | Counter (unchanged, mode 0 only) | `E7 {level<<7, dropped<<3} rise[4 BE] ts[4 BE] glitch 7E` |
| `1C` | 8 | `C1` | I2C event | `1C flags data ts[4 BE] C1` |
| `5B` | 10 | `B5` | SPI byte | `5B flags mosi miso ts[4 BE] nbits B5` |
| `D1` | 10 | `1D` | DIO-8 sample | `D1 flags levels changed ts[4 BE] 00 1D` |

**I2C flags (byte 1):** bit7 `start` (first byte after START), bit6 `rstart` (first byte after repeated
START), bit5 `stop` (STOP detected after this event), bit4 `data_valid` (data byte + ack meaningful),
bit3 `dropped`, bit2 `ack` (SDA low on 9th clock = ACK), bit1 `timeout` (bus idle — SCL **and** SDA both
high — for >1 ms mid-transaction; SCL held low = clock stretching, never a timeout),
bit0 `partial` (STOP, rSTART or timeout before the byte and its ACK clock completed; data holds the bits
received, MSB-aligned). **A partial record has `data_valid=0` and `ack=0`; receivers ignore its ack bit.**
(Ruled 2026-11-01.)
One record per completed byte (incl. address byte); a STOP with no pending byte is a record with
`data_valid=0, stop=1`. ts = µs when the event completes.

**SPI flags (byte 1):** bit7 `cs_start` (first byte after CS fell), bit6 `cs_end` (CS rose after this
byte), bit5 `partial` (nbits < 8), bit4 `data_valid`, bit3 `dropped`, bits2..0 = 0.
`nbits` = valid bits in mosi/miso (8 normal, 1–7 partial, 0 for an end-only record with `data_valid=0,
cs_end=1`). Partial bits are MSB-aligned for MSB-first, LSB-aligned for LSB-first. ts = µs at byte completion.
Decoder oversamples at 12 MHz (no SCK clock domain); supported SCK **≤ 1.5 MHz** (SCK high/low ≥ ~170 ns
after the 2-sample filter), modes 0–3, MSB/LSB first; the first SCK edge may follow CS fall immediately.
I2C SCL/SDA and SPI SCK pass a 2-sample (~167 ns) stability filter.

**DIO flags (byte 1):** bit3 `dropped`, bit0 `heartbeat` (periodic sample, `changed` = 0). `levels` bit i =
PROBE(i+1). `changed` = bits that changed since the previous emitted record. Emitted on any change after
a per-channel 1 µs glitch filter, plus a heartbeat every 100 ms. ts = µs of the change.

Byte 8 of DIO is reserved (0x00).

---

## 3. ESP → FPGA config block (replaces the raw 1-byte mode transaction)

Sent as **its own CS transaction** on the existing link SPI, MOSI:

```
C3 | gen u8 | probe_mode u8 | spi_cfg u8 | div0 u16 LE | div1 u16 LE | div2 u16 LE | div3 u16 LE | crc16 BE
```
14 bytes; CRC-16/CCITT-FALSE over bytes 0..11 (same CRC and byte order as the `5C` control block).

- A `C3` transaction **must not consume, ack or advance stream data**; MISO during it is don't-care and
  the ESP discards it. Latched only if CRC is correct and every field is in range; otherwise ignored.
- `gen`: ESP increments on each new config (wraps). FPGA stores the last applied value.
- `spi_cfg`: bit0 CPHA, bit1 CPOL, bit2 LSB-first, other bits 0.
- `divN` = round(12 000 000 / baud) for UART channel N (0 RS-232 RX, 1 RS-232 TX, 2 RS-485, 3 probe UART).
  Valid 16..65535 (750 000 .. 184 baud). FPGA reset defaults: 1250, 1250, 1250, 104 (9600/9600/9600/115200),
  probe_mode 0, spi_cfg 0, gen 0.
- **Echo:** every subsequent `5C` read frame carries the applied config in MISO bytes 3–4 (today fixed
  `00`): byte 3 = `gen`, byte 4 = `probe_mode`. (Agents: confirm the ESP frame parser/CRC treats bytes
  3–10 correctly; byte 5–10 stay `00`.)
- ESP sends the config after every link (re)sync and on every change, resends every 100 ms until the
  echo matches, and after 10 failed tries sets FPGA error bit `0x08 CONTROL` until confirmed.

---

## 4. BLE notification format (ESP → app, characteristic …0011)

```
[seq lo][seq hi][battery %][FPGA err][MCU err][protocol][len] | TLVs … (len bytes)
```
7-byte header, total ≤ 244 bytes, `len` = bytes after the header. One protocol per notification.

- `seq` u16 LE: one counter for every notification (all protocols and the heartbeat). Incremented
  only when `esp_ble_gatts_send_indicate` returns `ESP_OK`; reset to 0 on each new connection.
  Receivers reset their tracker on connect; gap = `(seq - last - 1) & 0xFFFF`, a gap in 1..0x7FFF = lost
  packets, otherwise = reorder/duplicate.
- `battery %`: 0..100, `FF` unknown.
- `FPGA err` bits: 01 SYNC, 02 LENGTH, 04 CRC, 08 CONTROL (incl. config unconfirmed), 10 DROPPED,
  20 LINK_OTHER, 40 CAPTURE, 80 UNKNOWN.
- `MCU err` bits: 01 SPI, 02 RECORDS, 04 DROPPED, 08 BLE, 10 GAUGE, 20 BOARD, 40 FRAME_LOSS;
  `FF` = no protocol selected (whole-byte sentinel).
- `protocol`: 00 I2C, 01 SPI, 02 DIO, 03 CAN, 04 UART, FF none. **Data notifications always carry
  their real protocol byte**, also before any SET_MODE (no-selection is signalled only by MCU err `FF`).
- Heartbeat = protocol `FF`, len 0. Sent whenever no other notification went out for ~1 s,
  **whether or not a protocol is selected**, so battery % and error bits stay current on a quiet bus.
  Its MCU err byte is `FF` only while nothing is selected, otherwise the current MCU error bits.
  (Clarified 2026-11-01.)
- **Overflow-only notification:** when records were dropped and nothing else carries the count, the ESP
  sends header + `03 02 <u16>` only, with the selected protocol, or protocol `FF` when nothing / all is
  selected. Receivers must read TLV 03 from a protocol-`FF` notification (heartbeat = `FF` with len 0 only).
- **Closing group:** if a segment ended with `cont_next` and the transaction then restarts (START, rSTART,
  CS fall), the ESP first sends an empty group carrying only `cont_prev`:
  I2C `14 04 ts 12 00 13 01 80`; SPI `23 04 ts 20 00 21 00 22 01 <80|mode>`. Zero-byte groups are valid.
- TLV = `type u8, length u8, value`. **Receivers skip unknown types** (FSR-APP-001) and reject a
  notification whose TLVs overrun `len` or whose `len` ≠ notification length − 7 (FSR-APP-002).
- `03 02 <u16 LE>` Overflow (records lost before framing), first TLV when nonzero, any protocol.

### CAN (protocol 03) — unchanged
Group per frame starting at `40`: `40 04 id LE32`, `41 01 dlc`, [`42 n data`], `43 02 {fpga flags, b0 rec
dropped | b1 link dropped}`, `44 04 ts LE32` (CAN bit-times).

### UART (protocol 04) — unchanged
Group per run starting at `52`: `52 01 chan`, `50 04 baud LE32` (now the live configured baud),
`54 04 ts µs LE32`, `51 n bytes`, [`53 01 status`: b0 ferr, b1 brk, b2 dropped].

### I2C (protocol 00) — new
One group per transaction segment, starting at `14`:
`14 04 ts µs LE32` (first event of the segment) · [`10 01 addr byte` = addr<<1 | R/W, present when the
segment starts at a START/rSTART] · [`11 n data bytes`] · `12 k ack bitmap` (k = ceil(nbytes/8); bit i of
byte i/8 = byte i of this segment counting the address byte first; 1 = ACK) · `13 01 status`.
Status: b0 start, b1 rstart, b2 stop, b3 timeout, b4 cont_next (transaction continues in a later
segment), b5 dropped, b6 partial (last byte incomplete), b7 cont_prev (segment continues an earlier one;
no `10`). A segment ends at STOP, rSTART, timeout, a full notification, or 10 ms with no new events.

### SPI (protocol 01) — new
One group per CS-assertion segment, starting at `23`:
`23 04 ts µs LE32` · `20 n MOSI` · `21 n MISO` (same n) · `22 01 status` · [`24 01 nbits` of the last byte,
only when partial].
Status: b0 CPHA, b1 CPOL, b2 cs_start, b3 cs_end, b4 cont_next, b5 dropped, b6 partial, b7 cont_prev.
Same segment-end rules as I2C (CS rise instead of STOP).

### DIO (protocol 02) — new
- Mode 1: repeated `30 07 flags levels changed ts µs LE32` — flags b0 heartbeat, b1 dropped;
  `levels` bit i = PROBE(i+1).
- Mode 0: P1.10 counter as `31 0A flags glitch rise LE32 ts µs LE32` — flags b0 level, b1 dropped.

---

## 5. Control channel (write …0012, reply notify …0013) — changes only

Frame unchanged: request `op, req_id, args…`; reply `op|0x80, req_id, status, data…`.
- `02 SET_MODE transport`: also sets probe_mode — SPI→2, I2C→3, DIO→1, UART→0; CAN and `FF` (all) leave
  it unchanged. Forwarding filter behaviour unchanged.
- `04 SET_BAUD channel, baud LE32`: real. BAD_ARGUMENT if divisor out of 16..65535. Reply OK once the
  FPGA echo confirms (timeout 500 ms → FAILED). GET_INFO capability bit `04` now set.
- `05 GET_BAUD channel` → live configured baud LE32.
- `08 SET_SPI_CFG cfg u8` (new): cfg = spi_cfg bits (§3). Reply OK on echo confirm. Capability bit `40`.
- `06 GET_STATUS` reply data grows 11 → 13 bytes: append `cfg_gen u8`, `probe_mode_confirmed u8`
  (`FF` = not yet confirmed). Receivers must accept 11 (old firmware) and 13.

---

## 6. Golden vectors (cross-layer test chain)

- `vectors/v3_records.txt` — written by the FPGA end-to-end testbench: one line per record,
  `R <hex bytes>` in stream order, covering every record type, mode switches and a baud change.
- `vectors/v3_notifications.txt` — written by the ESP host test from `v3_records.txt`: one line per BLE
  notification, `N <hex bytes>`, plus `# expect …` comment lines giving the decoded meaning, including at
  least one deliberate seq gap, one unknown TLV, one malformed notification marked `# reject`.
- Both files live in `validation/vectors/` and are copied into each repo's test fixtures.
