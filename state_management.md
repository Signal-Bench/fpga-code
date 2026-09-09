# State Management SPI Protocol — Design Notes

Draft/proposal only — none of this is implemented yet. See "Target System Architecture
(SignalBench)" in [CLAUDE.md](CLAUDE.md) for the subsystem-level description this
expands on. Command names below (`CMD_...`) are placeholders for opcodes that still
need to be assigned; nothing here is a spec until those are pinned down.

Recap of the link: the ESP32-H2 is SPI master, the FPGA is SPI slave. Every
transaction is MCU-initiated: one command byte, optionally followed by one data byte
for parameterized commands. The FPGA exposes write-only config registers (protocol
select, baud rate, trigger pattern) and a read-only status register (idle /
recording-in-progress / streaming-in-progress / active-protocol / recording-done).

## Example 1: Recording with a trigger pattern

Scenario: catch the first I2C transaction addressed to `0x36`.

1. **Select protocol** — MCU writes `CMD_SET_PROTOCOL` + data byte `I2C`.
2. **Load trigger pattern** — MCU writes `CMD_SET_TRIGGER` + data byte `0x36` (single
   byte here; see "Multi-byte trigger patterns" below for longer patterns).
3. **Request recording** — MCU sends `CMD_REQUEST_RECORD`.
4. **FPGA acks** — sets a ready/armed bit in the status register; doesn't start yet.
5. **MCU confirms ready** — read transaction, checks the ready bit.
6. **Start** — MCU sends `CMD_START`. Trigger subsystem starts comparing decoder
   output against `0x36` on a rolling basis; status flips to "armed."
7. **Bus does its thing** — target device traffic happens on SCL/SDA independent of
   the SPI link; nothing crosses the SPI bus during this window.
8. **Match** — decoder emits the address byte, comparator sees `0x36`, FPGA sets
   recording-in-progress and starts writing subsequent payload bytes into EBR.
9. **Capture ends** — buffer fills or a protocol end-condition hits (I2C STOP); FPGA
   clears recording-in-progress, sets recording-done.
10. **MCU polls status** — next status read shows recording-done set.
11. **Drain** — MCU issues read commands to pull the captured bytes out of EBR
    (format — raw stream vs. length-prefixed — still undecided; see "Open questions").
12. **Back to idle** — MCU sends stop/idle command (or FPGA auto-returns to idle once
    drained); status register clears.

## Example 2: Streaming (continuous decode, no trigger)

Scenario: continuously relay decoded UART bytes as they arrive, no capture buffer.

1. **Select protocol** — MCU writes `CMD_SET_PROTOCOL` + data byte `UART`.
2. **Set baud rate** — MCU writes `CMD_SET_BAUD` + data byte (protocol-specific param).
3. **Request streaming** — MCU sends `CMD_REQUEST_STREAM`.
4. **FPGA acks** — sets a ready bit in the status register.
5. **MCU confirms, starts** — read to check ready, then `CMD_START`. FPGA sets
   streaming-in-progress.
6. **Frames arrive asynchronously** — each completed decoded frame (one UART byte,
   one I2C transaction, etc.) is held in a small output buffer until the next SPI
   read, since the FPGA can't initiate a transfer as slave.
7. **MCU polls for frames** — repeated read transactions. If nothing new is queued,
   the FPGA needs to signal "empty" without colliding with real data values (a raw
   `0x00` sentinel doesn't work — decoded payload bytes can legitimately be `0x00`).
   Two ways to do this: a leading status/valid byte ahead of each frame (cheap,
   costs one extra byte per poll), or a dedicated GPIO data-ready line the MCU checks
   before bothering to poll (extra pin, but zero wasted SPI transactions). See the
   "streaming backpressure" open question below — not decided yet.
8. **Continues until stopped** — MCU sends `CMD_STOP`; FPGA clears
   streaming-in-progress, returns to idle.

## Example 3: Recording without a specific trigger ("record starting now")

Scenario: MCU wants to arm a capture but doesn't care about matching a byte pattern —
just start recording from the next protocol boundary.

Two ways to support this without bolting on a separate mode:

- **Option A — zero-length trigger pattern as a sentinel.** Reuse the existing
  trigger-compare datapath; if `pattern_len == 0`, the comparator always asserts
  match, so recording starts the moment the subsystem is armed and sees the next
  qualifying boundary (e.g. next I2C START, next UART start bit). Minimal FPGA logic
  change (one `pattern_len != 0` gate on the compare), but overloads a "magic value"
  that's easy to forget about later.
- **Option B — dedicated `CMD_RECORD_NOW` command.** Skips the trigger-arm step and
  pattern register entirely, jumps straight to recording-in-progress on `CMD_START`.
  Clearer state machine, explicit in the protocol, costs one more opcode.

Leaning toward **B** for clarity — it keeps "untriggered recording" a first-class,
self-documenting path in the command set rather than a special value someone has to
know to check for. Walkthrough for B:

1. **Select protocol** — as before.
2. **Request** — MCU sends `CMD_REQUEST_RECORD_NOW` (distinct from
   `CMD_REQUEST_RECORD`, which implies a trigger pattern is set).
3. **FPGA acks ready.**
4. **MCU confirms, starts** — `CMD_START`. FPGA jumps directly to
   recording-in-progress, no pattern comparison, capture begins at the next
   protocol-boundary edge (not mid-byte/mid-frame).
5. **Capture proceeds identically to the triggered case** — fills EBR or hits a
   protocol end-condition, sets recording-done.
6. **Drain** — same as Example 1, step 11.

## Multi-byte trigger patterns

Problem: the command protocol is one command byte + at most one data byte, so a
trigger pattern longer than one byte (e.g. a 2-byte register address, or an N-byte SPI
command sequence to watch for) needs several transactions to load.

Proposed loading sequence:

1. `CMD_SET_TRIGGER_LEN` + data byte `N` — sets how many bytes of the pattern are
   active (0 = untriggered, see Example 3 Option A; max is however many bytes the
   FPGA's trigger register array is sized for — keep this small, e.g. 4 bytes, given
   the UP5K's limited LUT/EBR budget already shared with decode and capture logic).
2. `CMD_SET_TRIGGER_BYTE` + data byte, sent once per pattern byte in order (byte 0
   first, byte N−1 last). FPGA appends each into an internal array indexed by a
   write-pointer that resets whenever `CMD_SET_TRIGGER_LEN` is issued.
   - Alternative: four distinct opcodes (`CMD_SET_TRIGGER_BYTE0`..`_BYTE3`) instead of
     one opcode + implicit pointer — trades a few extra opcode values for removing the
     pointer/reset state entirely. Given the command-byte opcode space is otherwise
     lightly used (protocol select, baud, trigger-len, request/start/stop ×2 modes,
     drain), this is cheap and arguably simpler to get right in both MCU firmware and
     FPGA logic.
3. **Comparator**: `pattern_reg[N-1:0][7:0]` plus `pattern_len`. As each new decoded
   byte arrives from the active decoder, shift it into a same-width rolling window,
   and assert match when `window[0 +: pattern_len] == pattern_reg[0 +: pattern_len]`
   — only the configured length is compared, the rest of the register is don't-care.
   This is a direct generalization of a fixed 1-byte comparator; no new concept, just
   parameterized width.

## Open questions / not yet decided

- Exact opcode values for every `CMD_...` placeholder above.
- **MOSI vs. MISO direction for streaming output** — the report text says the FPGA
  "clocks decoded frames out over MOSI," which is backwards for an SPI slave (slave
  drives MISO); needs confirming whether that's a writeup slip or an actual wiring
  mistake to fix.
- **Drain/read byte format** — raw byte stream (like the standalone `i2c_sniffer.v`'s
  leading `0xFF` sentinel + raw bytes convention) vs. a length-prefixed frame format
  for both the recording drain and per-frame streaming reads.
- **Streaming backpressure** — how the MCU knows a poll returned a fresh frame vs. an
  empty/stale one (leading valid byte vs. dedicated data-ready GPIO, per Example 2
  step 7).
- Max trigger pattern length the UP5K can realistically afford alongside decode logic
  and the capture RAM.
