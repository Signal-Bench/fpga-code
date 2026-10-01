# Review notes — problems hit and decisions made

Covers both `final_system/` (the full system: trigger, recording, framed records) and
`stream_system/` (raw streaming only). Written to be reviewed after the fact, so each
decision says what the alternative was and when to revisit it.

The other documents, for reference:
- [`../unified_top.md`](../unified_top.md) — the architecture argument, with measurements
- [`CHANGES.md`](CHANGES.md) — exact audit of what was modified in existing sniffers
- [`../stream_system/README.md`](../stream_system/README.md) — how to use and test the stream version

**Status of everything here:** simulated, synthesized, placed. **Nothing has been on
hardware.**

---

## 1. Decisions — both versions

### 1.1 One 24 MHz clock, PLL'd from the 12 MHz crystal
- **Problem:** `can_sniffer` must run from the crystal (ISO 11898-1 df < 0.98%, HFOSC is
  ±10%). Every other design ran from `SB_HFOSC`. They couldn't coexist.
- **Chose:** a single 24 MHz domain from `SB_PLL40_CORE`, fed by the crystal. A PLL
  multiplies exactly, so the crystal's accuracy carries through.
- **Alternative:** stay at 12 MHz, no PLL. Simpler and closes timing trivially, but it
  halves I2C/UART sampling resolution and caps an oversampled MCU link at ~3 MHz SCK.
- **Cost:** R16 must be shorted for *every* protocol now, not just CAN. Pin 35 is lost
  (1.4).
- **Revisit if** R16 can't be relied on in the final hardware.

### 1.2 CAN at BRP=1 rather than 24 tq/bit
- **Chose:** keep 12 tq/bit with tq = 2 core clocks, which is numerically identical to
  what was verified on the GM6020.
- **Alternative:** 24 tq/bit at BRP=0. Simulated slightly *better* (breaks at −18000 ppm
  instead of −16000), but it changes every timing parameter and would need
  re-qualification on the bench.

### 1.3 `can_bit_timing.v` gets an opt-in parameter, not an edit
- **Problem:** its resync arithmetic (lines 130–138, compare→add→mux→add→compare) caps
  fmax at ~23–24 MHz. Merged at 24 MHz, only 1 of 6 placement seeds passed.
- **Chose:** `PIPELINE_RESYNC`, **default 0**, so `can_sniffer` is bit-for-bit unchanged
  (all 23 of its tests re-run and pass, still 1292 LC). The unified tops set it to 1.
- **Precondition: `brp >= 1`.** At brp = 0 the precomputed values are one clock stale and
  the module is *wrong*. There's a simulation guard, and `final_system` clamps `can_brp`
  to ≥ 1 in hardware. **This is the sharpest edge introduced in this whole effort.**
  Anyone reusing `can_bit_timing` with `PIPELINE_RESYNC=1` must know it.

### 1.4 Pin 35 is reserved; GBIN pins are 20, 35, 37, 44 **and 23**
- **Found by** place-and-route failing: `PLL 'u_pll' couldn't be placed anywhere`. The PLL
  sits in pin 35's I/O site.
- `spi_sniffer.md` lists the GBIN pins as 20/35/37/44 and says 35/37/44 are free. **Two
  corrections are owed to that file and haven't been made:** pin 23 is also GBIN, and 35
  isn't free once a PLL exists.

### 1.5 Decoders selected by holding the others in reset, never by clock gating
- Clock gating needs a global buffer per gated clock (there are 8) and creates clock
  crossings for no benefit. Reset costs one LUT input per register bank.
- A protocol switch therefore always restarts the newly selected decoder from a clean
  state.

### 1.6 `SB_SPI` hard IP for the MCU link, not `spi_slave.v`
- Measured on the merged design: **114 LCs cheaper and ~2 MHz faster.**
- Also, `spi_slave.v` has a real metastability defect: its edge detect reads the *first*
  flop off an asynchronous pin. It's not used, so it hasn't been fixed.

### 1.7 SPI sniffer: capture both edges instead of muxing the clock
- `spi_sniffer.md` recommends a glitch-free clock mux and treats both-edges capture as the
  fallback. I inverted that: both-edges costs about 32 flops and removes a combinational
  mux from a clock net, plus the "only change mode while disarmed" rule.

### 1.8 Stream FIFO / capture RAM written by hand, not `common/util.v`'s `fifo`
- `util.v`'s `fifo` does `assign read_data = buffer[read_ptr]`, an asynchronous read that
  EBR can't do, so it builds itself from flops. That's why `spi_counter_stream` is ~1200
  LC with **zero** EBRs. Both new designs use a registered read so the memory lands in
  EBR. `util.v` itself is untouched.

---

## 2. Decisions — `stream_system` only

### 2.1 Raw bytes, no framing (your direction)
- My first design wrapped every event in an 8-byte header (sync, protocol, flags, length,
  timestamp). Dropped before it was built, once you said the firmware streams raw.

### 2.2 I2C: port `i2c_sniffer.v` rather than reuse `final_system`'s `sb_i2c`
- **Why:** "raw, including ACKs" is exactly what `i2c_sniffer.v` produces, and it carries
  three hardware-found fixes. `sb_i2c` decodes to byte + ACK flag, and turns out to lack
  all three fixes (see 4.1).
- **Verified** with `i2c_sniffer_tb.v`'s own driver tasks and its own expected bytes. All
  three transactions match, including the ACK-jammed-against-STOP case.
- **Differences, all consequences of streaming rather than buffering:** `0xFF` emitted at
  START rather than prepended by a drain; an empty START/STOP yields a lone `FF`; an idle
  timeout can't recall bytes already sent; no 128-byte limit.

### 2.3 UART: tag removed, both lines merged
- `uart_to_spi.v` sent `[48][01|02][data]`. Removing the tag is what "raw" asks for, but
  bytes from the two lines then become indistinguishable. Fine for testing one line.
- **Revisit if** both directions need capturing at once.

### 2.4 Command protocol copied from your `spi_counter_stream.v`, ready check included
- Your table lists only the `53 42 4D xx` selects. Your firmware *also* sends the
  `53 42 A5 5A` ready check and expects `53 42 4F 4B`, and it waits for `53 42 41 xx`
  ACKs in the follow-up transaction. I implemented all of it, so that firmware should
  work unchanged.
- Also copied: RX drained before TX refills (your silicon fix for receive overruns), all
  five config registers written once, SDBRE off, `SPITXDR` loaded only when there's
  something to send.
- I read your branch with `git show` and your working tree read-only. Nothing there was
  modified.

### 2.5 A select flushes the FIFO
- Matches `spi_counter_stream`'s `mode_switch`. Everything after the ACK belongs to the new
  protocol. One stale byte can still sit in `SPITXDR` and appear *before* the ACK.

### 2.6 Shared probe pins
- ch0 = pin 38 (SCL / UART TX line / CAN / DIO), ch1 = pin 42 (SDA / UART RX line).
  `i2c_sniffer`, `uart_to_spi` and `can_sniffer` *all* already used pin 38, so every rig
  plugs in unchanged.
- `final_system` uses dedicated pins per protocol instead, and its I2C is on 42/36, which
  **isn't** where `i2c_sniffer` had it. The two pinouts now differ; one should win.

### 2.7 Build-time UART baud (9600) and SPI mode (0)
- Your table has no opcodes for them. 9600 matches `send_pattern.c` and `uart_to_spi.v`.

---

## 3. Problems found while building, and what was done

### 3.1 Bugs the `final_system` testbench caught (fixed)
1. **The triggering event was dropped from its own capture.** `recording` is set
   non-blockingly, so it was still 0 in the cycle the trigger matched.
2. **The trigger fired one event late, with reversed byte order.** The window was compared
   before the current byte shifted in, and `trig0` matched the newest byte where
   `state_management.md` says it's the first byte on the wire.

### 3.2 A bug I reported that wasn't one
- `CHANGES.md` originally listed a third bug, "drain pointer runaway". **That was a
  misdiagnosis.** I changed the drain logic *before* instrumenting the testbench; the
  instrumented run then pointed at bug 1. Restoring the original policy with bugs 1 and 2
  fixed, `make sim` still passes. The change stays as a robustness measure (CS framing),
  and `CHANGES.md` now says so explicitly.
- **Lesson worth keeping:** I fixed what I *expected* was wrong before measuring. The
  instrumentation took five minutes and would have prevented it.

### 3.3 Fixes that broke timing (resolved)
- Correcting the trigger put `proto → mux → compare → FSM` on one combinational path, and
  `final_system` dropped to 2/6 seeds. Fixed with an event pipeline register (one clock of
  latency, 42 ns, timestamp captured in the same stage) and by precomputing the
  comparator terms that only involve registers. Back to 6/6 at 27.7–30.0 MHz.
- That precompute carries a precondition (`win` stable for one clock before an event).
  It's weaker than a constraint the packer already imposes (events closer than 24
  clocks are dropped), so it's documented rather than guarded.

### 3.4 A testbench that passed without testing anything (fixed)
- The stream version's "flush on switch" test first left one byte in the FIFO. That byte
  was clocked out during the command transaction, so the flush never ran and the test
  passed anyway. It now leaves 8 bytes queued and checks the FIFO goes 7 → 0.

### 3.5 Branch handling
- You stashed my uncommitted `final-system` work to switch branches. I moved it into a
  worktree at `../fpga-final-system` instead of touching your checkout. The stash is still
  there as a redundant backup; its contents are committed on `final-system`.

---

## 4. Open issues — not fixed

### 4.1 `final_system`'s `sb_i2c` regresses against `i2c_sniffer.v` — **fix before using it**
`stream_system` doesn't use it, which is why it's still open:
- assembles bytes even outside START…STOP, so it emits garbage when attached mid-traffic
- no 1 ms idle timeout, so it can't recover from a missed STOP
- **reintroduces `i2c_sniffer.md` bug 3:** STOP is checked before a coincident SCL edge
- partial bytes at STOP or repeated START are silently discarded

### 4.2 `final_system`'s `sb_uart` silently loses a byte on same-cycle collision
- If both lines finish a byte in the same clock, RX is dropped with no `dropped` flag.
  Fixed in `stream_system`'s `sbs_uart`, not back-ported.

### 4.3 `0xFF` ambiguity in the stream — the biggest one for testing
- Empty FIFO reads as `0xFF`; so does a real `0xFF`, and so does every I2C START marker.
  Readable by eye against a known pattern; software can't tell them apart. Options are in
  the stream README.

### 4.4 Stream bytes clocked out during a command are lost
- If the firmware discards what it receives while sending a command, those bytes are gone.
  Same as `spi_counter_stream`.

### 4.5 Inherited from `common/uart.v`
- Samples each bit 25–50% of the way in rather than centred, which reduces baud-error
  tolerance. Framing errors are silently dropped. CLAUDE.md's target design describes a
  centred 1.5-bit sampler; that doesn't exist yet.

### 4.6 `final_system`'s MCU opcodes are invented
- Placeholders written before your opcode table existed. `stream_system` uses your table
  and protocol; `final_system` doesn't yet.

### 4.7 Record format in `final_system` diverges from `spi_sniffer.md`
- Fixed 32-byte stride with `AA`/`55` markers instead of the length-prefixed format that
  document proposes. Fine for the shared packer; worth revisiting if SPI needs one record
  per transaction.

---

## 5. Not verified by anything

- **All hardware behaviour.** Neither bitstream has been flashed.
- **`SB_SPI`.** Both testbenches use a behavioural model that has no leading dummy byte and
  guesses what `TRDY` does while CS is high. Your branch's hardware captures are the only
  real data on this IP so far.
- **CAN end-to-end inside either unified top.** The decoder is covered by
  `can_frame_fsm_tb.v` and the pipelined bit timing by `can_bit_timing_tb.v`, but neither
  testbench builds CAN frames into the merged design.
- **SPI sniffer, DIO, and the overflow/drop paths** — never exercised by a testbench.
