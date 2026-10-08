# spi_frame_stream — framed, lossless sample stream to the ESP32-H2

Streams data from the FPGA to the MCU at **1 Mbps of payload** without losing
any of it. A fake sensor fills a 128 KB SPRAM buffer; the MCU, as SPI master,
drains it one frame per CS assertion. The FPGA deletes data **only after the
MCU confirms it arrived intact**, so a corrupted, cut-off or missed read
costs a resend, never data.

Data is lost only when the buffer fills up because the MCU stopped reading,
and every such drop is counted and reported in the next frame.

This is the protocol spec. The MCU side (`main/fpga_spi_live/` in the
Microcontroller repo) implements the same rules; keep the two in sync.

## Why it is built this way

| Problem with a raw stream | Fix here |
|---|---|
| Idle `FF` filler looks the same as real `FF` data | Every frame has a sample **count** |
| A byte lost at a CS boundary goes unnoticed | Every frame starts fresh at CS fall with a fixed preamble, and carries its **start offset** |
| Bit errors pass straight through | **CRC-16** over the whole frame |
| Data the MCU never received is gone | FPGA frees data only on the MCU's **ack** |

**The SPI slave is plain FPGA logic, not the `SB_SPI` hard IP.** The hard
IP keeps a byte loaded ahead of the shifter, and nobody has checked whether
that byte survives a CS toggle on this chip (`spi_hw_stream`'s
`+drop_preloaded_on_cs` test). A slave built in logic restarts the frame at
byte 0 on every CS fall and always knows which bit is on the wire. It also
makes the simulation exact rather than a guess about the hard IP.

## Throughput

At 2 MHz SCK, the rate the MCU uses today:

| Read size | Payload per read | Link ceiling |
|---|---|---|
| 1 KB | 1000 B | 1.95 Mbps |
| 2 KB | 2024 B | **1.98 Mbps** |

That is roughly double the 1 Mbps target, if the MCU reads back to back. The
fake sensor makes exactly 1 Mbps (62,500 samples/s × 16 bits). The buffer
holds 64 Ki samples, about one second at that rate, which covers MCU stalls.

**SCK ceiling: 3 MHz.** SCK, CS and MOSI are sampled through 2-flop
synchronizers at the 24 MHz core clock, so MISO changes up to 125 ns after
the falling edge that asks for it. That leaves 125 ns of setup at 2 MHz and
42 ns at 3 MHz. Since MISO changes late, it also stays valid at least 83 ns
after each falling edge. That covers a master that samples half a cycle
late, the ESP32-H2 sample-point question from `notes.md`.

This link is FPGA → MCU only. It has nothing to do with the 16 MHz the
sniffer front end will need to sample a target bus.

## Protocol

SPI mode 0, MSB first. Multi-byte fields are little-endian except the CRCs,
which are big-endian. CRC is CRC-16/CCITT-FALSE: poly `0x1021`, init
`0xFFFF`, no reflection, no final XOR; `"123456789"` → `0x29B1`.

### MCU → FPGA: control block (MOSI, start of every transfer)

| Byte | Field |
|---|---|
| 0 | `0x5C` magic |
| 1–4 | **ack**: offset of the next sample the MCU needs. Everything before it has arrived intact and may be freed. |
| 5–6 | **read_len**: length of this SPI transfer in bytes |
| 7 | flags: bit 0 = ACK_VALID (clear until the MCU has synced) |
| 8–9 | CRC-16 over bytes 0–7 |
| 10… | `0x00` |

### FPGA → MCU: frame (MISO, every transfer)

| Byte | Field |
|---|---|
| 0–2 | `A5 5A 01` sync and version |
| 3–10 | `0x00` (sent while the control block is still arriving) |
| 11–14 | **start**: stream offset of the first sample |
| 15–16 | **count**: samples in this frame |
| 17–20 | **dropped**: samples dropped since reset because the buffer was full |
| 21 | status: bit 0 ACK_APPLIED, bit 1 CTRL_BAD, bit 2 ACK_REJECTED, bit 3 DROPPED (since the previous frame) |
| 22… | count × u16 samples |
| 22+2·count | CRC-16 over bytes 0 to 21+2·count |
| … | `0xFF` to the end of the transfer |

Overhead is 24 bytes per frame. `count` is the smallest of: samples
buffered, `(read_len − 24) / 2`, and `MAX_FRAME_SAMPLES` (2036, for a
4 KB read). So the frame always fits the transfer.

### FPGA rules

- The control block is checked by byte 10. If it is valid and ACK_VALID is
  set, and `rd ≤ ack ≤ highest offset ever sent`, data before `ack` is freed
  (ACK_APPLIED). An ack outside that range is refused (ACK_REJECTED); acting
  on it would free samples nobody has received.
- The frame always starts at the oldest sample not yet acked. Every frame
  resends whatever the MCU has not confirmed.
- If the control block is corrupt (CTRL_BAD), nothing is freed and the
  frame is **empty**, since `read_len` cannot be trusted. An empty frame fits
  any read.
- When the buffer is full, a new sample is dropped and counted. Unacked
  data is never overwritten.

### MCU rules

1. Every read: build the control block from `expected`, the next sample
   offset needed, and clock the full `read_len`.
2. Check `A5 5A 01`, that the frame fits, and the CRC. If any check fails,
   ignore the frame and change nothing. The FPGA resends it next time.
3. Not synced yet, or ACK_REJECTED (the FPGA restarted): set `expected =
   start`.
4. `start < expected` means the FPGA missed our last ack. Skip the
   `expected − start` samples we already have and keep the rest. Do the
   math mod 2³².
5. `expected = start + count`. Send that as the ack on the next read.

**What the ack means today:** the MCU sends the next read, and so the ack,
only after BLE has accepted the previous frame's samples, so BLE congestion
backs up into this buffer. It does not mean the phone received them, and
the MCU still discards data while no phone is subscribed. See the BLE notes
in the MCU repo.

## Wiring

The same pins as `spi_hw_stream`, so the bench wiring does not change.

| Signal | UPduino pin | Direction |
|---|---|---|
| SCK  | gpio_11 | FPGA in |
| CS   | gpio_19 | FPGA in, active low |
| MOSI | gpio_21 | FPGA in (control block) |
| MISO | gpio_13 | FPGA out (frames) |
| flash CS | 16 | FPGA out, held high |

## LEDs (active low)

| LED | Meaning |
|---|---|
| RED | pulses when a sample is dropped: the buffer is full and the MCU is not keeping up |
| GREEN | on while the last control block from the MCU was valid |
| BLUE | pulses when a frame with samples is sent |

Healthy at 1 Mbps: green steady, blue flickering, red dark.

## Commands

```
make sim        # all three configurations below
make sim-zero   # 1 Ki-sample buffer, offsets from 0
make sim-small  # 1 Ki-sample buffer, offsets through the 2^32 wrap, overflow test
make sim-full   # real 64 Ki-sample 4-SPRAM buffer, through the wrap and the bank 3 -> 0 crossing
make vectors    # writes frames.txt: real frames for the MCU host test to replay
make build      # bitstream
make time       # icetime
make flash
make clean      # build/sim artifacts only
```

Build: **1389 LC (26%)**, all 4 SPRAM blocks, no EBR, no `SB_SPI`. The core
clock is 24 MHz; Fmax 41.3 MHz (nextpnr), 39.0 MHz (icetime).

## Verification

`make sim` passes in all three configurations. The testbench is both the SPI
master and a model of the MCU, and its CRC and parser are written
independently of the RTL. It checks:

- **1 Mbps sustained** at 2 MHz with nothing dropped and the backlog flat at
  about two reads' worth
- every sample delivered exactly once and in order; before any drop, each
  sample's value must equal its offset
- a corrupt control block, a corrupt frame, a transfer cut off mid-frame, an
  ack beyond anything sent, an MCU reboot, and read lengths from 32 to 2048
  bytes. None of these lose or duplicate data.
- an SCK sweep from 400 kHz to 3 MHz, including odd rates. Each rate lands
  the frame decision at a different point relative to the byte loads.
- buffer overflow: every missing sample is exactly one the FPGA reported
  dropped
- the 32-bit offset wrap and every SPRAM bank crossing
- memory safety on every clock cycle: no read of an unwritten or freed
  slot, and no overwrite of unacked data

To show the tests can actually fail, these deliberate bugs were planted
during development, and the testbench caught every one:
- accepting any ack
- skipping a sample
- ignoring the control CRC
- leaving a byte out of the CRC
- promising one sample more than is buffered
- removing the full check

That ran against the RTL before the timing-closure rework that pipelined the
decision and next-byte logic. The suite has not been rerun since.

The off-by-one-sample bug was the only one the data checks alone missed: the
sensor wrote that slot just before it was read. That is why the
memory-safety monitors exist.

**Not verified:** none of this has run on hardware. The SPRAM model comes
from yosys's `cells_sim.v`, which is the model this toolflow assumes, not
silicon. The 3 MHz ceiling is worked out from the synchronizer latency,
since simulation has no pad delays.
