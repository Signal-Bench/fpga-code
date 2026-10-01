# SignalBench FPGA → MCU stream: how to parse it

Specification for the ESP32-H2 firmware that reads decoded bus traffic from the
SignalBench FPGA (`stream_system/` on the `final-system` branch). This document is
self-contained: you shouldn't need to read the RTL.

**Verification status.** The two C files in this document were compiled with
`gcc -std=c99 -Wall -Wextra -pedantic` (no warnings) and run over a byte-for-byte capture
of MISO from the FPGA's simulation. They recovered every frame and every ACK the
testbench expected. **None of this has run on real hardware yet.** In particular, the
FPGA's SPI block was simulated with a behavioural model, not silicon.

---

## 1. The link

| | |
|---|---|
| Role | MCU is SPI **master**, FPGA is **slave** |
| Mode | SPI mode 0, MSB first, 8-bit |
| Clock | 2 MHz tested in simulation; earlier FPGA bring-up work found margin thin near 10 MHz |
| Pins (FPGA side) | SCK 11, CS 19, MOSI 21, MISO 13 |
| Reading | Clock out bytes with **MOSI = `0x00`**. `0x00` can never start a command. |

Everything the FPGA sends arrives on MISO as one continuous byte stream. **Read
boundaries (CS assertions) carry no meaning.** A frame can start in one read and finish
in the next, and a single read can contain several frames.

---

## 2. Commands (MCU → FPGA, on MOSI)

Send each command as **one transaction of exactly 4 bytes**.

| Command | Bytes | Response (FPGA → MCU) |
|---|---|---|
| Ready check | `53 42 A5 5A` | `53 42 4F 4B` ("SBOK") |
| Select I2C | `53 42 4D 00` | `53 42 41 00` |
| Select SPI | `53 42 4D 01` | `53 42 41 01` |
| Select DIO | `53 42 4D 02` | `53 42 41 02` |
| Select CAN | `53 42 4D 03` | `53 42 41 03` |
| Select UART | `53 42 4D 04` | `53 42 41 04` |

Rules:

1. **The response arrives in the *next* transaction**, not the one carrying the command.
   An SPI slave can't answer a byte it hasn't received yet. Do a read (e.g. 16 bytes)
   after the command and **look for the response anywhere in it**. In simulation it
   appeared at byte 0 and at byte 1; don't assume a fixed position. The parser below finds
   it for you.
2. **Bytes received while you are sending a command are stream data, and they're gone.**
   They left the FPGA's buffer when they were clocked out. Discard them, and **call
   `sbs_reset()` after every command**, because a frame you were halfway through can no
   longer be completed.
3. **A select flushes everything**: queued frames of the old protocol are discarded and the
   new decoder starts clean. Frames after the select ACK all belong to the new protocol.
   A single stale byte may still precede the ACK; the parser skips it.
4. **Unknown select values (`05`–`FF`) are ignored**: no response, protocol unchanged.
5. The FPGA powers up with **I2C** selected. Select explicitly anyway, so you get an ACK
   confirming the FPGA is alive and configured.
6. Avoid sending a ready check in the middle of streaming. Per rule 2 it costs you data.
   A select costs nothing extra, since it flushes anyway.

---

## 3. What arrives on MISO

The stream is a sequence of three kinds of thing, possibly separated by idle bytes:

| Kind | Starts with | Total size |
|---|---|---|
| Data frame | `53 42 4D` | 5 + length |
| Select ACK | `53 42 41` | 4 |
| Ready ACK | `53 42 4F` | 4 (`53 42 4F 4B`) |
| **Idle** | `FF` | 1, repeated: the FPGA had nothing to send |

**Idle `0xFF` only exists *between* frames.** Inside a frame, `0xFF` is ordinary data;
the length field tells you exactly where the frame ends.

### Data frame

```
 byte:   0    1    2    3                 4         5 ... 5+length-1
       ┌────┬────┬────┬─────────────────┬────────┬──────────────────┐
       │ 53 │ 42 │ 4D │ flags/protocol  │ length │ payload          │
       └────┴────┴────┴─────────────────┴────────┴──────────────────┘
```

| Field | Meaning |
|---|---|
| `53 42 4D` | Magic, "SBM". Every frame starts with it. |
| flags/protocol | **bits 3:0** protocol: `0` I2C, `1` SPI, `2` DIO, `3` CAN, `4` UART (same numbering as the select command)<br>**bit 7** `DROPPED`: data was lost (FPGA buffer full) somewhere between the previous delivered frame and this one<br>**bit 6** `CONT`: this frame continues the previous frame; see §6<br>**bits 5:4** reserved, always 0 |
| length | Number of **payload** bytes that follow, **1–255**. Does *not* count the 5 header bytes. Never 0. |
| payload | Protocol-specific; see §5 |

---

## 4. Parsing rules, and the reference parser

1. Feed bytes **one at a time**, in order, **across reads**. Keep parser state between
   reads.
2. Look for `53 42`; the third byte says what follows.
3. For a data frame, read the flags byte and length, then **exactly `length` bytes**.
4. **After a frame (or ACK) ends, the next byte must start a new `53 42`**. Anything else
   (normally idle `FF`) is skipped until one appears. Never look for the magic *inside* a
   payload; while reading a payload, every byte is data.
5. Sanity-check each header: protocol `0`–`4`, reserved bits `0`, length `≥ 1`. If any
   check fails, the "magic" was coincidental data seen during resynchronisation; drop back
   to hunting.
6. After sending any command, call `sbs_reset()`.

### `sbs_parser.h`

```c
/* SignalBench stream parser -- reference implementation from
 * MCU_FRAME_FORMAT.md.  Feed it every byte received on MISO, one at a time,
 * across SPI reads; it never needs to see a read boundary. */
#ifndef SBS_PARSER_H
#define SBS_PARSER_H
#include <stdint.h>

enum sbs_kind { SBS_FRAME, SBS_MODE_ACK, SBS_READY_ACK };

enum sbs_proto { SBS_I2C = 0, SBS_SPI = 1, SBS_DIO = 2, SBS_CAN = 3, SBS_UART = 4 };

#define SBS_FLAG_DROPPED 0x80   /* data was lost before this frame */
#define SBS_FLAG_CONT    0x40   /* continues the previous frame */

typedef struct {
	enum sbs_kind kind;
	uint8_t protocol;      /* FRAME: 0..4.  MODE_ACK: the protocol selected. */
	uint8_t flags;         /* FRAME: SBS_FLAG_* bits (protocol bits removed) */
	uint8_t length;        /* FRAME: payload bytes, 1..255 */
	uint8_t payload[255];
} sbs_msg_t;

typedef struct {
	uint8_t state, flags, length, count;
	sbs_msg_t msg;
} sbs_parser_t;

void sbs_reset(sbs_parser_t *p);
/* Returns 1 when p->msg holds a complete message, 0 otherwise. */
int  sbs_feed(sbs_parser_t *p, uint8_t b);
#endif
```

### `sbs_parser.c`

```c
#include "sbs_parser.h"

enum { HUNT0, HUNT1, TYPE, FLAGS, LEN, PAYLOAD, MODE_ACK, READY_ACK };

void sbs_reset(sbs_parser_t *p) { p->state = HUNT0; }

int sbs_feed(sbs_parser_t *p, uint8_t b)
{
	switch (p->state) {
	case HUNT0:                         /* 0xFF idle and junk land here */
		if (b == 0x53) p->state = HUNT1;
		return 0;
	case HUNT1:
		if (b == 0x42)      p->state = TYPE;
		else if (b != 0x53) p->state = HUNT0;
		return 0;
	case TYPE:
		if      (b == 0x4D) p->state = FLAGS;      /* data frame        */
		else if (b == 0x41) p->state = MODE_ACK;   /* 53 42 41 xx       */
		else if (b == 0x4F) p->state = READY_ACK;  /* 53 42 4F 4B       */
		else if (b == 0x53) p->state = HUNT1;
		else                p->state = HUNT0;
		return 0;
	case FLAGS:
		p->flags = b;
		p->state = LEN;
		return 0;
	case LEN:
		/* Sanity check: a false 53 42 4D inside payload usually fails it. */
		if ((p->flags & 0x0F) > SBS_UART || (p->flags & 0x30) != 0 || b == 0) {
			p->state = HUNT0;
			return 0;
		}
		p->length = b;
		p->count  = 0;
		p->state  = PAYLOAD;
		return 0;
	case PAYLOAD:
		p->msg.payload[p->count++] = b;
		if (p->count < p->length) return 0;
		p->msg.kind     = SBS_FRAME;
		p->msg.protocol = p->flags & 0x0F;
		p->msg.flags    = p->flags & 0xC0;
		p->msg.length   = p->length;
		p->state = HUNT0;                /* next byte must start 53 42 */
		return 1;
	case MODE_ACK:
		p->msg.kind     = SBS_MODE_ACK;
		p->msg.protocol = b;
		p->state = HUNT0;
		return 1;
	case READY_ACK:
		p->state = HUNT0;
		if (b != 0x4B) return 0;
		p->msg.kind = SBS_READY_ACK;
		return 1;
	}
	p->state = HUNT0;
	return 0;
}
```

### Using it

```c
sbs_parser_t parser;
sbs_reset(&parser);

/* after each SPI read of n bytes into rx[]: */
for (size_t i = 0; i < n; i++) {
	if (!sbs_feed(&parser, rx[i])) continue;
	const sbs_msg_t *m = &parser.msg;   /* valid until the next sbs_feed() */
	switch (m->kind) {
	case SBS_READY_ACK: /* FPGA alive */                       break;
	case SBS_MODE_ACK:  /* m->protocol is now selected */      break;
	case SBS_FRAME:     handle_frame(m);                       break;
	}
}
```

`parser.msg` is overwritten by the next completed message. Copy it out if you need to keep
it.

---

## 5. Payloads

### 5.1 UART — protocol `4`

**One frame per received byte**, or per pair if both lines delivered a byte at once. Each
byte is preceded by a **line tag**:

| length | payload | meaning |
|---|---|---|
| **2** | `[tag][byte]` | one line received a byte |
| **4** | `[01][byte A][02][byte B]` | both lines received a byte at the same moment |

| Tag | Line | FPGA pin | Usual meaning |
|---|---|---|---|
| `0x01` | line A | 38 | the target's **TX** line (target → peer) |
| `0x02` | line B | 42 | the target's **RX** line (peer → target) |

- **Parse by position**: tags are at offsets 0 and 2, data at 1 and 3. A data byte that
  happens to be `0x01` or `0x02` is never a tag. (Tested: `[01 02]` means "line A
  received `0x02`".)
- In a length-4 frame the order is always `01` then `02`. It says nothing about which byte
  arrived first; they were simultaneous.
- **"Simultaneous"** means both bytes finished on the same quarter-bit sampling tick: at
  9600 baud, within up to about 26 µs of each other. Bytes more than ¼ bit apart are always
  separate frames. A request on one line and its reply on the other are therefore normally
  two frames.
- Baud rate is **fixed at 9600** in this build (8-N-1, LSB first). There's no command to
  change it.
- A byte with a bad stop bit (framing error) is silently dropped and produces no frame.

Example: `send_pattern` sending `AA 12 34 FF FF 9A BC DE` on line A produces eight frames:

```
53 42 4D 04 02 01 AA
53 42 4D 04 02 01 12
53 42 4D 04 02 01 34
53 42 4D 04 02 01 FF     <- FF is data here: it's inside the payload
53 42 4D 04 02 01 FF
53 42 4D 04 02 01 9A
53 42 4D 04 02 01 BC
53 42 4D 04 02 01 DE
```

Both lines receiving `5C` at the same instant: `53 42 4D 04 04 01 5C 02 5C`.

### 5.2 I2C — protocol `0`

**One frame per segment**, where a segment runs from a START (or repeated START) to the
next STOP or repeated START.

```
payload = [ raw SDA bits, packed ... ][ info ]
          └── length - 1 bytes ──────┘└ 1 byte ┘
```

**The raw bits** are SDA sampled on every SCL rising edge, **ACK/NACK bits included**,
packed 8 to a byte, MSB first. I2C uses 9 clocks per byte (8 data bits plus ACK), so these
bytes do **not** line up with I2C bytes. You have to regroup them into 9-bit units; the
code below does it. The last byte is left-aligned and zero-padded.

**The info byte** (always last):

| bits | name | meaning |
|---|---|---|
| 7 | RSTART | this segment began with a **repeated** START (no STOP before it) |
| 6:5 | END | `00` STOP · `01` repeated START · `10` idle timeout (bus idle 1 ms with no STOP) · `11` split: continues in the next frame, which has `CONT` set |
| 4:3 | — | 0 |
| 2:0 | lastbits | valid bits in the last raw byte; **`0` means all 8**. If the payload is only the info byte, there are no raw bits. |

Total raw bits = `0` if `length == 1`, else `(length − 2) × 8 + (lastbits ? lastbits : 8)`.

Regrouped into 9-bit units, the raw bits give `[8 data bits][ACK]`, where ACK `0` = ACK
and `1` = NACK. The first unit after a START or repeated START is the **address byte**:
`addr = byte >> 1`, `R/W = byte & 1` (`1` = read). A segment usually ends with **1
leftover bit**: the SCL clock the master issues to set up STOP or repeated START. That's
expected; ignore it.

A typical register read, as the AS5600 does (write register address, repeated START,
read), is **two frames**: the first ends with END = `01`, the second has RSTART = 1.

#### Example, from simulation

Bus traffic: `START 0x36-W ACK 0x0B ACK  rSTART 0x36-R ACK 0x93 NACK  STOP`

```
53 42 4D 00 04 | 6C 05 A0 23     info 23 = END repeated START, lastbits 3
53 42 4D 00 04 | 6D 49 C0 83     info 83 = RSTART, END STOP, lastbits 3
```

Decoded with the code below:

```
START:  [addr 0x36 W] ACK 0b ACK   -> repeated START, 1 extra bit
rSTART: [addr 0x36 R] ACK 93 NACK  -> STOP, 1 extra bit
```

#### Reference decoder for I2C frames

```c
#include "sbs_parser.h"

/* Turns I2C frame payloads back into (byte, ACK/NACK) pairs.  Keep one of
 * these per stream; it carries 9-bit groups across CONT frames. */
typedef struct {
	uint16_t acc;      /* bits collected so far, newest in bit 0 */
	uint8_t  n;        /* how many (0..8) */
	uint8_t  is_addr;  /* next completed group is an address byte */
} sbs_i2c_t;

typedef void (*sbs_i2c_cb)(uint8_t data, int nack, int is_addr, void *ctx);

/* Feed one I2C frame (msg->protocol == SBS_I2C).  Calls on_byte for every
 * complete 9-bit group.  Returns the number of leftover bits (< 9) when the
 * segment ends, or -1 if it continues in a CONT frame. */
int sbs_i2c_frame(sbs_i2c_t *s, const sbs_msg_t *m, sbs_i2c_cb on_byte, void *ctx)
{
	uint8_t  info     = m->payload[m->length - 1];
	uint32_t ndata    = m->length - 1u;
	uint32_t lastbits = info & 0x07u;
	uint32_t nbits    = ndata ? (ndata - 1u) * 8u + (lastbits ? lastbits : 8u) : 0u;

	if (!(m->flags & SBS_FLAG_CONT)) {        /* a new segment: after (r)START */
		s->acc = 0; s->n = 0; s->is_addr = 1;
	}
	for (uint32_t i = 0; i < nbits; i++) {
		int bit = (m->payload[i >> 3] >> (7 - (i & 7))) & 1;
		s->acc = (uint16_t)((s->acc << 1) | bit);
		if (++s->n == 9) {
			on_byte((uint8_t)(s->acc >> 1), s->acc & 1, s->is_addr, ctx);
			s->acc = 0; s->n = 0; s->is_addr = 0;
		}
	}
	if (((info >> 5) & 3u) == 3u) return -1;   /* END = split, more follows */
	int left = s->n;
	s->acc = 0; s->n = 0;
	return left;
}
```

If you'd rather forward raw frames to the phone and decode there, the same algorithm
applies on that side.

### 5.3 SPI — protocol `1` (not tested on hardware or in the testbench)

One frame per chip-select assertion on the probed bus. Payload is
`[MOSI byte][MISO byte]` pairs, so length is always even. Transactions longer than 127 pairs
are split with `CONT`. Bits after the last whole byte of a transaction are dropped. SPI mode
0 only in this build.

### 5.4 CAN — protocol `3` (not tested on hardware or in the testbench)

One frame per CAN frame, payload always **18 bytes**:

| Offset | Size | Field |
|---|---|---|
| 0 | 1 | flags `{ide, rtr, overload, ack_ok, crc_ok, err[2:0]}` (bit 7 → bit 0) |
| 1 | 4 | identifier, big-endian, 29 bits right-justified (standard IDs use the low 11) |
| 5 | 1 | DLC in the low 4 bits |
| 6 | 8 | data; the first `min(DLC, 8)` bytes are valid (none for remote frames) |
| 14 | 4 | timestamp, big-endian, in **bit times** (µs at 1 Mbit/s), latched at start of frame |

`err`: 0 none, 1 stuff, 2 CRC, 3 form, 4 no ACK, 5 unattributed, 6 bus stuck dominant.
Fixed at 1 Mbit/s.

### 5.5 DIO — protocol `2` (not tested on hardware or in the testbench)

One frame per edge on pin 38, payload **5 bytes**: `[level][timestamp µs, 4 bytes
big-endian]`, where level is `01` after a rising edge and `00` after a falling one. An edge
within ~6 FPGA clocks (250 ns) of the previous one is missed.

---

## 6. Flags

**`DROPPED` (bit 7).** The FPGA's 4 KB buffer filled because it wasn't read fast enough,
and at least one whole frame was thrown away before this one. Frames are never truncated:
what you receive is always complete, but something is missing before it. Report it, and
for I2C/SPI, discard any partial transaction you were assembling.

**`CONT` (bit 6).** This frame carries on from the previous one, because a single I2C
segment or SPI transaction didn't fit in 255 bytes. For I2C, keep feeding the same
`sbs_i2c_t`; the decoder joins the bitstream across the boundary. UART never sets `CONT`.
If a frame has both `CONT` and `DROPPED`, the start of the transaction it continues was
lost.

---

## 7. Throughput: the one thing that will bite in practice

The SPI link at 2 MHz can move ~250 kB/s, but you only get what you **poll**. Reading 64
bytes every 10 ms is **6.4 kB/s**.

| Traffic | Frame bytes per second | Fits in 6.4 kB/s? |
|---|---|---|
| `send_pattern` (8 bytes, then 10 ms pause, 9600 baud) | ~3.1 kB/s | yes |
| one UART line **saturated** at 9600 | 960 frames × 7 B = **6.7 kB/s** | **no**, slowly overflows |
| both UART lines saturated at 9600 | up to 13.4 kB/s | no |
| I2C at 100 kHz, **saturated** | ~12.5 kB/s | no |
| I2C sensor polling (AS5600-style) | well under 1 kB/s | yes |

Each UART byte costs 7 bytes on the wire (5 header + tag + data). When the FPGA can't keep
up, frames are dropped and `DROPPED` appears on the next one. The buffer absorbs about 4 KB
of backlog before that happens.

**Suggested fix: poll adaptively.** If a read ended inside a frame, or didn't end in idle
`FF` bytes with the parser between frames, read again immediately instead of waiting for
the next 10 ms tick:

```c
/* after feeding a read: more data is waiting unless we're between frames
 * and the tail of the read was idle */
int more_waiting = (parser.state != 0 /* HUNT0 */) || (rx[n - 1] != 0xFF);
```

---

## 8. Quick reference

```
Command:   53 42 4D xx     select (xx: 00 I2C, 01 SPI, 02 DIO, 03 CAN, 04 UART)
           53 42 A5 5A     ready check
ACKs:      53 42 41 xx     selected xx         (in the NEXT transaction)
           53 42 4F 4B     ready               (in the NEXT transaction)
Frame:     53 42 4D  FP  LEN  payload[LEN]
           FP: bit7 DROPPED, bit6 CONT, bits3:0 protocol
Idle:      FF (only between frames)
UART:      LEN 2: [tag][b]   LEN 4: [01][a][02][b]     tag 01 = pin 38, 02 = pin 42
I2C:       [raw SDA bits incl. ACKs, 9 per I2C byte][info]
           info: b7 RSTART, b6:5 END (00 STOP 01 rSTART 10 timeout 11 split), b2:0 lastbits
After any command: sbs_reset().
```
