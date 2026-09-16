

cmos for uart ttl for spi

SPI MSB

make build
make flash


Notes from 25:
- gpio 28 does not work
- MSB SPI


Notes from 31:
- LSB UART
- Polarity Idle high


Notes from april 7:
- 3 bugs
    - only stores the read part when no magnet
    - with magnet it does not store the last ack byte for the write part
    - with magnet it does send the write byte at the end but logic analyzer does not pick it up
Notes from Sept 16 (CAN sniffer bring-up):
- VP230 (SN65HVD230) transceiver: do NOT tie RS to 3.3V. "Standby / Listen Only"
  mode leaves RXD stuck recessive - no data at 1 Mbit/s. Confirmed with a scope:
  CANH/CANL carried clean traffic while RXD sat permanently high. Chip marking was
  genuinely VP230, not a mis-populated VP231. Leaving RS on the breakout's fitted
  10k pulldown (slope-control mode) works immediately.
  Datasheet says the opposite - sec 10.4.3 calls RS-high "Listen Only", Table 2 says
  RXD "Mirrors Bus State", and sec 8.8 receiver timing has no RS condition. An absent
  degraded-timing spec is not a guarantee of full-rate operation in that mode.
- Consequence: driver is ENABLED in slope-control mode, so the TXD/CTX strap to 3.3V
  is the entire passivity guarantee. Solder it. Never leave it floating.
- UPduino header pin 2 is VIO, NOT a 3.3V supply - it is the bank I/O voltage input
  and floats by default (R19/R26 unpopulated), meters ~2.3V of leakage. 3.3V is pin 9.
  Quick rail check: 3.3V powers the FT232H, so if lsusb shows 0403:6014 the rail is ok.
- Logic-analyzer threshold for RXD: 1.65V. Not a 2.5V "CMOS" preset - receiver VOH min
  is 2.4V, below it.
- Dev Board Type C has no CAN termination on any of its 4 connectors, and wires each
  bus to two parallel connectors (pass-through). GM6020 DIP 4 = terminator, DIP 1-3 =
  ID with 000 invalid.
