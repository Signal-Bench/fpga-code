

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