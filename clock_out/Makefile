filename = clock_out
pcf_file = ../common/upduino_v3.pcf

build:
	yosys -p "synth_ice40 -top top -json $(filename).json" $(filename).v
	nextpnr-ice40 --up5k --package sg48 --json $(filename).json --pcf $(pcf_file) --asc $(filename).asc
	icepack $(filename).asc $(filename).bin

flash:
	iceprog -d i:0x0403:0x6014 $(filename).bin # Flash to FTDI

clean:
	rm -rf $(filename).blif $(filename).asc $(filename).bin $(filename).json
