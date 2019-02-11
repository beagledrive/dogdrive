PROJ = blink
BUILD =./BUILD
DEVICE = 8k
FOOTPRNT = tq144:4k

SRC = top.v
SRC +=
PIN_SRC = pinmap.pcf

.PHONY: all load clean


all:
	mkdir -p $(BUILD)
	yosys -q -p "synth_ice40 -top top -blif $(BUILD)/$(PROJ).blif" $(SRC)
	arachne-pnr -d $(DEVICE) -P $(FOOTPRNT) -p $(PIN_SRC) -o $(BUILD)/$(PROJ).asc $(BUILD)/$(PROJ).blif
	icepack $(BUILD)/$(PROJ).asc $(BUILD)/$(PROJ).bin
	echo -e "\x0\x0\x0\x0\x0\x0\x0" >> $(BUILD)/$(PROJ).bin

load:
	sh /home/debian/load-fw/bw-prog.sh $(BUILD)/$(PROJ).bin

clean:
	rm -rf ./$(BUILD)/
