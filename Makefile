#
# Makefile for both curses version and arduino version
#
# To make curses version (builds both debug and non-debug executables):
#		make
#
# Creates executables 'dsky' and 'dsky_debug'.
#
# To make arduino version:
#		make sketch
#
# To upload arduino version
#		make upload
#

#
# Arduino command line programs
#
ARDUINO_CLI=/M/kjs/ARDUINO/bin/arduino-cli
AVR_OBJDUMP=/M/kjs/ARDUINO/.arduino15/packages/arduino/tools/avr-gcc/7.3.0-atmel3.6.1-arduino7/bin/avr-objdump

DIR=KennysOpenDSKY

$(DIR)/dsky: $(DIR)/KennysOpenDSKY.cpp $(DIR)/kennysagc.h
	gcc -DCURSES_SIMULATOR -lncurses $(DIR)/KennysOpenDSKY.cpp -o $(DIR)/dsky
	gcc -DCURSES_SIMULATOR -DDSKY_DEBUG -g -lncurses $(DIR)/KennysOpenDSKY.cpp -o $(DIR)/dsky_debug

$(DIR)/kennysagc.h: $(DIR)/kennysagc.asm
	./assembler.py $(DIR)/kennysagc.asm

sketch:
	$(ARDUINO_CLI) compile -e --fqbn arduino:avr:nano KennysOpenDSKY
	$(AVR_OBJDUMP) -d KennysOpenDSKY/build/arduino.avr.nano/KennysOpenDSKY.ino.elf > $(DIR)/KennysOpenDSKY.dump

upload:
	$(ARDUINO_CLI) upload -p /dev/ttyUSB0 --fqbn arduino:avr:uno KennysOpenDSKY
