# KennysOpenDSKY
Kenny's Open DSKY Software

![alt text](images/panorama_dsky.jpg "")

## Table Of Contents
+ [KennysOpenDSKY](#kennysopendsky)
  + [Table Of Contents](#table-of-contents)
  + [Description](#description)
  + [Acknowledgments](#acknowledgments)
  + [Features](#features)
  + [Files](#files)
  + [Memory Usage](#memory-usage)
  + [Dependencies](#dependencies)
    + [Arduino](#arduino)
    + [Linux](#linux)
  + [Compiling for Arduino Nano](#compiling-for-arduino-nano)
  + [Compiling for Linux](#compiling-for-linux)
    + [Running the Curses Sumulator](#running-the-curses-sumulator)
  + [Compiling for Windows/Macos](#compiling-for-windowsmacos)
  + [Using the Assembler](#using-the-assembler)
  + [Assembly Language](#assembly-language)
    + [Virtual Machine Architecture](#virtual-machine-architecture)
    + [Comments](#comments)
    + [Labels](#labels)
    + [Scope Brackets](#scope-brackets)
    + [Directives](#directives)
    + [Instruction Mnemonic Suffixes](#instruction-mnemonic-suffixes)
    + [Instruction Arguments](#instruction-arguments)
    + [Instructions](#instructions)
  + [Running the Curses Simulator](#running-the-curses-simulator)
    + [Keys](#keys)
    + [log file](#log-file)
    + [persistent data](#persistent-data)
  + [DSKY Usage](#dsky-usage)
    + [General Notes:](#general-notes)
    + [VERBs, NOUNs, and PROGRAMs](#verbs-nouns-and-programs)
    + [Verbs](#verbs)
    + [Nouns](#nouns)
    + [Verb-Nouns](#verb-nouns)
    + [Programs](#programs)
  + [Green Plexi-glass Modification](#green-plexi-glass-modification)
  + [Review of the Open DSKY Kit](#review-of-the-open-dsky-kit)
    + [Sticker 1](#sticker-1)
    + [Sticker 2](#sticker-2)
    + [Mini-DSKY](#mini-dsky)
  + [Author](#author)

## Description
This project contains C/C++ source code for the Arduino nano which
will control the Open DSKY kickstarter kit.

This project also contains source code for Linux/Windows/Macos which
runs as `ncurses` text-based simulator of the arduino software. This
will be referred to as the **curses simulator**.

The **curses simulator** allows for testing and debugging the complete software
without having to upload to the Arduino nano.

![alt text](images/v16n36.jpg "")

## Acknowledgments
This is an original implementation of code needed to drive the
Open DSKY hardware. However, I derived many routines from these sources:

+ **S&T Geotronics** James Sanderson and Marc Tessier for creating the
	Open DSKY kit. <http://www.stgeotronics.com>

+ **Scott Pavlovec** github project. <https://github.com/scottpav/OpenDSKY>. This
	contained the reference implementation which I used to talk to the hardware.

+ The functionality offered was inspired by the functionality offered by
		the **Apollo 50th Anniversary** project which came pre-installed with the Open DSKY kit.
	(Apollo Education Experience Program, Cumming Georgia).
	Thanks **Bill Walker** for an incredible software and user manual.
	Website 1: <http://apolloexperience.com>.
	Website 2: <https://www.gofundme.com/apollo-education-experience-project>

+ The audio clips provided here are taken from the **Apollo 50th Anniversary** project SD card.

+ The following website has information about the virtual apollo guidance
	computer: <http://www.ibiblio.org/apollo/>.

+ An online apollo DSKY emulator <https://svtsim.com/moonjs/agc.html>. I used this great tool to
	tweak my code to better reflect how an actual Apollo DSKY worked.

## Features
+ A virtual machine and byte code interpreter
+ Two threads of control: One thread is a background task, which
	runs major modes. The other thread is a foreground task which runs simple verbs/nouns.
+ Ability to enter values using the keypad instead of +/- keys.
+ More accurately reproduces the Apollo DSKY interface.
+ **Curses Simulator** - test and develop your code on your computer before uploading it to the Arduino.

## Files
The main source code is in `KennysOpenDSKY.cpp`. 

+ `KennysOpenDSKY.cpp` - the main source code file C/C++.
+ `kennysagc.asm` - the assembly code for the verb/noun/prog programs.
+ `kennysagc.h` - the assembled code. produced by running `assembler.py`.
+ `KennysOpenDSKY.ino` - an empty file to satisfy the Arduino CLI sketch requirements.
+ `KennysOpenDSKY.dump` - a dump of the AVR nano assembly code for the Arduino version
+ `assembler.py` - Python 3 program which assembles the assembly code into byte codes.
+ `Makefile` - A simple makefile to compile on Linux and also compile/upload the
		sketch using the Arduino CLI tools.
+ `dsky` - The **curses simulator** executable produced on linux
+ `dsky_debug` - The **curses simulator** executable produced on linux with debugging symbols (-g).
+ `log.txt` - a log file for debugging when running the **curses simulator**.
+ `persist.txt` - emulates the EEPROM and RTC RAM area when using the **curses simulator**.
+ `audio` - directory containing the SD card audio files for the MP3 player.
+ `images` - directory containing images used for documentation purposes. Also contains a
	`pdf` and `pages` document for use with the **Green Plexi-glass** modification.

## Memory Usage
The current build uses the following memory on the Arduino:

```text
$ make sketch
arduino-cli compile -e --fqbn arduino:avr:nano KennysOpenDSKY
Sketch uses 22292 bytes (72%) of program storage space. Maximum is 30720 bytes.
Global variables use 1288 bytes (62%) of dynamic memory, leaving 760 bytes for local variables.
Maximum is 2048 bytes.

Used library      Version Path                                                                          
Adafruit NeoPixel 1.12.0  /M/kjs/ARDUINO/Arduino/libraries/Adafruit_NeoPixel                            
LedControl        1.0.6   /M/kjs/ARDUINO/Arduino/libraries/LedControl                                   
TinyGPSPlus       1.0.3   /M/kjs/ARDUINO/Arduino/libraries/TinyGPSPlus                                  
Wire              1.0     /M/kjs/ARDUINO/.arduino15/packages/arduino/hardware/avr/1.8.6/libraries/Wire  
EEPROM            2.0     /M/kjs/ARDUINO/.arduino15/packages/arduino/hardware/avr/1.8.6/libraries/EEPROM

Used platform Version Path                                                         
arduino:avr   1.8.6   /M/kjs/ARDUINO/.arduino15/packages/arduino/hardware/avr/1.8.6
```

## Dependencies
### Arduino
This section describes the third party libraries needed to compile
the sketch for Arduino.

+ **Wire** - standard arduino library. I2C communications.
+ **EEPROM** - standard arduino library for reading/writing the 2K eeprom memory.
+ **Adafruit NeoPixel** - library used to illuminate the neo pixels.
+ **LedControl** - library used to talk to 7-segment LEDs.
+ **TinyGPS++** - library to read the GPS device and parse its output.
+ **Arduino CLI tools for linux** - I don't use the Arduino IDE. I use `arduino-cli` tool.
+ **python3** - this is needed to run `assembler.py`. The assembler is a simple text only
		python program which should work on most python installations.

### Linux
This section describes the notable libraries needed to compile
the curses simulator on linux/macos/windows.

+ **ncurses** - this is a library for drawing simple text based user interfaces. It
		is widely available, and comes installed by default on most linux distros.

+ **sigaction()** - this is capability of most unixes. It is included on all linux operating
			systems. Special porting may be needed for Windows or Macos. It is used to
			implement the 100ms timer.

+ **getrandom()** - this is a library function provdided by linux. It is used to provide
			random numbers to the code for blinking the **Uplink Acty**
			and **Comp Acty** lights. As well as the random number assembly instruction.

+ **python3** - this is needed to run `assembler.py`. The assembler is a simple text only
		python program which should work on most python installations.


## Compiling for Arduino Nano
I use the Arduino Command Line Interface. I run this under linux on a Raspberry Pi.
The included ```Makefile``` shows the commands needed to compile the sketch and
upload to your Open DSKY kit.

To assemble the assembly code use:
```
	$ ./assembler.py kennysagc.asm
```
This will produce a file called `kennysagc.h` which is included by `KennysOpenDSKY.cpp`.

To compile the sketch use:
```
	$ arduino-cli compile -e --fqbn arduino:avr:nano KennysOpenDSKY
```

To upload the sketch use:
```
	$ arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:uno KennysOpenDSKY
```
Replace **/dev/ttyUSB0** with whatever is required on your system.

The provided simple `Makefile` encapsulates these commands. This builds the sketch:
```
	$ make sketch
```

This uploads the sketch to your Arduino Nano:
```
	$ make upload
```


## Compiling for Linux
To compile on linux `gcc` is used. The program itself is written in simple C/C++.
(Does not use any fancy features of C++ beyond `class`).

To assemble the assembly code use:
```
	$ ./assembler.py kennysagc.asm
```
This produces a file caled `kennysagc.h` which is included by `KennysOpenDSKY.cpp`.

This builds the `dsky` executable:
```
	$ gcc -DCURSES_SIMULATOR -lncurses KennysOpenDSKY.cpp -o dsky
```

This builds the `dsky_debug` executable (containing debug symbols):
```
	$ gcc -DCURSES_SIMULATOR -DDSKY_DEBUG -g -lncurses KennysOpenDSKY.cpp -o dsky_debug
```

The provided simple Makefile builds both executables with this command:
```
	$ make
```

### Running the Curses Sumulator
To run the executable simply run it as follows (no command line arguments are needed):
```
	$ ./dsky
```

or,

```
	$ ./dsky_debug
```

or (when debugging),
```
	$ gdb ./dsky_debug
```

## Compiling for Windows/Macos
I don't know how to compile for these platforms. Make sure
you have the ```ncurses``` library available. Make sure you have sigaction() available.
Make sure you have getrandom(). The compilation should be pretty straight forward.

## Using the Assembler
The program ``assembler.py`` is a simple one pass assembler written in python.
You will need python3 to recompile the assembly. This github repository
however contains a pre-compiled version of the assembly.

## Assembly Language
This section documents all the assembly instructions.

### Virtual Machine Architecture
Here is the CPU architecture of the virtual machine:

![alt text](images/cpuarch.jpg "")

The CPU consists of three 32-bit registers called **A**, **B** and **C**. Different instructions
operate on different registers. The **C** register has special indirect addressing modes
associared with it, so it can be thought of as the "index" register. The **A** register
can be thought of as the "accumulator", although many of the same instructions also
work with the **B** register. These registers are 32-bits wide.

The stack pointer **SP** points into the RAM area. The RAM area consists of 128 memory locations
each capable of storing a 32-bit value.

The program counter **PC** points into the Program[] array which contains the byte codes
that were assembled using the assembler.

The program as access to a bank of RAM locations which are 32-bits wide. There are 128 RAM locations.
This is to hold variables for the assembly programs. It also contains a small stack for each cpu.

The program bytes codes are read-only. You can store data tables in this memory area. It
mostly contains the subroutines.

There are two CPU's defined. **cpu 0** is the background thread. **cpu 1** is the foreground thread.
The background thread is designed to run major modes or PROGRAMS. The foreground task runs verbs and noun
combinations.

The foreground task can be paused and a new foreground task stacked on top of the currently
running foreground task. For example, If `V16 N36` is running to show the current time.
If the user runs the `V35` (LAMP TEST) verb, then this will pause the current verb/noun and
run the LAMP TEST code. When the LAMP TEST finishes the previous Verb/Noun will be resumed.

### Comments
Use C++ `//` style comments in the assembly code. I.e.,

```
Loop:
		BRANCH Loop			// loop forever
```

### Labels
Labels are symbol which appear

### Scope Brackets
The curly braces `{` and `}` are scope brakcet. They can appear on a line
by themselves. Any labels defined within scoped brackets are removed from
the symbol table after the end scope bracket. This allows reusing symbols
within the body of a function.

```
VERB_34:
{				// begin scope
Loop:
		GOTO Loop
}				// end scope

VERB_35:
{				// begin scope
Loop:	GOTO Loop
}				// end scope
```

The symbol `Loop` was reused becuse it appeares in seperate scope blocks.
The symbols `VERB_34` and `VERB_35` are not inside of scope brackets. These
symbols will be available for adding to the Verb[] dispatch table.

### Directives
The three directives are `DATA8`, `DATA16` and 'DEFINE'.

```
		DATA8		0xFF
		DATA8		0x1F
		DATA16		SomeLabel
		DEFINE Symbol = 100
		DEFINE JUNK   = 0x45
```

The `DATA` directives compile raw data into the Program instructions stream.
This can be used to implement lookup tables. The `DEFINE` directive associates
a value with a symbol.

### Instruction Mnemonic Suffixes

The instruction mnemonics use these suffixes to indicate the addressing modes/arguments:

+ `_IMM8` - Takes an immediate 8-bit value
+ `_IMM16` - Takes an immediate 16-bit value
+ `_IMM32` - Takes an immediate 16-bit value
+ `_DIRECT` - Takes an 8-bit value which represents a RAM memory location
+ `_CDIRECT` - Takes an 8-bit address constant and adds it to the C register
				to form the effective address. `C+<addr>`
+ `_INDIRECT_C` - The C register contains the address to RAM. RAM[C]
+ `_U`			- the instruction operates in the unsigned domain
+ `_OCT`		- the instruction operates using Octal radix instead of decimal

### Instruction Arguments
These are the types of arguments instructions can have:

+ `<offset>` - An 8-bit signed value which is added to the program counter when the branch is taken.
+ `<imm8>` - An 8-bit value provided immediately in the Program byte code stream
+ `<imm16>` - A 16-bit value provided immediately in the Program byte code stream
+ `<imm32>` - A 32-bit value provided immediately in the Program byte code stream
+ `<addr>` - An 8-bit unsigned value which refers to a RAM location.
+ `<addrMin>` - An 8-bit unsigned value which refers to a RAM location. Refers to the minimum value
		in the range of values.
+ `<addrMax>` - An 8-bit unsigned value which refers to a RAM location. Refers to the maximum value in
		the range of values.

In the assembly syntax argument are provided seperated by whitspace. Numeric literal can use
signed decimal notation or unsined hex notation. I.e.,

```
    0x4E            // hex literal
    -4              // decimal literal 8-bits
    1234            // decimal literal 16-bits
    1234999         // decimal literal 32-bits
    0x4E001F2F      // 32-bit hex literal
```

### Instructions

Here are all the assembly instructions:

| Mnemonic          | Arguments  | Description                                                          |
|-------------------|------------|----------------------------------------------------------------------|
| MOV_R1_A          |            | Move the BCD encoded contents of DSKY register R1 into A             |
| MOV_R2_A          |            | Move the BCD encoded contents of DSKY register R2 into A             |
| MOV_R3_A          |            | Move the BCD encoded contents of DSKY register R3 into A             |
| MOV_A_R1          |            | Move the BCD encoded contents of register A into DSKY register R1    |
| MOV_A_R2          |            | Move the BCD encoded contents of register A into DSKY register R2    |
| MOV_A_R3          |            | Move the BCD encoded contents of register A into DSKY register R3    |
| DECODE_A_FROM_OCT |            | Convert BCD encoding of octal value in A into a INT                  |
| DECODE_A_FROM_DEC |            | Convert BCD encoding of decimal into a INT                           |
| ENCODE_A_TO_OCT   |            | Encode A contents as BCD encoded octal                               |
| ENCODE_A_TO_DEC   |            | Encode A contents as BCD encoded decimal (plus/minus sign)           |
| ENCODE_A_TO_UDEC  |            | Encode Positive decimal value (No plus sign) into BCD                |
| MOV_A_B           |            | Move contents of A into B                                            |
| MOV_A_C           |            | Move contents of A into C                                            |
| MOV_B_A           |            | Move contents of B into A                                            |
| MOV_B_C           |            | Move contents of B into C                                            |
| MOV_C_A           |            | Move contents of C into A                                            |
| MOV_C_B           |            | Move contents of C into B                                            |
| LD_A_DIRECT       | addr       | Load into the A register the contents of RAM at addr                 |
| LD_B_DIRECT       | addr       | Load into the B register the contents of RAM at addr                 |
| LD_C_DIRECT       | addr       | Load into the C register the contents of RAM at addr                 |
| LD_A_IMM32        | imm32      | Load into the A register the immediate 32-bit value give by imm32    |
| LD_B_IMM32        | imm32      | Load into the B register the immediate 32-bit value give by imm32    |
| LD_C_IMM32        | imm32      | Load into the C register the immediate 32-bit value give by imm32    |
| LD_A_IMM16        | imm16      | Load into the A register the immediate 16-bit value give by imm16    |
| LD_B_IMM16        | imm16      | Load into the B register the immediate 16-bit value give by imm16    |
| LD_C_IMM16        | imm16      | Load into the C register the immediate 16-bit value give by imm16    |
| LD_A_IMM8         | imm8       | Load into the A register the immediate 8-bit value give by imm8      |
| LD_B_IMM8         | imm8       | Load into the B register the immediate 8-bit value give by imm8      |
| LD_C_IMM8         | imm8       | Load into the C register the immediate 8-bit value give by imm8      |
| LD_A_CDIRECT      | addr       | Load A register with contents of RAM at address C+addr               |
| LD_B_CDIRECT      | addr       | Load B register with contents of RAM at address C+addr               |
| LD_A_INDIRECT_C   |            | Load A register with contents of RAM at address given by C           |
| LD_B_INDIRECT_C   |            | Load B register with contents of RAM at address given by C           |
| ST_A_DIRECT       | addr       | Store A register to the RAM location given by addr                   |
| ST_B_DIRECT       | addr       | Store B register to the RAM location given by addr                   |
| ST_C_DIRECT       | addr       | Store C register to the RAM location given by addr                   |
| ST_A_CDIRECT      | addr       | Store A register to the RAM location given by C+addr                 |
| ST_B_CDIRECT      | addr       | Store B register to the RAM location given by C+addr                 |
| ST_A_INDIRECT_C   |            | Store A register to the RAM location given by C                      |
| ST_B_INDIRECT_C   |            | Store B register to the RAM location given by C                      |
| CLR_A             |            | Clear the A register to a value of 0                                 |
| CLR_B             |            | Clear the B register to a value of 0                                 |
| CLR_C             |            | Clear the C register to a value of 0                                 |
| INC_A             |            | Increment the A register by 1                                        |
| INC_B             |            | Increment the B register by 1                                        |
| INC_C             |            | Increment the C register by 1                                        |
| DEC_A             |            | Decrement the A register by 1                                        |
| DEC_B             |            | Decrement the B register by 1                                        |
| DEC_C             |            | Decrement the C register by 1                                        |
| PUSH_A            |            | Store the A register in RAM at address given by SP. Decrement SP     |
| PUSH_B            |            | Store the B register in RAM at address given by SP. Decrement SP     |
| PUSH_C            |            | Store the C register in RAM at address given by SP. Decrement SP     |
| POP_A             |            | Increment SP. Load the A register from RAM address given by SP       |
| POP_B             |            | Increment SP. Load the B register from RAM address given by SP       |
| POP_C             |            | Increment SP. Load the C register from RAM address given by SP       |
| CALL              | addr16     | Call subroutine at addr16. Push PC+3 on the stack                    |
| RET               |            | Pop program counter from the stack                                   |
| GOTO              | addr16     | Store addr16 into the program counter PC                             |
| BRANCH            | offset     | Add signed 8-bit value to program counter                            |
| BRANCH_A_GT_B     | offset     | If A >  B then add the branch offset to PC. Else next instruction    |
| BRANCH_A_GE_B     | offset     | If A >= B then add the branch offset to PC. Else next instruction    |
| BRANCH_A_LE_B     | offset     | If A <= B then add the branch offset to PC. Else next instruction    |
| BRANCH_A_LT_B     | offset     | If A < B then add the branch offset to PC. Else next instruction     |
| BRANCH_A_EQ_B     | offset     | If A == B then add the branch offset to PC. Else next instruction    |
| BRANCH_A_NE_B     | offset     | If A != B then add the branch offset to PC. Else next instruction    |
| BRANCH_A_GT_DIRECT | addr offset | If A >  RAM[addr] then add offset to PC. Else next instr.          |
| BRANCH_A_GE_DIRECT | addr offset | If A >= RAM[addr] then add offset to PC. Else next instr.          |
| BRANCH_A_LE_DIRECT | addr offset | If A <= RAM[addr] then add offset to PC. Else next instr.          |
| BRANCH_A_LT_DIRECT | addr offset | If A < RAM[addr] then add offset to PC. Else next instr.           |
| BRANCH_A_EQ_DIRECT | addr offset | If A == RAM[addr] then add offset to PC. Else next instr.          |
| BRANCH_A_NE_DIRECT | addr offset | If A != RAM[addr] then add offset to PC. Else next instr.          |
| BRANCH_A_GT_IMM8   | imm8 offset | If A >  imm8 then add offset to PC. Else next instr.               |
| BRANCH_A_GE_IMM8   | imm8 offset | If A >= imm8 then add offset to PC. Else next instr.               |
| BRANCH_A_LE_IMM8   | imm8 offset | If A <= imm8 then add offset to PC. Else next instr.               |
| BRANCH_A_LT_IMM8   | imm8 offset | If A < imm8 then add offset to PC. Else next instr.                |
| BRANCH_A_EQ_IMM8   | imm8 offset | If A == imm8 then add offset to PC. Else next instr.               |
| BRANCH_A_NE_IMM8   | imm8 offset | If A != imm8 then add offset to PC. Else next instr.               |
| BRANCH_A_GT_IMM16  | imm16 offset | If A >  imm16 then add offset to PC. Else next instr.             |
| BRANCH_A_GE_IMM16  | imm16 offset | If A >= imm16 then add offset to PC. Else next instr.             |
| BRANCH_A_LE_IMM16  | imm16 offset | If A <= imm16 then add offset to PC. Else next instr.             |
| BRANCH_A_LT_IMM16  | imm16 offset | If A < imm16 then add offset to PC. Else next instr.              |
| BRANCH_A_EQ_IMM16  | imm16 offset | If A == imm16 then add offset to PC. Else next instr.             |
| BRANCH_A_NE_IMM16  | imm16 offset | If A != imm16 then add offset to PC. Else next instr.             |
| BRANCH_B_GT_DIRECT | addr offset | If B > RAM[addr] then add offset to PC. Else next instr.           |
| BRANCH_B_GE_DIRECT | addr offset | If B >= RAM[addr] then add offset to PC. Else next instr.          |
| BRANCH_B_LE_DIRECT | addr offset | If B <= RAM[addr] then add offset to PC. Else next instr.          |
| BRANCH_B_LT_DIRECT | addr offset | If B < RAM[addr] then add offset to PC. Else next instr.           |
| BRANCH_B_EQ_DIRECT | addr offset | If B == RAM[addr] then add offset to PC. Else next instr.          |
| BRANCH_B_NE_DIRECT | addr offset | If B != RAM[addr] then add offset to PC. Else next instr.          |
| BRANCH_B_GT_IMM8   | imm8 offset | If B > imm8 then add offset to PC. Else next instr.                |
| BRANCH_B_GE_IMM8   | imm8 offset | If B >= imm8 then add offset to PC. Else next instr.               |
| BRANCH_B_LE_IMM8   | imm8 offset | If B <= imm8 then add offset to PC. Else next instr.               |
| BRANCH_B_LT_IMM8   | imm8 offset | If B < imm8 then add offset to PC. Else next instr.                |
| BRANCH_B_EQ_IMM8   | imm8 offset | If B == imm8 then add offset to PC. Else next instr.               |
| BRANCH_B_NE_IMM8   | imm8 offset | If B != imm8 then add offset to PC. Else next instr.               |
| BRANCH_B_GT_IMM16  | imm16 offset | If B > imm16 then add offset to PC. Else next instr.              |
| BRANCH_B_GE_IMM16  | imm16 offset | If B >= imm16 then add offset to PC. Else next instr.             |
| BRANCH_B_LE_IMM16  | imm16 offset | If B <= imm16 then add offset to PC. Else next instr.             |
| BRANCH_B_LT_IMM16  | imm16 offset | If B < imm16 then add offset to PC. Else next instr.              |
| BRANCH_B_EQ_IMM16  | imm16 offset | If B == imm16 then add offset to PC. Else next instr.             |
| BRANCH_B_NE_IMM16  | imm16 offset | If B != imm16 then add offset to PC. Else next instr.             |
| BRANCH_NOT_TIMER1  | offset    | branch if timer1 (100ms) hasn't triggered, else next instruction     |
| BRANCH_NOT_TIMER2  | offset    | branch if timer2 (200ms) hasn't triggered, else next instruction     |
| BRANCH_NOT_TIMER3  | offset    | branch if timer3 (300ms) hasn't triggered, else next instruction     |
| BRANCH_NOT_TIMER4  | offset    | branch if timer4 (1s) hasn't triggered, else next instruction        |
| BRANCH_NOT_TIMER5  | offset    | branch if timer5 (2s) hasn't triggered, else next instruction        |
| SWAP_A_B           |           | Swap registers A and B                                               |
| SWAP_A_C           |           | Swap registers A and C                                               |
| SWAP_B_C           |           | Swap registers B and C                                               |
| ADD_A_B            |           | A = A + B                                                            |
| ADD_B_A            |           | B = B + A                                                            |
| SUB_A_B            |           | A = A - B                                                            |
| SUB_B_A            |           | B = B - A                                                            |
| MUL_A_B            |           | A = A * B                                                            |
| MUL_B_A            |           | B = B * A                                                            |
| DIV_A_B            |           | A = A / B                                                            |
| DIV_B_A            |           | B = B / A                                                            |
| MOD_A_B            |           | A = A % B                                                            |
| MOD_B_A            |           | B = B % A                                                            |
| AND_A_B            |           | A = A & B                                                            |
| AND_B_A            |           | B = B & A                                                            |
| OR_A_B             |           | A = A \| B                                                           |
| OR_B_A             |           | B = B \| A                                                           |
| OR_A_IMM32         | imm32     | A = A \| imm32                                                       |
| OR_B_IMM32         | imm32     | B = B \| imm32                                                       |
| AND_A_IMM32        | imm32     | A = A & imm32                                                        |
| AND_B_IMM32        | imm32     | B = B & imm32                                                        |
| LSHIFT_A_IMM8      | imm8      | A = A << imm8                                                        |
| LSHIFT_B_IMM8      | imm8      | B = B << imm8                                                        |
| RSHIFT_A_IMM8      | imm8      | A = A >> imm8                                                        |
| RSHIFT_B_IMM8      | imm8      | B = B >> imm8                                                        |
| NOT_A              |           | A = ~A                                                               |
| NOT_B              |           | B = ~B                                                               |
| NEG_A              |           | A = -A                                                               |
| NEG_B              |           | B = -B                                                               |
| MOV_A_VERB         |   | Move LSB byte of register A into VERB DSKY field (assumed to be BCD encoded) |
| MOV_A_NOUN         |   | Move LSB byte of register A into NOUN DSKY field (assumed to be BCD encoded) |
| MOV_A_PROG         |   | Move LSB byte of register A into PROG DSKY field (assumed to be BCD encoded) |
| MOV_NOUN_A         |   | Move NOUN DSKY field into LSB byte of register A                             |
| MOV_VERB_A         |   | Move VERB DSKY field into LSB byte of register A                             |
| MOV_PROG_A         |   | Move PROG DSKY field into LSB byte of register A                             |
| BLINK_VERB         | imm8      | 1 or 0. enable/disable blinking of VERB digits                       |
| BLINK_NOUN         | imm8      | 1 or 0. enable/disable blinking of NOUN digits                       |
| BLINK_PROG         | imm8      | 1 or 0. enable/disable blinking of PROG digits                       |
| BLINK_KEYREL       | imm8      | 1 or 0. enable/disable blinking of KEYREL status light               |
| BLINK_OPRERR       | imm8      | 1 or 0. enable/disable blinking of OPR ERR status light              |
| BLINK_R1           | imm8      | 1 or 0. enable/disable blinking of DSKY R1 digits                    |
| BLINK_R2           | imm8      | 1 or 0. enable/disable blinking of DSKY R2 digits                    |
| BLINK_R3           | imm8      | 1 or 0. enable/disable blinking of DSKY R3 digits                    |
| LT_UPLINK_ACTY     | imm8      | 1 or 0. turn on/turn off the UPLINK ACTY status light                |
| LT_NO_ATT          | imm8      | 1 or 0. turn on/turn off the NO ATT status light                     |
| LT_STBY            | imm8      | 1 or 0. turn on/turn off the STBY status light                       |
| LT_OPR_ERR         | imm8      | 1 or 0. turn on/turn off the OPR ERR status light                    |
| LT_KEY_REL         | imm8      | 1 or 0. turn on/turn off the KEY REL status light                    |
| LT_NA1             | imm8      | 1 or 0. turn on/turn off the NA1 status light                        |
| LT_NA2             | imm8      | 1 or 0. turn on/turn off the NA2 status light                        |
| LT_TEMP            | imm8      | 1 or 0. turn on/turn off the TEMP status light                       |
| LT_GIMBAL_LOCK     | imm8      | 1 or 0. turn on/turn off the GIMBAL LOCK status light                |
| LT_PROG_ALRM       | imm8      | 1 or 0. turn on/turn off the PROG ALRM status light                  |
| LT_RESTART         | imm8      | 1 or 0. turn on/turn off the RESTART status light                    |
| LT_TRACKER         | imm8      | 1 or 0. turn on/turn off the TRACKER status light                    |
| LT_ALT             | imm8      | 1 or 0. turn on/turn off the ALT status light                        |
| LT_VEL             | imm8      | 1 or 0. turn on/turn off the VEL status light                        |
| LT_COMP_ACTY       | imm8      | 1 or 0. turn on/turn off the COMP ACTY light                         |
| LT_VERB            | imm8      | 1 or 0. turn on/turn off the VERB light                              |
| LT_NOUN            | imm8      | 1 or 0. turn on/turn off the NOUN light                              |
| LT_PROG            | imm8      | 1 or 0. turn on/turn off the PROG light                              |
| LT_ALL             | imm8      | 1 or 0. turn on/turn off ALL status lights                           |
| UPLINK_PROB_IMM8   | imm8 | set UPLINK ACTY random blink probability to imm8 0=off, 255=always on     |
| COMPACTY_PROB_IMM8 | imm8 | set COMP ACTY random blink probability to imm8 0=off, 255=always on       |
| GPS_LAT_A          |           | Read GPS unit and place Latitude into A                              |
| GPS_LON_A          |           | Read GPS unit and place Longtitude into A                            |
| GPS_YEAR_A         |           | Read GPS unit and place Year into A                                  |
| GPS_MON_A          |           | Read GPS unit and place Month into A                                 |
| GPS_DAY_A          |           | Read GPS unit and place Day into A                                   |
| GPS_HH_A           |           | Read GPS unit and place Hours into A                                 |
| GPS_MM_A           |           | Read GPS unit and place Minutes into A                               |
| GPS_SS_A           |           | Read GPS unit and place Seconds into A                               |
| BRANCH_TIMESTAMP_LT| addr1 addr2 offset | branch if timestamp1 less than timestamp2                   |
| TIMESTAMP_DIFF_A   | addr1 addr2 | diff timestamp1 in addr1 and timestamp2 in addr2 put result into A |
| RTC_TIMESTAMP_DIRECT | addr    | store entire RTC timestamp to addr+0 and addr+1                      |
| RTC_DAY_A          |           | Read RTC and place DAY field into A register  (BCD encoded)          |
| RTC_YEAR_A         |           | Read RTC and place YEAR field into A register (BCD encoded)          |
| RTC_MON_A          |           | Read RTC and place MONTH field into A register (BCD encoded)         |
| RTC_HH_A           |           | Read RTC and place HOURS field into A register (BCD encoded)         |
| RTC_MM_A           |           | Read RTC and place MINUTES field into A register (BCD encoded)       |
| RTC_SS_A           |           | Read RTC and place SECONDS field into A register (BCD encoded)       |
| RTC_MEM_A          | addr      | Read RTC RAM address addr and place into A register                  |
| RTC_A_MEM          | addr      | Write LSB of A register into RTC RAM address addr                    |
| RTC_MEM_A_CDIRECT  | addr      | Read RTC RAM address addr+C and place into A register                |
| RTC_A_MEM_CDIRECT  | addr      | Write LSB of A register into RTC RAM address addr+C                  |
| IMU_ACCX_A         |           | Move IMU Acceleration X value into A                                 |
| IMU_ACCY_A         |           | Move IMU Acceleration Y value into A                                 |
| IMU_ACCZ_A         |           | Move IMU Acceleration Z value into A                                 |
| IMU_PITCH_A        |           | Move IMU Pitch value into A                                          |
| IMU_ROLL_A         |           | Move IMU Roll value into A                                           |
| IMU_YAW_A          |           | Move IMU Yaw value into A                                            |
| IMU_TEMP_A         |           | Move IMU Temp value into A                                           |
| MP3_PLAY_A         |           | play track number indicated by register A                            |
| MP3_STOP           |           | stop playing any track. (play the silence clip)                      |
| EEPROM_WRITE_A_CDIRECT |       | write A register byte to EEPROM[C]                                   |
| EEPROM_READ_A_CDIRECT  |       | read into A register byte from EEPROM[C]                             |
| WAIT1              |           | pause CPU until 100ms timer triggers, then proceed to next instr.    |
| WAIT2              |           | pause CPU until 200ms timer triggers, then proceed to next instr.    |
| WAIT3              |           | pause CPU until 300ms timer triggers, then proceed to next instr.    |
| WAIT4              |           | pause CPU until 1s timer triggers, then proceed to next instr.       |
| WAIT5              |           | pause CPU until 2s timer triggers, then proceed to next instr.       |
| INPUT_NOUN         |  | Read a NOUN from the keyboard. A=value means good. A=-1 means bad             |
| INPUT_R1           |  | Read a signed decimal number into R1. A=value good. A=-1 Bad                  |
| INPUT_R2           |  | Read a signed decimal number into R2. A=value good. A=-1 Bad                  |
| INPUT_R3           |  | Read a signed decimal number into R3. A=value good. A=-1 Bad                  |
| INPUT_R1_U         |  | Read a unsigned decimal number into R1. A=value good. A=-1 Bad                |
| INPUT_R2_U         |  | Read a unsigned decimal number into R2. A=value good. A=-1 Bad                |
| INPUT_R3_U         |  | Read a unsigned decimal number into R3. A=value good. A=-1 Bad                |
| INPUT_R1_OCT       |  | Read an octal value into R1. A=value good. A=-1 Bad                           |
| INPUT_R2_OCT       |  | Read an octal value into R2. A=value good. A=-1 Bad                           |
| INPUT_R3_OCT       |  | Read an octal value into R3. A=value good. A=-1 Bad                           |
| ADJUST_R1          | addrMin addrMax | use +/- keys to adjust value in R1 up and down.              |
| ADJUST_R2          | addrMin addrMax | use +/- keys to adjust value in R2 up and down.              |
| ADJUST_R3          | addrMin addrMax | use +/- keys to adjust value in R3 up and down.              |
| ADJUST_R1_OCT      | addrMin addrMax | use +/- keys to adjust value in R1 up and down. (octal)      |
| ADJUST_R2_OCT      | addrMin addrMax | use +/- keys to adjust value in R2 up and down. (octal)      |
| ADJUST_R3_OCT      | addrMin addrMax | use +/- keys to adjust value in R3 up and down. (octal)      |
| PROG8_A_INDIRECT_C |           | A = Program[C] read byte from program memory                         |
| PROG16_A_INDIRECT_C|           | A = Program[C] read 16-bit word from program memory C, C+1           |
| PROG32_A_INDIRECT_C|           | A = Program[C] read 32-bit word from program memory C, C+1, C+2, C+3 |
| ADD_A_IMM8         | imm8      | Add signed byte imm8 to A register                                   |
| ADD_B_IMM8         | imm8      | Add signed byte imm8 to B register                                   |
| ADD_C_IMM8         | imm8      | Add signed byte imm8 to C register                                   |
| EMPTY_STACK        |           | Reset the stack to be empty for the CPU core                         |
| RUN_PROG_A         | | cause cpu core 0 to run program located at 16-bit address in A. Reset stack    |
| CALL_CINDIRECT     |           | call a subroutine whose address is in the C register                 |
| PUSH_DSKY          |           | push DSKY state on stack                                             |
| POP_DSKY           |           | pop DSKY state on stack                                              |
| MOV_A_AGC_FLAGS2   |           | Move A register to the AGC_FLAGS2 field   DELETED                    |
| MOV_AGC_FLAGS2_A   |           | Move AGC_FLAGS2 field into A register     DELETED                    |


## Running the Curses Simulator
The **curses simulator** is a text based 'ncurses' application. You run
the program from any text terminal and you will see a simple text screen
that represents the DSKY display and DSKY keyboard.

Run with this command,
```
	$ ./dsky
```

![alt text](images/ncurses2.jpg "")

### Keys
The keys map to your keyboard thusly. Only lower case keys are accepted.
+ '0' ... '9' - Digits
+ '+'	- Plus
+ '-'	- Minus
+ 'v'	- VERB
+ 'n'	- NOUN
+ 'p'	- PRO (Proceed)
+ 'c'	- CLR (Clear)
+ 'k'	- KEY REL (Key Release)
+ 'e' or 'Enter' - ENTR (Enter)
+ 'r'	- RSET (Reset)
+ 'q'	- Quit the DSKY simulator

The behavior of this program should be identical to the behavior it
will have when run on the Arduino Open DSKY hardware. The GPS and IMU
devices are simulated with *fake* data. The MP3 player only shows audio
as a text line at the bottom of the window. The audio shows how many seconds
remain in the audio clip. But no sound will play! The Real Time cLock (RTC)
is simulated but it uses the linux date and time to initialize itself. The
real time clock can be set by the user to a different date/time in the simulator.

### log file
When running the **curses simulator** the file ```./logfile.txt``` is produced.
It is used for debugging purposes. Also contains the memory sizes of some data structures.

### persistent data
The file ```./persist.txt``` contains a simulated EEPROM storage and simulated RTC clock RAM.
This allows the **curses simulator** to retain information between runs of the program.

## DSKY Usage
This section describes the general usage of the DSKY. The interface was modeled
by my experimentation with a faithful DSKY simulator (See <https://svtsim.com/moonjs/agc.html>).

### General Notes:
+ **ENTR** not required after each verb or noun entry. Pressing **ENTR** causes
	the DSKY to take action on the currently showing verb/noun fields.

+ **KEY REL** - When the **KEY REL** light is blinking, then the KEY REL button can be
	pressed to cancel the entry of a noun/verb.

+ **OPR ERR** - When **OPR ERR** light is blinking then the user should press **RSET** to reset
	this.

+ You cannot launch a program using the **PRO** key. Instead you must enter `V37` `ENTR` and
	then type in a two digit program number which appears in the noun field. Then `ENTR` again
	to launch the program.

### VERBs, NOUNs, and PROGRAMs
This section documents the available VERB/NOUN combinations and the PROGRAM's.

### Verbs

| VERB | Description                                        |
|------|----------------------------------------------------|
| V06  | Display Selected Value                             |
| V16  | Monitor Selected Values                            |
| V21  | Enter value (R1 only)                              |
| V22  | Enter value (R2 only)                              |
| V25  | Enter values (R1 + R2 + R3)                        |
| V26  | Load values from external source                   |
| V35  | LAMP TEST                                          |
| V36  | Fresh Start                                        |
| V37  | Execute Major PROGRAM                              |
| V69  | Force Restart                                      |
| V82  | Monitor Orbital Parameters                         |

### Nouns
| NOUN | Description                                        |
|------|----------------------------------------------------|
| N17  | IMU Linear Acceleration values (XXXX, YYYY, ZZZZ)  |
| N18  | IMU Gyro acceleration values (ROLL, PITCH, YAW)    |
| N19  | RTC DATE, TIME, IMU TEMP                           |
| N31  | Time from AGC initialization                       |
| N32  | Time from Perigee                                  |

### Verb-Nouns
| VERB-NOUN | Description                                        |
|-----------|----------------------------------------------------|
|V06 N17    | Display IMU linear accel values                    |
|V06 N18    | Display IMU gyro accel values                      |
|V16 N19    | Display RTC Date/Time and IMU temp                 |
|V06 N31    | Display time from AGC Init                         |
|V06 N32    | display time to perigee                            |
|V06 N36    | Display RTC time                                   |
|V06 N37    | display RTC date                                   |
|V06 N65    | display met                                        |
|V06 N38    | Display GPS time                                   |
|V06 N39    | Display GPS date                                   |
|V16 N17    | Monitor IMU linear accel values                    |
|V16 N18    | Monitor IMU Gyro accel values                      |
|V16 N19    | Monitor RTC Date/Time and IMU temp                 |
|V16 N31    | Monitor time from agc init                         |
|V16 N34    | Monitor/Stop Time From event                       |
|V16 N35    | Monitor/Stop timer count to event                  |
|V16 N36    | Monitor RTC Time                                   |
|V16 N37    | Monitor RTC Date                                   |
|V16 N38    | Monitor GPS time                                   |
|V16 N39    | Monitor GPS date                                   |
|V16 N43    | Monitor GPS coordinates                            |
|V16 N44    | Monitor Orbital Parameters                         |
|V16 N65    | Monitor MET                                        |
|V16 N68    | A11 Lunar Landing simulation                       |
|V16 N87    | Monitor IMU linear accel values (with random 1202 alarms)  |
|V16 N98    | Play selected audio clip R1=clip, R2=index adj factor      |
|V21 N98    | select number of audio clip                        |
|V22 N98    | Enter index adj factor                             |
|V25 N34    | Set/Start timer count from event                   |
|V25 N35    | Set/Start timer count to event                     |
|V25 N36    | Set RTC Clock Manually                             |
|V25 N37    | Set RTC Date Manually                              |
|V26 N36    | Set RTC Clock from GPS                             |
|V26 N37    | Set RTC Date from GPS                              |
|V35        | Lamp test                                          |
|V36        | Fresh Start                                        |
|V37 N00    | P00 enter idle mode (run major mode, instead of N00 use Nxx)  |
|V37 N01    | P01 A11 Launch Simulation                          |
|V37 N06    | P06 Simulates putting AGC into standby mode        |
|V37 N11    | P11 Monitor IMU ACcel values                       |
|V37 N61    | P61 playback JFK i believe                         |
|V37 N62    | P62 playback JFK We choose                         |
|V37 N68    | P68 playback A8 Genesis                            |
|V37 N69    | P69 playback A11 eagle has landed                  |
|V37 N70    | P70 playback A11 we have a problem                 |
|V69        | Force restart                                      |
|V82        | Monitor Orbital parameters                         |

### Programs
| PROG | Description                                                |
|------|------------------------------------------------------------|
| P00  | Poo                                                        |
| P01  | Apollo 11 Launch Simulation                                |
| P06  | Simulate putting AGC into standby mode                     |
| P11  | Display IMU linear acceleration values (same as V16 N18)   |
| P61  | play short version of JFK "I believe"                      |
| P62  | play short version of JFK "we choose"                      |
| P68  | play short version of Apollo 8 genesis clip                |
| P69  | play Apollo 11 the eagle has landed clip                   |
| P70  | play short version of Apollo 13 "problem" clip             |

## Green Plexi-glass Modification
This section describes my modification to the Open DSKY. I decided
I didn't like how fuzzy the LED digits were. I decided to use a green
plexi-glass window to make the LED region nicer to look at and avoid
the fuzzyness and dimness of the original kit.

![alt text](images/v69.jpg "")

I bought this product from amazon:
```
	Transparent Green Acrylic Sheet (12" x 20", 1/16" / 1.5mm)
	(MakerStock Store)
```
Make sure the thickness is 1.5mm.

I measured the desired size and used a box cutter knife to cut the plastic.
I used a metal ruler to maintain a straight line. After 10 to 12 cuts I broke
the plastic along a straight table edge. The plastic broke cleanly along the
line I had scored with the razor.

![alt text](images/gpm1.jpg "")

I then used a utility knife to scrape the size to fit perfectly.

![alt text](images/gpm2.jpg "")

I did the same cutting operation to the provided clear plastic that came
with the Open DSKY kit.

![alt text](images/gpm3.jpg "")

I used duct tape to adhere my green plexi-glass sheet to the table I was working on
so that the sheet and ruler wouldn't move.

The sticker that came with the Open DSKY kit I cut in half. I retained the left hand side
of the sticker for the caution and warning lights pane. But I threw away the right hand side.

Using my laser printer I printed out `screen_stuff.pdf` and used scissors to cut out a the
black region plus verb/noun/prog text. This I carefully positioned over the verb/noun/prog
lights.

I also cut out the block of **plus** symbols. These I overlayed over the plus 7-segment LEDs.

I used a small piece of scotch tape to hold these overlays in place while I assembled the
display and bezel. Best results are achieved if the black ink covers anything inside that
might reflect light.

## Review of the Open DSKY Kit
This section contains my thoughts on the Open DSKY kit.

Overall I loved it. It requires a lot of detail work for final assembly.
However this is a kit and as the creators say in various YouTube videos
this is a DIY (Do It Yourself) project. The most important letter is the "Y" letter.

Final assembly required me to use a dremel grinding tool to carve out plastic
from the back in order to ensure the front cover fits snuggly onto the cicruit board.

I didn't like the "sticker" which you use to cover over the beautiful LED's, so I
customized my device.

The electronics and provided circuit board were excellent. All the parts were well labeled.
It makes for a greate arduino platform to play with many different devices: GPS, IMU, MP3 player
and Real Time Clock.

Little details included with the Kit were also very nice. There were two stickers that you
affix to the case which look like official NASA tracking signage. There was also a tiny
little 3d-printed DSKY.

### Sticker 1
![alt text](images/sticker1.jpg "")

### Sticker 2
![alt text](images/sticker2.jpg "")

### Mini-DSKY
![alt text](images/minidsky.jpg "")

The assembly instructions were a little sparse. But this increased my feeling of satisfaction
when I successfully built the thing. How to perform final assembly was not well documented.

Be careful with the buttons. Using wire cutters to cut off the plastic bump on all the buttons
can result in inadvertently cutting the wire lead that illuminates the button. Thankfully
they provide an extra button if you screw one of them up.

Make sure the GPS unit is flush with the circuit board when you solder it on.

All the components came in indiviual plastic and were labeled. There was a inventory sheet
showing all the components. The kit arrived in good  packaging. I definately felt the kit was worth the $600.

There are good online schematics. The sample source code is pretty gross, but also pretty easy
reverse engineer. Unfortunately there lacks a schematic showing how the MP3 player was wired up.

## Author
Ken Stauffer<BR>
New York, NY.<BR>
2/14/2024
