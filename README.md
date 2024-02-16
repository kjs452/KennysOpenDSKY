# KennysOpenDSKY
Kenny's Open DSKY Software

## Description
This project contains C/C++ source code for the Arduino nano which
will control the Open DSKY kickstarter kit.

This project also contains source code for Linux/Windows/Macos which
runs as ```ncurses``` text-based simulator of the arduino software. This
will be referred to as the **curses simulator**.

The **curses simulator** allows for testing and deugging the complete software
without having to upload to the Arduino nano.

![alt text](images/v16n36.jpg "")

## Acknowledgments
This is an original implementation of code needed to drive the
Open DSKY hardware. However, I derived many routines from these sources:

+ **S&T Geotronics** James Sanderson and Marc Tessier for creating the
	Open DSKY kit.

+ **Scott Pavlovec** github project. <https://github.com/scottpav>. This
	contained the reference implementation which I used to talk to the hardware.

+ The functionality offered was inspired by the functionality offered by
		the **Apollo 50th Anniversary** project which came pre-installed with the Open DSKY kit.
	(Apollo Education Experience Program, Cumming Georgia).
	Thanks **Bill Walker** for an incredible software and user manual.
	Website 1: <http://apolloexperience.com>.
	Website 2: <https://www.gofundme.com/apollo-education-experience-project>

+ The audio clips provided here are taken from the **Apollo 50th Anniversary** project SD card.

+ This website which has information about the virtual apollo guidance
	computer <http://www.ibiblio.org/apollo/>.

+ An online apollo DSKY emulator <https://svtsim.com/moonjs/agc.html>. I used this interface to
	tweak my code to better reflect how the actual DSKY worked.

## Features
+ A virtual machine and byte code interpreter
+ Two threads of control: One thread is a backround task, which
	runs major modes. The other thread is a foreground task which runs simple verbs/nouns.
+ Ability to enter values using the keypad instead of +/- keys.

## Files

## Dependencies
### Arduino
This section describes the third party libraries needed to compile
the sketch for Arduino.

+ Wire - standard arduino library
+ EEPROM - standard arduino library for reading/writing the 2K eeprom memory.
+ Adafruit NeoPixel - library used to illuminate the neo pixels
+ LedControl - library used to talk to LEDs
+ TinyGPS++ - library to read the GPS device and parse its output.
+ DFPlayerMini_Fast - library to play audio clips

### Linux
This section describes the notable libraries needed to compile
the curses simulator on linux/macos/windows.

+ **ncurses** - this is a library for drawing simple text based user interfaces. It
		is widely available, and comes installed by default on most linux distros.

+ **sigalarm** - this is capability of most unixes. It is included on all linux operating
			systems. Special porting may be needed for Windows or Macos.


## Compiling for Arduino Nano
I use the Arduino Command Line Interface. I run this under linux on a Raspberry Pi.
The included ```Makefile``` shows the commands needed to compile the sketch and
upload to your Open DSKY kit.

## Compiling for Linux

## Compiling for Windows/Macos
I don't know how to compile for these platforms. Make sure
you have the ```ncurses``` library available. Make sure you have sigalarm() available.
The compiliation should be pretty straight forward.

## Assembly Language
This section documents all the assembly instructions.

## Using the Assembler
The program ``assembler.py`` is a simple one pass assembler written in python.
You will need python to recompile the assembly. This github repository
however contains a pre-compiled version of the assembly.

## Running the curses simulator
The **curses simulator** is a text based 'ncurses' application. You run
the program from any text terminal and you will see a simple text screen
that represents the DSKY display and DSKY keyboard.

![alt text](images/ncurses2.jpg "")

### Keys
The keys map to you keyboard thusly. Only lower case keys are accepted.
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

### log file
The file ```./logfile.txt``` is produced which can be used for debugging purposes.

### persistent data
The file ```./persist.txt``` contains a simulated EEPROM storage and simulated RTC clock RAM.
This allows the **curses simulator** to retain information between running the program.

## DSKY Usage
This section describes the general usage of the DSKY. The interface was modeled
by my experimentation with a faithful DSKY simulator (See http:www.XXXXXXXXX).

## VERBs, NOUNs, and PROGRAMs
This section documents the avalable VERB/NOUN combinations and the PROGRAM's.

## Green Plexiglass Modification
This section describes my modification to the Open DSKY. I decided
I didn't like how fuzzy the LED digits were. I decided to use a green
plexi-glass window to make the LED region nicer to look at and avoid
the fuzzyness and dimness of the original kit.

![alt text](images/v69.jpg "")

## Review of the Open DSKY Kit
This section contains my thoughts on the Open DSKY kit.

Overall I loved it. It requires a lot of detail work for final assembly.
However this is a kit and as the creators say in various YouTube videos
this is a DIY (Do It Yourself) project. The most important letter is the "Y" letter.

Final assembly required me to use a dremel grinding tool to carve out plastic
from the back in order to ensure the front cover fits snuggly onto the cicruit board.

I didn't like the "sticker" which you use to cover over the beautiful LED's, so I
customized my device.

The electronics and provided circuit board were excellend. All the parts were well labeled.
It makes for a greate arduino platform to play with many different devices: GPS, IMU, MP3 player
and Real Time Clock.

Little details included with the Kit were also very nice. There were two stickers that you
affix to the case which look like official NASA tracking signage. There was also a tiny
little 3d-printed DSKY.

The assembly instructions were a little sparse. But this increased my feeling of satisfaction
when I successfully built the thing. How to perform final assembly was not well documented.

Be careful with the buttons. Using wire cutters to cut off the plastic bump on all the buttons
can result in inadvertently cutting the wire lead that illuminates the button. Thankfully
they provide an extra button if you screw one of them up.

Make sure the GPS unit is flush with the circuit board when you solder it on.

All the components came in indiviual plastic and were labeled. There was a inventory sheet
showing all the components. The kit arrived in good  packaging. I definately felt the kit was worth the $600.

There are good online schematics. The sample source code is pretty gross, but also pretty easy
reverse engineer.

## Author
Mr. Kenneth James Stauffer Jr., the second, B.Sc, FAA Drone pilot
2/14/2024
