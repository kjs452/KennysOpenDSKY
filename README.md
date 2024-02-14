# KennysOpenDSKY
Kenny's Open DSKY Software

## Description
This project contains C/C++ source code for the arduino nano which
will control the Open DSKY hardware kit.

This project also contains source code for Linux/Windows/Macos which
runs as ```ncurses``` text-based simulator of the arduino software. This
will be referred to as the **curses simulator**.

The **curses simulator** allows for testing and deugging the complete software
without having to upload to the arduino nano.

## Acknowledgments
This is an original implementation of code needed to drive the
Open DSKY hardware. However, I derived many routines from these sources:

+ Scott Pav's github project.

+ The functionality offered was inspired by the functionality offered by
		the Apollo 50th project which came pre-installed with the Open DSKY kit.


## Features
+ A virtual machine and byte code interpreter
+ Two threads of control: One thread is a backround task, which
	runs major modes. The other thread is a foreground task which runs simple verbs/nouns.
+ Ability to enter values using the keypad instead of +/- keys.

## Files


## Dependencies

## Compiling for Arduino Nano

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

## Green Plexiglass Modification
This section describes my modification to the Open DSKY. I decided
I didn't like how fuzzy the LED digits were. I decided to use a green
plexi-glass window to make the LED region nicer to look at and avoid
the fuzzyness and dimness of the original kit.

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
