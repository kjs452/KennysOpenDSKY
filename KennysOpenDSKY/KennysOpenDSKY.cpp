//
// Kenny's Open DSKY Software
// 1/31/2024
//
// This software compiles for arduino and is installable on the 'Open DSKY' kickstarter kit.
// The same source file compiles on linux/windows/macos with 'ncurses' for a faster development cycle.
//
// This single file contains two versions:
//	1) Arduino sketch
//	2) Linux/Macos/Window Curses Simulator	(when the CURSES_SIMULATOR macro is defined)
//
// This program emulates the Apollo Guidance Computer DSKY interface.
// It contains a byte code interpreter for running programs written in
// assembly instructions. These programs form the Verbs, Nouns and Programs.
//
// This program uses a state machine to handle keyboard input. It tries
// to emulates the original Apollo DSKY as much as possible.
//
// Two virtual threads are available:
//	1) Background thread for running major modes (PROG's)
//	2) Foreground thread for running foreground tasks (Verbs/Nouns)
//
// Forground tasks can stack on top of a previous foreground tasks and
// restores to the previous run state on RETURN.
//
// This program is broken into these sections:
//
//	SECTION 1: Basic Type definitions
//	SECTION 2: Infrastructure globals
//	SECTION 3: Apollo Guidance Computer data structure definitions
//	SECTION 4: Miscellaneous Arduino Mocks
//	SECTION 5: 'Adafruit_NeoPixel' Mock
//	SECTION 6: 'LedControl' Mock
//	SECTION 7: 'Random()' Mock Routines
//	SECTION 8: 'Wire' and 'Serial' Mock
//	SECTION 9: MP3 Player Curses Routines
//	SECTION 10: 'EEPROM' Mock
//	SECTION 11: Globals Variables (Read-Only & Read/Write)
//	  SUB-SECTION 11a: Read-Only globals (constants in PROGMEM)
//	  SUB-SECTION 11b: Read-Write globals (variables in RAM)
//	SECTION 12: load/store mock RTC RAM and mock EEPROM data
//	SECTION 13: Curses window start/end
//	SECTION 14: DSKY redraw routines (arduino and curses)
//	SECTION 15: Keyboard reading (arduino and curses)
//	SECTION 16: IMU Reading routine
//	SECTION 17: Apollo Guidance Computer routines
//	SECTION 18: Signal/ISR for Timing control (arduino and curses)
//	SECTION 19: Sketch setup() routine
//	SECTION 20: Sketch loop() routine
//	SECTION 21: MAIN() - calls setup() and loop()
//
// NOTES:
//	* When adding/removing VM instructions there are 4 places to edit:
//		1) the enum INSTRUCTIONS
//		2) the debug string table Mnemonics[]
//		3) The switch statement in agc_execute()
//		4) The InstructionTable{} associative array in assembler.py
//
// #define macros:
//	CURSES_SIMULATOR		- enable this on linux/macos/windows for the curses simulator
//	DSKY_DEBUG				- only works when CURSES_SIMULATOR is also defined.
//							  enables extra debug logging messages
//

//////////////////////////////////////////////////////////////////////
//
// SECTION 1: Basic Type definitions
//

#ifdef CURSES_SIMULATOR
#include <curses.h>
#include <sys/random.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <signal.h>
#include <errno.h>

typedef	signed char		int8_t;
typedef unsigned char	uint8_t;
typedef unsigned short	uint16_t;
typedef int				int32_t;

//
// Mock PROGMEM access macros
//
#define PROGMEM	/* nothing */
#define pgm_read_byte_near(address_short)	(*((uint8_t*)(address_short)))
#define pgm_read_word_near(address_short)	(*((uint16_t*)(address_short)))
#define pgm_read_dword_near(address_short)	(*((uint32_t*)(address_short)))

//
// Mock timer registers
//
static uint8_t TCCR1A;
static uint8_t TCCR1B;
static uint8_t TCNT1;
static uint16_t OCR1A;
static uint8_t TIMSK1;
#define WGM12	1
#define CS12	12
#define OCIE1A	1

//
// mock neopixel configuation defines
//
#define NEO_GRB			0			// mock value
#define NEO_KHZ800		0			// mock value

#else

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <LedControl.h>
#include <Wire.h>
#include <stdint.h>
#include <EEPROM.h>

#endif

// this macro definition conflicts with 'SP' struct member in struct CPU.
#ifdef SP
#undef SP
#endif

#define ERR (-1)			// returned from readKeyboard() to indicate "nothing pressed"
#define ODDROWDIVIDERVOLTAGE1	225
#define ODDROWDIVIDERVOLTAGE2	370
#define ODDROWDIVIDERVOLTAGE3	510
#define ODDROWDIVIDERVOLTAGE4	650
#define ODDROWDIVIDERVOLTAGE5	790
#define ODDROWDIVIDERVOLTAGE6	930

#define EVENROWDIVIDERVOLTAGE1	200
#define EVENROWDIVIDERVOLTAGE2	330
#define EVENROWDIVIDERVOLTAGE3	455
#define EVENROWDIVIDERVOLTAGE4	577
#define EVENROWDIVIDERVOLTAGE5	700
#define EVENROWDIVIDERVOLTAGE6	823
#define EVENROWDIVIDERVOLTAGE7	930

#define RTC_ADDR		0x68		// I2C address of the RTC DS1307
#define MPU_ADDR		0x69		// I2C address of the MPU-6050
#define TRACKDB_LEN		17			// number of audio tracks on SD card
#define PIN				6			// neopixel access pin
#define NUMPIXELS		18
#define GPS_SW			7 

//
// These macros wrap access to PROGRAM MEMORY arrays.
// Four PROGMEM arrays are stored in program memory:
//	1) StateMachine[] for the DSKY user input state transistion graph
//	2) Program[] array for running AGC Virtual Machine assembly code (kennysagc.h)
//	3) Verbs[] array for looking up valid verbs and their start address
//	4) accdbm[] array (Accumulated Days By Month) for date math
//
#define StateMachineAcc(state, input)	\
							(const uint8_t)(pgm_read_byte_near((uint8_t*)StateMachine + (10*state) + input))
#define ProgramAccU8(pc)	(const uint8_t)(pgm_read_byte_near(Program + pc))
#define ProgramAcc8(pc)		(const int8_t)(pgm_read_byte_near(Program + pc))
#define ProgramAcc16(pc)	(const int16_t)(pgm_read_word_near(Program + pc))
#define ProgramAcc32(pc)	(const int32_t)(pgm_read_dword_near(Program + pc))
#define VerbAcc(i)			(const uint8_t)(pgm_read_byte_near(&Verbs[i].cmd))
#define VerbStartAcc(i)		(const uint16_t)(pgm_read_word_near(&(Verbs[i].start)))
#define accdbmAcc(i)		(const uint16_t)(pgm_read_word_near(accdbm + (i)))

//////////////////////////////////////////////////////////////////////
//
// SECTION 2: Infrastructure globals
//
#ifdef CURSES_SIMULATOR

static FILE *logfp;				// logging file
static WINDOW *c_win;			// curses window

#endif

//////////////////////////////////////////////////////////////////////
//
// SECTION 3: Apollo Guidance Computer data structure definitions
//

//////////////////////////////////////////////////////////////////////
//
// DSKY display interface
//

//
// These macros represent the Red,Green,Blue components for
// setting the neopixel color. Only intended to be used in this
// context:
//	neoPixels.setPixelColor(LAMP_STBY, NP_COLOR_YELLOW);
//
#define NP_COLOR_WHITE		100,100,100
#define NP_COLOR_GREEN		0,100,0			/* not used */
#define NP_COLOR_YELLOW		100,100,0
#define NP_COLOR_ORANGE		255,165,0		/* not used */
#define NP_COLOR_RED		100,0,0			/* not used */
#define NP_COLOR_OFF		0,0,0

//
// This enum represents the NeoPixel address-id for
// each NeoPixel on the OpenDSKY
//
enum LAMP_NO {
	LAMP_NOUN			= 0,
	LAMP_PROG			= 1,
	LAMP_VERB			= 2,
	LAMP_COMP_ACTY		= 3,

	LAMP_TEMP			= 4,
	LAMP_GIMBAL_LOCK	= 5,
	LAMP_PROG_ALARM		= 6,
	LAMP_RESTART		= 7,
	LAMP_TRACKER		= 8,
	LAMP_ALT			= 9,
	LAMP_VEL			= 10,

	LAMP_NA2			= 11,	// lampClk
	LAMP_NA1			= 12,	// lampPosition
	LAMP_OPR_ERR		= 13,
	LAMP_KEY_REL		= 14,
	LAMP_STBY			= 15,
	LAMP_NO_ATT			= 16,
	LAMP_UPLINK_ACTY	= 17,
};

//
// NOTE: The status light enum also contains these
// special flag bits:
//		LS_HIDE_R3, RS_HIDE_R2, VNPC_HIDE_R1
//		VNPC_HIDE_VERB, VNPC_HIDE_NOUN, VNPC_HIDE_PROG
// These bits allow the turning off of different LED fields,
// which is used to support blinking effect.
//

//
// The status lights in the left column
// (Also contains the LS_HIDE_R3 flag)
//
enum LEFT_STATUS
{
	LS_UPLINK_ACTY	= 1<<0,
	LS_NO_ATT		= 1<<1,
	LS_STBY			= 1<<2,
	LS_KEY_REL		= 1<<3,
	LS_OPR_ERR		= 1<<4,
	LS_NA1			= 1<<5,
	LS_NA2			= 1<<6,
	LS_HIDE_R3		= 1<<7,			// when set, R3 is not shown (used for blinking R3)
};

//
// The status lights in the right column
// (Also contains the LS_HIDE_R2 flag)
//
enum RIGHT_STATUS
{
	RS_TEMP			= 1<<0,
	RS_GIMBAL_LOCK	= 1<<1,
	RS_PROG			= 1<<2,
	RS_RESTART		= 1<<3,
	RS_TRACKER		= 1<<4,
	RS_ALT			= 1<<5,
	RS_VEL			= 1<<6,
	RS_HIDE_R2		= 1<<7,			// when set R2, is not shown (used for blinking R2)
};

//
// The Verb/Noun/Prog/CompActy lights
// (Also contains the HIDE flags for NOUN,VERB, PROG, R1)
//
enum VNPC_LIGHTS
{
	VNPC_VERB		= 1<<0,			// verb neo pixel
	VNPC_NOUN		= 1<<1,			// noun neo pixel
	VNPC_PROG		= 1<<2,			// prog neo pixel
	VNPC_COMP_ACTY	= 1<<3,			// comp acty neo pixel
	VNPC_HIDE_VERB	= 1<<4,			// when set, verb numerals are not shown (used for blinking)
	VNPC_HIDE_NOUN	= 1<<5,			// when set, noun numerals are not shown (used for blinking)
	VNPC_HIDE_PROG	= 1<<6,			// when set, prog numerals are not shown (used for blinking)
	VNPC_HIDE_R1	= 1<<7,			// when set, R1 is not shown (used for blinking)
};

//
// These flags indicate which element of the DSKY is blinking
//
enum BLINK_FLAGS {
	BLINKF_VERB = 1<<0,			// enable VERB digits to blink
	BLINKF_NOUN = 1<<1,			// enable NOUN digits to blink
	BLINKF_PROG = 1<<2,			// enable PROG digits to blink
	BLINKF_R1 = 1<<3,			// enable R1 digits to blink
	BLINKF_R2 = 1<<4,			// enable R2 digits to blink
	BLINKF_R3 = 1<<5,			// enable R3 digits to blink
	BLINKF_OPRERR = 1<<6,		// enable OPR ERR light to blink
	BLINKF_KEYREL = 1<<7,		// enable KEY REL light to blink
};

//
// DSKY Binary Coded Decimal (BCD)
// Each nibble (4-bit field) encodes a single digit
// as well as 'space', '+' and '-'.
//
//	+------+------+
//  | 0000 | 0000 |
//	+------+------+
//   digit1 digit0
//
// Hex digit:
//	0,1,2,3,4,5,6,7,8,9 - Digit Value
//  A					- blank (space)
//	B					- plus (+)	only makes sense for most signifigant digit of R1, R2, R3
//	C					- minus (-)	only makes sense for most signifigant digit of R1, R2, R3
//	D					- NOT USED
//	F					- NOT USED
//
// This Binary Coded Decimal encoding is used for the VERB, NOUN, PROG, R1, R2, and R3 fields.
//

//
// This structure encodes the entire display state
//
struct DSKY {
	uint8_t R1[3];		// BCD - binary coded decimal
	uint8_t R2[3];		// BCD - binary coded decimal
	uint8_t R3[3];		// BCD - binary coded decimal
	uint8_t verb;		// BCD - binary coded decimal
	uint8_t noun;		// BCD - binary coded decimal
	uint8_t prog;		// BCD - binary coded decimal
	uint8_t lstatus;	// LEFT_STATUS
	uint8_t rstatus;	// RIGHT_STATUS
	uint8_t vnpc;		// VNPC_LIGHTS verb noun prog 'comp acty' lights
	uint8_t blink;		// BLINK_FLAGS - what elements are blinking
};

//
// This enum defines the edges of the state machine graph.
// (The input types for the state machine, which correspond to a pressed keyboard key)
//
enum DSKY_INPUT {
	I_VERB,
	I_NOUN,
	I_PRO,
	I_CLR,
	I_KREL,
	I_ENTR,
	I_RSET,
	I_PLUS,			// +
	I_MINUS,		// -
	I_DIGIT,		// 0 thru 9
};

//
// This enum defines the nodes of the state machine graph
//
enum DSKY_STATE {
	S_ST,		// start state
	S_ERR,		// operator error state
	S_NOP,		// no op. remain in previous state. avoid taking any actions

	S_ENT,		// ENTR. initiate action on verb/noun
	S_KRL,		// KEY REL. clean up on bad input
	S_RST,		// RSET pressed. clear all status flags (except KEY REL) -> returns to S_ST
	S_IN,		// cancel current input instruction -> returns to S_N1
	S_IV,		// cancel current input instruction -> returns to S_V1
	S_IE,		// input error occured, show OPR ERR flash -> returns to S_ST after RSET pressed

	S_V1,		// verb
	S_V2,		// 1st verb digit
	S_VA,		// 2nd verb digit, verb accept		-> returns to S_ST
	S_VE,		// verb error		-> returns to S_ST (after clearing oprerr and keyrel)

	S_N1,		// noun
	S_N2,		// 1st noun digit
	S_NA,		// 2nd noun digit, noun accept		-> returns to S_ST
	S_NE,		// noun error		-> returns to S_ST (after clearing oprerr and keyrel)

	S_NI1,		// input noun instruction - waiting for 1st digit
	S_NI2,		// got 1st noun digit - waiting for 2nd digit
	S_NI3,		// got 2nd noun digit - waiting for ENTR
	S_NIA,		// got ENTR. noun accept		-> returns to S_ST

	S_IR1,		// input number into register
	S_IR2,		// got plus. transistion on 1st digit
	S_IR3,		// got minus. transistion on 1st digit
	S_IR4,		// got 1st digit.
	S_IR5,		// got 2nd digit.
	S_IR6,		// got 3rd digit.
	S_IR7,		// got 4th digit.
	S_IR8,		// got 5th digit.
	S_IRA,		// accept

	S_IO1,		// input octal number into register
	S_IO2,		// got 1st digit.
	S_IO3,		// got 2nd digit.
	S_IO4,		// got 3rd digit.
	S_IO5,		// got 4th digit.
	S_IO6,		// got 5th digit.
	S_IOA,		// accept

	S_IP1,		// input (PRO) proceed key
	S_IPA,		// PRO key pressed

	S_IPR,		// required (PRO) proceed key
};

//
// Used in the execution of BRANCH instructions and the do_cmp() function.
//
enum CMP_OP {
	CMP_GT,		// greater than
	CMP_GE,		// greater than or equals
	CMP_LT,		// less than
	CMP_LE,		// less then or equals
	CMP_EQ,		// equals
	CMP_NE,		// not equals
};

//
// These elements can blink. When turning blinking off,
// the element must be unhidden if it is hidden.
// this enum lets the instruction decoder specify
// what is to be unhidden when blink is turned off.
//
enum UNHIDE {
	UNHIDE_R1,
	UNHIDE_R2,
	UNHIDE_R3,
	UNHIDE_VERB,
	UNHIDE_NOUN,
	UNHIDE_PROG,
	UNHIDE_KEYREL,
	UNHIDE_OPRERR,
};

// up to 256 instructions are possible. stored as uint8_t.
// Opcodes range from 0 to 255.
// This is the machine language for the Apollo Guidance Computer VM.
enum AGC_INSTRUCTION
{
	MOV_R1_A = 0,
	MOV_R2_A,
	MOV_R3_A,
	MOV_A_R1,
	MOV_A_R2,
	MOV_A_R3,
	DECODE_A_FROM_OCT,	// Convert BCD encoding of octal value into a INT
	DECODE_A_FROM_DEC,	// Convert BCD encodiing of decimal into a INT
	ENCODE_A_TO_OCT,	// Encode A contents as BCD encoded octal
	ENCODE_A_TO_DEC,	// Encode A contents as BCD encoded decimal (plus/minus sign)
	ENCODE_A_TO_UDEC,	// Encode Positive decimal value (No plus sign) into BCD
	MOV_A_B,
	MOV_A_C,
	MOV_B_A,
	MOV_B_C,
	MOV_C_A,
	MOV_C_B,
	LD_A_DIRECT,		// <addr>
	LD_B_DIRECT,		// <addr>
	LD_C_DIRECT,		// <addr>
	LD_A_IMM32,			// #00 00 00 00
	LD_B_IMM32,			// #00 00 00 00
	LD_C_IMM32,			// #00 00 00 00
	LD_A_IMM16,			// #00 00
	LD_B_IMM16,			// #00 00
	LD_C_IMM16,			// #00 00
	LD_A_IMM8,			// #00
	LD_B_IMM8,			// #00
	LD_C_IMM8,			// #00
	LD_A_CINDIRECT,		// 		load A with (C)
	LD_B_CINDIRECT,		//		load B with (C)
	ST_A_DIRECT,		// <addr>
	ST_B_DIRECT,		// <addr>
	ST_C_DIRECT,		// <addr>
	ST_A_CINDIRECT,		//			store A to (C)
	ST_B_CINDIRECT,		//			store B to (C)
	CLR_A,
	CLR_B,
	CLR_C,
	INC_A,
	INC_B,
	INC_C,
	DEC_A,
	DEC_B,
	DEC_C,
	PUSH_A,
	PUSH_B,
	PUSH_C,
	POP_A,
	POP_B,
	POP_C,
	CALL,				// <addr_2byte>
	RET,
	GOTO,				// <addr_2byte>
	BRANCH,				// <offset>		relative +127/-128
	BRANCH_A_GT_B,		// <offset>
	BRANCH_A_GE_B,		// <offset>
	BRANCH_A_LE_B,		// <offset>
	BRANCH_A_LT_B,		// <offset>
	BRANCH_A_EQ_B,		// <offset>
	BRANCH_A_NE_B,		// <offset>
	BRANCH_A_GT_DIRECT,	// <addr> 	<offset>
	BRANCH_A_GE_DIRECT,	// <addr>	<offset>
	BRANCH_A_LE_DIRECT,	// <addr>	<offset>
	BRANCH_A_LT_DIRECT,	// <addr>	<offset>
	BRANCH_A_EQ_DIRECT,	// <addr>	<offset>
	BRANCH_A_NE_DIRECT,	// <addr>	<offset>
	BRANCH_A_GT_IMM8,	// #00	<offset>
	BRANCH_A_GE_IMM8,	// #00	<offset>
	BRANCH_A_LE_IMM8,	// #00	<offset>
	BRANCH_A_LT_IMM8,	// #00	<offset>
	BRANCH_A_EQ_IMM8,	// #00	<offset>
	BRANCH_A_NE_IMM8,	// #00	<offset>
	BRANCH_A_GT_IMM16,	// #00 00	<offset>
	BRANCH_A_GE_IMM16,	// #00 00	<offset>
	BRANCH_A_LE_IMM16,	// #00 00	<offset>
	BRANCH_A_LT_IMM16,	// #00 00	<offset>
	BRANCH_A_EQ_IMM16,	// #00 00	<offset>
	BRANCH_A_NE_IMM16,	// #00 00	<offset>
	BRANCH_B_GT_DIRECT,	// <addr>	<offset>
	BRANCH_B_GE_DIRECT,	// <addr>	<offset>
	BRANCH_B_LE_DIRECT,	// <addr>	<offset>
	BRANCH_B_LT_DIRECT,	// <addr>	<offset>
	BRANCH_B_EQ_DIRECT,	// <addr>	<offset>
	BRANCH_B_NE_DIRECT,	// <addr>	<offset>
	BRANCH_B_GT_IMM8,	// #00	<offset>
	BRANCH_B_GE_IMM8,	// #00	<offset>
	BRANCH_B_LE_IMM8,	// #00	<offset>
	BRANCH_B_LT_IMM8,	// #00	<offset>
	BRANCH_B_EQ_IMM8,	// #00	<offset>
	BRANCH_B_NE_IMM8,	// #00	<offset>
	BRANCH_B_GT_IMM16,	// #00 00	<offset>
	BRANCH_B_GE_IMM16,	// #00 00	<offset>
	BRANCH_B_LE_IMM16,	// #00 00	<offset>
	BRANCH_B_LT_IMM16,	// #00 00	<offset>
	BRANCH_B_EQ_IMM16,	// #00 00	<offset>
	BRANCH_B_NE_IMM16,	// #00 00	<offset>
	BRANCH_NOT_TIMER1,	// <offset>	branch if timer1 hasn't triggered, else clear and do next instruction
	BRANCH_NOT_TIMER2,	// <offset>
	BRANCH_NOT_TIMER3,	// <offset>
	BRANCH_NOT_TIMER4,	// <offset>
	BRANCH_NOT_TIMER5,	// <offset>
	SWAP_A_B,
	SWAP_A_C,
	SWAP_B_C,
	ADD_A_B,			// a = a+b
	ADD_B_A,
	SUB_A_B,			// a = a-b
	SUB_B_A,
	MUL_A_B,			// a = a*b
	MUL_B_A,
	DIV_A_B,			// a = a/b
	DIV_B_A,			// b = b/a
	MOD_A_B,			// a = a % b
	MOD_B_A,			// b = b % a
	AND_A_B,			// a = a & b
	AND_B_A,
	OR_A_B,				// a = a | b
	OR_B_A,
	OR_A_IMM32,			// a = a | imm32
	OR_B_IMM32,			// b = b | imm32
	AND_A_IMM32,		// a = a & imm32
	AND_B_IMM32,		// b = b & imm32
	LSHIFT_A_IMM8,		// #00		a = a << IMM
	LSHIFT_B_IMM8,		// #00
	RSHIFT_A_IMM8,		// #00		a = a >> IMM
	RSHIFT_B_IMM8,		// #00
	NOT_A,				// a = ~a
	NOT_B,
	NEG_A,				// a = -a
	NEG_B,				// b = -b
	MOV_A_VERB,			// bcd encoded. use only least signifigant byte
	MOV_A_NOUN,
	MOV_A_PROG,
	MOV_NOUN_A,
	MOV_VERB_A,
	MOV_PROG_A,
	BLINK_VERB,			// 0|1	enabled/disable blinking of VERB digits
	BLINK_NOUN,			// 0|1	enabled/disable blinking of NOUN digits
	BLINK_PROG,			// 0|1
	BLINK_KEYREL,		// 0|1
	BLINK_OPRERR,		// 0|1
	BLINK_R1,			// 0|1
	BLINK_R2,			// 0|1
	BLINK_R3,			// 0|1
	LT_UPLINK_ACTY,		// 0|1
	LT_NO_ATT,			// 0|1
	LT_STBY,			// 0|1
	LT_OPR_ERR,			// 0|1
	LT_KEY_REL,			// 0|1
	LT_NA1,				// 0|1
	LT_NA2,				// 0|1
	LT_TEMP,			// 0|1
	LT_GIMBAL_LOCK,		// 0|1
	LT_PROG_ALRM,		// 0|1
	LT_RESTART,			// 0|1
	LT_TRACKER,			// 0|1
	LT_ALT,				// 0|1
	LT_VEL,				// 0|1
	LT_COMP_ACTY,		// 0|1	turn COMP_ACTY light on/off
	LT_VERB,			// 0|1	turn VERB light on/off
	LT_NOUN,			// 0|1	turn NOUN light on/off
	LT_PROG,			// 0|1	turn PROG light on/off
	LT_ALL,				// 0|1	turn all lights on or off
	UPLINK_PROB_IMM8,	// <val8>		set UPLINK ACTY random blink probability with immediate byte
	COMPACTY_PROB_IMM8,	// <val8>		set COMP ACTY random blink probability with immediate byte
	GPS_LAT_A,
	GPS_LON_A,
	GPS_YEAR_A,
	GPS_MON_A,
	GPS_DAY_A,
	GPS_HH_A,
	GPS_MM_A,
	GPS_SS_A,
	BRANCH_TIMESTAMP_LT,	// <addr1> <addr2>  branch if timestamp1 less than timestamp2
	TIMESTAMP_DIFF_A,		// <addr1> <addr2>	diff timestamp1 in addr1 and timestamp2 in addr2 => A
	RTC_TIMESTAMP_DIRECT,	// <addr>	store RTC timestamp to addr+0 and addr+1
	RTC_DAY_A,
	RTC_YEAR_A,
	RTC_MON_A,
	RTC_HH_A,
	RTC_MM_A,
	RTC_SS_A,
	RTC_MEM_A_CINDIRECT,	// <addr>		56 byte memory	C
	RTC_A_MEM_CINDIRECT,	// <addr>		56 byte memory	C
	IMU_READ_DIRECT,		// <addr>		store IMU data into <addr+0> ... <addr+6> locations
	MP3_PLAY_A,				// play track indicated by register A
	EEPROM_WRITE_A_CINDIRECT,	// write A register byte to EEPROM[C]
	EEPROM_READ_A_CINDIRECT,	// read into A register byte from EEPROM[C]
	WAIT1,					// 100ms wait for wait flag to become 1, don't advance PC until it becomes 1.
	WAIT2,					// 200ms
	WAIT3,					// 300ms
	WAIT4,					// 1s
	WAIT5,					// 2s
	INPUT_NOUN,
	INPUT_R1,
	INPUT_R2,
	INPUT_R3,
	INPUT_R1_OCT,	// read an octal value from keyboard and put into R1	A=result 0=good key=bad
	INPUT_R2_OCT,	// read an octal value from keyboard and put into R2	A=result 0=good key=bad
	INPUT_R3_OCT,	// read an octal value from keyboard and put into R3	A=result 0=good key=bad
	INPUT_PROCEED,	// ask user to press PRO key (proceed)	A=0 proceed, A=-1 cancelled
	INPUT_REQ_PROCEED,	// required proceed pressA=1 proceed, A=0
	PROG8_A_CINDIRECT,		// A = Program[C]		byte
	PROG16_A_CINDIRECT,		// A = Program[C]		short (little endian)
	PROG32_A_CINDIRECT,		// A = Program[C]		int (little endian)
	ADD_A_IMM8,				// #00
	ADD_B_IMM8,				// #00		B = B + IMM8
	ADD_C_IMM8,				// #00
	EMPTY_STACK,			// reset stack to being empty
	RUN_PROG_A,				// Run major mode program whose 16-bit address is in 'A'
	RUN_MINOR_A,			// Run minor mode whose 16-bit address is in 'A'
	CALL_CINDIRECT,			// call a routine using C register
	PUSH_DSKY,				// push DSKY state on stack
	POP_DSKY,				// push DSKY state on stack
	RANDOM_A,				// Populate A with a random 16-bit value
	AGC_INSTRUCTIONS_LEN,	// number of instruction opcode must be <= 255
};

#ifdef DSKY_DEBUG
//
// Only used when DSKY_DEBUG is enabled. Allows for printing out the
// instructions being executed. Keep this in sync with the above enum.
//
static const char *Mnemonics[] = {
	"MOV_R1_A",
	"MOV_R2_A",
	"MOV_R3_A",
	"MOV_A_R1",
	"MOV_A_R2",
	"MOV_A_R3",
	"DECODE_A_FROM_OCT",	// Convert BCD encoding of octal value into a INT
	"DECODE_A_FROM_DEC",	// Convert BCD encodfing of decimal into a INT
	"ENCODE_A_TO_OCT",	// Encode A contents as BCD encoded octal
	"ENCODE_A_TO_DEC",	// Encode A contents as BCD encoded decimal (plus/minus sign)
	"ENCODE_A_TO_UDEC",	// Encode Positive decimal value (No plus sign) into BCD
	"MOV_A_B",
	"MOV_A_C",
	"MOV_B_A",
	"MOV_B_C",
	"MOV_C_A",
	"MOV_C_B",
	"LD_A_DIRECT",		// <addr>
	"LD_B_DIRECT",		// <addr>
	"LD_C_DIRECT",		// <addr>
	"LD_A_IMM32",			// #00 00 00 00
	"LD_B_IMM32",			// #00 00 00 00
	"LD_C_IMM32",			// #00 00 00 00
	"LD_A_IMM16",			// #00 00
	"LD_B_IMM16",			// #00 00
	"LD_C_IMM16",			// #00 00
	"LD_A_IMM8",			// #00
	"LD_B_IMM8",			// #00
	"LD_C_IMM8",			// #00
	"LD_A_CINDIRECT",		// 		load A with (C)
	"LD_B_CINDIRECT",		//		load B with (C)
	"ST_A_DIRECT",			// <addr>
	"ST_B_DIRECT",			// <addr>
	"ST_C_DIRECT",			// <addr>
	"ST_A_CINDIRECT",		//			store A to (C)
	"ST_B_CINDIRECT",		//			store B to (C)
	"CLR_A",
	"CLR_B",
	"CLR_C",
	"INC_A",
	"INC_B",
	"INC_C",
	"DEC_A",
	"DEC_B",
	"DEC_C",
	"PUSH_A",
	"PUSH_B",
	"PUSH_C",
	"POP_A",
	"POP_B",
	"POP_C",
	"CALL",				// <addr_2byte>
	"RET",
	"GOTO",				// <addr_2byte>
	"BRANCH",				// <offset>		relative +127/-128
	"BRANCH_A_GT_B",		// <offset>
	"BRANCH_A_GE_B",		// <offset>
	"BRANCH_A_LE_B",		// <offset>
	"BRANCH_A_LT_B",		// <offset>
	"BRANCH_A_EQ_B",		// <offset>
	"BRANCH_A_NE_B",		// <offset>
	"BRANCH_A_GT_DIRECT",	// <addr> 	<offset>
	"BRANCH_A_GE_DIRECT",	// <addr>	<offset>
	"BRANCH_A_LE_DIRECT",	// <addr>	<offset>
	"BRANCH_A_LT_DIRECT",	// <addr>	<offset>
	"BRANCH_A_EQ_DIRECT",	// <addr>	<offset>
	"BRANCH_A_NE_DIRECT",	// <addr>	<offset>
	"BRANCH_A_GT_IMM8",	// #00	<offset>
	"BRANCH_A_GE_IMM8",	// #00	<offset>
	"BRANCH_A_LE_IMM8",	// #00	<offset>
	"BRANCH_A_LT_IMM8",	// #00	<offset>
	"BRANCH_A_EQ_IMM8",	// #00	<offset>
	"BRANCH_A_NE_IMM8",	// #00	<offset>
	"BRANCH_A_GT_IMM16",	// #00 00	<offset>
	"BRANCH_A_GE_IMM16",	// #00 00	<offset>
	"BRANCH_A_LE_IMM16",	// #00 00	<offset>
	"BRANCH_A_LT_IMM16",	// #00 00	<offset>
	"BRANCH_A_EQ_IMM16",	// #00 00	<offset>
	"BRANCH_A_NE_IMM16",	// #00 00	<offset>
	"BRANCH_B_GT_DIRECT",	// <addr>	<offset>
	"BRANCH_B_GE_DIRECT",	// <addr>	<offset>
	"BRANCH_B_LE_DIRECT",	// <addr>	<offset>
	"BRANCH_B_LT_DIRECT",	// <addr>	<offset>
	"BRANCH_B_EQ_DIRECT",	// <addr>	<offset>
	"BRANCH_B_NE_DIRECT",	// <addr>	<offset>
	"BRANCH_B_GT_IMM8",	// #00	<offset>
	"BRANCH_B_GE_IMM8",	// #00	<offset>
	"BRANCH_B_LE_IMM8",	// #00	<offset>
	"BRANCH_B_LT_IMM8",	// #00	<offset>
	"BRANCH_B_EQ_IMM8",	// #00	<offset>
	"BRANCH_B_NE_IMM8",	// #00	<offset>
	"BRANCH_B_GT_IMM16",	// #00 00	<offset>
	"BRANCH_B_GE_IMM16",	// #00 00	<offset>
	"BRANCH_B_LE_IMM16",	// #00 00	<offset>
	"BRANCH_B_LT_IMM16",	// #00 00	<offset>
	"BRANCH_B_EQ_IMM16",	// #00 00	<offset>
	"BRANCH_B_NE_IMM16",	// #00 00	<offset>
	"BRANCH_NOT_TIMER1",	// <offset>	branch if timer1 hasn't triggered, else clear and do next instruction
	"BRANCH_NOT_TIMER2",	// <offset>
	"BRANCH_NOT_TIMER3",	// <offset>
	"BRANCH_NOT_TIMER4",	// <offset>
	"BRANCH_NOT_TIMER5",	// <offset>
	"SWAP_A_B",
	"SWAP_A_C",
	"SWAP_B_C",
	"ADD_A_B",			// a = a+b
	"ADD_B_A",
	"SUB_A_B",			// a = a-b
	"SUB_B_A",
	"MUL_A_B",			// a = a*b
	"MUL_B_A",
	"DIV_A_B",			// a = a/b
	"DIV_B_A",			// b = b/a
	"MOD_A_B",			// a = a % b
	"MOD_B_A",			// b = b % a
	"AND_A_B",			// a = a & b
	"AND_B_A",
	"OR_A_B",				// a = a | b
	"OR_B_A",
	"OR_A_IMM32",
	"OR_B_IMM32",
	"AND_A_IMM32",
	"AND_B_IMM32",
	"LSHIFT_A_IMM8",		// #00		a = a << IMM
	"LSHIFT_B_IMM8",		// #00
	"RSHIFT_A_IMM8",		// #00		a = a >> IMM
	"RSHIFT_B_IMM8",		// #00
	"NOT_A",				// a = ~a
	"NOT_B",
	"NEG_A",				// a = -a
	"NEG_B",				// b = -b
	"MOV_A_VERB",			// bcd encoded. use only least signifigant byte
	"MOV_A_NOUN",
	"MOV_A_PROG",
	"MOV_NOUN_A",
	"MOV_VERB_A",
	"MOV_PROG_A",
	"BLINK_VERB",			// 0|1	enabled/disable blinking of VERB digits
	"BLINK_NOUN",			// 0|1	enabled/disable blinking of NOUN digits
	"BLINK_PROG",			// 0|1
	"BLINK_KEYREL",		// 0|1
	"BLINK_OPRERR",		// 0|1
	"BLINK_R1",			// 0|1
	"BLINK_R2",			// 0|1
	"BLINK_R3",			// 0|1
	"LT_UPLINK_ACTY",		// 0|1
	"LT_NO_ATT",			// 0|1
	"LT_STBY",			// 0|1
	"LT_OPR_ERR",			// 0|1
	"LT_KEY_REL",			// 0|1
	"LT_NA1",				// 0|1
	"LT_NA2",				// 0|1
	"LT_TEMP",			// 0|1
	"LT_GIMBAL_LOCK",		// 0|1
	"LT_PROG_ALRM",		// 0|1
	"LT_RESTART",			// 0|1
	"LT_TRACKER",			// 0|1
	"LT_ALT",				// 0|1
	"LT_VEL",				// 0|1
	"LT_COMP_ACTY",		// 0|1	turn COMP_ACTY light on/off
	"LT_VERB",			// 0|1	turn VERB light on/off
	"LT_NOUN",			// 0|1	turn NOUN light on/off
	"LT_PROG",			// 0|1	turn PROG light on/off
	"LT_ALL",			// 0|1	turn all lights on or off
	"UPLINK_PROB_IMM8",
	"COMPACTY_PROB_IMM8",
	"GPS_LAT_A",
	"GPS_LON_A",
	"GPS_YEAR_A",
	"GPS_MON_A",
	"GPS_DAY_A",
	"GPS_HH_A",
	"GPS_MM_A",
	"GPS_SS_A",
	"BRANCH_TIMESTAMP_LT",
	"TIMESTAMP_DIFF_A",
	"RTC_TIMESTAMP_DIRECT",
	"RTC_DAY_A",
	"RTC_YEAR_A",
	"RTC_MON_A",
	"RTC_HH_A",
	"RTC_MM_A",
	"RTC_SS_A",
	"RTC_MEM_A_CINDIRECT",	// <addr>	56 byte memory	C
	"RTC_A_MEM_CINDIRECT",	// <addr>	56 byte memory	C
	"IMU_READ_DIRECT",
	"MP3_PLAY_A",				// play track indicated by register A
	"EEPROM_WRITE_A_CINDIRECT",	// <2byteaddr>			<2byteaddr>+C
	"EEPROM_READ_A_CINDIRECT",	// <2byteaddr>			<2byteaddr>+C
	"WAIT1",					// wait for wait flag to become 1, don't advance PC until it becomes 1.
	"WAIT2",
	"WAIT3",
	"WAIT4",
	"WAIT5",
	"INPUT_NOUN",
	"INPUT_R1",
	"INPUT_R2",
	"INPUT_R3",
	"INPUT_R1_OCT",	// read an octal value from keyboard and put into R1	A=result 0=good key=bad
	"INPUT_R2_OCT",	// read an octal value from keyboard and put into R2	A=result 0=good key=bad
	"INPUT_R3_OCT",	// read an octal value from keyboard and put into R3	A=result 0=good key=bad
	"INPUT_PROCEED",
	"INPUT_REQ_PROCEED",
	"PROG8_A_CINDIRECT",		// A = Program[C]		byte
	"PROG16_A_CINDIRECT",		// A = Program[C]		short (little endian)
	"PROG32_A_CINDIRECT",		// A = Program[C]		int (little endian)
	"ADD_A_IMM8",				// #00
	"ADD_B_IMM8",				// #00		B = B + IMM8
	"ADD_C_IMM8",				// #00
	"EMPTY_STACK",			// reset stack to being empty
	"RUN_PROG_A",				// Run major mode program in 'A'
	"RUN_MINOR_A",				// Run minor mode in 'A'
	"CALL_CINDIRECT",			// call a routine using C register
	"PUSH_DSKY",				// push DSKY state on stack
	"POP_DSKY",				// push DSKY state on stack
	"RANDOM_A",				// Populate A with a random 16-bit value
	"AGC_INSTRUCTIONS_LEN",	// number of instruction opcode must be <= 127
};
#endif

enum AGC_FLAGS {
	AGC_TIMER1 = 1<<0,				// 100ms 1/10 second
	AGC_TIMER2 = 1<<1,				// 200ms 1/5 second
	AGC_TIMER3 = 1<<2,				// 300ms 3/10 second
	AGC_TIMER4 = 1<<3,				// 1000ms 1 second
	AGC_TIMER5 = 1<<4,				// 2000ms 2 second
	AGC_NA1 = 1<<5,
	AGC_NA2 = 1<<6,
	AGC_NA3 = 1<<7,
};

// each cpu core has a 'flags' register with these bits
enum CPU_FLAGS {
	CPU_NOT_USED	= 1<<0,		// not used
	CPU_TIMER1		= 1<<1,		// 100ms	1/10 second
	CPU_TIMER2		= 1<<2,		// 200ms	1/5 second
	CPU_TIMER3		= 1<<3,		// 300ms	3/10 second
	CPU_TIMER4		= 1<<4,		// 1000ms	1 second
	CPU_TIMER5		= 1<<5,		// 2000ms	2 seconds
	CPU_NEED_INPUT	= 1<<6,		// cpu is paused until user input received
	CPU_GOT_INPUT	= 1<<7,		// indicates that user input has been received (placed into A register)
};

#define CPU0_STACK	127		// initial stack pointer for Agc.cpu[0]
#define CPU1_STACK	112		// initial stack pointer for Agc.cpu[1]

//
// the cpu core, there are two of these.
//	Agc.cpu[0]		runs the major mode (PROG's)
//	Agc.cpu[1]		runs a foreground task (VERB's)
//
struct CPU
{
	int32_t		regs[3];		// general purpose registers A, B, C
	uint8_t		SP;				// stack pointer. points into RAM
	uint16_t	PC;				// program counter. points into 'Program[]' array
	uint8_t		flags;			// CPU_FLAGS
};

//
// Everything about the Apollo Guidance Computer is contained
// in this structure. (except Verbs[] table and Program[] byte codes)
//
struct APOLLO_GUIDANCE_COMPUTER
{
	DSKY		dsky;			// the current DSKY display state
	DSKY		prev;			// the previous DSKY display state
	CPU			cpu[2];			// two cpu cores (cpu=0 major mode; cpu=1 foreground task)
	int32_t		RAM[128];		// read-write memory shared between cpu's
	uint8_t		state;			// DSKY_STATE (state machine state)
	uint8_t		flags;			// AGC_FLAGS
	uint8_t		input_reg;		// dsky register (1, 2, or 3) used by user input instructions
	uint8_t		uplink_prob;	// probability UPLINK ACTY light will be on. 0=off, 255 max rate
	uint8_t		compacty_prob;	// probability COMP ACTY light will be on. 0=off, 255 max rate
};

//
// Used to launch a VERB given its number 'cmd'.
//
struct DISPATCH_ENTRY {
	uint8_t		cmd;			// verb number (BCD encoded)
	uint16_t	start;			// start location in 'Program[]' array
};

//////////////////////////////////////////////////////////////////////
//
// SECTION 4: Miscellaneous Arduino Mocks
//
//
#ifdef CURSES_SIMULATOR
static void cli()
{
}

static void sei()
{
}

static uint8_t analogRead(uint8_t pin)
{
	return 0;
}

//
// Mock pinMode() routine
//
#define INPUT 0
#define OUTPUT 1
#define A0 0
#define A1 1
#define A2 2
#define A7 7
#define LOW 0

void pinMode(uint8_t pin, uint8_t mode)
{
}

void digitalWrite(uint8_t pin, uint8_t sig)
{
}

//
// Mock delay() routine
//
static void delay(uint16_t ms)
{
}

int16_t map(int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static
int16_t constrain(int16_t value, int16_t lower, int16_t upper)
{
	return (value < lower) ? lower : ( (value > upper) ? upper : value);
}

#endif

//////////////////////////////////////////////////////////////////////
//
// SECTION 5: 'Adafruit_NeoPixel' Mock
//
#ifdef CURSES_SIMULATOR
class Adafruit_NeoPixel
{
public:
	Adafruit_NeoPixel(uint8_t numpixels, uint8_t pin, uint8_t flags)
	{
	}

	void begin()
	{
	}
};
#endif

//////////////////////////////////////////////////////////////////////
//
// SECTION 6: 'LedControl' Mock
//
#ifdef CURSES_SIMULATOR
class LedControl
{
public:
	LedControl(uint8_t x, uint8_t y, uint8_t z, uint8_t w)
	{
	}

	void shutdown(uint8_t index, bool state)
	{
	}

	void setIntensity(uint8_t index, uint8_t intensity)
	{
	}

	void clearDisplay(uint8_t index)
	{
	}
};
#endif

//////////////////////////////////////////////////////////////////////
//
// SECTION 7: 'Random()' Mock Routines
//
#ifdef CURSES_SIMULATOR
static void randomSeed(uint8_t seed)
{
}

static uint32_t random(uint32_t val)
{
	uint32_t x;
	getrandom(&x, sizeof(x), GRND_RANDOM);
	x = x % val;
	return x;
}
#endif

#ifdef CURSES_SIMULATOR
//////////////////////////////////////////////////////////////////////
//
// SECTION 8: 'Wire' and 'Serial' Mock
//
// Emulates the RTC and IMU devices
// This is a minimal mock 'Wire' class that can simulate the RTC and IMU devices.
//
class TwoWire
{
public:
	// call this every second to increment the real time clock simulated date/time.
	// also tweaks the IMU values randomly.
	static void tick()
	{
		seconds = dec2bcd( bcd2dec(seconds) + 1);
		if( bcd2dec(seconds) > 59 )
		{
			seconds = 0x00;
			minutes = dec2bcd( bcd2dec(minutes) + 1);
		}

		if( bcd2dec(minutes) > 59 )
		{
			minutes = 0x00;
			hours = dec2bcd( bcd2dec(hours) + 1);
		}

		if( h24 && bcd2dec(hours) > 23 )
		{
			hours = 0x00;
			day = dec2bcd( bcd2dec(day) + 1 );
			dow += 1;
			if( dow > 7 )
				dow = 1;
		}
		else if( !h24 && bcd2dec(hours) > 12 )
		{
			hours = 0x01;
			if( pm ) {
				day = dec2bcd( bcd2dec(day) + 1 );
				dow += 1;
				if( dow > 7 )
					dow = 1;
				pm = false;
			} else {
				pm = true;
			}
		}

		if( bcd2dec(day) > month_duration[ bcd2dec(month) ] )
		{
			day = 0x01;
			month = dec2bcd( bcd2dec(month) + 1 );
		}

		if( bcd2dec(month) > 12 )
		{
			month = 0x01;
			year = dec2bcd( bcd2dec(year) + 1 );
		}
	}

	static uint8_t get_rtc_ram(uint8_t addr)
	{
		if( addr >= 56 )
		{
			fprintf(logfp, "Wire.get_rtc_ram(%d) bad address\n", addr);
			return 0;
		}
		return RTC_RAM[addr];
	}

	static void set_rtc_ram(uint8_t addr, uint8_t b)
	{
		if( addr >= 56 )
		{
			fprintf(logfp, "Wire.set_rtc_ram(%d) bad address\n", addr);
			return;
		}
		RTC_RAM[addr] = b;
	}
	
	static void begin()
	{
		m_device = 0x00;
		m_startAddr = 0x00;
		m_addr = 0x00;
		m_needAddr = true;

		time_t x = time(NULL);
		struct tm tm = *localtime(&x);

		year = dec2bcd(tm.tm_year % 100);
		month = dec2bcd(tm.tm_mon + 1);
		day = dec2bcd(tm.tm_mday);
		hours = dec2bcd(tm.tm_hour);
		minutes = dec2bcd(tm.tm_min);
		seconds = dec2bcd(tm.tm_sec);
		dow = dec2bcd(tm.tm_wday+1);
		h24 = true;
		pm = false;
		fprintf(logfp, "H: %02x M: %02x S: %02x\n", hours, minutes, seconds);
	}

	static void beginTransmission(uint8_t device)
	{
		m_device = device;
		m_needAddr = true;
	}

	static void endTransmission(uint8_t sendStop)
	{
	}

	static void requestFrom(uint8_t device, uint8_t len, uint8_t sendStop)
	{
		m_device = device;
		m_len = len;
	}

	static int available()
	{
		return (m_addr - m_startAddr);
	}

	static void write(uint8_t val)
	{
#ifdef DSKY_DEBUG
		fprintf(logfp, "TwoWire.Write() m_needAddr=%d m_addr=%02x val=%02x\n", m_needAddr, m_addr, val);
#endif
		if( m_needAddr )
		{
			m_needAddr = false;
			m_addr = val;
		} else {
			writeByte(m_device, m_addr, val);
			m_addr++;
		}
	}

	static uint8_t read()
	{
		uint8_t byte;

		byte = readByte(m_device, m_addr);
#ifdef DSKY_DEBUG
		fprintf(logfp, "ReadByte(%02X) = %02X\n", m_addr, byte);
#endif
		m_addr++;
		return byte;
	}

private:
	static uint8_t dec2bcd(uint8_t val)
	{
		return ((val / 10) << 4) | (val % 10);
	}

	static uint8_t bcd2dec(uint8_t val)
	{
		return ((val & 0xf0) >> 4) * 10 + (val & 0x0f);
	}

	static uint8_t readByte(uint8_t device, uint8_t addr)
	{
		if( device == RTC_ADDR )
		{
			addr = addr & 0x3F;

			if( addr < 0x08 )
			{
				switch(addr)
				{
				case 0x00:	// seconds
					return seconds;

				case 0x01:	// minutes
					return minutes;

				case 0x02:	// hours
					if( h24 )
						return hours;
					else
						return hours | (1<<6) | (((pm) ? 1 : 0) << 5);

				case 0x03:	// dow
					return dow;

				case 0x04:	// day
					return day;

				case 0x05:	// month
					return month;

				case 0x06:	// year
					return year % 100;

				case 0x07:	// control
					return 0x00;
				}
			}
			else if( addr <= 0x3F )
			{
				return RTC_RAM[addr - 0x08];
			}
		}
		else if( device == MPU_ADDR )		// IMU
		{
			addr = addr & 0xff;
			switch( addr )
			{
			case 0x3B:	// (ACCEL_XOUT_H)
				return random(0xff);

			case 0x3C:	// (ACCEL_XOUT_L)
				return random(0xff);

			case 0x3D:	// (ACCEL_YOUT_H)
				return random(0xff);

			case 0x3E:	// (ACCEL_YOUT_L)
				return random(0xff);

			case 0x3F:	// (ACCEL_ZOUT_H)
				return random(0xff);

			case 0x40:	// (ACCEL_ZOUT_L)
				return random(0xff);

			case 0x41:	// (TEMP_OUT_H)
				return random(0xff);

			case 0x42:	// (TEMP_OUT_L)
				return random(0xff);

			case 0x43:	// (GYRO_XOUT_H)
				return random(0xff);

			case 0x44:	// (GYRO_XOUT_L)
				return random(0xff);

			case 0x45:	// (GYRO_YOUT_H)
				return random(0xff);

			case 0x46:	// (GYRO_YOUT_L)
				return random(0xff);

			case 0x47:	// (GYRO_ZOUT_H)
				return random(0xff);

			case 0x48:	// (GYRO_ZOUT_L)
				return random(0xff);
			}

		}
		return 0;
	}

	static void writeByte(uint8_t device, uint8_t addr, uint8_t val)
	{
		if( device == 0x68 )		// RTC
		{
			addr = addr & 0x3F;

			if( addr < 0x08 ) {
				switch(addr)
				{
				case 0x00:	// seconds
					seconds = val;
					break;

				case 0x01:	// minutes
					minutes = val;
					break;

				case 0x02:	// hours
					if( val & 0x40 ) {
						h24 = false;
						pm = (val & 0x20) ? false : true;
						hours = val & 0x1f;
					} else {
						h24 = true;
						hours = val & 0x3f;
					}
					hours = val;
					break;

				case 0x03:	// dow
					dow = val;
					break;

				case 0x04:	// day
					day = val;
					break;

				case 0x05:	// month
					month = val;
					break;

				case 0x06:	// year
					year = val;
					break;

				case 0x07:	// control
					break;
				}
			} else if( addr <= 0x3F ) {
				RTC_RAM[addr - 0x08] = val;
			}
		}
	}

	static uint8_t m_device;
	static uint8_t m_startAddr;
	static uint8_t m_addr;
	static uint8_t m_len;
	static uint8_t m_needAddr;
	static uint8_t seconds;
	static uint8_t minutes;
	static uint8_t hours;
	static bool h24;
	static bool pm;
	static uint8_t dow;
	static uint8_t day;
	static uint8_t month;
	static uint8_t year;
	static uint8_t RTC_RAM[56];
	static uint8_t month_duration[12];
};

uint8_t TwoWire::m_device;
uint8_t TwoWire::m_startAddr;
uint8_t TwoWire::m_addr;
uint8_t TwoWire::m_len;
uint8_t TwoWire::m_needAddr;
uint8_t TwoWire::seconds;
uint8_t TwoWire::minutes;
uint8_t TwoWire::hours;
bool    TwoWire::h24;
bool    TwoWire::pm;
uint8_t TwoWire::dow;
uint8_t TwoWire::day;
uint8_t TwoWire::month;
uint8_t TwoWire::year;
uint8_t TwoWire::RTC_RAM[56];
uint8_t TwoWire::month_duration[12] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

// Mock Stream class
class Stream
{
public:
	void begin(uint16_t baud)
	{
	}

	void write(uint8_t *buf, uint8_t len)
	{
	}
};

static Stream Serial;		// Mock Serial object
static TwoWire Wire;		// Mock Wire object

#endif

//////////////////////////////////////////////////////////////////////
//
// SECTION 9: MP3 Player Curses Routines
//
// These routines render the audio track being played on
// the bottom line of the curses window.
//
#ifdef CURSES_SIMULATOR

static const struct {
	const char	*name;
	int			duration;		// seconds
} trackdb[] = {
	"",											0,			// not a valid track number
	"01-Silence",								2,			// first valid track number
	"02-Alarm w lead",							60,
	"03-JFK I Believe - Short w lead",			14,
	"04-JFK We Choose - Short w lead",			10,
	"05-A8 Genesis - Short w lead",				43,
	"06-A11 Eagle Has Landed - Short w lead",	6,
	"07-A13 problem - Short w lead",			13,
	"08-A11 Go for Landing w lead",				5,
	"09-A11 Launch to Orbit w lead",			13,
	"10-A12 Conrad Bugs Bunny w lead",			20,
	"11-A11 One Small Step w lead",				14,
	"12-A17 Strolling - Long w lead",			48,
	"13-JFK I Believe - Long w lead",			27,
	"14-JFK We Choose - Long w lead",			28,
	"15-A8 Genesis Long w lead",				60+54,
	"16-A17 Orange Soil - Short w lead",		34,
	"17-A13 problem - Long w lead",				18,			// last valid track number
};

static int mp3_remaining;
static int mp3_track;

//
// print out mp3 playing status
//
void mp3_update_curses()
{
	char buf[100];

	wmove(c_win, 23, 0);
	if( mp3_track == 0 ) {
		waddstr(c_win, "              --No Audio--               ");
	} else {
		snprintf(buf, sizeof(buf), " %-36.36s %03d", trackdb[mp3_track].name, mp3_remaining);
		waddstr(c_win, buf);
	}
}

//
// Call this every second to update the CURSES SIMULATOR audio text
//
static void mp3_increment_1s()
{
	if( mp3_track > 0 )
	{
		mp3_remaining--;
		if( mp3_remaining < 0 )
		{
			mp3_track = 0;
		}
		mp3_update_curses();
	}
}

static void mp3_play(uint8_t track_number)
{
	if( track_number > TRACKDB_LEN )
	{
		fprintf(logfp, "Invalid track number %d\n", track_number);
		return;
	}
	fprintf(logfp, "Playing track number %d\n", track_number);
	mp3_track = track_number;
	mp3_remaining = trackdb[mp3_track].duration;
	mp3_update_curses();
}
#endif

//////////////////////////////////////////////////////////////////////
//
// SECTION 10: 'EEPROM' Mock
//
#ifdef CURSES_SIMULATOR
class EEPROMClass
{
public:
	uint8_t read(uint16_t addr)
	{
		if( addr >= 2048 )
		{
			fprintf(logfp, "EEPROM.read(%04x) bad address\n", addr);
			return 0;
		}
		return storage[addr];
	}

	void write(uint16_t addr, uint8_t value)
	{
		if( addr >= 2048 )
		{
			fprintf(logfp, "EEPROM.write(%04x, %02x) bad address\n", addr, value);
			return;
		}
		storage[addr] = value;
	}

	void update(uint16_t addr, uint8_t value)
	{
		if( read(addr) != value )
		{
			write(addr, value);
		}
	}

private:
	uint8_t storage[2048];		// simulated EEPROM 2K memory
};

static EEPROMClass EEPROM;	// Mock EEPROM object

#endif

//////////////////////////////////////////////////////////////////////
//
// SECTION 11: Globals Variables (Read-Only & Read/Write)
//
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
//
// SUB-SECTION 11a: Read-Only globals (constants in PROGMEM)
//

//
// The machine code for each VERB and PROG
// This is the "byte code" for the virtual machine.
// All programs and verbs are contained in this program space.
//
// Program[] array comes from the assembly code 'kennysagc.asm'.
// which is assembled into the file, "./kennysagc.h".
// The program which creates this file is called: "assembler.py".
//
#include "./kennysagc.h"			// Program[] array

// Verb Table
static const DISPATCH_ENTRY Verbs[] PROGMEM = {
	{ 0x01, LBL_VERB_01 },
	{ 0x02, LBL_VERB_02 },
	{ 0x03, LBL_VERB_03 },
	{ 0x04, LBL_VERB_04 },
	{ 0x05, LBL_VERB_05 },
	{ 0x06, LBL_VERB_06 },
	{ 0x09, LBL_VERB_09 },
	{ 0x10, LBL_VERB_10 },
	{ 0x16, LBL_VERB_16 },
	{ 0x21, LBL_VERB_21 },
	{ 0x22, LBL_VERB_22 },
	{ 0x25, LBL_VERB_25 },
	{ 0x35, LBL_VERB_35 },
	{ 0x36, LBL_VERB_36 },
	{ 0x37, LBL_VERB_37 },
	{ 0x69, LBL_VERB_69 },
};

//
// This 2 dimensional array forms a state machine for handling
// user keyboard input. This table forms a state transistion graph.
// This table is indexed by the (state, input) variables:
//
//		new_state = StateMachine[state][input]
//
// Since the state machine table is stored in program memory (PROGMEM) this
// #define accessor macro must be used:
//
//		new_state = StateMachineAcc(state, input)
//
//	'state' is one of the enumeration's in DSKY_STATE.
//	'input' is one of the enumeration's in DSKY_INPUT (keypress character).
//
//	Each time a key press occurs we transistion to a new node in this graph.
//	'S_ST' is the start state.
//	'S_NOP' is a psuedo state, it keeps the current state but doesn't take action.
//
// There are other start states which are triggered by Apollo Guidance Computer
// virtual machine instructions. These are for prompting the user to enter data:
//
// Start State			Instruction(s)
//	S_ST		<none>										main start state
//	S_NI1		INPUT_NOUN									input noun
//	S_IR1		INPUT_R1, INPUT_R2, INPUT_R3, 				input register (decimal)
//	S_IO1		INPUT_R1_OCT, INPUT_R2_OCT, INPUT_R3_OCT	input register (octal)
//	S_IP1		INPUT_PROCEED								wait for PRO key press
//	S_IRP		INPUT_REQ_PROCEED							wait for PRO key press
//

static const uint8_t StateMachine[][10] PROGMEM = {
/* Input		VERB	NOUN	PRO		CLR		KREL	ENTR	RSET	PLUS	MINUS	DIGIT	*/
/*              =======	=======	=======	=======	=======	=======	=======	=======	=======	======= */
/* ST */		S_V1,	S_N1,	S_NOP,	S_NOP,	S_KRL,	S_ENT,	S_RST,	S_NOP,	S_NOP,	S_NOP,
/* ERR */		S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_ERR,	S_NOP,	S_ERR,	S_NOP,	S_NOP,	S_NOP,
/* NOP */		S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,

/* ENT */		S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,
/* KRL */		S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,
/* RST */		S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,
/* IN */		S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,
/* IV */		S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,
/* IE */		S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_IE,	S_NOP,	S_NOP,	S_NOP,

/* V1 */		S_V1,	S_N1,	S_VE,	S_NOP,	S_KRL,	S_VE,	S_NOP,	S_NOP,	S_NOP,	S_V2,
/* V2 */		S_V1,	S_N1,	S_VE,	S_NOP,	S_KRL,	S_VE,	S_NOP,	S_NOP,	S_NOP,	S_VA,
/* VA */		S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,
/* VE */		S_VE,	S_VE,	S_VE,	S_VE,	S_VE,	S_VE,	S_VE,	S_VE,	S_VE,	S_VE,

/* N1 */		S_V1,	S_N1,	S_NE,	S_NOP,	S_KRL,	S_NE,	S_NOP,	S_NOP,	S_NOP,	S_N2,
/* N2 */		S_V1,	S_N1,	S_NE,	S_NOP,	S_KRL,	S_NE,	S_NOP,	S_NOP,	S_NOP,	S_NA,
/* NA */		S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,
/* NE */		S_NE,	S_NE,	S_NE,	S_NE,	S_NE,	S_NE,	S_NE,	S_NE,	S_NE,	S_NE,

/* NI1 */		S_IV,	S_NI1,	S_NOP,	S_NOP,	S_NOP,	S_IE,	S_NOP,	S_NOP,	S_NOP,	S_NI2,
/* NI2 */		S_IV,	S_NI1,	S_NOP,	S_NOP,	S_NOP,	S_NIA,	S_NOP,	S_NOP,	S_NOP,	S_NI3,
/* NI3 */		S_IV,	S_NI1,	S_NOP,	S_NOP,	S_NOP,	S_NIA,	S_NOP,	S_NOP,	S_NOP,	S_NOP,
/* NIA */		S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,

/* IR1 */		S_IV,	S_IN,	S_NOP,	S_IR1,	S_NOP,	S_NOP,	S_NOP,	S_IR2,	S_IR3,	S_NOP,
/* IR2 */		S_IV,	S_IN,	S_NOP,	S_IR1,	S_NOP,	S_IE,	S_NOP,	S_NOP,	S_NOP,	S_IR4,
/* IR3 */		S_IV,	S_IN,	S_NOP,	S_IR1,	S_NOP,	S_IE,	S_NOP,	S_NOP,	S_NOP,	S_IR4,
/* IR4 */		S_IV,	S_IN,	S_NOP,	S_IR1,	S_NOP,	S_IRA,	S_NOP,	S_NOP,	S_NOP,	S_IR5,
/* IR5 */		S_IV,	S_IN,	S_NOP,	S_IR1,	S_NOP,	S_IRA,	S_NOP,	S_NOP,	S_NOP,	S_IR6,
/* IR6 */		S_IV,	S_IN,	S_NOP,	S_IR1,	S_NOP,	S_IRA,	S_NOP,	S_NOP,	S_NOP,	S_IR7,
/* IR7 */		S_IV,	S_IN,	S_NOP,	S_IR1,	S_NOP,	S_IRA,	S_NOP,	S_NOP,	S_NOP,	S_IR8,
/* IR8 */		S_IV,	S_IN,	S_NOP,	S_IR1,	S_NOP,	S_IRA,	S_NOP,	S_NOP,	S_NOP,	S_NOP,
/* IRA */		S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,

/* Input		VERB	NOUN	PRO		CLR		KREL	ENTR	RSET	PLUS	MINUS	DIGIT	*/
/*              =======	=======	=======	=======	=======	=======	=======	=======	=======	======= */

/* IO1 */		S_IV,	S_IN,	S_NOP,	S_IO1,	S_NOP,	S_IE,	S_NOP,	S_IE,	S_IE,	S_IO2,
/* IO2 */		S_IV,	S_IN,	S_NOP,	S_IO1,	S_NOP,	S_IOA,	S_NOP,	S_IE,	S_IE,	S_IO3,
/* IO3 */		S_IV,	S_IN,	S_NOP,	S_IO1,	S_NOP,	S_IOA,	S_NOP,	S_IE,	S_IE,	S_IO4,
/* IO4 */		S_IV,	S_IN,	S_NOP,	S_IO1,	S_NOP,	S_IOA,	S_NOP,	S_IE,	S_IE,	S_IO5,
/* IO5 */		S_IV,	S_IN,	S_NOP,	S_IO1,	S_NOP,	S_IOA,	S_NOP,	S_IE,	S_IE,	S_IO6,
/* IO6 */		S_IV,	S_IN,	S_NOP,	S_IO1,	S_NOP,	S_IOA,	S_NOP,	S_IE,	S_IE,	S_NOP,
/* IOA */		S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,

/* IP1 */		S_IV,	S_IN,	S_IPA,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,
/* IPA */		S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,

/* IPR */		S_NOP,	S_NOP,	S_IPA,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,	S_NOP,
};

//
// Accumulated days in year by month. Used by timestamp_diff().
//
static
const uint16_t accdbm[12] PROGMEM = { 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334 };

//////////////////////////////////////////////////////////////////////
//
// SUB-SECTION 11b: Read-Write globals (variables in RAM)
//
static APOLLO_GUIDANCE_COMPUTER Agc;
static Adafruit_NeoPixel neoPixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
static LedControl ledControl = LedControl(12, 10, 11, 4);
static uint8_t audioTrack;

#ifdef CURSES_SIMULATOR
//////////////////////////////////////////////////////////////////////
//
// SECTION 12: load/store mock RTC RAM and mock EEPROM data
//
//	The file to be read/written is: "./persist.txt"
//	If the file doesn't exist then initialize persistent storage with 0xFF bytes.
//

static
void read_persistent_storage()
{
	FILE *fp;
	char line[200];
	int b;		// byte value being composed
	int d;		// current digit index being processed (0 or 1).
	int digit;	// the digit being processed as a 4 bit value
	int lineno;
	char *p;
	int absolute_address;
	int i;

	fp = fopen("./persist.txt", "r");
	if( fp == NULL )
	{
		fprintf(logfp, "Unable to open ./persist.txt for reading (%s)\n", strerror(errno) );
		fprintf(logfp, "populating RTC RAM and EEPROM with 0xFF\n");

		//
		// When the file doesn't exist initialize the two persistent storage areas with 0xFF.
		//
		for(i=0; i < 56; i++)
		{
			Wire.set_rtc_ram(i, 0xFF);
		}

		for(i=0; i < 2048; i++)
		{
			EEPROM.write(i, 0xFF);
		}
		return;
	}

	absolute_address = 0;
	lineno = 0;
	while( ! feof(fp) )
	{
		p = fgets(line, sizeof(line), fp);
		if( p == NULL )		// EOF
			break;

		lineno += 1;

		if( line[0] == ';' )		// skip comments
			continue;

		if( line[0] == '\n' )		// skip blank lines
			continue;

		d = 0;
		b = 0;
		p = line;
		while( *p != '\n' ) {
			if( *p >= '0' && *p <= '9' ) {
				digit = *p - '0';
			} else if( *p >= 'A' && *p <= 'F' ) {
				digit = *p - 'A' + 10;
			} else {
				fprintf(logfp, "Parsing error line %d, expecting hex digit but got %d\n", lineno, *p);
				fclose(fp);
				return;
			}

			b = b*16 + digit;

			d += 1;
			p++;
			if( d <= 1 )
				continue;		// get 2nd nibble of hex byte

			d = 0;

			// got byte in 'b', now store it:

			if( absolute_address < 56 ) {
				Wire.set_rtc_ram(absolute_address, b);
			} else if( absolute_address < 56 + 2048 ) {
				EEPROM.update(absolute_address - 56, b);
			} else {
				fprintf(logfp, "reading persistent storage: Line %d, invalid address %04x (%02x)\n",
							lineno, absolute_address, b);
				fclose(fp);
				return;
			}

			absolute_address += 1;
			b = 0;

			if( *p == ' ' ) {
				p++;
			} else if( *p == '\n' ) {
				// no action
			} else {
				fprintf(logfp, "Parsing error line %d, expecting space or newline, got %d\n", lineno, *p);
				fclose(fp);
				return;
			}
		}
	}

	fclose(fp);
}

//
// Write the contents of the mock real-time clock data and EEPROM to disk.
//
static
void write_persistent_storage()
{
	FILE *fp;
	int i, col;
	uint8_t b;

	fp = fopen("./persist.txt", "w");
	if( fp == NULL )
	{
		fprintf(logfp, "Unable to open ./persist.txt for writing (%s)\n", strerror(errno) );
		return;
	}

	// Write Real Time Clock RAM
	fprintf(fp, "; Real Time Clock RAM 0x08 - 0x3F\n");

	col = 0;
	for(i=0; i < 56; i++)
	{
		b = Wire.get_rtc_ram(i);
		fprintf(fp, "%02X ", b);
		col++;
		if( col == 16 ) {
			col = 0;
			fprintf(fp, "\n");
		}
	}
	fprintf(fp, "\n");

	fprintf(fp, "; EEPROM 0x000 - 0x3ff\n");
	col = 0;
	for(i=0; i < 2048; i++)
	{
		b = EEPROM.read(i);
		fprintf(fp, "%02X ", b);
		col++;
		if( col == 16 ) {
			col = 0;
			fprintf(fp, "\n");
		}
	}

	fclose(fp);
}
#endif

#ifdef CURSES_SIMULATOR
//////////////////////////////////////////////////////////////////////
//
// SECTION 13: Curses window start/end
//
static
void curses_window_start()
{
	const int DSKY_WIDTH = 42;
	const int DSKY_HEIGHT = 24;

	int h, w;
	int nlines, ncols, y0, x0;

	getmaxyx(stdscr, h, w);
	fprintf(logfp, "dim h=%d w=%d\n", h, w);

	if( h < DSKY_HEIGHT || w < DSKY_WIDTH ) {
		fprintf(logfp, "screen too small. %dx%d required\n", DSKY_HEIGHT, DSKY_WIDTH);
		return;
	}

	y0 = h/2 - DSKY_HEIGHT/2;
	x0 = w/2 - DSKY_WIDTH/2;

	fprintf(logfp, "y0=%d, x0=%d\n", y0, x0);

	c_win = newwin(DSKY_HEIGHT, DSKY_WIDTH, y0, x0);

	wclear(c_win);

	wattron(c_win, A_NORMAL);

	static const char *dsky_layout[] = {
			"+---------+---------+--------------------+",	// 0
			"| UPLINK  |  TEMP   |   COMP      PROG   |",	// 1
			"|  ACTY   |         |   ACTY       88    |",	// 2
			"+---------+---------+                    |",	// 3
			"| NO ATT  | GIMBAL  |   VERB      NOUN   |",	// 4
			"|         |  LOCK   |    88        88    |",	// 5
			"+---------+---------+                    |",	// 6
			"|  STBY   |  PROG   |   --------------   |",	// 7
			"+---------+---------+       + 88888      |",	// 8
			"| KEY REL | RESTART |   --------------   |",	// 9
			"+---------+---------+       + 88888      |",	// 10
			"| OPR ERR | TRACKER |   --------------   |",	// 11
			"+---------+---------+       + 88888      |",	// 12
			"|    .    |   ALT   |                    |",	// 13
			"+---------+---------+                    |",	// 14
			"|    .    |   VEL   |                    |",	// 15
			"+---------+---------+--------------------+",	// 16
			"|                                        |",	// 17
			"|   NOUN  +  7  8  9   CLR   ENTR        |",	// 18
			"|         -  4  5  6   PRO               |",	// 19
			"|   VERB  0  1  2  3  KEYREL RSET        |",	// 20
			"|                                    QUIT|",	// 21
			"+----------------------------------------+",	// 22
			"                                         ",	// 23
	};

	for(h=0; h < DSKY_HEIGHT; h++) {
		wmove(c_win, h, 0);
		waddstr(c_win, dsky_layout[h]);
		fprintf(logfp, "h=%d ->%s<-\n", h,dsky_layout[h]);
	}

	wrefresh(c_win);

	nodelay(c_win, TRUE);
}

void curses_window_end()
{
	delwin(c_win);
}

//
// Returns the coordinates to the keyboard character text
// on the simulated DSKY keyboard
//
static
void curses_key_pos(int8_t key, int *y, int *x)
{
	switch(key)
	{
	case '0':	*y = 20; *x = 10; break;
	case '1':	*y = 20; *x = 13; break;
	case '2':	*y = 20; *x = 16; break;
	case '3':	*y = 20; *x = 19; break;
	case '4':	*y = 19; *x = 13; break;
	case '5':	*y = 19; *x = 16; break;
	case '6':	*y = 19; *x = 19; break;
	case '7':	*y = 18; *x = 13; break;
	case '8':	*y = 18; *x = 16; break;
	case '9':	*y = 18; *x = 19; break;
	case '+':	*y = 18; *x = 10; break;
	case '-':	*y = 19; *x = 10; break;
	case 'k':	*y = 20; *x = 22; break;	// KEY REL
	case 'n':	*y = 18; *x = 4; break;		// NOUN
	case 'v':	*y = 20; *x = 4; break;		// VERB
	case 'p':	*y = 19; *x = 23; break;	// PRO
	case 'c':	*y = 18; *x = 23; break;	// CLR
	case 'r':	*y = 20; *x = 29; break;	// RSET
	case 'e':	*y = 18; *x = 29; break;	// ENTR
	case '\n':	*y = 18; *x = 29; break;	// ENTR
	case 'q':	*y = 21; *x = 37; break;	// QUIT
	default:
		fprintf(logfp, "curses_key_pos Invalid key = %d\n", key);
		break;
	}
}

//
// returns the keyboard text for a keyboard 'key'
//
static
const char *curses_key_text(int8_t key)
{
	switch(key)
	{
	case '0':	return "0";
	case '1':	return "1";
	case '2':	return "2";
	case '3':	return "3";
	case '4':	return "4";
	case '5':	return "5";
	case '6':	return "6";
	case '7':	return "7";
	case '8':	return "8";
	case '9':	return "9";
	case '+':	return "+";
	case '-':	return "-";
	case 'k':	return "KEYREL";
	case 'n':	return "NOUN";
	case 'v':	return "VERB";
	case 'p':	return "PRO";
	case 'c':	return "CLR";
	case 'r':	return "RSET";
	case 'e':	return "ENTR";
	case '\n':	return "ENTR";
	case 'q':	return "QUIT";
	default:
		fprintf(logfp, "curses_key_text Invalid key = %d\n", key);
		return "";
		break;
	}
}

//
// Read a keyboard character. Non-blocking operation.
//
// Returns ERR (-1) if no key is ready.
// Returns only these characters:
//		0, 1, 2, 3, 4, 5, 6, 7, 8, 9
//		+, -
//		e	ENTR
//		v	VERB
//		n	NOUN
//		c	CLR
//		p	PRO
//		k	KEK REL
//		r	RSET
//		q	quit
//
// any typed invalid characters will be ignored.
//
int curses_readkey()
{
	static uint8_t c_key = 0;			// last read key (starts at 0)

	int ch;
	const char *ktext;
	int y, x;

	for(;;) {
		ch = wgetch(c_win);
		if( ch == ERR )
			return ch;

		if( ch == '\n' )
			ch = 'e';

		if( index("0123456789+-evncpkrq", ch) != NULL )
			break;
	}

	fprintf(logfp, "prev key=%d ch=%d\n", c_key, ch);

	if( c_key != 0 ) {
		curses_key_pos(c_key, &y, &x);
		ktext = curses_key_text(c_key);
		wmove(c_win, y, x);
		wattroff(c_win, A_REVERSE);
		waddstr(c_win, ktext);
		fprintf(logfp, "key=%d, y=%d, x=%d ktext=%s\n", c_key, y, x, ktext);
	}

	c_key = ch;
	curses_key_pos(c_key, &y, &x);
	ktext = curses_key_text(c_key);
	wmove(c_win, y, x);
	wattron(c_win, A_REVERSE);
	waddstr(c_win, ktext);
	wattroff(c_win, A_REVERSE);

	wmove(c_win, 0, 0);

	wrefresh(c_win);

	return ch;
}

static
void curses_redraw_digit(int digit, int y, int x)
{
	char ch;

	if( digit >= 0 && digit <= 9 ) {
		ch = '0' + digit;
	} else if( digit == 0xA ) {		// SPACE
		ch = ' ';
	} else if( digit == 0xB ) {		// PLUS
		ch = '+';
	} else if( digit == 0xC ) {		// MINUS
		ch = '-';
	} else {
		ch = '\'';
	}
	wmove(c_win, y, x);
	waddch(c_win, ch);
}

static
void curses_redraw_status(int flag, int y, int x, const char *txt)
{
	if( flag ) {
		wattron(c_win, A_REVERSE);
	}

	wmove(c_win, y, x);
	waddstr(c_win, txt);

	if( flag ) {
		wattroff(c_win, A_REVERSE);
	}
}
#endif

//////////////////////////////////////////////////////////////////////
//
// SECTION 14: DSKY redraw routines (arduino and curses)
//
static
void dsky_init(DSKY *dsky)
{
	dsky->R1[0] = 0xAA;
	dsky->R1[1] = 0xAA;
	dsky->R1[2] = 0xAA;

	dsky->R2[0] = 0xAA;
	dsky->R2[1] = 0xAA;
	dsky->R2[2] = 0xAA;

	dsky->R3[0] = 0xAA;
	dsky->R3[1] = 0xAA;
	dsky->R3[2] = 0xAA;

	dsky->noun = 0xAA;
	dsky->verb = 0xAA;
	dsky->prog = 0xAA;

	dsky->lstatus = 0;
	dsky->rstatus = 0;
	dsky->vnpc = VNPC_VERB | VNPC_NOUN | VNPC_PROG;
	dsky->blink = 0;
}

static
bool dsky_changed(DSKY *curr, DSKY *prev)
{
	return memcmp(curr, prev, sizeof(DSKY)) != 0;
}

#ifdef CURSES_SIMULATOR
#else
//
// 'val' is a BCD sign:
//	0xA			maps to space
//	0xB			maps to plus
//	0xC			maps to minus
//
// Retuns the value needed to be passed to the ledControl to set that LED.
//
static
uint8_t dsky_sign(uint8_t val)
{
	if( val == 0xA )
		return B00000000;	// space
	else if( val == 0xB )
		return B01110100;	// plus sign
	else if( val == 0xC )
		return B00100100;	// minus sign
	else
		return B01000000;	// bogus sign (for debugging)
}

//
// val can be a digit value (0x0 - 0x9) or space (0xA).
//
static
void dsky_digit(uint8_t row, uint8_t col, uint8_t val)
{
	if( val == 0xA )
		ledControl.setRow(row, col, B00000000);
	else
		ledControl.setDigit(row, col, val, false);
	return 0;
}
#endif

#define BitNotEq(curr, prev, bit)		(((curr) & bit) != ((prev) & bit))

//
// Make DSKY display look like 'curr'
// (prev is what is currently showing)
//
static
void dsky_redraw(DSKY *curr, DSKY *prev)
{
	uint8_t showNP;

	// the whole reason for 'curr' and 'prev' is for this optimization
	if( ! dsky_changed(curr, prev) )
		return;

#ifdef CURSES_SIMULATOR
	if( curr->R1[0] != prev->R1[0] || BitNotEq(curr->vnpc, prev->vnpc, VNPC_HIDE_R1) ) {
		if( (curr->vnpc & VNPC_HIDE_R1) != 0 ) {
			curses_redraw_digit(0x0A, 8, 28);
			curses_redraw_digit(0x0A, 8, 30);		
		} else {
			curses_redraw_digit((curr->R1[0] & 0xf0) >> 4, 8, 28);
			curses_redraw_digit((curr->R1[0] & 0x0f) >> 0, 8, 30);
		}
	}

	if( curr->R1[1] != prev->R1[1] || BitNotEq(curr->vnpc, prev->vnpc, VNPC_HIDE_R1) ) {
		if( (curr->vnpc & VNPC_HIDE_R1) != 0 ) {
			curses_redraw_digit(0x0A, 8, 31);
			curses_redraw_digit(0x0A, 8, 32);
		} else {
			curses_redraw_digit((curr->R1[1] & 0xf0) >> 4, 8, 31);
			curses_redraw_digit((curr->R1[1] & 0x0f) >> 0, 8, 32);
		}
	}

	if( curr->R1[2] != prev->R1[2] || BitNotEq(curr->vnpc, prev->vnpc, VNPC_HIDE_R1) ) {
		if( (curr->vnpc & VNPC_HIDE_R1) != 0 ) {
			curses_redraw_digit(0x0A, 8, 33);
			curses_redraw_digit(0x0A, 8, 34);
		} else {
			curses_redraw_digit((curr->R1[2] & 0xf0) >> 4, 8, 33);
			curses_redraw_digit((curr->R1[2] & 0x0f) >> 0, 8, 34);
		}
	}

	if( curr->R2[0] != prev->R2[0] || BitNotEq(curr->rstatus, prev->rstatus, RS_HIDE_R2) ) {
		if( (curr->rstatus & RS_HIDE_R2) != 0 ) {
			curses_redraw_digit(0x0A, 10, 28);
			curses_redraw_digit(0x0A, 10, 30);
		} else {
			curses_redraw_digit((curr->R2[0] & 0xf0) >> 4, 10, 28);
			curses_redraw_digit((curr->R2[0] & 0x0f) >> 0, 10, 30);
		}
	}

	if( curr->R2[1] != prev->R2[1] || BitNotEq(curr->rstatus, prev->rstatus, RS_HIDE_R2) ) {
		if( (curr->rstatus & RS_HIDE_R2) != 0 ) {
			curses_redraw_digit(0x0A, 10, 31);
			curses_redraw_digit(0x0A, 10, 32);
		} else {
			curses_redraw_digit((curr->R2[1] & 0xf0) >> 4, 10, 31);
			curses_redraw_digit((curr->R2[1] & 0x0f) >> 0, 10, 32);
		}
	}

	if( curr->R2[2] != prev->R2[2] || BitNotEq(curr->rstatus, prev->rstatus, RS_HIDE_R2) ) {
		if( (curr->rstatus & RS_HIDE_R2) != 0 ) {
			curses_redraw_digit(0x0A, 10, 33);
			curses_redraw_digit(0x0A, 10, 34);
		} else {
			curses_redraw_digit((curr->R2[2] & 0xf0) >> 4, 10, 33);
			curses_redraw_digit((curr->R2[2] & 0x0f) >> 0, 10, 34);
		}
	}

	if( curr->R3[0] != prev->R3[0] || BitNotEq(curr->lstatus, prev->lstatus, LS_HIDE_R3) ) {
		if( (curr->lstatus & LS_HIDE_R3) != 0 ) {
			curses_redraw_digit(0x0A, 12, 28);
			curses_redraw_digit(0x0A, 12, 30);
		} else {
			curses_redraw_digit((curr->R3[0] & 0xf0) >> 4, 12, 28);
			curses_redraw_digit((curr->R3[0] & 0x0f) >> 0, 12, 30);
		}
	}

	if( curr->R3[1] != prev->R3[1] || BitNotEq(curr->lstatus, prev->lstatus, LS_HIDE_R3) ) {
		if( (curr->lstatus & LS_HIDE_R3) != 0 ) {
			curses_redraw_digit(0x0A, 12, 31);
			curses_redraw_digit(0x0A, 12, 32);
		} else {
			curses_redraw_digit((curr->R3[1] & 0xf0) >> 4, 12, 31);
			curses_redraw_digit((curr->R3[1] & 0x0f) >> 0, 12, 32);
		}
	}

	if( curr->R3[2] != prev->R3[2] || BitNotEq(curr->lstatus, prev->lstatus, LS_HIDE_R3) ) {
		if( (curr->lstatus & LS_HIDE_R3) != 0 ) {
			curses_redraw_digit(0x0A, 12, 33);
			curses_redraw_digit(0x0A, 12, 34);
		} else {
			curses_redraw_digit((curr->R3[2] & 0xf0) >> 4, 12, 33);
			curses_redraw_digit((curr->R3[2] & 0x0f) >> 0, 12, 34);
		}
	}

	if( curr->verb != prev->verb || BitNotEq(curr->vnpc, prev->vnpc, VNPC_HIDE_VERB) ) {
		if( (curr->vnpc & VNPC_HIDE_VERB) != 0 ) {
			curses_redraw_digit(0x0A, 5, 25);
			curses_redraw_digit(0x0A, 5, 26);
		} else {
			curses_redraw_digit((curr->verb & 0xf0) >> 4, 5, 25);
			curses_redraw_digit((curr->verb & 0x0f) >> 0, 5, 26);
		}
	}

	if( curr->noun != prev->noun || BitNotEq(curr->vnpc, prev->vnpc, VNPC_HIDE_NOUN) ) {
		if( (curr->vnpc & VNPC_HIDE_NOUN) != 0 ) {
			curses_redraw_digit(0x0A, 5, 35);
			curses_redraw_digit(0x0A, 5, 36);
		} else {
			curses_redraw_digit((curr->noun & 0xf0) >> 4, 5, 35);
			curses_redraw_digit((curr->noun & 0x0f) >> 0, 5, 36);
		}
	}

	if( curr->prog != prev->prog || BitNotEq(curr->vnpc, prev->vnpc, VNPC_HIDE_PROG) ) {
		if( (curr->vnpc & VNPC_HIDE_PROG) != 0 ) {
			curses_redraw_digit(0x0A, 2, 35);
			curses_redraw_digit(0x0A, 2, 36);
		} else {
			curses_redraw_digit((curr->prog & 0xf0) >> 4, 2, 35);
			curses_redraw_digit((curr->prog & 0x0f) >> 0, 2, 36);
		}
	}

	if( curr->lstatus != prev->lstatus ) {
		curses_redraw_status((curr->lstatus & LS_UPLINK_ACTY ), 1, 2, "UPLINK");
		curses_redraw_status((curr->lstatus & LS_UPLINK_ACTY ), 2, 3, "ACTY");
		curses_redraw_status((curr->lstatus & LS_NO_ATT), 4, 2, "NO ATT");
		curses_redraw_status((curr->lstatus & LS_STBY), 7, 3, "STBY");
		curses_redraw_status((curr->lstatus & LS_KEY_REL), 9, 2, "KEY REL");
		curses_redraw_status((curr->lstatus & LS_OPR_ERR), 11, 2, "OPR ERR");
		curses_redraw_status((curr->lstatus & LS_NA1), 13, 5, ".");
		curses_redraw_status((curr->lstatus & LS_NA2), 15, 5, ".");
	}

	if( curr->rstatus != prev->rstatus ) {
		curses_redraw_status((curr->rstatus & RS_TEMP), 1, 13, "TEMP");
		curses_redraw_status((curr->rstatus & RS_GIMBAL_LOCK), 4, 12, "GIMBAL");
		curses_redraw_status((curr->rstatus & RS_GIMBAL_LOCK), 5, 13, "LOCK");
		curses_redraw_status((curr->rstatus & RS_PROG), 7, 13, "PROG");
		curses_redraw_status((curr->rstatus & RS_RESTART), 9, 12, "RESTART");
		curses_redraw_status((curr->rstatus & RS_TRACKER), 11, 12, "TRACKER");
		curses_redraw_status((curr->rstatus & RS_ALT), 13, 14, "ALT");
		curses_redraw_status((curr->rstatus & RS_VEL), 15, 14, "VEL");
	}

	if( curr->vnpc != prev->vnpc ) {
		curses_redraw_status((curr->vnpc & VNPC_VERB), 4, 24, "VERB");
		curses_redraw_status((curr->vnpc & VNPC_NOUN), 4, 34, "NOUN");
		curses_redraw_status((curr->vnpc & VNPC_PROG), 1, 34, "PROG");
		curses_redraw_status((curr->vnpc & VNPC_COMP_ACTY), 1, 24, "COMP");
		curses_redraw_status((curr->vnpc & VNPC_COMP_ACTY), 2, 24, "ACTY");
	}

	wmove(c_win, 0, 0);
	wrefresh(c_win);
#else
	if( curr->R1[0] != prev->R1[0] || BitNotEq(curr->vnpc, prev->vnpc, VNPC_HIDE_R1) ) {
		if( (curr->vnpc & VNPC_HIDE_R1) != 0 ) {
			ledControl.setRow(1, 0, 0);
			ledControl.setRow(1, 1, 0);
		} else {
			ledControl.setRow(1, 0, dsky_sign((curr->R1[0] & 0xf0) >> 4));
			dsky_digit(1, 1, (curr->R1[0] & 0x0f) >> 0);
		}
	}

	if( curr->R1[1] != prev->R1[1] || BitNotEq(curr->vnpc, prev->vnpc, VNPC_HIDE_R1) ) {
		if( (curr->vnpc & VNPC_HIDE_R1) != 0 ) {
			ledControl.setRow(1, 2, 0);
			ledControl.setRow(1, 3, 0);
		} else {
			dsky_digit(1, 2, (curr->R1[1] & 0xf0) >> 4);
			dsky_digit(1, 3, (curr->R1[1] & 0x0f) >> 0);
		}
	}

	if( curr->R1[2] != prev->R1[2] || BitNotEq(curr->vnpc, prev->vnpc, VNPC_HIDE_R1) ) {
		if( (curr->vnpc & VNPC_HIDE_R1) != 0 ) {
			ledControl.setRow(1, 4, 0);
			ledControl.setRow(1, 5, 0);
		} else {
			dsky_digit(1, 4, (curr->R1[2] & 0xf0) >> 4);
			dsky_digit(1, 5, (curr->R1[2] & 0x0f) >> 0);
		}
	}

	if( curr->R2[0] != prev->R2[0] || BitNotEq(curr->rstatus, prev->rstatus, RS_HIDE_R2) ) {
		if( (curr->rstatus & RS_HIDE_R2) != 0 ) {
			ledControl.setRow(2, 0, 0);
			ledControl.setRow(2, 1, 0);
		} else {
			ledControl.setRow(2, 0, dsky_sign((curr->R2[0] & 0xf0) >> 4));
			dsky_digit(2, 1, (curr->R2[0] & 0x0f) >> 0);
		}
	}

	if( curr->R2[1] != prev->R2[1] || BitNotEq(curr->rstatus, prev->rstatus, RS_HIDE_R2) ) {
		if( (curr->rstatus & RS_HIDE_R2) != 0 ) {
			ledControl.setRow(2, 2, 0);
			ledControl.setRow(2, 3, 0);
		} else {
			dsky_digit(2, 2, (curr->R2[1] & 0xf0) >> 4);
			dsky_digit(2, 3, (curr->R2[1] & 0x0f) >> 0);
		}
	}

	if( curr->R2[2] != prev->R2[2] || BitNotEq(curr->rstatus, prev->rstatus, RS_HIDE_R2) ) {
		if( (curr->rstatus & RS_HIDE_R2) != 0 ) {
			ledControl.setRow(2, 4, 0);
			ledControl.setRow(2, 5, 0);
		} else {
			dsky_digit(2, 4, (curr->R2[2] & 0xf0) >> 4);
			dsky_digit(2, 5, (curr->R2[2] & 0x0f) >> 0);
		}
	}

	if( curr->R3[0] != prev->R3[0] || BitNotEq(curr->lstatus, prev->lstatus, LS_HIDE_R3) ) {
		if( (curr->lstatus & LS_HIDE_R3) != 0 ) {
			ledControl.setRow(3, 0, 0);
			ledControl.setRow(3, 1, 0);
		} else {
			ledControl.setRow(3, 0, dsky_sign((curr->R3[0] & 0xf0) >> 4));
			dsky_digit(3, 1, (curr->R3[0] & 0x0f) >> 0);
		}
	}

	if( curr->R3[1] != prev->R3[1] || BitNotEq(curr->lstatus, prev->lstatus, LS_HIDE_R3) ) {
		if( (curr->lstatus & LS_HIDE_R3) != 0 ) {
			ledControl.setRow(3, 2, 0);
			ledControl.setRow(3, 3, 0);
		} else {
			dsky_digit(3, 2, (curr->R3[1] & 0xf0) >> 4);
			dsky_digit(3, 3, (curr->R3[1] & 0x0f) >> 0);
		}
	}

	if( curr->R3[2] != prev->R3[2] || BitNotEq(curr->lstatus, prev->lstatus, LS_HIDE_R3) ) {
		if( (curr->lstatus & LS_HIDE_R3) != 0 ) {
			ledControl.setRow(3, 4, 0);
			ledControl.setRow(3, 5, 0);
		} else {
			dsky_digit(3, 4, (curr->R3[2] & 0xf0) >> 4);
			dsky_digit(3, 5, (curr->R3[2] & 0x0f) >> 0);
		}
	}

	if( curr->verb != prev->verb || BitNotEq(curr->vnpc, prev->vnpc, VNPC_HIDE_VERB) ) {
		if( (curr->vnpc & VNPC_HIDE_VERB) != 0 ) {
			ledControl.setRow(0, 0, 0);
			ledControl.setRow(0, 1, 0);
		} else {
			dsky_digit(0, 0, (curr->verb & 0xf0) >> 4);
			dsky_digit(0, 1, (curr->verb & 0x0f) >> 0);
		}
	}

	if( curr->noun != prev->noun || BitNotEq(curr->vnpc, prev->vnpc, VNPC_HIDE_NOUN) ) {
		if( (curr->vnpc & VNPC_HIDE_NOUN) != 0 ) {
			ledControl.setRow(0, 4, 0);
			ledControl.setRow(0, 5, 0);
		} else {
			dsky_digit(0, 4, (curr->noun & 0xf0) >> 4);
			dsky_digit(0, 5, (curr->noun & 0x0f) >> 0);
		}
	}

	if( curr->prog != prev->prog || BitNotEq(curr->vnpc, prev->vnpc, VNPC_HIDE_PROG) ) {
		if( (curr->vnpc & VNPC_HIDE_PROG) != 0 ) {
			ledControl.setRow(0, 2, 0);
			ledControl.setRow(0, 3, 0);
		} else {
			dsky_digit(0, 2, (curr->prog & 0xf0) >> 4);
			dsky_digit(0, 3, (curr->prog & 0x0f) >> 0);
		}
	}

	showNP = false;

	if( curr->lstatus != prev->lstatus ) {
		if( curr->lstatus & LS_UPLINK_ACTY )
		    neoPixels.setPixelColor(LAMP_UPLINK_ACTY, NP_COLOR_WHITE);
		else
		    neoPixels.setPixelColor(LAMP_UPLINK_ACTY, NP_COLOR_OFF);

		if( curr->lstatus & LS_NO_ATT )
		    neoPixels.setPixelColor(LAMP_NO_ATT, NP_COLOR_WHITE);
		else
		    neoPixels.setPixelColor(LAMP_NO_ATT, NP_COLOR_OFF);

		if( curr->lstatus & LS_STBY )
		    neoPixels.setPixelColor(LAMP_STBY, NP_COLOR_WHITE);
		else
		    neoPixels.setPixelColor(LAMP_STBY, NP_COLOR_OFF);

		if( curr->lstatus & LS_KEY_REL )
		    neoPixels.setPixelColor(LAMP_KEY_REL, NP_COLOR_WHITE);
		else
		    neoPixels.setPixelColor(LAMP_KEY_REL, NP_COLOR_OFF);

		if( curr->lstatus & LS_OPR_ERR )
		    neoPixels.setPixelColor(LAMP_OPR_ERR, NP_COLOR_WHITE);
		else
		    neoPixels.setPixelColor(LAMP_OPR_ERR, NP_COLOR_OFF);

		if( curr->lstatus & LS_NA1 )
		    neoPixels.setPixelColor(LAMP_NA1, NP_COLOR_WHITE);
		else
		    neoPixels.setPixelColor(LAMP_NA1, NP_COLOR_OFF);

		if( curr->lstatus & LS_NA2 )
		    neoPixels.setPixelColor(LAMP_NA2, NP_COLOR_WHITE);
		else
		    neoPixels.setPixelColor(LAMP_NA2, NP_COLOR_OFF);

		showNP = true;
	}

	if( curr->rstatus != prev->rstatus ) {
		if( curr->rstatus & RS_TEMP )
		    neoPixels.setPixelColor(LAMP_TEMP, NP_COLOR_YELLOW);
		else
		    neoPixels.setPixelColor(LAMP_TEMP, NP_COLOR_OFF);

		if( curr->rstatus & RS_GIMBAL_LOCK )
		    neoPixels.setPixelColor(LAMP_GIMBAL_LOCK, NP_COLOR_YELLOW);
		else
		    neoPixels.setPixelColor(LAMP_GIMBAL_LOCK, NP_COLOR_OFF);

		if( curr->rstatus & RS_PROG )
		    neoPixels.setPixelColor(LAMP_PROG_ALARM, NP_COLOR_YELLOW);
		else
		    neoPixels.setPixelColor(LAMP_PROG_ALARM, NP_COLOR_OFF);

		if( curr->rstatus & RS_RESTART )
		    neoPixels.setPixelColor(LAMP_RESTART, NP_COLOR_YELLOW);
		else
		    neoPixels.setPixelColor(LAMP_RESTART, NP_COLOR_OFF);

		if( curr->rstatus & RS_TRACKER )
		    neoPixels.setPixelColor(LAMP_TRACKER, NP_COLOR_YELLOW);
		else
		    neoPixels.setPixelColor(LAMP_TRACKER, NP_COLOR_OFF);

		if( curr->rstatus & RS_ALT )
		    neoPixels.setPixelColor(LAMP_ALT, NP_COLOR_YELLOW);
		else
		    neoPixels.setPixelColor(LAMP_ALT, NP_COLOR_OFF);

		if( curr->rstatus & RS_VEL )
		    neoPixels.setPixelColor(LAMP_VEL, NP_COLOR_YELLOW);
		else
		    neoPixels.setPixelColor(LAMP_VEL, NP_COLOR_OFF);

		showNP = true;
	}

	if( curr->vnpc != prev->vnpc ) {
		if( curr->vnpc & VNPC_VERB )
		    neoPixels.setPixelColor(LAMP_VERB, NP_COLOR_GREEN);
		else
		    neoPixels.setPixelColor(LAMP_VERB, NP_COLOR_OFF);

		if( curr->vnpc & VNPC_NOUN )
		    neoPixels.setPixelColor(LAMP_NOUN, NP_COLOR_GREEN);
		else
		    neoPixels.setPixelColor(LAMP_NOUN, NP_COLOR_OFF);

		if( curr->vnpc & VNPC_PROG )
		    neoPixels.setPixelColor(LAMP_PROG, NP_COLOR_GREEN);
		else
		    neoPixels.setPixelColor(LAMP_PROG, NP_COLOR_OFF);

		if( curr->vnpc & VNPC_COMP_ACTY )
		    neoPixels.setPixelColor(LAMP_COMP_ACTY, NP_COLOR_GREEN);
		else
		    neoPixels.setPixelColor(LAMP_COMP_ACTY, NP_COLOR_OFF);

		showNP = true;
	}

	if( showNP ) {
		neoPixels.show();
	}
#endif

	*prev = *curr;
}

//////////////////////////////////////////////////////////////////////
//
// SECTION 15: Keyboard reading (arduino and curses)
//
#ifdef CURSES_SIMULATOR
static
int8_t readKeyboard()
{
	return curses_readkey();
}

#else

static
int8_t sampleKeyboard()
{
	int value_row1 = analogRead(A0);
	int value_row2 = analogRead(A1);
	int value_row3 = analogRead(A2);

	if( (value_row1 > ODDROWDIVIDERVOLTAGE6)
			&& (value_row2 > ODDROWDIVIDERVOLTAGE6)
			&& (value_row3 > ODDROWDIVIDERVOLTAGE6))
	{
        return ERR;	// no key pressed
	}

    // keyboard ~top row
	else if( value_row1 < ODDROWDIVIDERVOLTAGE1 ) return 'v';		// VERB
	else if( value_row1 < ODDROWDIVIDERVOLTAGE2 ) return '+';
	else if( value_row1 < ODDROWDIVIDERVOLTAGE3 ) return '7';
	else if( value_row1 < ODDROWDIVIDERVOLTAGE4 ) return '8';
	else if( value_row1 < ODDROWDIVIDERVOLTAGE5 ) return '9';
	else if( value_row1 < ODDROWDIVIDERVOLTAGE6 ) return 'c';		// CLR

	// keyboard ~middle row
	else if( value_row2 < EVENROWDIVIDERVOLTAGE1 ) return 'n';		// NOUN
	else if( value_row2 < EVENROWDIVIDERVOLTAGE2 ) return '-';
	else if( value_row2 < EVENROWDIVIDERVOLTAGE3 ) return '4';
	else if( value_row2 < EVENROWDIVIDERVOLTAGE4 ) return '5';
	else if( value_row2 < EVENROWDIVIDERVOLTAGE5 ) return '6';
	else if( value_row2 < EVENROWDIVIDERVOLTAGE6 ) return 'p';		// PRO
	else if( value_row2 < EVENROWDIVIDERVOLTAGE7 ) return 'e';		// ENTR

	// keyboard ~bottom row
	else if( value_row3 < ODDROWDIVIDERVOLTAGE1 ) return '0';
	else if( value_row3 < ODDROWDIVIDERVOLTAGE2 ) return '1';
	else if( value_row3 < ODDROWDIVIDERVOLTAGE3 ) return '2';
	else if( value_row3 < ODDROWDIVIDERVOLTAGE4 ) return '3';
	else if( value_row3 < ODDROWDIVIDERVOLTAGE5 ) return 'k';		// KEY-REL
	else if( value_row3 < ODDROWDIVIDERVOLTAGE6 ) return 'r';		// RSET
	else {
		return ERR; // no key pressed
	}
}

static
int8_t readKeyboard()
{
	static int8_t s0, s1;

	s0 = sampleKeyboard();
	if( s0 == ERR ) {
		s1 = s0;
		return ERR;
	} else if( s0 == s1 ) {
		return ERR;
	} else {
		delay(10);
		s1 = sampleKeyboard();
		if( s0 == s1 ) {
			return s0;
		} else {
			s0 = s1 = ERR;
			return ERR;
		}
	}
}
#endif

//////////////////////////////////////////////////////////////////////
//
// SECTION 16: IMU Reading routine
//
// IMU https://github.com/griegerc/arduino-gy521/blob/master/gy521-read-angle/gy521-read-angle.ino
//
const int16_t	ACCEL_OFFSET		PROGMEM = 200;
const int16_t	GYRO_OFFSET			PROGMEM = 151;		// 151
const int16_t	GYRO_SENSITITY		PROGMEM = 131;		// 131 is sensivity of gyro from data sheet
//const float		GYRO_SCALE			PROGMEM = 0.2;		//  0.02 by default - tweak as required
const float		GYRO_TEMP_DRIFT		PROGMEM = 0.02;		//  0.02 by default - tweak as required
const int16_t	GYRO_GRANGE			PROGMEM = 2;		// Gforce Range
const int16_t	ACCEL_SCALE			PROGMEM = 16384;	// Scalefactor of Accelerometer
const float		LOOP_TIME			PROGMEM = 0.15;		// 0.1 = 100ms

// change this to your system until gyroCorrX displays 0 if the DSKY sits still
const int16_t	GYRO_OFFSET_X		PROGMEM = 2;

// change this to your system until gyroCorrY displays 0 if the DSKY sits still
const int16_t	GYRO_OFFSET_Y		PROGMEM = 0;

// change this to your system until gyroCorrZ displays 0 if the DSKY sits still
const int16_t	GYRO_OFFSET_Z		PROGMEM = 0;

// change this to your system until accAngleX displays 0 if the DSKY sits still
const int16_t	ACC_OFFSET_X		PROGMEM = 2;

// change this to your system until accAngleY displays 0 if the DSKY sits still
const int16_t	ACC_OFFSET_Y		PROGMEM = 3;

// change this to your system until accAngleZ displays 0 if the DSKY sits still
const int16_t	ACC_OFFSET_Z		PROGMEM = 0;

static
void readIMU(uint8_t addr)
{
	int16_t accValueX;
	int16_t accValueY;
	int16_t accValueZ;
	int16_t gyroValueX;
	int16_t gyroValueY;
	int16_t gyroValueZ;
	int16_t temp;

	int16_t accCorrX;
	int16_t accCorrY;
	int16_t accCorrZ;

	float accAngleX;
	float accAngleY;
	float accAngleZ;

	float gyroAngleX;
	float gyroAngleY;
	float gyroAngleZ; 
	float gyroCorrX;
	float gyroCorrY;
	float gyroCorrZ;

	Wire.beginTransmission(MPU_ADDR);
	Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
	Wire.endTransmission(false);
	Wire.requestFrom(MPU_ADDR, 14, true);  // request a total of 14 registers
  
	accValueX =  (Wire.read() << 8) | Wire.read();	// 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
	accValueY =  (Wire.read() << 8) | Wire.read();	// 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
	accValueZ =  (Wire.read() << 8) | Wire.read();	// 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
	temp =       (Wire.read() << 8) | Wire.read();	// 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
	gyroValueX = (Wire.read() << 8) | Wire.read();	// 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
	gyroValueY = (Wire.read() << 8) | Wire.read();	// 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
	gyroValueZ = (Wire.read() << 8) | Wire.read();	// 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

	temp = (temp / 340.00 + 36.53);		// equation for temperature in degrees C from datasheet

	accCorrX = accValueX - ACCEL_OFFSET;
	accCorrX = map(accCorrX, -ACCEL_SCALE, ACCEL_SCALE, -90, 90);
	accAngleX = constrain(accCorrX, -90, 90);

	// our IMU sits upside down in the DSKY, so we have to flip the angle
	accAngleX = -accAngleX;
	accAngleX = accAngleX + ACC_OFFSET_X;

	accCorrY = accValueY - ACCEL_OFFSET;
	accCorrY = map(accCorrY, -ACCEL_SCALE, ACCEL_SCALE, -90, 90);

	accAngleY = constrain(accCorrY, -90, 90);
	accAngleY = accAngleY + ACC_OFFSET_Y;

	accCorrZ = accValueZ - ACCEL_OFFSET;
	accCorrZ = map(accCorrZ, -ACCEL_SCALE, ACCEL_SCALE, -90, 90);
	accAngleZ = constrain(accCorrZ, -90, 90);

	// our IMU sits upside down in the DSKY, so we have to flip the angle
	accAngleZ = -accAngleZ;
	accAngleZ = accAngleZ + ACC_OFFSET_Z;

	gyroCorrX = ((float)gyroValueX / GYRO_SENSITITY) + GYRO_OFFSET_X;
	gyroAngleX = (gyroCorrX * GYRO_GRANGE) * -LOOP_TIME;
	gyroCorrY = ((float)gyroValueY / GYRO_SENSITITY) + GYRO_OFFSET_Y;
	gyroAngleY = (gyroCorrY * GYRO_GRANGE) * -LOOP_TIME;
	gyroCorrZ = ((float)gyroValueZ / GYRO_SENSITITY) + GYRO_OFFSET_Z;
	gyroAngleZ = (gyroCorrZ * GYRO_GRANGE) * -LOOP_TIME;

	Agc.RAM[addr+0] = (int16_t)(gyroAngleX * 100);		// IMU_GYROX
	Agc.RAM[addr+1] = (int16_t)(gyroAngleY * 100);		// IMU_GYROY
	Agc.RAM[addr+2] = (int16_t)(gyroAngleZ * 100);		// IMU_GYROZ
	Agc.RAM[addr+3] = (int16_t)(accAngleX * 100);		// IMU_ACCX
	Agc.RAM[addr+4] = (int16_t)(accAngleY * 100);		// IMU_ACCY
	Agc.RAM[addr+5] = (int16_t)(accAngleZ * 100);		// IMU_ACCZ
	Agc.RAM[addr+6] = temp;								// IMU_TEMP
}

//////////////////////////////////////////////////////////////////////
//
// SECTION 17: Apollo Guidance Computer routines
//
static
void agc_cpu_init(int c, CPU *cpu)
{
	memset(cpu, 0, sizeof(CPU));
	cpu->SP = (c==0) ? CPU0_STACK : CPU1_STACK;		// each cpu has uses a different area in RAM for their stack
	cpu->PC = (c==0) ? LBL_MAIN_CPU0 : LBL_MAIN_CPU1;
}

static
void agc_init()
{
	memset(&Agc.prev, 0xff, sizeof(DSKY));		// make it look different so complete repaint occurs
	dsky_init( &Agc.dsky );
	agc_cpu_init(0, &Agc.cpu[0] );
	agc_cpu_init(1, &Agc.cpu[1] );
	Agc.state = 0;
	Agc.flags = 0;
	Agc.dsky.blink = 0;
	Agc.uplink_prob = 0;
	Agc.compacty_prob = 0;
}

static
bool do_cmp(int op, int32_t lhs, int32_t rhs)
{
	switch(op)
	{
	case CMP_GT:	return lhs > rhs;
	case CMP_GE:	return lhs >= rhs;
	case CMP_LT:	return lhs < rhs;
	case CMP_LE:	return lhs <= rhs;
	case CMP_EQ:	return lhs == rhs;
	case CMP_NE:	return lhs != rhs;
	}
	return false;
}

static
int32_t decode_from_bcd_dec(int32_t val)
{
	int32_t res;
	int32_t mask;
	uint8_t sign;
	uint8_t digit;
	int8_t i;

	sign = (val & 0x00f00000) >> 20;
	res = 0;
	for(i=16; i >= 0; i -= 4)
	{
		mask = (uint32_t)0x0f << i;
		digit = (val & mask) >> i;
		if( digit == 0xA )
			continue;
		res = res * 10 + digit;
	}

	if( sign == 0xc )
		res = -res;

	return res;
}

//
// 'val' is assumed to contain an octal value BCD encoded.
// I.e., 0x00001770
//
static
int32_t decode_from_bcd_oct(int32_t val)
{
	int32_t res;
	int32_t mask;
	uint8_t digit;
	int8_t i;

	res = 0;
	for(i=16; i >= 0; i -= 4)
	{
		mask = (uint32_t)0x0f << i;
		digit = (val & mask) >> i;
		if( digit == 0xA )
			continue;
		res = res * 8 + digit;
	}

	return res;
}

//////////////////////////////////////////////////////////////////////
//
// date/time diff logic
//
// https://stackoverflow.com/questions/13932909/difference-between-two-dates-in-c

#define IsLeapG(yr)		((yr%4)==0)

static
uint16_t Godn(uint8_t yy1, uint8_t yy2)
{
	uint16_t i, bb;

	bb = 0;
	for(i = yy1; i < yy2; i++) {
		bb += 365;
		if( IsLeapG(i) ) bb += 1;
	}
	return bb;
}

// Day of the Year
static
uint16_t rbdug(uint8_t d, uint8_t m, uint8_t y)
{
	uint16_t a;

	a = accdbmAcc(m-1) + d;

	if( IsLeapG(y) && m > 2 )
		a += 1;
	return a;
}

//
// date2 - date1
//
static
int16_t DatDif(uint8_t d1, uint8_t m1, uint8_t y1, uint8_t d2, uint8_t m2, uint8_t y2)
{
	int16_t suma;

	suma = rbdug(d2, m2, y2) - rbdug(d1, m1, y1);

	if( y1 != y2 ) {
		if( y1 < y2 ) {
			suma += Godn(y1, y2);
		} else {
			suma -= Godn(y2, y1);
		}
	}
	return suma;
}

//
//	diff two timestamps: addr2 - addr1
//
// addr1 points to a timestamp in RAM
// addr2 points to a timestamp in RAM
//
//	addr1 + 0:	00 YY MM DD
//	addr1 + 1:	00 HH MM SS
//
// Return a BCD encoded duration, which represents
//	the difference between timestamp1 and timestamp2.
//
// Duration is encoded as:
//		sH HH MM SS			's' is 0xB (+) or 0xC (-)
//
static
uint32_t timestamp_diff(uint8_t addr1, uint8_t addr2)
{
	uint32_t dt1, dt2;
	uint32_t tm1, tm2;

	uint8_t y1, y2;		// year field
	uint8_t m1, m2;		// month field and minutes field
	uint8_t d1, d2;		// day field
	uint8_t h1, h2;		// hours field
	uint8_t s1, s2;		// seconds field

	int32_t totsec1, totsec2, diff;
	int16_t days;
	uint16_t hours;
	uint8_t minutes;
	uint8_t seconds;
	bool neg;
	uint32_t result;

	dt1 = Agc.RAM[addr1];
	dt2 = Agc.RAM[addr2];

	d1 = ((dt1 & 0xf0) >> 4) * 10 + ((dt1 & 0x0f) >> 0);
	d2 = ((dt2 & 0xf0) >> 4) * 10 + ((dt2 & 0x0f) >> 0);

	m1 = ((dt1 & 0xf000) >> 12) * 10 + ((dt1 & 0x0f00) >> 8);
	m2 = ((dt2 & 0xf000) >> 12) * 10 + ((dt2 & 0x0f00) >> 8);

	y1 = ((dt1 & 0xf00000) >> 20) * 10 + ((dt1 & 0x0f0000) >> 16);
	y2 = ((dt2 & 0xf00000) >> 20) * 10 + ((dt2 & 0x0f0000) >> 16);

	days = DatDif(d1, m1, y1, d2, m2, y2);
	if( days >= 0 )
	{
		d1 = 0;
		d2 = days;
	} else {
		d1 = -days;
		d2 = 0;
	}

	tm1 = Agc.RAM[addr1+1];
	tm2 = Agc.RAM[addr2+1];

	h1 = ((tm1 & 0xf00000) >> 20) * 10 + ((tm1 & 0x0f0000) >> 16);
	h2 = ((tm2 & 0xf00000) >> 20) * 10 + ((tm2 & 0x0f0000) >> 16);

	m1 = ((tm1 & 0xf000) >> 12) * 10 + ((tm1 & 0x0f00) >> 8);
	m2 = ((tm2 & 0xf000) >> 12) * 10 + ((tm2 & 0x0f00) >> 8);

	s1 = ((tm1 & 0xf0) >> 4) * 10 + ((tm1 & 0x0f) >> 0);
	s2 = ((tm2 & 0xf0) >> 4) * 10 + ((tm2 & 0x0f) >> 0);

	totsec1 = (uint32_t)d1*24*3600 + (uint32_t)h1*3600 + m1*60 + s1;
	totsec2 = (uint32_t)d2*24*3600 + (uint32_t)h2*3600 + m2*60 + s2;

	diff = totsec2 - totsec1;

	neg = false;
	if( diff < 0 )
	{
		diff = -diff;
		neg = true;
	}

	hours = diff / 3600;
	diff = diff % 3600;
	minutes = diff / 60;
	diff = diff % 60;
	seconds = diff;

	result = (uint32_t)(hours / 100) << 24;
	hours = hours % 100;
	result |= (uint32_t)(hours / 10) << 20;
	result |= (uint32_t)(hours % 10) << 16;

	result |= (uint16_t)(minutes / 10) << 12;
	result |= (uint16_t)(minutes % 10) << 8;

	result |= (seconds / 10) << 4;
	result |= (seconds % 10) << 0;

	if( neg ) {
		result |= 0xC0000000;
	} else {
		result |= 0xB0000000;
	}

#ifdef DSKY_DEBUG
	// generates a lot of log messages. change 'false' to 'true' to enable.
	if( false ) {
		fprintf(logfp, "dt1=%08X tm1=%08X - dt2=%08X tm2=%08X result=%08X\n",
			dt1, tm1, dt2, tm2, result);
	}
#endif

	return result;
}

//
// The John F. Kennedy routine!
// Play audio track 'jfk'
//
static
void jfk(uint8_t jfk)
{
	while( audioTrack != jfk ) {
		pinMode(9, OUTPUT);
		delay(100);
		pinMode(9, INPUT);
		delay(100);
		audioTrack++;
		if( audioTrack > TRACKDB_LEN ) {
            audioTrack = 1;
        }
    }

#ifdef CURSES_SIMULATOR
	mp3_play(audioTrack);
#endif

	pinMode(9, OUTPUT);
	delay(100);
	pinMode(9, INPUT);
	audioTrack++;
	if( audioTrack > TRACKDB_LEN ) {
		audioTrack = 1;
    }
}

//
// execute cpu 'c' for one instruction.
// c is 0 or 1.
//	0 - is the background task (major PROG mode)
//	1 - is the foreground task
//
static
void agc_execute_cpu(uint8_t c)
{
	CPU *cpu;
	uint8_t instruction;
	uint8_t reg1, reg2;
	uint8_t *rx;
	int8_t offset;
	uint8_t op;
	int32_t tmp;
	int32_t res;
	bool cmp;
	int8_t imm8;
	int16_t imm16;
	uint8_t unhide;
#ifdef DSKY_DEBUG
	uint16_t prevPC;
#endif

	cpu = &Agc.cpu[c];

#ifdef DSKY_DEBUG
	prevPC = cpu->PC;
#endif

	instruction = ProgramAccU8(cpu->PC++);

	switch(instruction)
	{
	case MOV_R1_A:
		rx = Agc.dsky.R1; reg2 = 0; goto mov_rx_reg2;

	case MOV_R2_A:
		rx = Agc.dsky.R2; reg2 = 0; goto mov_rx_reg2;

	case MOV_R3_A:
		rx = Agc.dsky.R3; reg2 = 0; goto mov_rx_reg2;

	case MOV_A_R1:
		reg1 = 0; rx = Agc.dsky.R1; goto mov_reg1_rx;

	case MOV_A_R2:
		reg1 = 0; rx = Agc.dsky.R2; goto mov_reg1_rx;

	case MOV_A_R3:
		reg1 = 0; rx = Agc.dsky.R3; goto mov_reg1_rx;

	case DECODE_A_FROM_OCT:
		cpu->regs[0] = decode_from_bcd_oct(cpu->regs[0]);
		break;

	case DECODE_A_FROM_DEC:
		cpu->regs[0] = decode_from_bcd_dec(cpu->regs[0]);
		break;

	case ENCODE_A_TO_OCT:
		tmp = cpu->regs[0];
		res = 0;
		res |= (tmp % 8);
		tmp /= 8;
		res |= (tmp % 8) << 4;
		tmp /= 8;
		res |= (tmp % 8) << 8;
		tmp /= 8;
		res |= (tmp % 8) << 12;
		tmp /= 8;
		res |= (tmp % 8) << 16;
		res |= (uint32_t)0x0a << 20;
		cpu->regs[0] = res;
		break;

	case ENCODE_A_TO_DEC:
		imm8 = 0x0B; goto encode_a_to_decimal;

	case ENCODE_A_TO_UDEC:
		imm8 = 0x0A; goto encode_a_to_decimal;

	case MOV_A_B:
		reg1 = 0; reg2 = 1; goto mov_reg1_reg2;

	case MOV_A_C:
		reg1 = 0; reg2 = 2; goto mov_reg1_reg2;

	case MOV_B_A:
		reg1 = 1; reg2 = 0; goto mov_reg1_reg2;

	case MOV_B_C:
		reg1 = 1; reg2 = 2; goto mov_reg1_reg2;

	case MOV_C_A:
		reg1 = 2; reg2 = 0; goto mov_reg1_reg2;

	case MOV_C_B:
		reg1 = 2; reg2 = 1; goto mov_reg1_reg2;

	case LD_A_DIRECT:
		reg1 = 0; goto ld_reg1_direct;

	case LD_B_DIRECT:
		reg1 = 1; goto ld_reg1_direct;

	case LD_C_DIRECT:
		reg1 = 2; goto ld_reg1_direct;

	case LD_A_IMM32:
		reg1 = 0; goto ld_reg1_imm32;

	case LD_B_IMM32:
		reg1 = 1; goto ld_reg1_imm32;

	case LD_C_IMM32:
		reg1 = 2; goto ld_reg1_imm32;

	case LD_A_IMM16:
		reg1 = 0; goto ld_reg1_imm16;

	case LD_B_IMM16:
		reg1 = 1; goto ld_reg1_imm16;

	case LD_C_IMM16:
		reg1 = 2; goto ld_reg1_imm16;

	case LD_A_IMM8:
		reg1 = 0; goto ld_reg1_imm8;

	case LD_B_IMM8:
		reg1 = 1; goto ld_reg1_imm8;

	case LD_C_IMM8:
		reg1 = 2; goto ld_reg1_imm8;

	case LD_A_CINDIRECT:
		reg1 = 0; goto ld_reg1_indirect_c;

	case LD_B_CINDIRECT:
		reg1 = 1; goto ld_reg1_indirect_c;

	case ST_A_DIRECT:
		reg1 = 0; goto st_reg1_direct;

	case ST_B_DIRECT:
		reg1 = 1; goto st_reg1_direct;

	case ST_C_DIRECT:
		reg1 = 2; goto st_reg1_direct;

	case ST_A_CINDIRECT:
		reg1 = 0; goto st_reg1_indirect_c;

	case ST_B_CINDIRECT:
		reg1 = 1; goto st_reg1_indirect_c;

	case CLR_A:
		reg1 = 0; goto clr_reg1;

	case CLR_B:
		reg1 = 1; goto clr_reg1;

	case CLR_C:
		reg1 = 2; goto clr_reg1;

	case INC_A:
		reg1 = 0; goto inc_reg1;

	case INC_B:
		reg1 = 1; goto inc_reg1;

	case INC_C:
		reg1 = 2; goto inc_reg1;

	case DEC_A:
		reg1 = 0; goto dec_reg1;

	case DEC_B:
		reg1 = 1; goto dec_reg1;

	case DEC_C:
		reg1 = 2; goto dec_reg1;

	case PUSH_A:
		reg1 = 0; goto push_reg1;

	case PUSH_B:
		reg1 = 1; goto push_reg1;

	case PUSH_C:
		reg1 = 2; goto push_reg1;

	case POP_A:
		reg1 = 0; goto pop_reg1;

	case POP_B:
		reg1 = 1; goto pop_reg1;

	case POP_C:
		reg1 = 2; goto pop_reg1;

	case CALL:
		tmp = ProgramAcc16(cpu->PC);
		cpu->PC += 2;
		Agc.RAM[cpu->SP--] = cpu->PC;
		cpu->PC = tmp;
		break;

	case RET:
		tmp = Agc.RAM[++cpu->SP];
		if( (tmp & 0xffff0000) != 0 ) {
			Agc.dsky.verb = (tmp >> 24) & 0xff;
			Agc.dsky.noun = (tmp >> 16) & 0xff;
		}
		cpu->PC = (tmp & 0xffff);
		break;

	case GOTO:
		cpu->PC = ProgramAcc16(cpu->PC);
		break;

	case BRANCH:
		offset = ProgramAcc8(cpu->PC++);
		cpu->PC += offset;
		break;

	case BRANCH_A_GT_B:
		reg1 = 0; reg2 = 1; op = CMP_GT; goto branch_reg1_op_reg2;

	case BRANCH_A_GE_B:
		reg1 = 0; reg2 = 1; op = CMP_GE; goto branch_reg1_op_reg2;

	case BRANCH_A_LE_B:
		reg1 = 0; reg2 = 1; op = CMP_LE; goto branch_reg1_op_reg2;

	case BRANCH_A_LT_B:
		reg1 = 0; reg2 = 1; op = CMP_LT; goto branch_reg1_op_reg2;

	case BRANCH_A_EQ_B:
		reg1 = 0; reg2 = 1; op = CMP_EQ; goto branch_reg1_op_reg2;

	case BRANCH_A_NE_B:
		reg1 = 0; reg2 = 1; op = CMP_NE; goto branch_reg1_op_reg2;

	case BRANCH_A_GT_DIRECT:
		reg1 = 0; op = CMP_GT; goto branch_reg1_op_direct;

	case BRANCH_A_GE_DIRECT:
		reg1 = 0; op = CMP_GE; goto branch_reg1_op_direct;

	case BRANCH_A_LE_DIRECT:
		reg1 = 0; op = CMP_LE; goto branch_reg1_op_direct;

	case BRANCH_A_LT_DIRECT:
		reg1 = 0; op = CMP_LT; goto branch_reg1_op_direct;

	case BRANCH_A_EQ_DIRECT:
		reg1 = 0; op = CMP_EQ; goto branch_reg1_op_direct;

	case BRANCH_A_NE_DIRECT:
		reg1 = 0; op = CMP_NE; goto branch_reg1_op_direct;

	case BRANCH_A_GT_IMM8:
		reg1 = 0; op = CMP_GT; goto branch_reg1_op_imm8;

	case BRANCH_A_GE_IMM8:
		reg1 = 0; op = CMP_GE; goto branch_reg1_op_imm8;

	case BRANCH_A_LE_IMM8:
		reg1 = 0; op = CMP_LE; goto branch_reg1_op_imm8;

	case BRANCH_A_LT_IMM8:
		reg1 = 0; op = CMP_LT; goto branch_reg1_op_imm8;

	case BRANCH_A_EQ_IMM8:
		reg1 = 0; op = CMP_EQ; goto branch_reg1_op_imm8;

	case BRANCH_A_NE_IMM8:
		reg1 = 0; op = CMP_NE; goto branch_reg1_op_imm8;

	case BRANCH_A_GT_IMM16:
		reg1 = 0; op = CMP_GT; goto branch_reg1_op_imm16;

	case BRANCH_A_GE_IMM16:
		reg1 = 0; op = CMP_GE; goto branch_reg1_op_imm16;

	case BRANCH_A_LE_IMM16:
		reg1 = 0; op = CMP_LE; goto branch_reg1_op_imm16;

	case BRANCH_A_LT_IMM16:
		reg1 = 0; op = CMP_LT; goto branch_reg1_op_imm16;

	case BRANCH_A_EQ_IMM16:
		reg1 = 0; op = CMP_EQ; goto branch_reg1_op_imm16;

	case BRANCH_A_NE_IMM16:
		reg1 = 0; op = CMP_NE; goto branch_reg1_op_imm16;

	case BRANCH_B_GT_DIRECT:
		reg1 = 1; op = CMP_GT; goto branch_reg1_op_direct;

	case BRANCH_B_GE_DIRECT:
		reg1 = 1; op = CMP_GE; goto branch_reg1_op_direct;

	case BRANCH_B_LE_DIRECT:
		reg1 = 1; op = CMP_LE; goto branch_reg1_op_direct;

	case BRANCH_B_LT_DIRECT:
		reg1 = 1; op = CMP_LT; goto branch_reg1_op_direct;

	case BRANCH_B_EQ_DIRECT:
		reg1 = 1; op = CMP_EQ; goto branch_reg1_op_direct;

	case BRANCH_B_NE_DIRECT:
		reg1 = 1; op = CMP_NE; goto branch_reg1_op_direct;

	case BRANCH_B_GT_IMM8:
		reg1 = 1; op = CMP_GT; goto branch_reg1_op_imm8;

	case BRANCH_B_GE_IMM8:
		reg1 = 1; op = CMP_GE; goto branch_reg1_op_imm8;

	case BRANCH_B_LE_IMM8:
		reg1 = 1; op = CMP_LE; goto branch_reg1_op_imm8;

	case BRANCH_B_LT_IMM8:
		reg1 = 1; op = CMP_LT; goto branch_reg1_op_imm8;

	case BRANCH_B_EQ_IMM8:
		reg1 = 1; op = CMP_EQ; goto branch_reg1_op_imm8;

	case BRANCH_B_NE_IMM8:
		reg1 = 1; op = CMP_NE; goto branch_reg1_op_imm8;

	case BRANCH_B_GT_IMM16:
		reg1 = 1; op = CMP_GT; goto branch_reg1_op_imm16;

	case BRANCH_B_GE_IMM16:
		reg1 = 1; op = CMP_GE; goto branch_reg1_op_imm16;

	case BRANCH_B_LE_IMM16:
		reg1 = 1; op = CMP_LE; goto branch_reg1_op_imm16;

	case BRANCH_B_LT_IMM16:
		reg1 = 1; op = CMP_LT; goto branch_reg1_op_imm16;

	case BRANCH_B_EQ_IMM16:
		reg1 = 1; op = CMP_EQ; goto branch_reg1_op_imm16;

	case BRANCH_B_NE_IMM16:
		reg1 = 1; op = CMP_NE; goto branch_reg1_op_imm16;

	case BRANCH_NOT_TIMER1:
		op = CPU_TIMER1; goto branch_not_timer_op;

	case BRANCH_NOT_TIMER2:
		op = CPU_TIMER2; goto branch_not_timer_op;

	case BRANCH_NOT_TIMER3:
		op = CPU_TIMER3; goto branch_not_timer_op;

	case BRANCH_NOT_TIMER4:
		op = CPU_TIMER4; goto branch_not_timer_op;

	case BRANCH_NOT_TIMER5:
		op = CPU_TIMER5; goto branch_not_timer_op;

	case SWAP_A_B:
		reg1 = 0; reg2 = 1; goto swap_reg1_reg2;

	case SWAP_A_C:
		reg1 = 0; reg2 = 2; goto swap_reg1_reg2;

	case SWAP_B_C:
		reg1 = 1; reg2 = 2; goto swap_reg1_reg2;

	case ADD_A_B:
		reg1 = 0; reg2 = 1; goto add_reg1_reg2;

	case ADD_B_A:
		reg1 = 1; reg2 = 0; goto add_reg1_reg2;

	case SUB_A_B:
		reg1 = 0; reg2 = 1; goto sub_reg1_reg2;

	case SUB_B_A:
		reg1 = 1; reg2 = 0; goto sub_reg1_reg2;

	case MUL_A_B:
		reg1 = 0; reg2 = 1; goto mul_reg1_reg2;

	case MUL_B_A:
		reg1 = 1; reg2 = 0; goto mul_reg1_reg2;

	case DIV_A_B:
		reg1 = 0; reg2 = 1; goto div_reg1_reg2;

	case DIV_B_A:
		reg1 = 1; reg2 = 0; goto div_reg1_reg2;

	case MOD_A_B:
		reg1 = 0; reg2 = 1; goto mod_reg1_reg2;

	case MOD_B_A:
		reg1 = 1; reg2 = 0; goto mod_reg1_reg2;

	case AND_A_B:
		reg1 = 0; reg2 = 1; goto and_reg1_reg2;

	case AND_B_A:
		reg1 = 1; reg2 = 0; goto and_reg1_reg2;

	case OR_A_B:
		reg1 = 0; reg2 = 1; goto or_reg1_reg2;

	case OR_B_A:
		reg1 = 1; reg2 = 0; goto or_reg1_reg2;

	case OR_A_IMM32:
		reg1 = 0; goto or_reg1_imm32;

	case OR_B_IMM32:
		reg1 = 1; goto or_reg1_imm32;

	case AND_A_IMM32:
		reg1 = 0; goto and_reg1_imm32;

	case AND_B_IMM32:
		reg1 = 1; goto and_reg1_imm32;

	case LSHIFT_A_IMM8:
		reg1 = 0; goto lshift_reg1;

	case LSHIFT_B_IMM8:
		reg1 = 1; goto lshift_reg1;

	case RSHIFT_A_IMM8:
		reg1 = 0; goto rshift_reg1;

	case RSHIFT_B_IMM8:
		reg1 = 1; goto rshift_reg1;

	case NOT_A:
		reg1 = 0; goto not_reg1;

	case NOT_B:
		reg1 = 1; goto not_reg1;

	case NEG_A:
		reg1 = 0; goto neg_reg1;

	case NEG_B:
		reg1 = 1; goto neg_reg1;

	case MOV_A_VERB:
		Agc.dsky.verb = cpu->regs[0];
		break;

	case MOV_A_NOUN:
		Agc.dsky.noun = cpu->regs[0];
		break;

	case MOV_A_PROG:
		Agc.dsky.prog = cpu->regs[0];
		break;

	case MOV_NOUN_A:
		cpu->regs[0] = Agc.dsky.noun;
		break;

	case MOV_VERB_A:
		cpu->regs[0] = Agc.dsky.verb;
		break;

	case MOV_PROG_A:
		cpu->regs[0] = Agc.dsky.prog;
		break;

	case BLINK_VERB:
		op = BLINKF_VERB; unhide = UNHIDE_VERB; goto blink_op;

	case BLINK_NOUN:
		op = BLINKF_NOUN; unhide = UNHIDE_NOUN; goto blink_op;

	case BLINK_PROG:
		op = BLINKF_PROG; unhide = UNHIDE_PROG; goto blink_op;

	case BLINK_KEYREL:
		op = BLINKF_KEYREL; unhide = UNHIDE_KEYREL; goto blink_op;

	case BLINK_OPRERR:
		op = BLINKF_OPRERR; unhide = UNHIDE_OPRERR; goto blink_op;

	case BLINK_R1:
		op = BLINKF_R1; unhide = UNHIDE_R1; goto blink_op;

	case BLINK_R2:
		op = BLINKF_R2; unhide = UNHIDE_R2; goto blink_op;

	case BLINK_R3:
		op = BLINKF_R3; unhide = UNHIDE_R3; goto blink_op;

	case LT_UPLINK_ACTY:
		op = LS_UPLINK_ACTY; goto light_lstatus_op;

	case LT_NO_ATT:
		op = LS_NO_ATT; goto light_lstatus_op;

	case LT_STBY:
		op = LS_STBY; goto light_lstatus_op;

	case LT_KEY_REL:
		op = LS_KEY_REL; goto light_lstatus_op;

	case LT_OPR_ERR:
		op = LS_OPR_ERR; goto light_lstatus_op;

	case LT_NA1:
		op = LS_NA1; goto light_lstatus_op;

	case LT_NA2:
		op = LS_NA2; goto light_lstatus_op;

	case LT_TEMP:
		op = RS_TEMP; goto light_rstatus_op;

	case LT_GIMBAL_LOCK:
		op = RS_GIMBAL_LOCK; goto light_rstatus_op;

	case LT_PROG_ALRM:
		op = RS_PROG; goto light_rstatus_op;

	case LT_RESTART:
		op = RS_RESTART; goto light_rstatus_op;

	case LT_TRACKER:
		op = RS_TRACKER; goto light_rstatus_op;

	case LT_ALT:
		op = RS_ALT; goto light_rstatus_op;

	case LT_VEL:
		op = RS_VEL; goto light_rstatus_op;

	case LT_COMP_ACTY:
		op = VNPC_COMP_ACTY; goto light_vnpc_op;

	case LT_VERB:
		op = VNPC_VERB; goto light_vnpc_op;

	case LT_NOUN:
		op = VNPC_NOUN; goto light_vnpc_op;

	case LT_PROG:
		op = VNPC_PROG; goto light_vnpc_op;

	case LT_ALL:
		imm8 = ProgramAcc8(cpu->PC++);
		if( imm8 ) {
			Agc.dsky.vnpc = 0xff;
			Agc.dsky.lstatus = 0xff;
			Agc.dsky.rstatus = 0xff;
		} else {
			Agc.dsky.vnpc = 0x00;
			Agc.dsky.lstatus = 0x00;
			Agc.dsky.rstatus = 0x00;
		}
		break;

	case UPLINK_PROB_IMM8:
		Agc.uplink_prob = ProgramAccU8(cpu->PC++);
		break;

	case COMPACTY_PROB_IMM8:
		Agc.compacty_prob = ProgramAccU8(cpu->PC++);
		break;

	case GPS_LAT_A:
		break;

	case GPS_LON_A:
		break;

	case GPS_YEAR_A:
		break;

	case GPS_MON_A:
		break;

	case GPS_DAY_A:
		break;

	case GPS_HH_A:
		break;

	case GPS_MM_A:
		break;

	case GPS_SS_A:
		break;

	case BRANCH_TIMESTAMP_LT:
		reg1 = ProgramAccU8(cpu->PC++);
		reg2 = ProgramAccU8(cpu->PC++);
		if( Agc.RAM[reg1+0] < Agc.RAM[reg2+0] ) {
			cpu->PC += ProgramAcc8(cpu->PC++);
		} else if( Agc.RAM[reg1+1] < Agc.RAM[reg2+1] ) {
			cpu->PC += ProgramAcc8(cpu->PC++);
		} else {
			cpu->PC++;
		}
		break;

	case TIMESTAMP_DIFF_A:
		reg1 = ProgramAccU8(cpu->PC++);
		reg2 = ProgramAccU8(cpu->PC++);
		cpu->regs[0] = timestamp_diff(reg1, reg2);
		break;

	case RTC_TIMESTAMP_DIRECT:
		op = ProgramAcc8(cpu->PC++);

		Wire.beginTransmission(RTC_ADDR);
		Wire.write(0x00);
		Wire.endTransmission(true);
		Wire.requestFrom(RTC_ADDR, 7 , true);
		tmp = Wire.read() & 0x7f;				// ss (exclude CH bit)
		tmp |= (uint16_t)Wire.read() << 8;		// mm
		tmp |= (uint32_t)Wire.read() << 16;		// hh
		Agc.RAM[op+1] = tmp;

		Wire.read();							// dow
		tmp = Wire.read();						// day
		tmp |= (uint16_t)Wire.read() << 8;		// month
		tmp |= (uint32_t)Wire.read() << 16;		// year
		Agc.RAM[op+0] = tmp;
		break;

	case RTC_DAY_A:
		op = 0x04; goto rtc_op_a;

	case RTC_YEAR_A:
		op = 0x06; goto rtc_op_a;

	case RTC_MON_A:
		op = 0x05; goto rtc_op_a;

	case RTC_HH_A:
		op = 0x02; goto rtc_op_a;

	case RTC_MM_A:
		op = 0x01; goto rtc_op_a;

	case RTC_SS_A:
		op = 0x00; goto rtc_op_a;

	case RTC_MEM_A_CINDIRECT:
		break;

	case RTC_A_MEM_CINDIRECT:
		Wire.beginTransmission(RTC_ADDR);
		Wire.write(cpu->regs[2] & 0xff);		// RTC address in C
		Wire.write(cpu->regs[0] & 0xff);		// write byte in A to RTC address
		Wire.endTransmission(true);
		break;

	case IMU_READ_DIRECT:
		readIMU(ProgramAccU8(cpu->PC++));
		break;

	case MP3_PLAY_A:
		jfk(cpu->regs[0] & 0xff);
		break;

	case EEPROM_WRITE_A_CINDIRECT:
		EEPROM.update(cpu->regs[2], cpu->regs[0]);
		break;

	case EEPROM_READ_A_CINDIRECT:
		cpu->regs[0] = EEPROM.read(cpu->regs[2]);
		break;

	case WAIT1:
		if( cpu->flags & CPU_TIMER1 ) {
			cpu->flags &= ~CPU_TIMER1;
		} else {
			cpu->PC--;
		}
		break;

	case WAIT2:
		if( cpu->flags & CPU_TIMER2 ) {
			cpu->flags &= ~CPU_TIMER2;
		} else {
			cpu->PC--;
		}
		break;

	case WAIT3:
		if( cpu->flags & CPU_TIMER3 ) {
			cpu->flags &= ~CPU_TIMER3;
		} else {
			cpu->PC--;
		}
		break;

	case WAIT4:
		if( cpu->flags & CPU_TIMER4 ) {
			cpu->flags &= ~CPU_TIMER4;
		} else {
			cpu->PC--;
		}
		break;

	case WAIT5:
		if( cpu->flags & CPU_TIMER5 ) {
			cpu->flags &= ~CPU_TIMER5;
		} else {
			cpu->PC--;
		}
		break;

	case INPUT_NOUN:
		reg1 = 0 /*NOT USED*/; op = S_NI1; goto input_reg1_op;
		break;

	case INPUT_R1:
		reg1 = 0; op = S_IR1; goto input_reg1_op;

	case INPUT_R2:
		reg1 = 1; op = S_IR1; goto input_reg1_op;

	case INPUT_R3:
		reg1 = 2; op = S_IR1; goto input_reg1_op;

	case INPUT_R1_OCT:
		reg1 = 0; op = S_IO1; goto input_reg1_op;
		break;

	case INPUT_R2_OCT:
		reg1 = 1; op = S_IO1; goto input_reg1_op;
		break;

	case INPUT_R3_OCT:
		reg1 = 2; op = S_IO1; goto input_reg1_op;
		break;

	case INPUT_PROCEED:
		reg1 = 0; op = S_IP1; goto input_reg1_op;		// reg1 not used in this context

	case INPUT_REQ_PROCEED:
		reg1 = 0; op = S_IPR; goto input_reg1_op;		// reg1 not used in this context

	case PROG8_A_CINDIRECT:
		cpu->regs[0] = ProgramAcc8(cpu->regs[2]);
		break;

	case PROG16_A_CINDIRECT:
		// arduino compiler is little endian, assembler must make sure to pack values that way
		cpu->regs[0] = ProgramAcc16(cpu->regs[2]);
		break;

	case PROG32_A_CINDIRECT:
		// arduino compiler is little endian, assembler must make sure to pack values that way
		cpu->regs[0] = ProgramAcc32(cpu->regs[2]);
		break;

	case ADD_A_IMM8:
		reg1 = 0; goto add_reg1_imm8;
		break;

	case ADD_B_IMM8:
		reg1 = 1; goto add_reg1_imm8;
		break;

	case ADD_C_IMM8:
		reg1 = 2; goto add_reg1_imm8;
		break;

	case EMPTY_STACK:
		if( c == 0 ) {
			cpu->SP = CPU0_STACK;
		} else {
			cpu->SP = CPU1_STACK;
		}
		break;

	case RUN_PROG_A:
		Agc.cpu[0].PC = cpu->regs[0];
		Agc.cpu[0].SP = CPU0_STACK;
		break;

	case RUN_MINOR_A:
		Agc.cpu[1].PC = cpu->regs[0];
		Agc.cpu[1].SP = CPU1_STACK;
		break;

	case CALL_CINDIRECT:
		Agc.RAM[cpu->SP--] = cpu->PC;
		cpu->PC = cpu->regs[2];
		break;

	case PUSH_DSKY:
		cpu->SP -= 3;
		memcpy(&Agc.RAM[cpu->SP], &Agc.dsky, sizeof(DSKY));
		cpu->SP--;
		break;

	case POP_DSKY:
		cpu->SP++;
		memcpy(&Agc.dsky, &Agc.RAM[cpu->SP], sizeof(DSKY));
		cpu->SP += 3;
		break;

	case RANDOM_A:
		cpu->regs[0] = random(0xffff);
		break;
	}

	goto done;

mov_reg1_rx:
	rx[0] = (cpu->regs[reg1] & 0xff0000 ) >> 16;
	rx[1] = (cpu->regs[reg1] & 0x00ff00 ) >> 8;
	rx[2] = (cpu->regs[reg1] & 0x0000ff ) >> 0;
	goto done;

mov_rx_reg2:
	cpu->regs[reg2] = ((uint32_t)rx[0] << 16) | ((uint32_t)rx[1] << 8) | (rx[2] << 0);
	goto done;

mov_reg1_reg2:
	cpu->regs[reg2] = cpu->regs[reg1];
	goto done;

clr_reg1:
	cpu->regs[reg1] = 0;
	goto done;

inc_reg1:
	cpu->regs[reg1]++;
	goto done;

dec_reg1:
	cpu->regs[reg1]--;
	goto done;

push_reg1:
	Agc.RAM[cpu->SP--] = cpu->regs[reg1];
	goto done;

pop_reg1:
	cpu->regs[reg1] = Agc.RAM[++cpu->SP];
	goto done;

swap_reg1_reg2:
	tmp = cpu->regs[reg1];
	cpu->regs[reg1] = cpu->regs[reg2];
	cpu->regs[reg2] = tmp;
	goto done;

ld_reg1_direct:
	cpu->regs[reg1] = Agc.RAM[ProgramAccU8(cpu->PC++)];
	goto done;

ld_reg1_imm32:
	cpu->regs[reg1] = ProgramAcc32(cpu->PC);
	cpu->PC += 4;
	goto done;

ld_reg1_imm16:
	cpu->regs[reg1] = ProgramAcc16(cpu->PC);
	cpu->PC += 2;
	goto done;

ld_reg1_imm8:
	cpu->regs[reg1] = ProgramAcc8(cpu->PC);
	cpu->PC += 1;
	goto done;

ld_reg1_indirect_c:
	cpu->regs[reg1] = Agc.RAM[cpu->regs[2]];
	goto done;

st_reg1_direct:
	Agc.RAM[ProgramAccU8(cpu->PC++)] = cpu->regs[reg1];
	goto done;

st_reg1_indirect_c:
	Agc.RAM[cpu->regs[2]] = cpu->regs[reg1];
	goto done;

add_reg1_reg2:
	cpu->regs[reg1] += cpu->regs[reg2];
	goto done;

sub_reg1_reg2:
	cpu->regs[reg1] -= cpu->regs[reg2];
	goto done;

mul_reg1_reg2:
	cpu->regs[reg1] *= cpu->regs[reg2];
	goto done;

div_reg1_reg2:
	cpu->regs[reg1] /= cpu->regs[reg2];
	goto done;

mod_reg1_reg2:
	cpu->regs[reg1] %= cpu->regs[reg2];
	goto done;

and_reg1_reg2:
	cpu->regs[reg1] &= cpu->regs[reg2];
	goto done;

or_reg1_reg2:
	cpu->regs[reg1] |= cpu->regs[reg2];
	goto done;

or_reg1_imm32:
	cpu->regs[reg1] |= ProgramAcc32(cpu->PC);
	cpu->PC += 4;
	goto done;

and_reg1_imm32:
	cpu->regs[reg1] &= ProgramAcc32(cpu->PC);
	cpu->PC += 4;
	goto done;

lshift_reg1:
	cpu->regs[reg1] <<= ProgramAcc8(cpu->PC++);
	goto done;

rshift_reg1:
	cpu->regs[reg1] >>= ProgramAcc8(cpu->PC++);
	goto done;

not_reg1:
	cpu->regs[reg1] = ~cpu->regs[reg1];
	goto done;

neg_reg1:
	cpu->regs[reg1] = -cpu->regs[reg1];
	goto done;

branch_reg1_op_reg2:
	cmp = do_cmp(op, cpu->regs[reg1], cpu->regs[reg2]);
	if( cmp ) {
		cpu->PC += ProgramAcc8(cpu->PC++);
	} else {
		cpu->PC++;
	}
	goto done;

branch_reg1_op_direct:
	cmp = do_cmp(op, cpu->regs[reg1], Agc.RAM[ ProgramAccU8(cpu->PC++) ]);
	if( cmp ) {
		cpu->PC += ProgramAcc8(cpu->PC++);
	} else {
		cpu->PC++;
	}
	goto done;

branch_reg1_op_imm8:
	imm8 = ProgramAcc8(cpu->PC++);
	cmp = do_cmp(op, cpu->regs[reg1], imm8);
	if( cmp ) {
		cpu->PC += ProgramAcc8(cpu->PC++);
	} else {
		cpu->PC++;
	}
	goto done;

branch_reg1_op_imm16:
	imm16 = ProgramAcc16(cpu->PC);
	cpu->PC += 2;
	cmp = do_cmp(op, cpu->regs[reg1], imm16);
	if( cmp ) {
		cpu->PC += ProgramAcc8(cpu->PC++);
	} else {
		cpu->PC++;
	}
	goto done;

branch_not_timer_op:
	if( (cpu->flags & op) == 0 ) {
		cpu->PC += ProgramAcc8(cpu->PC++);
	} else {
		cpu->flags &= ~op;
		cpu->PC++;
	}
	goto done;

add_reg1_imm8:
	imm8 = ProgramAcc8(cpu->PC++);
	cpu->regs[reg1] += imm8;
	goto done;

blink_op:
	imm8 = ProgramAcc8(cpu->PC++);
	if( imm8 ) {
		Agc.dsky.blink |= op;
	} else {
		Agc.dsky.blink &= ~op;
		switch(unhide) {
		case UNHIDE_R1:			Agc.dsky.vnpc &= ~VNPC_HIDE_R1; break;
		case UNHIDE_R2:			Agc.dsky.rstatus &= ~RS_HIDE_R2; break;
		case UNHIDE_R3:			Agc.dsky.lstatus &= ~LS_HIDE_R3; break;
		case UNHIDE_VERB:		Agc.dsky.vnpc &= ~VNPC_HIDE_VERB; break;
		case UNHIDE_NOUN:		Agc.dsky.vnpc &= ~VNPC_HIDE_NOUN; break;
		case UNHIDE_PROG:		Agc.dsky.vnpc &= ~VNPC_HIDE_PROG; break;
		case UNHIDE_KEYREL:		Agc.dsky.lstatus &= ~LS_KEY_REL; break;
		case UNHIDE_OPRERR:		Agc.dsky.lstatus &= ~LS_OPR_ERR; break;
		}
	}
	goto done;

light_lstatus_op:
	imm8 = ProgramAcc8(cpu->PC++);
	if( imm8 )
		Agc.dsky.lstatus |= op;
	else
		Agc.dsky.lstatus &= ~op;
	goto done;

light_rstatus_op:
	imm8 = ProgramAcc8(cpu->PC++);
	if( imm8 )
		Agc.dsky.rstatus |= op;
	else
		Agc.dsky.rstatus &= ~op;
	goto done;

light_vnpc_op:
	imm8 = ProgramAcc8(cpu->PC++);
	if( imm8 )
		Agc.dsky.vnpc |= op;
	else
		Agc.dsky.vnpc &= ~op;
	goto done;

input_reg1_op:
	if( (cpu->flags & CPU_NEED_INPUT) == 0 ) {
		cpu->flags |= CPU_NEED_INPUT;
		Agc.input_reg = reg1;
		Agc.state = op;
		cpu->PC--;
	} else if( (cpu->flags & CPU_GOT_INPUT) == 0 ) {
		cpu->PC--;
	} else if( (cpu->flags & CPU_GOT_INPUT) != 0 ) {
		cpu->flags &= ~(CPU_GOT_INPUT | CPU_NEED_INPUT);
	}
	goto done;

encode_a_to_decimal:
	// 'imm8' contains the plus sign symbol (0x0b or 0x0a)
	tmp = cpu->regs[0];
	op = (tmp < 0);
	if(op) tmp = -tmp;
	res = 0;
	res |= (tmp % 10);
	tmp /= 10;
	res |= (tmp % 10) << 4;
	tmp /= 10;
	res |= (tmp % 10) << 8;
	tmp /= 10;
	res |= (tmp % 10) << 12;
	tmp /= 10;
	res |= (tmp % 10) << 16;
	res |= (uint32_t)(op ? 0x0c : imm8) << 20;
	cpu->regs[0] = res;
	goto done;

rtc_op_a:
	Wire.beginTransmission(RTC_ADDR);
	Wire.write(op);
	Wire.endTransmission(true);
	Wire.requestFrom(RTC_ADDR, 1, true);
	cpu->regs[0] = Wire.read();
	goto done;

done:

#ifdef DSKY_DEBUG
	// this generates a lot of log messages, change 'false' to 'true' to enable
	if( false && c == 1 && prevPC != 0 ) {
		fprintf(logfp, "cpu %d: pc: %d, sp: %d, Instruction = %s, A=%d B=%d C=%d A=%02X B=%02X C=%02X\n",
						c, prevPC, cpu->SP, Mnemonics[instruction],
						cpu->regs[0], cpu->regs[1], cpu->regs[2],
						cpu->regs[0], cpu->regs[1], cpu->regs[2] );
		fflush(logfp);
	}
#endif

	return;
}

//
// Cause cpu[c] to call subroutine located at 'start'
// The previous verb and noun are also packed into the same stack slot.
//		+------+------+-----+-----+
//		| verb | noun | PCL | PCH |		<--- SP
//		+------+------+-----+-----+
//
static
void agc_call(uint8_t c, uint8_t prev_verb, uint8_t prev_noun, uint16_t start)
{
	Agc.RAM[ Agc.cpu[c].SP-- ] = ((uint32_t)prev_verb << 24) | ((uint32_t)prev_noun << 16) | Agc.cpu[c].PC;
	Agc.cpu[c].PC = start;
}

//
// See if cpu[0] or cpu[1] is running an INPUT instruction
// and if so, unblock it and populate the A register with R0|R1|R2.
//
static
uint8_t agc_accept_input(uint8_t R0, uint8_t R1, uint8_t R2)
{
	uint8_t c;

	for(c=0; c < 2; c++)
	{
		if( Agc.cpu[c].flags & CPU_NEED_INPUT )
		{
#ifdef CURSES_SIMULATOR
			fprintf(logfp, "agc_accept_input c:%d R0=%02X R1=%02X R2=%02X\n", c, R0, R1, R2);
#endif
			Agc.cpu[c].flags |= CPU_GOT_INPUT;
			Agc.cpu[c].regs[0] = ((uint32_t)R0 << 16) | ((uint32_t)R1 << 8) | (R2 << 0);
			return 1;
		}
	}
	return 0;
}

//
// See if cpu[0] or cpu[1] is running an INPUT instruction
// and if so unblock it and populate the A register with -1
// to indicate an error.
//
static
uint8_t agc_reject_input()
{
	uint8_t c;

	for(c=0; c < 2; c++)
	{
		if( Agc.cpu[c].flags & CPU_NEED_INPUT )
		{
			Agc.cpu[c].flags |= CPU_GOT_INPUT;
			Agc.cpu[c].regs[0] = -1;
			return 1;
		}
	}
	return 0;
}

//
// reg is 0, 1 or 2. Maps to a pointer to the dsky register R1, R2 or R3.
//
static
uint8_t *agc_get_reg(uint8_t reg)
{
	switch(reg) {
	case 0:		return Agc.dsky.R1;
	case 1:		return Agc.dsky.R2;
	case 2:		return Agc.dsky.R3;
	}
	return NULL;
}

//////////////////////////////////////////////////////////////////////
//
// Main execution loop for the Apollo Guidance Computer
// This routine contains its own "forever" loop.
// Because of this it is not necessary to run this in loop().
//
//////////////////////////////////////////////////////////////////////
static
void apollo_guidance_computer()
{
	int8_t ch;
	uint8_t input;
	uint8_t new_state;
	uint8_t digit;
	uint8_t i;
	uint8_t* regp;
	bool found;
	uint8_t prev_verb;
	uint8_t prev_noun;

	agc_init();

	prev_verb = Agc.dsky.verb;
	prev_noun = Agc.dsky.noun;

	input = -1;
	Agc.state = S_ST;

	for(;;) {
		agc_execute_cpu(0);
		agc_execute_cpu(1);

		ch = readKeyboard();

		if( ch == ERR ) {
			//
			// Execute current computer program
			//
#ifdef CURSES_SIMULATOR
		} else if( ch == 'q' ) {
			break;
#endif
		} else {
			if( ch >= '0' && ch <= '9' ) {
				input = I_DIGIT;
				digit = ch - '0';
			} else {
				switch(ch) {
				case 'v': input = I_VERB; break;
				case 'n': input = I_NOUN; break;
				case 'p': input = I_PRO; break;
				case 'c': input = I_CLR; break;
				case 'k': input = I_KREL; break;
				case 'e': input = I_ENTR; break;
				case 'r': input = I_RSET; break;
				case '+': input = I_PLUS; break;
				case '-': input = I_MINUS; break;
				}
			}

			new_state = StateMachineAcc(Agc.state, input);

			if( new_state == S_NOP ) {
				continue;
			}

			Agc.state = new_state;

			switch(Agc.state) {
			case S_ST:
				break;

			case S_ERR:
				if( input == I_RSET ) {
					Agc.dsky.blink &= ~BLINKF_OPRERR;
					Agc.dsky.lstatus &= ~LS_OPR_ERR;
					Agc.dsky.verb = prev_verb;
					Agc.dsky.noun = prev_noun;
				} else if( input == I_KREL ) {
					Agc.dsky.blink &= ~BLINKF_KEYREL;
					Agc.dsky.lstatus &= ~LS_KEY_REL;
					Agc.dsky.verb = prev_verb;
					Agc.dsky.noun = prev_noun;
				}

				if( (Agc.dsky.blink & BLINKF_OPRERR) == 0
						&& (Agc.dsky.blink & BLINKF_KEYREL) == 0 ) {
					Agc.state = S_ST;
				}
				break;

			case S_ENT:
				found = false;
				for(i=0; i < sizeof(Verbs)/sizeof(Verbs[0]); i++)
				{
					if( Agc.dsky.verb == VerbAcc(i) ) {
						found = true;
						break;
					}
				}

				if(found) {
					agc_call(1, prev_verb, prev_noun, VerbStartAcc(i));
				} else {
					Agc.dsky.verb = prev_verb;
					Agc.dsky.noun = prev_noun;
				}
				Agc.state = S_ST;
				break;

			case S_KRL:
				Agc.state = S_ST;
				if( Agc.dsky.blink & BLINKF_KEYREL ) {
					Agc.dsky.verb = prev_verb;
					Agc.dsky.noun = prev_noun;
					Agc.dsky.blink &= ~BLINKF_KEYREL;
					Agc.dsky.lstatus &= ~LS_KEY_REL;
				}
				break;

			case S_RST:
				Agc.state = S_ST;
				Agc.dsky.lstatus &= BLINKF_KEYREL;
				Agc.dsky.rstatus = 0;
				if( Agc.dsky.blink & BLINKF_OPRERR ) {
					Agc.dsky.blink &= ~BLINKF_OPRERR;
				}
				break;

			case S_IN:
				agc_reject_input();
				Agc.state = S_N1;
				break;

			case S_IV:
				agc_reject_input();
				Agc.state = S_V1;
				break;

			case S_IE:
				if( input != I_RSET ) {
					Agc.dsky.blink |= BLINKF_OPRERR;
					// agc_reject_input();
				} else if( input == I_RSET ) {
					agc_reject_input();
					Agc.dsky.blink &= ~BLINKF_OPRERR;
					Agc.dsky.lstatus &= ~LS_OPR_ERR;
					Agc.state = S_ST;
				}
				break;

			case S_V1:
				prev_verb = Agc.dsky.verb;
				prev_noun = Agc.dsky.noun;
				Agc.dsky.verb = 0xAA;
				Agc.dsky.blink |= BLINKF_KEYREL;
				break;

			case S_V2:
				Agc.dsky.verb = (digit << 4) | (Agc.dsky.verb & 0x0f);
				break;

			case S_VA:
				Agc.dsky.verb = (digit) | (Agc.dsky.verb & 0xf0);
				Agc.dsky.blink &= ~BLINKF_KEYREL;
				Agc.dsky.lstatus &= ~LS_KEY_REL;
				Agc.state = S_ST;
				break;

			case S_VE:
				Agc.dsky.blink |= BLINKF_OPRERR;
				Agc.state = S_ERR;
				break;

			case S_N1:
				prev_verb = Agc.dsky.verb;
				prev_noun = Agc.dsky.noun;
				Agc.dsky.noun = 0xAA;
				Agc.dsky.blink |= BLINKF_KEYREL;
				break;

			case S_N2:
				Agc.dsky.noun = (digit << 4) | (Agc.dsky.noun & 0x0f);
				break;

			case S_NA:
				Agc.dsky.noun = (digit) | (Agc.dsky.noun & 0xf0);
				Agc.dsky.blink &= ~BLINKF_KEYREL;
				Agc.dsky.lstatus &= ~LS_KEY_REL;
				Agc.state = S_ST;
				break;

			case S_NE:
				Agc.dsky.blink |= BLINKF_OPRERR;
				Agc.state = S_ERR;
				break;

			case S_NI1:
				Agc.dsky.noun = 0xAA;		// KJS TODO - this action never get called
				break;

			case S_NI2:
				Agc.dsky.noun = (digit << 4) | (Agc.dsky.noun & 0x0f);
				break;

			case S_NI3:
				Agc.dsky.noun = (digit) | (Agc.dsky.noun & 0xf0);
				break;

			case S_NIA:
				agc_accept_input(0xAA, 0xAA, Agc.dsky.noun);
				Agc.state = S_ST;
				break;

			case S_IR1:
				// called from CLR, also start state
				regp = agc_get_reg(Agc.input_reg);
				regp[0] = regp[1] = regp[2] = 0xAA;
				break;

			case S_IR2:
				regp = agc_get_reg(Agc.input_reg);
				regp[0] = regp[1] = regp[2] = 0xAA;		// clear entry field
				regp[0] = 0xBA;
				break;

			case S_IR3:
				regp = agc_get_reg(Agc.input_reg);
				regp[0] = regp[1] = regp[2] = 0xAA;		// clear entry field
				regp[0] = 0xCA;
				break;

			case S_IR4:
				regp[0] = (regp[0] & 0xf0) | (digit);
				break;

			case S_IR5:
				regp[1] = (regp[1] & 0x0f) | (digit) << 4;
				break;

			case S_IR6:
				regp[1] = (regp[1] & 0xf0) | (digit);
				break;

			case S_IR7:
				regp[2] = (regp[2] & 0x0f) | (digit) << 4;
				break;

			case S_IR8:
				regp[2] = (regp[2] & 0xf0) | (digit);
				break;

			case S_IRA:
				agc_accept_input(regp[0], regp[1], regp[2]);
				Agc.state = S_ST;
				break;

			case S_IO1:
				// called from CLR, also start state
				regp = agc_get_reg(Agc.input_reg);
				regp[0] = regp[1] = regp[2] = 0xAA;
				break;

			case S_IO2:
				if( digit <= 7 ) {
					regp = agc_get_reg(Agc.input_reg);
					regp[0] = regp[1] = regp[2] = 0xAA;		// clear entry field
					regp[0] = 0xA0 | (digit);
				} else {
					Agc.dsky.blink |= BLINKF_OPRERR;
					Agc.state = S_IE;
				}
				break;

			case S_IO3:
				if( digit <= 7 ) {
					regp[1] = (regp[1] & 0x0f) | (digit) << 4;
				} else {
					Agc.dsky.blink |= BLINKF_OPRERR;
					Agc.state = S_IE;
				}
				break;

			case S_IO4:
				if( digit <= 7 ) {
					regp[1] = (regp[1] & 0xf0) | (digit);
				} else {
					Agc.dsky.blink |= BLINKF_OPRERR;
					Agc.state = S_IE;
				}
				break;

			case S_IO5:
				if( digit <= 7 ) {
					regp[2] = (regp[2] & 0x0f) | (digit) << 4;
				} else {
					Agc.dsky.blink |= BLINKF_OPRERR;
					Agc.state = S_IE;
				}
				break;

			case S_IO6:
				if( digit <= 7 ) {
					regp[2] = (regp[2] & 0xf0) | (digit);
				} else {
					Agc.dsky.blink |= BLINKF_OPRERR;
					Agc.state = S_IE;
				}
				break;

			case S_IOA:
				agc_accept_input(regp[0], regp[1], regp[2]);
				Agc.state = S_ST;
				break;

			case S_IP1:
				break;

			case S_IPA:
				agc_accept_input(0, 0, 0);
				Agc.state = S_ST;
				break;

			case S_IPR:
				break;
			}
		}

		if( Agc.flags & AGC_TIMER1 )
		{
			Agc.flags &= ~AGC_TIMER1;

			union {
				uint32_t l;
				uint8_t x[4];		// x[0] - least signifigant byte, x[3] - most signifigant byte
			} u;

			u.l = random(0x10000);

			if( u.x[0] < Agc.uplink_prob ) {
				Agc.dsky.lstatus |= LS_UPLINK_ACTY;
			} else {
				Agc.dsky.lstatus &= ~LS_UPLINK_ACTY;
			}

			if( u.x[1] < Agc.compacty_prob ) {
				Agc.dsky.vnpc |= VNPC_COMP_ACTY;
			} else {
				Agc.dsky.vnpc &= ~VNPC_COMP_ACTY;
			}
	
			if( Agc.dsky.blink & BLINKF_OPRERR )
			{
				if( Agc.dsky.lstatus & LS_OPR_ERR )
					Agc.dsky.lstatus &= ~LS_OPR_ERR;
				else
					Agc.dsky.lstatus |= LS_OPR_ERR;
			}

			if( Agc.dsky.blink & BLINKF_KEYREL )
			{
				if( Agc.dsky.lstatus & LS_KEY_REL )
					Agc.dsky.lstatus &= ~LS_KEY_REL;
				else
					Agc.dsky.lstatus |= LS_KEY_REL;
			}

			if( Agc.dsky.blink & BLINKF_R1 )
			{
				if( Agc.dsky.vnpc & VNPC_HIDE_R1 )
					Agc.dsky.vnpc &= ~VNPC_HIDE_R1;
				else
					Agc.dsky.vnpc |= VNPC_HIDE_R1;
			}

			if( Agc.dsky.blink & BLINKF_R2 )
			{
				if( Agc.dsky.rstatus & RS_HIDE_R2 )
					Agc.dsky.rstatus &= ~RS_HIDE_R2;
				else
					Agc.dsky.rstatus |= RS_HIDE_R2;
			}

			if( Agc.dsky.blink & BLINKF_R3 )
			{
				if( Agc.dsky.lstatus & LS_HIDE_R3 )
					Agc.dsky.lstatus &= ~LS_HIDE_R3;
				else
					Agc.dsky.lstatus |= LS_HIDE_R3;
			}

			if( Agc.dsky.blink & BLINKF_VERB )
			{
				if( Agc.dsky.vnpc & VNPC_HIDE_VERB )
					Agc.dsky.vnpc &= ~VNPC_HIDE_VERB;
				else
					Agc.dsky.vnpc |= VNPC_HIDE_VERB;
			}

			if( Agc.dsky.blink & BLINKF_NOUN )
			{
				if( Agc.dsky.vnpc & VNPC_HIDE_NOUN )
					Agc.dsky.vnpc &= ~VNPC_HIDE_NOUN;
				else
					Agc.dsky.vnpc |= VNPC_HIDE_NOUN;
			}

			if( Agc.dsky.blink & BLINKF_PROG )
			{
				if( Agc.dsky.vnpc & VNPC_HIDE_PROG )
					Agc.dsky.vnpc &= ~VNPC_HIDE_PROG;
				else
					Agc.dsky.vnpc |= VNPC_HIDE_PROG;
			}
		}

#if 0 // not used
		if( Agc.flags & AGC_TIMER2 )
		{
			Agc.flags &= ~AGC_TIMER2;
		}

		if( Agc.flags & AGC_TIMER3 )
		{
			Agc.flags &= ~AGC_TIMER3;
		}
#endif

#ifdef CURSES_SIMULATOR
		if( Agc.flags & AGC_TIMER4 )
		{
			Agc.flags &= ~AGC_TIMER4;

			mp3_increment_1s();
			Wire.tick();
		}
#endif

#if 0 // not used
		if( Agc.flags & AGC_TIMER5 )
		{
			Agc.flags &= ~AGC_TIMER5;
		}
#endif

		dsky_redraw(&Agc.dsky, &Agc.prev);
	}
}

//////////////////////////////////////////////////////////////////////
//
// SECTION 18: Signal/ISR for Timing control (arduino and curses)
//
// signal handler 100ms
//	Set the CPU_TIMERx flag for both cpu's
//
// Two signal handlers are implemented:
//	1) CURSES SIMULATOR: Uses Unix Signals, via sigaction(SIGALRM)
//	2) Arduino: Uses interrupts and built-in timer 1. Declares an Interrupt Service Routine (ISR)
//
//	From the single 100ms timer we derive lower frequency timers:
//		200ms		called TIMER2
//		300ms		called TIMER3
//		1s			called TIMER4
//		2s			called TIMER5
//
#ifdef CURSES_SIMULATOR
static
void Signal_Handler(int signo)
{
	static uint8_t timer_count = 0;

	if( signo == SIGALRM )
	{
		// fprintf(logfp, "Signal happened\n");
		timer_count++;

		// 100ms
		if( timer_count % 1 == 0 ) {
			Agc.cpu[0].flags |= CPU_TIMER1;
			Agc.cpu[1].flags |= CPU_TIMER1;
			Agc.flags |= AGC_TIMER1;
		}

		// 200ms
		if( timer_count % 2 == 0 ) {
			Agc.cpu[0].flags |= CPU_TIMER2;
			Agc.cpu[1].flags |= CPU_TIMER2;
			Agc.flags |= AGC_TIMER2;
		}

		// 300ms
		if( timer_count % 3 == 0 ) {
			Agc.cpu[0].flags |= CPU_TIMER3;
			Agc.cpu[1].flags |= CPU_TIMER3;
			Agc.flags |= AGC_TIMER3;
		}

		// 1000ms
		if( timer_count % 10 == 0 ) {
			Agc.cpu[0].flags |= CPU_TIMER4;
			Agc.cpu[1].flags |= CPU_TIMER4;
			Agc.flags |= AGC_TIMER4;
		}

		// 2000ms
		if( timer_count % 20 == 0 ) {
			Agc.cpu[0].flags |= CPU_TIMER5;
			Agc.cpu[1].flags |= CPU_TIMER5;
			Agc.flags |= AGC_TIMER5;
			timer_count = 0;
		}
	}
}

#else

//
// Interrupt service routine for TIMER1
// Configured to interrupt the cpu every 100ms
//
ISR(TIMER1_COMPA_vect)
{
	static uint8_t timer_count = 0;

	timer_count++;

	// 100ms
	if( timer_count % 1 == 0 ) {
		Agc.cpu[0].flags |= CPU_TIMER1;
		Agc.cpu[1].flags |= CPU_TIMER1;
		Agc.flags |= AGC_TIMER1;
	}

	// 200ms
	if( timer_count % 2 == 0 ) {
		Agc.cpu[0].flags |= CPU_TIMER2;
		Agc.cpu[1].flags |= CPU_TIMER2;
		Agc.flags |= AGC_TIMER2;
	}

	// 300ms
	if( timer_count % 3 == 0 ) {
		Agc.cpu[0].flags |= CPU_TIMER3;
		Agc.cpu[1].flags |= CPU_TIMER3;
		Agc.flags |= AGC_TIMER3;
	}

	// 1000ms
	if( timer_count % 10 == 0 ) {
		Agc.cpu[0].flags |= CPU_TIMER4;
		Agc.cpu[1].flags |= CPU_TIMER4;
		Agc.flags |= AGC_TIMER4;
	}

	// 2000ms
	if( timer_count % 20 == 0 ) {
		Agc.cpu[0].flags |= CPU_TIMER5;
		Agc.cpu[1].flags |= CPU_TIMER5;
		Agc.flags |= AGC_TIMER5;
		timer_count = 0;
	}
}
#endif

//////////////////////////////////////////////////////////////////////
//
// SECTION 19: Sketch setup() routine
//
static
void setup()
{
	Serial.begin(9600);

	Wire.begin();
	Wire.beginTransmission(MPU_ADDR);
	Wire.write(0x6B);  // PWR_MGMT_1 register
	Wire.write(0);	   // set to zero (wakes up the MPU-6050)
	Wire.endTransmission(true);

	audioTrack = 1;

	pinMode(A0, INPUT);
	pinMode(A1, INPUT);
	pinMode(A2, INPUT);
	pinMode(A7, INPUT);
	pinMode(GPS_SW, OUTPUT);
	digitalWrite(GPS_SW, LOW);		// disable GPS, enable Serial
	
	randomSeed(analogRead(A7));
	neoPixels.begin();

	for (int index = 0; index < 4; index++) {
		ledControl.shutdown(index,false);
		ledControl.setIntensity(index, 15);
		ledControl.clearDisplay(index);
	}

	//////////////////////////////////////////////////////////////////////
	//
	// set timer1 interrupt at 100ms
	//
	cli();				// stop interrupts

	TCCR1A = 0;			// set entire TCCR1A register to 0
	TCCR1B = 0;			// same for TCCR1B
	TCNT1  = 0;			//initialize counter value to 0

	// calculation:
	// .1 * (16*10^6) / (1024) - 1		// .1 seconds using 1024 prescalar
	// 1561.5
	// .1 * (16*10^6) / (256) - 1		// .1 seconds, using 256 prescalar
	// 6249.00000000000000000000

	// set compare match register for 100ms increments
	OCR1A = 6249;		// = .1 * (16*10^6) / (1024) - 1 (must be <65536)

	// turn on CTC mode
	TCCR1B |= (1 << WGM12);

	// Set CS12 bit for 256 prescaler
	TCCR1B |= (1 << CS12);  

	// enable timer compare interrupt
	TIMSK1 |= (1 << OCIE1A);

	sei();				// enable interrupts
}

//////////////////////////////////////////////////////////////////////
//
// SECTION 20: Sketch loop() routine
//
static
void loop()
{
	//
	// Run the whole shabang
	//
	apollo_guidance_computer();
}

#ifdef CURSES_SIMULATOR
//////////////////////////////////////////////////////////////////////
//
// SECTION 21: MAIN() - calls setup() and loop()
//
// This is only used in the linux/macos/windows CURSES SIMULATOR version.
//
int main(int argc, char *argv[])
{
	logfp = fopen("./log.txt", "w");
	if( logfp == NULL )
	{
		fprintf(stderr, "Unable to open log.txt\n");
		exit(1);
	}

	fprintf(logfp, "Agc size = %d\n", sizeof(APOLLO_GUIDANCE_COMPUTER));
	fprintf(logfp, "DSKY size = %d\n", sizeof(DSKY));
	fprintf(logfp, "CPU size = %d\n", sizeof(CPU));
	fprintf(logfp, "StateMachine size = %d\n", sizeof(StateMachine));
	fprintf(logfp, "Program size = %d\n", sizeof(Program));
	fprintf(logfp, "Verbs dispatch size = %d\n", sizeof(Verbs));
	fprintf(logfp, "instruction opcodes size = %d\n", AGC_INSTRUCTIONS_LEN);

	audioTrack = 0;

	//
	// Setup curses
	//
	initscr();
	cbreak();
	noecho();
	clear();

	curses_window_start();
	mp3_update_curses();

	//
	// setup signal handler (100ms repeating timer)
	//
	struct sigaction psa;
	psa.sa_handler = Signal_Handler;
	sigaction(SIGALRM, &psa, NULL);

	struct itimerval itv;
	itv.it_interval.tv_sec = 0;
	itv.it_interval.tv_usec = 100000;	// 100,000usec == 100ms
	itv.it_value.tv_sec = 0;
	itv.it_value.tv_usec = 100000;   // 100,000 = 1/10th second

	setitimer(ITIMER_REAL, &itv, NULL);

	// load (or initialize) persistent storage data from disk file: ./persist.txt
	read_persistent_storage();

	setup();

//	for(;;)			// we don't want to run forever in CURSES_SIMULATOR (so user can quit with 'q')
	{
		loop();
	}

	//
	// we get here only if the user pressed 'q'
	//

	// store persistent storage data to disk in file ./persist.txt
	write_persistent_storage();

	curses_window_end();
	endwin();

	fclose(logfp);
	exit(0);
}
#endif
