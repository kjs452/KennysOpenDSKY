
// general variables for minor mode
DEFINE	TMP1 = 0
DEFINE	TMP2 = 1
DEFINE	TMP3 = 2
DEFINE	TMP4 = 3

DEFINE	V01_R1 = 5
DEFINE	V01_R2 = 6
DEFINE	V01_R3 = 7

DEFINE	RTC_SEC = 8
DEFINE	RTC_S100 = 9

DEFINE	AGCINIT = 10			// timestamp for AGC Initialization (10 and 11)
DEFINE	TS1 = 12				// generic timestamp vairable 12 and 13.

DEFINE	PROG_TMP1 = 14			// general variable for major mode PROGs

DEFINE	CONTMODE = 15			// monitor once or continiously (0=once, 1=continiously)

DEFINE	SELCLIP = 16			// selected clip to play
DEFINE	ADJFACT = 17			// adjustment factor
DEFINE	CLIP_HAS_PLAYED = 18	// 0=clip hasn't played yet in V06 N98 (or V16 N98), 1=has played

DEFINE	ImuGyroX = 19			// use this address for IMU_READ_DIRECT
DEFINE	ImuGyroY = 20
DEFINE	ImuGyroZ = 21
DEFINE	ImuAccX = 22
DEFINE	ImuAccY = 23
DEFINE	ImuAccZ = 24
DEFINE	ImuTemp = 25

// idle loop for for cpu 0
MAIN_CPU0:
{
	CALL	Init
	// NOTE: this falls thru to MAIN_CPU1
}

// idle loop for cpu 1	(cpu 0 comes here on VERB_36)
MAIN_CPU1:
{
	EMPTY_STACK
loop:
	BRANCH loop
}

//
// Initialization stuff
//
Init:
{
	LD_A_IMM8				1
	ST_A_DIRECT				SELCLIP		// set selected clip to be "1"
	RTC_TIMESTAMP_DIRECT	AGCINIT		// reset the "time since AGC power up"
	RET
}

//
// Run major mode
//
VERB_37:
{
			LD_A_IMM8		0xAA
			MOV_A_NOUN			// clear noun field
			BLINK_VERB	1
			BLINK_NOUN	1

			INPUT_NOUN
			BRANCH_A_LT_IMM8	0	Error
			LD_B_IMM16	0x00FF
			AND_A_B
			MOV_A_B

			LD_C_IMM16	Programs

Next:		PROG8_A_CINDIRECT
			BRANCH_A_EQ_IMM8	-1		NotFound
			INC_C
			ST_A_DIRECT			PROG_TMP1
			PROG16_A_CINDIRECT
			ADD_C_IMM8			2
			BRANCH_B_EQ_DIRECT	PROG_TMP1	Found
			BRANCH	Next

Programs:	DATA8		0x00
			DATA16		PROG_00
			DATA8		0x01
			DATA16		PROG_01
			DATA8		0x02
			DATA16		PROG_02
			DATA8		0x11
			DATA16		PROG_11
			DATA8		0x60
			DATA16		PROG_60
			DATA8		0x61
			DATA16		PROG_61
			DATA8		0x62
			DATA16		PROG_62
			DATA8		0x68
			DATA16		PROG_68
			DATA8		0x69
			DATA16		PROG_69
			DATA8		0x70
			DATA16		PROG_70
			DATA8		0x42
			DATA16		PROG_42
			DATA8		0x06
			DATA16		PROG_06
			DATA8		-1

Found:
			SWAP_A_B
			// ENCODE_A_TO_DEC
			MOV_A_PROG
			SWAP_A_B
			RUN_PROG_A
			BRANCH Done

Error:
NotFound:
			BLINK_OPRERR	1

Done:
			BLINK_VERB	0
			BLINK_NOUN	0
			RET
}

// LAMP TEST
VERB_35:
{
	PUSH_A
	PUSH_B
	PUSH_C
	PUSH_DSKY

	// Do lights
	LT_UPLINK_ACTY	1
	LT_TEMP			1
	WAIT1

	LT_NO_ATT		1
	LT_GIMBAL_LOCK	1
	WAIT1

	LT_NO_ATT		1
	LT_GIMBAL_LOCK	1
	WAIT1

	LT_STBY			1
	LT_PROG_ALRM	1
	WAIT1

	LT_KEY_REL		1
	LT_RESTART		1
	WAIT1

	LT_OPR_ERR		1
	LT_TRACKER		1
	WAIT1

	LT_NA1			1
	LT_ALT			1
	WAIT1

	LT_NA2			1
	LT_VEL			1
	WAIT1

	// do digits
	LD_A_IMM32		0x00B88888
	MOV_A_PROG
	WAIT1
	MOV_A_VERB
	MOV_A_NOUN
	WAIT1
	MOV_A_R1
	WAIT1
	MOV_A_R2
	WAIT1
	MOV_A_R3
	WAIT2

	BLINK_VERB		1
	BLINK_NOUN		1
	BLINK_PROG		1
	BLINK_KEYREL	1
	BLINK_OPRERR	1

	WAIT5
	WAIT5
	WAIT5

// restore state
	POP_DSKY
	POP_C
	POP_B
	POP_A
	RET
}

// view memory
//	0000 - 3777		2K EEPROM
//	4000 - 4067		RTC RAM (0 to 55)
VERB_01:
{
		EMPTY_STACK
		MOV_NOUN_A
		BRANCH_A_NE_IMM8	0x02	error

		CALL				EnterAddressR3
		BRANCH_B_LT_IMM8	0		error
		BRANCH_B_LE_IMM16	2047	eeprom		// EEPROM memory	(0000 - 3777 octal)
		BRANCH				rtc					// RTC memory		(4000 - 4067 octal)

eeprom:
		MOV_B_C
		EEPROM_READ_A_CINDIRECT
		BRANCH				continue
rtc:
		LD_A_IMM16			2040				// adjust address 2048-8
		SUB_B_A
		MOV_B_C

		RTC_MEM_A_CINDIRECT

continue:
		ENCODE_A_TO_OCT
		MOV_A_R1
		BRANCH			done

error:	BLINK_OPRERR	1

done:	BLINK_VERB		0
		BLINK_NOUN		0
x:		BRANCH		x
}

//
// Prompt for octal address in R3.
// This code used by: V21-N02 and V01-N02
//
//	Modifies:
//		- Registers A, B
//		- Starts VERB and NOUN fields blinking
//		- clears R1, R2, R3
//	
//	Returns address in B (decoded from BCD octal)
//		Returns -1 in B for error.
//
EnterAddressR3:
{
		BLINK_VERB		1
		BLINK_NOUN		1
		LD_A_IMM32		0xAAAAAA
		MOV_A_R1
		MOV_A_R2
		MOV_A_R3

		INPUT_R3_OCT
		BRANCH_A_LT_IMM8	0	error
		DECODE_A_FROM_OCT
		MOV_A_B
		ENCODE_A_TO_OCT			// reformat
		MOV_A_R3

		BRANCH_B_GT_IMM16	2103	error		// address to large
		RET

error:
		LD_B_IMM8		-1
		RET
}

VERB_02:
{
		LD_A_IMM32		-12345
		ENCODE_A_TO_DEC
		MOV_A_R1

		BLINK_R2		1
		LD_A_IMM32		0xAAAAAA
		MOV_A_R2

		LD_A_IMM32		99091
		ENCODE_A_TO_DEC
		MOV_A_R3

		INPUT_R2
		BRANCH_A_LT_IMM8	0		bad
		DECODE_A_FROM_DEC
		ENCODE_A_TO_UDEC
		BRANCH			good
bad:	LD_A_IMM32		0xAAAAAA
good:	MOV_A_R2
		BLINK_R2		0
		RET
}

VERB_03:
{
		UPLINK_PROB_IMM8	128
		COMPACTY_PROB_IMM8	128

		LD_A_IMM32	0xAAAAAA
		ST_A_DIRECT	TMP1

		LD_A_IMM32	0xA1201A
		MOV_A_R1

		LD_A_IMM32	-9091
		ENCODE_A_TO_DEC
		MOV_A_R2

		BLINK_NOUN	1
		BLINK_VERB	1
		LD_A_DIRECT	TMP1
		MOV_A_R3

		INPUT_R3_OCT
		BRANCH_A_LT_IMM8	0	bad
		DECODE_A_FROM_OCT
		ENCODE_A_TO_OCT
		BRANCH			good
bad:	LD_A_DIRECT		TMP1
good:	MOV_A_R3
		BLINK_NOUN	0
		BLINK_VERB	0
		RET
}

//
// three seperate incrementing and decrementing numbers
//
VERB_04:
{
		EMPTY_STACK
		CLR_A
		ST_A_DIRECT	V01_R1
		ST_A_DIRECT	V01_R2
		LD_A_IMM8	-20
		ST_A_DIRECT	V01_R3

LOOP:
		WAIT3
		LD_A_DIRECT	V01_R1
		BRANCH_A_LE_IMM16	10000	L01
		CLR_A
		BRANCH OVER1
L01:	INC_A
OVER1:	ST_A_DIRECT	V01_R1
		ENCODE_A_TO_DEC
		MOV_A_R1

		BRANCH_NOT_TIMER4	LOOP

		LD_A_DIRECT	V01_R2
		BRANCH_A_LE_IMM16		3000	L02
		CLR_A
		BRANCH OVER2
L02:	INC_A
OVER2:	ST_A_DIRECT	V01_R2
		ENCODE_A_TO_DEC
		MOV_A_R2

		BRANCH_NOT_TIMER5	LOOP

		LD_A_DIRECT	V01_R3
		BRANCH_A_LE_IMM16		3000	L03
		CLR_A
		BRANCH OVER3
L03:	INC_A
OVER3:	ST_A_DIRECT	V01_R3
		ENCODE_A_TO_DEC
		MOV_A_R3

		BRANCH	LOOP
}

//
// This verb clears the foreground task and clears the DSKY
//
VERB_05:
{
	EMPTY_STACK
	LD_A_IMM32	0xAAAA0000		// cause the prev_verb and prev_noun to be cleared on return
	LD_B_IMM16 MAIN_CPU1
	OR_A_B
	PUSH_A

	LD_A_IMM32	0x00AAAAAA		// clear register fields
	MOV_A_R1
	MOV_A_R2
	MOV_A_R3

	RET					// return to address 0 and clear Verb/Noun fields
}

//
// Fresh Start
//
VERB_36:
{
	LD_A_IMM8	MAIN_CPU1
	RUN_PROG_A
	EMPTY_STACK
	LD_A_IMM32			0xAAAAAA
	MOV_A_R1
	MOV_A_R2
	MOV_A_R3
	MOV_A_VERB
	MOV_A_NOUN
	MOV_A_PROG
	LT_ALL			0

	UPLINK_PROB_IMM8		0
	COMPACTY_PROB_IMM8		0

	LD_A_IMM8			4
loop:
	LT_RESTART			1
	WAIT4
	LT_RESTART			0
	WAIT4
	DEC_A
	BRANCH_A_GT_IMM8	0	loop
done:
	LT_NOUN				1
	WAIT2
	LT_PROG				1
	WAIT2
	LT_VERB				1
	WAIT2
	UPLINK_PROB_IMM8	0xff
	LT_TEMP				1
	WAIT2
	LT_NO_ATT			1
	LT_GIMBAL_LOCK		1
	WAIT2
	LT_STBY				1
	LT_PROG_ALRM		1
	WAIT2
	LT_KEY_REL			1
	LT_RESTART			1
	WAIT2
	LT_OPR_ERR			1
	LT_TRACKER			1
	WAIT2
	LT_NA1				1
	LT_ALT				1
	WAIT2
	LT_NA2				1
	LT_VEL				1
	WAIT5

	LD_A_IMM32			0xB88888
	CALL				doleds

	WAIT5
	WAIT5
	WAIT5

	LT_ALL				0
	LT_NOUN				1
	LT_PROG				1
	LT_VERB				1
	UPLINK_PROB_IMM8	0
	LT_NO_ATT			1
	LD_A_IMM32			0xAAAAAA
	CALL				doleds

forever:
	BRANCH	forever

doleds:
	MOV_A_R1
	MOV_A_R2
	MOV_A_R3
	MOV_A_NOUN
	MOV_A_VERB
	MOV_A_PROG
	RET
}

//
// Force restart (clears the Agc time since power up)
//
VERB_69:
{
	CALL		Init
	CALL		VERB_36
}

PROG_00:
PROG_01:
PROG_02:
PROG_60:
{
		BRANCH PROG_00
}

PROG_11:
{
		WAIT5				// wait for cpu1 to return from V37 N11
		WAIT5
		LD_A_IMM8	0xAA
		MOV_A_NOUN
		MOV_A_VERB
		LD_A_IMM16		MAIN_CPU1	// end whatever is running in cpu1
		RUN_MINOR_A
again:	CALL	Query_IMU_Accel
		BRANCH	again
}

// P61 playback JFK i believe
PROG_61:
{
	LD_A_IMM8	3
	MP3_PLAY_A
f:	BRANCH f
}

// P62 playback JFK We choose
PROG_62:
{
	LD_A_IMM8	4
	MP3_PLAY_A
f:	BRANCH f
}

// P68 playback A8 Genesis
PROG_68:
{
	LD_A_IMM8	5
	MP3_PLAY_A
f:	BRANCH f
}

// P69 playback A11 eagle has landed
PROG_69:
{
	LD_A_IMM8	6
	MP3_PLAY_A
f:	BRANCH f
}

// P70 playback A11 we have a problem
PROG_70:
{
	LD_A_IMM8	17
	MP3_PLAY_A
f:	BRANCH f
}

//
// Blinky: Randomly light caution and warning lamps
//
PROG_42:
{
			RANDOM_A
			LD_B_IMM8		3
			MOD_A_B							// A contains random delay in seconds between 0 and 3

waiting:	WAIT4							// wait 1 second
			DEC_A
			BRANCH_A_GE_IMM8	0	waiting		// waiting

			// change lamps
			RANDOM_A
			LD_B_IMM8		22				// pick a light 0-21
			MOD_A_B							// A is a value between 0 and 21
			LD_B_IMM8		3
			MUL_A_B							// A = A*3
			LD_B_IMM16		LampList
			ADD_B_A							// B = LampList + A
			MOV_B_C
			CALL_CINDIRECT

			BRANCH PROG_42

LampList:						// LampList consists of 21 3-byte functions

On_NOATT:		LT_NO_ATT	1			// 0
				RET
On_STBY:		LT_STBY		1			// 1
				RET
On_NA1:			LT_NA1		1			// 2
				RET
On_NA2:			LT_NA2		1
				RET
On_TEMP:		LT_TEMP		1
				RET
On_GIMBAL_LOCK:	LT_GIMBAL_LOCK	1		// 5
				RET
On_PROG_ALRM:	LT_PROG_ALRM	1
				RET
On_RESTART:		LT_RESTART	1
				RET
On_TRACKER:		LT_TRACKER	1
				RET
On_ALT:			LT_ALT	1
				RET
On_VEL:			LT_VEL	1				// 10
				RET
Off_NOATT:		LT_NO_ATT	0
				RET
Off_STBY:		LT_STBY		0
				RET
Off_NA1:		LT_NA1		0
				RET
Off_NA2:		LT_NA2		0
				RET
Off_TEMP:		LT_TEMP		0			// 15
				RET
Off_GIMBAL_LOCK: LT_GIMBAL_LOCK	0
				RET
Off_PROG_ALRM:	LT_PROG_ALRM	0
				RET
Off_RESTART:	LT_RESTART	0
				RET
Off_TRACKER:	LT_TRACKER	0
				RET
Off_ALT:		LT_ALT	0				// 20
				RET
Off_VEL:		LT_VEL	0				// 21
				RET
}

//
// simulate Kevin Bacon / Gary Sinise Agc shutdown / Agc startup
//
// 1) 
//	V 50	V 25		Blinking
//	R1 = +00062
//	R2 = +00000
//	R3 = +00000
//
PROG_06:
{
		// begin standby
		LD_A_IMM16		MAIN_CPU1		// idle loop for cpu 1
		RUN_MINOR_A
		EMPTY_STACK
		UPLINK_PROB_IMM8	0
		COMPACTY_PROB_IMM8	0

		LD_A_IMM8	0x50
		MOV_A_VERB
		LD_A_IMM8	0x25
		MOV_A_NOUN

		BLINK_VERB		0		// verb/noun are already blinking at this point
		BLINK_NOUN		0		// so turn off first. this is needed to syncronize blinking
								// for all three fields.

		BLINK_VERB		1
		BLINK_NOUN		1
		BLINK_PROG		1

		LD_A_IMM32	0xB00062
		MOV_A_R1
		LD_A_IMM32	0xB00000
		MOV_A_R2
		LD_A_IMM32	0xB00000
		MOV_A_R3

		INPUT_PROCEED
		BRANCH_A_LT_IMM8		0	cancel

		// Turn everything off, except STBY light
		LT_ALL		0
		LT_STBY		1
		WAIT3
		WAIT3
		CALL		clr

		INPUT_REQ_PROCEED			// required proceed

		LT_STBY				0
		WAIT4
		UPLINK_PROB_IMM8	255
		WAIT4
		LT_TEMP				1
		WAIT4
		LT_NO_ATT			1
		WAIT4
		LT_PROG_ALRM		1
		WAIT4
		LT_STBY				1
		WAIT4
		LT_TRACKER			1
		WAIT4
		LT_OPR_ERR			1
		WAIT4
		WAIT4

		LT_ALL				0
		UPLINK_PROB_IMM8	0

		WAIT3
		LD_A_IMM8			3
rloop:	LT_RESTART			1
		WAIT3
		WAIT3
		LT_RESTART			0
		WAIT3
		WAIT3
		DEC_A
		BRANCH_A_GE_IMM8	0		rloop

		WAIT3
		LT_PROG				1
		WAIT3
		LT_NOUN				1
		WAIT3
		LT_VERB				1
		WAIT3
		COMPACTY_PROB_IMM8	255
		WAIT3

		LD_A_IMM8		0x16
		MOV_A_VERB
		LD_A_IMM8		0x20
		MOV_A_NOUN
		LD_A_IMM8		0x46
		MOV_A_PROG

		LD_A_IMM32		0xC00000
		MOV_A_R1

		LD_A_IMM32		0xB00180
		MOV_A_R2

		LD_A_IMM32		0xB00000
		MOV_A_R3

		UPLINK_PROB_IMM8	10
		COMPACTY_PROB_IMM8	100

		BRANCH			forever

cancel:	CALL			clr
		BRANCH			forever

forever:
		BRANCH		forever

clr:
		BLINK_NOUN		0
		BLINK_VERB		0
		BLINK_PROG		0
		LD_A_IMM32		0xAAAAAA
		MOV_A_NOUN
		MOV_A_VERB
		MOV_A_R1
		MOV_A_R2
		MOV_A_R3
		MOV_A_PROG
		RET
}

//
// VERB 16 & VERB 06
//
// This verb runs forever, it overwrites any running foreground task(s).
//
// Monitor various devices:
// V16 N17		Monitor IMU linear accel values
// V16 N18		Monitor IMU Gyro accel values
// V16 N19		Monitor RTC Date/Time and IMU temp
// V16 N31		Monitor time from agc init
// V16 N34		Monitor/Stop Time From even
// V16 N35		Monitor/Stop timer count to event
// V16 N36		Monitor RTC Time
// V16 N37		Monitor RTC Date
// V16 N38		Monitor GPS time
// V16 N39		Monitor GPS date
// V16 N43		Monitor GPS coordinates
// V16 N44		Monitor Orbital Parameters
// V16 N65		Monitor MET
// V16 N68		A11 Lunar Landing simulation
// V16 N87		Monitor IMU linear accel values (with random 1202 alarms)
// V16 N98		Play selected audio clicp R1=clip, R2=index adj factor
//
VERB_06:
{
	CLR_A
	ST_A_DIRECT		CONTMODE		// 0 means monitor once
	BRANCH VERB_06_16
}

VERB_16:
{
	LD_A_IMM8			1			// 1 means monitor continiously
	ST_A_DIRECT			CONTMODE
	BRANCH VERB_06_16
}

VERB_06_16:
{
		EMPTY_STACK

		CLR_A												// special hack for N98
		ST_A_DIRECT				CLIP_HAS_PLAYED

		UPLINK_PROB_IMM8		10
		COMPACTY_PROB_IMM8		10

		MOV_NOUN_A
		MOV_A_B

		LD_C_IMM16					NounTable
Next:	PROG8_A_CINDIRECT
		BRANCH_A_EQ_IMM8	-1		NotFound
		AND_A_IMM32			0xff		// ensure byte is unsigned
		INC_C
		ST_A_DIRECT			TMP1
		PROG16_A_CINDIRECT
		BRANCH_B_EQ_DIRECT	TMP1	Found
		ADD_C_IMM8			2
		BRANCH	Next

NounTable:
		DATA8	0x17
		DATA16	Query_IMU_Accel

		DATA8	0x18
		DATA16	Query_Gyro_Accel

		DATA8	0x19
		DATA16	Query_RTC_DateTimeTemp

		DATA8	0x31
		DATA16	Query_AgcInitTime

		DATA8	0x34
		DATA16	Query_TimeFromEvent

		DATA8	0x35
		DATA16	Query_TimeToEvent

		DATA8	0x36
		DATA16	Query_RTC_Time

		DATA8	0x37
		DATA16	Query_RTC_Date

		DATA8	0x38
		DATA16	Query_GPS_Time

		DATA8	0x39
		DATA16	Query_GPS_Date

		DATA8	0x43
		DATA16	Query_GPS_Coord

		DATA8	0x44
		DATA16	Query_Orbital_Params

		DATA8	0x65
		DATA16	Query_MET

		DATA8	0x68
		DATA16	Query_Major_Mode

		DATA8	0x87
		DATA16	Query_IMU_1202

		DATA8	0x98
		DATA16	Query_AudioClipPlaying

		DATA8	-1

Found:
		MOV_A_C
		ST_C_DIRECT		TMP1

		LD_A_DIRECT			CONTMODE
		BRANCH_A_NE_IMM8	0	Monitor

		LD_C_DIRECT			TMP1
		CALL_CINDIRECT
forever:
		BRANCH				forever

Monitor:
		LD_C_DIRECT			TMP1
		CALL_CINDIRECT
		BRANCH				Monitor

NotFound:
		BLINK_OPRERR		1
		RET
}

Query_IMU_Accel:
{
	IMU_READ_DIRECT		ImuGyroX

	LD_A_DIRECT			ImuAccX
	ENCODE_A_TO_DEC
	MOV_A_R1

	LD_A_DIRECT			ImuAccY
	ENCODE_A_TO_DEC
	MOV_A_R2

	LD_A_DIRECT			ImuAccZ
	ENCODE_A_TO_DEC
	MOV_A_R3

	WAIT3
	RET
}

Query_Gyro_Accel:
{
	IMU_READ_DIRECT		ImuGyroX

	LD_A_DIRECT			ImuGyroX
	ENCODE_A_TO_DEC
	MOV_A_R1

	LD_A_DIRECT			ImuGyroY
	ENCODE_A_TO_DEC
	MOV_A_R2

	LD_A_DIRECT			ImuGyroZ
	ENCODE_A_TO_DEC
	MOV_A_R3

	WAIT3
	RET
}

Query_RTC_DateTimeTemp:
{
	RTC_MON_A
	LSHIFT_A_IMM8	12
	MOV_A_B
	RTC_DAY_A
	OR_B_A
	MOV_B_A
	LD_B_IMM32		0xB00A00
	OR_A_B
	MOV_A_R1

	RTC_HH_A
	LSHIFT_A_IMM8	8
	MOV_A_B
	RTC_MM_A
	OR_B_A
	MOV_B_A
	LD_B_IMM32		0xB00000
	OR_A_B
	MOV_A_R2

	IMU_READ_DIRECT		ImuGyroX

	LD_A_DIRECT			ImuTemp
	ENCODE_A_TO_DEC
	MOV_A_R3

	WAIT5
	WAIT5
	RET
}

Query_AgcInitTime:
{
	RTC_TIMESTAMP_DIRECT	TS1
	TIMESTAMP_DIFF_A		TS1 AGCINIT
	MOV_A_B					// save in B
	AND_A_IMM32				0x0FFF0000
	RSHIFT_A_IMM8			16
	OR_A_IMM32			0xB00000
	MOV_A_R1

	MOV_B_A
	AND_A_IMM32			0xFF00
	RSHIFT_A_IMM8		8
	OR_A_IMM32			0xB00000
	MOV_A_R2

	MOV_B_A
	AND_A_IMM32			0x00FF
	OR_A_IMM32			0xB00000
	MOV_A_R3

	RET
}

Query_TimeFromEvent:
{
}

Query_TimeToEvent:
{
}

Query_RTC_Time:
{
	RTC_HH_A
	LD_B_IMM8		0x3F
	AND_A_B
	LD_B_IMM32		0xB00000
	OR_A_B
	MOV_A_R1
	RTC_MM_A
	OR_A_B
	MOV_A_R2

	RTC_SS_A
	BRANCH_A_EQ_DIRECT	RTC_SEC		skip
	ST_A_DIRECT			RTC_SEC

	CLR_B
	ST_B_DIRECT			RTC_S100
	BRANCH				over

skip:
	LD_B_DIRECT			RTC_S100
	ADD_B_IMM8			10
	ST_B_DIRECT			RTC_S100

over:
	SWAP_A_B
	ENCODE_A_TO_DEC
	SWAP_A_B

	LSHIFT_A_IMM8		8
	OR_A_B
	OR_A_IMM32			0xB00000

	MOV_A_R3
	WAIT1				// wait 1/10 second
	RET
}

Query_RTC_Date:
{
	RTC_YEAR_A
	DECODE_A_FROM_DEC
	LD_B_IMM16		2000
	ADD_A_B
	ENCODE_A_TO_DEC
	MOV_A_R1

	RTC_MON_A
	OR_A_IMM32		0xB00000
	MOV_A_R2

	RTC_DAY_A
	OR_A_IMM32		0xB00000
	MOV_A_R3
	WAIT5
	RET
}

Query_GPS_Time:
{
}

Query_GPS_Date:
{
}

Query_GPS_Coord:
{
}

Query_Orbital_Params:
{
}

Query_MET:
{
}

Query_Major_Mode:
{
}

Query_IMU_1202:
{
		CALL		Query_Gyro_Accel
		RANDOM_A
		BRANCH_A_GT_IMM16		1500	noalrm

		LD_A_IMM32	0xAAAAAA
		MOV_A_R3
		LD_B_IMM32	0xAA1201
		RANDOM_A
		BRANCH_A_GT_IMM16		16000	skip
		INC_B					// 1201 becomes 1202
skip:	SWAP_A_B
		MOV_A_R1
		MOV_A_R2

		LD_A_IMM8		2
		MP3_PLAY_A				// alarm track (track no. = 2)
		LT_PROG_ALRM	1
		INPUT_REQ_PROCEED
		LT_PROG_ALRM	0
		LD_A_IMM32	0xAAAAAA
		MOV_A_R1
		MOV_A_R2
		LD_A_IMM8		1
		MP3_PLAY_A				// silence track (track no. = 1)

noalrm:	RET
}

Query_AudioClipPlaying:
{
		LD_A_DIRECT			CLIP_HAS_PLAYED
		BRANCH_A_NE_IMM8	0	done

		INC_A
		ST_A_DIRECT			CLIP_HAS_PLAYED		// set to 1

		LD_A_DIRECT			SELCLIP
		MOV_A_B
		ENCODE_A_TO_DEC
		MOV_A_R1

		LD_A_DIRECT			ADJFACT
		ENCODE_A_TO_DEC
		MOV_A_R2

		LD_A_IMM32			0xAAAAAA
		MOV_A_R3

		MOV_B_A
		MP3_PLAY_A

done:	RET
}

VERB_09:
{
	EMPTY_STACK

	// check noun
	MOV_NOUN_A
	DECODE_A_FROM_DEC
	BRANCH_A_LT_IMM8	1	error
	BRANCH_A_GT_IMM8	4	error
	DEC_A
	LSHIFT_A_IMM8		1			// A=A*2
	LD_B_IMM16			Add
	ADD_A_B
	ST_A_DIRECT			TMP3

	BLINK_R1		1
	BLINK_R2		1
	LD_A_IMM32		0xAAAAAA
	MOV_A_R1
	MOV_A_R2
	MOV_A_R3

	INPUT_R1
	BRANCH_A_LT_IMM8	0		error
	DECODE_A_FROM_DEC
	ST_A_DIRECT		TMP1
	BLINK_R1		0
	ENCODE_A_TO_DEC
	MOV_A_R1

	INPUT_R2
	BRANCH_A_LT_IMM8	0		error
	DECODE_A_FROM_DEC
	ST_A_DIRECT		TMP2
	BLINK_R2		0
	ENCODE_A_TO_DEC
	MOV_A_R2

	LD_A_DIRECT		TMP1
	LD_B_DIRECT		TMP2
	LD_C_DIRECT		TMP3
	CALL_CINDIRECT

	ENCODE_A_TO_DEC
	MOV_A_R3

	BRANCH		done

error:
	BLINK_OPRERR	1
done:
	BRANCH done

Add:	ADD_A_B		// N01
		RET
Mul:	MUL_A_B		// N02
		RET
Div:	DIV_A_B		// N03
		RET
Mod:	MOD_A_B		// N04
		RET
}

// Decimal to Octal conversion N01
// Octal to Decimal conversion N02
//
VERB_10:
{
		EMPTY_STACK
		BLINK_R1		1
		LD_A_IMM32		0xAAAAAA
		MOV_A_R1
		MOV_A_R2
		MOV_A_R3

		MOV_NOUN_A
		DECODE_A_FROM_DEC
		BRANCH_A_EQ_IMM8	1	DecimalToOctal
		BRANCH_A_EQ_IMM8	2	OctalToDecimal
		BRANCH				error

DecimalToOctal:
		INPUT_R1
		BLINK_R1			0
		BRANCH_A_LT_IMM8	0		error
		DECODE_A_FROM_DEC
		MOV_A_B
		ENCODE_A_TO_DEC
		MOV_A_R1
		MOV_B_A

		LD_C_IMM8			0
		BRANCH_A_GT_IMM8	0		notneg	// check negative
		NEG_A								// A = -A
		LD_C_IMM8			1
notneg:
		ENCODE_A_TO_OCT

		MOV_C_B
		BRANCH_B_NE_IMM8	1		over
		LD_B_IMM32			0x0fffff
		AND_A_B
		LD_B_IMM32			0xC00000
		OR_A_B
over:
		MOV_A_R2
		BRANCH				done

OctalToDecimal:
		INPUT_R1_OCT
		BLINK_R1			0
		BRANCH_A_LT_IMM8	0		error
		DECODE_A_FROM_OCT
		MOV_A_B
		ENCODE_A_TO_OCT
		MOV_A_R1
		MOV_B_A
		ENCODE_A_TO_DEC
		MOV_A_R2
		BRANCH				done

error:
		BLINK_OPRERR		1

done:
		BRANCH	done
}

//
// Enter R1 which is a clip number.	V21 N98
//
VERB_21:
{
		MOV_NOUN_A
		BRANCH_A_EQ_IMM16	0x98	skip
		BRANCH_A_EQ_IMM16	0x02	VERB_21_N02
		BRANCH				error

skip:
		LD_A_IMM32		0xAAAAAA
		MOV_A_R2
		MOV_A_R3
		LD_A_DIRECT		SELCLIP
		ENCODE_A_TO_DEC
		MOV_A_R1
		BLINK_R1		1
		INPUT_R1
		BLINK_R1		0
		BRANCH_A_LT_IMM8	0	error
		DECODE_A_FROM_DEC
		MOV_A_B
		ENCODE_A_TO_DEC
		MOV_A_R1
		MOV_B_A
		BRANCH_A_LT_IMM8	1	error		// valid track numbers 1 through 17
		BRANCH_A_GT_IMM8	17	error		// update this code to reflect new SD card

		ST_A_DIRECT			SELCLIP

		LD_A_IMM8			0x16
		MOV_A_VERB
		GOTO				VERB_16

error:
		BLINK_OPRERR	1
x:		BRANCH			x
}

//
// Write memory. Enter octal address into R3
// Then enter octal value 0-377 into R1.
//
VERB_21_N02:
{
		EMPTY_STACK

		CALL			EnterAddressR3
		BRANCH_B_LT_IMM8	0		error

		INPUT_R1_OCT
		BRANCH_A_LT_IMM8	0		error
		DECODE_A_FROM_OCT
		MOV_A_C
		ENCODE_A_TO_OCT
		MOV_A_R1

		MOV_C_A
		BRANCH_A_GT_IMM16	0xff	error		// Error if octal value was outside range 0-377

		BRANCH_B_LE_IMM16	2047	eeprom		// EEPROM memory	(0000 - 3777 octal)
		BRANCH				rtc					// RTC memory		(4000 - 4067 octal)

eeprom:
		MOV_B_C
		EEPROM_WRITE_A_CINDIRECT
		BRANCH				done
rtc:
		PUSH_A
		LD_A_IMM16			2040				// adjust address 2048-8
		SUB_B_A
		POP_A
		MOV_B_C
		RTC_A_MEM_CINDIRECT
		BRANCH			done

error:	BLINK_OPRERR	1

done:	BLINK_VERB		0
		BLINK_NOUN		0
x:		BRANCH			x
}

//
// Enter R2 which is an index adjustment factor. V21 N98
//
VERB_22:
{
		MOV_NOUN_A
		BRANCH_A_NE_IMM16	0x98	error

		LD_A_IMM32		0xAAAAAA
		MOV_A_R1
		MOV_A_R3
		LD_A_DIRECT		ADJFACT
		ENCODE_A_TO_DEC
		MOV_A_R2
		BLINK_R2		1
		INPUT_R2
		BLINK_R2		0
		BRANCH_A_LT_IMM8	0	error
		DECODE_A_FROM_DEC
		MOV_A_B
		ENCODE_A_TO_DEC
		MOV_A_R2
		MOV_B_A
		BRANCH_A_LT_IMM8	0	error		// valid adjustment factor is 0 through 16
		BRANCH_A_GT_IMM8	16	error		// update this code to reflect new SD card

		ST_A_DIRECT			ADJFACT

		LD_A_IMM8			0x16
		MOV_A_VERB
		GOTO				VERB_16

error:
		BLINK_OPRERR	1
x:		BRANCH			x
}

//
// V25 N36		Set RTC Time (HH, MM, SS)
// V25 N37		Set RTC Date (YYYY, MM, DD)
//
// TMP1		address of populate function
// TMP2		address of set function
//
VERB_25:
{
		MOV_NOUN_A
		BRANCH_A_EQ_IMM8	0x36	ptime
		BRANCH_A_EQ_IMM8	0x37	pdate
		BRANCH				error

ptime:	CALL				PopulateTime
		BRANCH				over
pdate:	CALL				PopulateDate

over:
		BLINK_R1		1
		INPUT_R1
		BRANCH_A_LT_IMM8	0		error
		BLINK_R1		0
		DECODE_A_FROM_DEC
		MOV_A_B
		ENCODE_A_TO_DEC
		MOV_A_R1

		MOV_NOUN_A
		BRANCH_A_EQ_IMM8	0x36	CheckHH
		BRANCH_A_EQ_IMM8	0x37	CheckYYYY

doR2:
		BLINK_R2		1
		INPUT_R2
		BRANCH_A_LT_IMM8	0		error
		BLINK_R2		0
		DECODE_A_FROM_DEC
		MOV_A_B
		ENCODE_A_TO_DEC
		MOV_A_R2

		MOV_NOUN_A
		BRANCH_A_EQ_IMM8	0x36	CheckMM
		BRANCH_A_EQ_IMM8	0x37	CheckMON

doR3:
		BLINK_R3		1
		INPUT_R3
		BRANCH_A_LT_IMM8	0		error
		BLINK_R3		0
		DECODE_A_FROM_DEC
		MOV_A_B
		ENCODE_A_TO_DEC
		MOV_A_R3

		MOV_NOUN_A
		BRANCH_A_EQ_IMM8	0x36	CheckSS
		BRANCH_A_EQ_IMM8	0x37	CheckDAY

set:
		MOV_NOUN_A
		BRANCH_A_EQ_IMM8	0x36	stime
		BRANCH_A_EQ_IMM8	0x37	sdate

stime:	CALL			SetTime
		GOTO			success

sdate:	CALL			SetDate
		GOTO			success

error:
		BLINK_OPRERR	1
x:		BRANCH			x

CheckHH:
		BRANCH_B_LT_IMM8	0	error
		BRANCH_B_GT_IMM8	23	error
		BRANCH				doR2
CheckMM:
		BRANCH_B_LT_IMM8	0	error
		BRANCH_B_GT_IMM8	59	error
		BRANCH				doR3
CheckSS:
		BRANCH_B_LT_IMM8	0	error
		BRANCH_B_GT_IMM8	59	error
		GOTO				set
CheckYYYY:
		BRANCH_B_LT_IMM16	2000	error
		BRANCH_B_GT_IMM16	2099	error
		BRANCH				doR2
CheckMON:
		BRANCH_B_LT_IMM8	0	error
		BRANCH_B_GT_IMM8	12	error
		BRANCH				doR3
CheckDAY:
		BRANCH_B_LT_IMM8	1	error
		BRANCH_B_GT_IMM8	31	error
		GOTO				set

PopulateTime:
		RTC_HH_A
		OR_A_IMM32		0xB00000
		MOV_A_R1
		RTC_MM_A
		OR_A_IMM32		0xB00000
		MOV_A_R2
		RTC_SS_A
		OR_A_IMM32		0xB00000
		MOV_A_R3
		RET

PopulateDate:
		RTC_YEAR_A
		OR_A_IMM32		0xB02000
		MOV_A_R1
		RTC_MON_A
		OR_A_IMM32		0xB00000
		MOV_A_R2
		RTC_DAY_A
		OR_A_IMM32		0xB00000
		MOV_A_R3
		RET

SetDate:
		LD_C_IMM8			4		// 4=day 5=month 6=year
		MOV_R3_A					// day
		RTC_A_MEM_CINDIRECT
		INC_C

		MOV_R2_A					// month
		RTC_A_MEM_CINDIRECT
		INC_C

		MOV_R1_A					// year
		AND_A_IMM32			0xFF
		RTC_A_MEM_CINDIRECT
		RET

SetTime:
		// clear seconds to avoid incrementing minutes
		// in the middle of the set operation
		CLR_C
		SWAP_A_B
		CLR_A
		RTC_A_MEM_CINDIRECT			// seconds
		SWAP_A_B

		LD_C_IMM8			2		// 2=hours 1=minutes 0=seconds
		MOV_R1_A					// hours
		RTC_A_MEM_CINDIRECT
		DEC_C

		MOV_R2_A					// minutes
		RTC_A_MEM_CINDIRECT
		DEC_C

		MOV_R3_A					// second (do seconds last for more accuracy!)
		RTC_A_MEM_CINDIRECT
		RET

		// jump to the corresponding V16 N36 (time), V16 N37 (date)
success:
		EMPTY_STACK
		LD_A_IMM8		0x16
		MOV_A_VERB
		GOTO			VERB_16
}

ASM_END:
