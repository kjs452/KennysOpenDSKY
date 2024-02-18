
//////////////////////////////////////////////////////////////////////
//
// This file automatically generated by 'assembler.py'
// Input File: 'KennysOpenDSKY/kennysagc.asm'
// Command: $ ./assembler.py KennysOpenDSKY/kennysagc.asm
//
// This is a C source code header file which defines an enum and a program array.
// The enum defines global labels and defined symbols.
// The program array is called 'Program[]', it contains the byte codes that
// the Apollo Guidance Computer virtual machine will execute.
//
//

enum ASM_SYMBOLS {
	TMP1 = 0,    // DEFINE
	TMP2 = 1,    // DEFINE
	TMP3 = 2,    // DEFINE
	TMP4 = 3,    // DEFINE
	V01_R1 = 5,    // DEFINE
	V01_R2 = 6,    // DEFINE
	V01_R3 = 7,    // DEFINE
	RTC_SEC = 8,    // DEFINE
	RTC_S100 = 9,    // DEFINE
	AGCINIT = 10,    // DEFINE
	TS1 = 12,    // DEFINE
	PROG_TMP1 = 14,    // DEFINE
	LBL_MAIN = 0,    // LABEL
	LBL_VERB_37 = 2,    // LABEL
	LBL_VERB_35 = 80,    // LABEL
	LBL_VERB_01 = 158,    // LABEL
	LBL_VERB_02 = 218,    // LABEL
	LBL_VERB_03 = 258,    // LABEL
	LBL_VERB_04 = 306,    // LABEL
	LBL_VERB_05 = 330,    // LABEL
	LBL_VERB_36 = 346,    // LABEL
	LBL_VERB_69 = 460,    // LABEL
	LBL_PROG_00 = 465,    // LABEL
	LBL_PROG_01 = 465,    // LABEL
	LBL_PROG_02 = 465,    // LABEL
	LBL_PROG_60 = 465,    // LABEL
	LBL_PROG_61 = 468,    // LABEL
	LBL_PROG_62 = 473,    // LABEL
	LBL_PROG_68 = 478,    // LABEL
	LBL_PROG_69 = 483,    // LABEL
	LBL_PROG_70 = 488,    // LABEL
	LBL_PROG_42 = 493,    // LABEL
	LBL_Help = 584,    // LABEL
	LBL_Help2 = 590,    // LABEL
	LBL_Help3 = 599,    // LABEL
	LBL_VERB_16 = 605,    // LABEL
	LBL_Query_IMU_Accel = 691,    // LABEL
	LBL_Query_Gyro_Accel = 707,    // LABEL
	LBL_Query_RTC_DateTimeTemp = 707,    // LABEL
	LBL_Query_AgcInitTime = 741,    // LABEL
	LBL_Query_TimeFromEvent = 787,    // LABEL
	LBL_Query_TimeToEvent = 787,    // LABEL
	LBL_Query_RTC_Time = 787,    // LABEL
	LBL_Query_RTC_Date = 832,    // LABEL
	LBL_Query_GPS_Time = 856,    // LABEL
	LBL_Query_GPS_Date = 856,    // LABEL
	LBL_Query_GPS_Coord = 856,    // LABEL
	LBL_Query_Orbital_Params = 856,    // LABEL
	LBL_Query_MET = 856,    // LABEL
	LBL_Query_Major_Mode = 856,    // LABEL
	LBL_Query_IMU_1202 = 856,    // LABEL
	LBL_Query_AudioClipPlaying = 856,    // LABEL
	LBL_ASM_END = 856,    // LABEL
};

static const uint8_t Program[] PROGMEM = {
    // MAIN:
    // {
        BRANCH, 0xFE /* MAIN=-2 */, 
    // }
    //
    // Run major mode
    //
    // VERB_37:
    // {
        LD_A_IMM8, 0xAA, 
        MOV_A_NOUN, 
        BLINK_VERB, 0x01, 
        BLINK_NOUN, 0x01, 
        INPUT_NOUN, 
        LD_B_IMM16, 0xFF, 0x00, 
        AND_A_B, 
        MOV_A_B, 
        BRANCH_B_LT_IMM8, 0x00, 0x37 /* Error=+55 */, 
        LD_C_IMM16, 0x24, 0x00 /* Programs=36 */, 
    // Next:
        PROG8_A_INDIRECT_C, 
        BRANCH_A_EQ_IMM8, 0xFF, 0x30 /* NotFound=+48 */, 
        INC_C, 
        ST_A_DIRECT, PROG_TMP1, 
        PROG16_A_INDIRECT_C, 
        ADD_C_IMM8, 0x02, 
        BRANCH_B_EQ_DIRECT, PROG_TMP1, 0x21 /* Found=+33 */, 
        BRANCH, 0xF1 /* Next=-15 */, 
    // Programs:
        0x00, 
        0xD1, 0x01 /* PROG_00=465 */, 
        0x01, 
        0xD1, 0x01 /* PROG_01=465 */, 
        0x02, 
        0xD1, 0x01 /* PROG_02=465 */, 
        0x60, 
        0xD1, 0x01 /* PROG_60=465 */, 
        0x61, 
        0xD4, 0x01 /* PROG_61=468 */, 
        0x62, 
        0xD9, 0x01 /* PROG_62=473 */, 
        0x68, 
        0xDE, 0x01 /* PROG_68=478 */, 
        0x69, 
        0xE3, 0x01 /* PROG_69=483 */, 
        0x70, 
        0xE8, 0x01 /* PROG_70=488 */, 
        0x42, 
        0xED, 0x01 /* PROG_42=493 */, 
        0xFF, 
    // Found:
        SWAP_A_B, 
    // ENCODE_A_TO_DEC
        MOV_A_PROG, 
        SWAP_A_B, 
        RUN_PROG_A, 
        BRANCH, 0x02 /* Done=+2 */, 
    // NotFound:
    // Error:
        BLINK_OPRERR, 0x01, 
    // Done:
        BLINK_VERB, 0x00, 
        BLINK_NOUN, 0x00, 
        RET, 
    // }
    // LAMP TEST
    // VERB_35:
    // {
        PUSH_A, 
        PUSH_B, 
        PUSH_C, 
        PUSH_DSKY, 
    // Do lights
        LT_UPLINK_ACTY, 0x01, 
        LT_TEMP, 0x01, 
        WAIT1, 
        LT_NO_ATT, 0x01, 
        LT_GIMBAL_LOCK, 0x01, 
        WAIT1, 
        LT_NO_ATT, 0x01, 
        LT_GIMBAL_LOCK, 0x01, 
        WAIT1, 
        LT_STBY, 0x01, 
        LT_PROG_ALRM, 0x01, 
        WAIT1, 
        LT_KEY_REL, 0x01, 
        LT_RESTART, 0x01, 
        WAIT1, 
        LT_OPR_ERR, 0x01, 
        LT_TRACKER, 0x01, 
        WAIT1, 
        LT_NA1, 0x01, 
        LT_ALT, 0x01, 
        WAIT1, 
        LT_NA2, 0x01, 
        LT_VEL, 0x01, 
        WAIT1, 
    // do digits
        LD_A_IMM32, 0x88, 0x88, 0xB8, 0x00, 
        MOV_A_PROG, 
        WAIT1, 
        MOV_A_VERB, 
        MOV_A_NOUN, 
        WAIT1, 
        MOV_A_R1, 
        WAIT1, 
        MOV_A_R2, 
        WAIT1, 
        MOV_A_R3, 
        WAIT2, 
        BLINK_VERB, 0x01, 
        BLINK_NOUN, 0x01, 
        BLINK_PROG, 0x01, 
        BLINK_KEYREL, 0x01, 
        BLINK_OPRERR, 0x01, 
        WAIT5, 
        WAIT5, 
        WAIT5, 
    // restore state
        POP_DSKY, 
        POP_C, 
        POP_B, 
        POP_A, 
        RET, 
    // }
    // VERB_01:
    // {
        EMPTY_STACK, 
        CLR_A, 
        ST_A_DIRECT, V01_R1, 
        ST_A_DIRECT, V01_R2, 
        LD_A_IMM8, 0xEC, 
        ST_A_DIRECT, V01_R3, 
    // LOOP:
        WAIT3, 
        LD_A_DIRECT, V01_R1, 
        BRANCH_A_LE_IMM16, 0x10, 0x27, 0x03 /* L01=+3 */, 
        CLR_A, 
        BRANCH, 0x01 /* OVER1=+1 */, 
    // L01:
        INC_A, 
    // OVER1:
        ST_A_DIRECT, V01_R1, 
        ENCODE_A_TO_DEC, 
        MOV_A_R1, 
        BRANCH_NOT_TIMER4, 0xEF /* LOOP=-17 */, 
        LD_A_DIRECT, V01_R2, 
        BRANCH_A_LE_IMM16, 0xB8, 0x0B, 0x03 /* L02=+3 */, 
        CLR_A, 
        BRANCH, 0x01 /* OVER2=+1 */, 
    // L02:
        INC_A, 
    // OVER2:
        ST_A_DIRECT, V01_R2, 
        ENCODE_A_TO_DEC, 
        MOV_A_R2, 
        BRANCH_NOT_TIMER5, 0xDF /* LOOP=-33 */, 
        LD_A_DIRECT, V01_R3, 
        BRANCH_A_LE_IMM16, 0xB8, 0x0B, 0x03 /* L03=+3 */, 
        CLR_A, 
        BRANCH, 0x01 /* OVER3=+1 */, 
    // L03:
        INC_A, 
    // OVER3:
        ST_A_DIRECT, V01_R3, 
        ENCODE_A_TO_DEC, 
        MOV_A_R3, 
        GOTO, 0xA8, 0x00 /* LOOP=168 */, 
    // }
    // VERB_02:
    // {
        LD_A_IMM32, 0xC7, 0xCF, 0xFF, 0xFF, 
        ENCODE_A_TO_DEC, 
        MOV_A_R1, 
        BLINK_R2, 0x01, 
        LD_A_IMM32, 0xAA, 0xAA, 0xAA, 0x00, 
        MOV_A_R2, 
        LD_A_IMM32, 0x13, 0x83, 0x01, 0x00, 
        ENCODE_A_TO_DEC, 
        MOV_A_R3, 
        INPUT_R2, 
        BRANCH_A_LT_IMM8, 0x00, 0x05 /* bad=+5 */, 
        DECODE_A_FROM_DEC, 
    //		ENCODE_A_TO_DEC
        ENCODE_A_TO_UDEC, 
        GOTO, 0xFE, 0x00 /* good=254 */, 
    // bad:
        LD_A_IMM32, 0xAA, 0xAA, 0xAA, 0x00, 
    // good:
        MOV_A_R2, 
        BLINK_R2, 0x00, 
        RET, 
    // }
    // VERB_03:
    // {
        UPLINK_PROB_IMM8, 0x80, 
        COMPACTY_PROB_IMM8, 0x80, 
        LD_A_IMM32, 0xAA, 0xAA, 0xAA, 0x00, 
        ST_A_DIRECT, TMP1, 
        LD_A_IMM32, 0x1A, 0x20, 0xA1, 0x00, 
        MOV_A_R1, 
        LD_A_IMM32, 0x7D, 0xDC, 0xFF, 0xFF, 
        ENCODE_A_TO_DEC, 
        MOV_A_R2, 
        BLINK_NOUN, 0x01, 
        BLINK_VERB, 0x01, 
        LD_A_DIRECT, TMP1, 
        MOV_A_R3, 
        INPUT_R3_OCT, 
        BRANCH_A_LT_IMM8, 0x00, 0x05 /* bad=+5 */, 
        DECODE_A_FROM_OCT, 
        ENCODE_A_TO_OCT, 
        GOTO, 0x2C, 0x01 /* good=300 */, 
    // bad:
        LD_A_DIRECT, TMP1, 
    // good:
        MOV_A_R3, 
        BLINK_NOUN, 0x00, 
        BLINK_VERB, 0x00, 
        RET, 
    // }
    // VERB_04:
    // {
        PUSH_A, 
        PUSH_B, 
        PUSH_C, 
        CALL, 0x48, 0x02 /* Help=584 */, 
        ENCODE_A_TO_DEC, 
        MOV_A_R1, 
        MOV_B_A, 
        ENCODE_A_TO_DEC, 
        MOV_A_R2, 
        MOV_C_A, 
        ENCODE_A_TO_DEC, 
        MOV_A_R3, 
        POP_C, 
        POP_B, 
        POP_A, 
        WAIT4, 
        WAIT4, 
        WAIT4, 
        WAIT4, 
        WAIT4, 
        WAIT4, 
        RET, 
    // }
    //
    // This verb clears the foreground task and clears the DSKY
    //
    // VERB_05:
    // {
        EMPTY_STACK, 
        LD_A_IMM32, 0x00, 0x00, 0xAA, 0xAA, 
        PUSH_A, 
        LD_A_IMM32, 0xAA, 0xAA, 0xAA, 0x00, 
        MOV_A_R1, 
        MOV_A_R2, 
        MOV_A_R3, 
        RET, 
    // }
    //
    // Fresh Start
    //
    // VERB_36:
    // {
        EMPTY_STACK, 
        LD_A_IMM32, 0xAA, 0xAA, 0xAA, 0x00, 
        MOV_A_R1, 
        MOV_A_R2, 
        MOV_A_R3, 
        MOV_A_VERB, 
        MOV_A_NOUN, 
        MOV_A_PROG, 
        LT_ALL, 0x00, 
        UPLINK_PROB_IMM8, 0x00, 
        COMPACTY_PROB_IMM8, 0x00, 
        LD_A_IMM8, 0x04, 
    // loop:
        LT_RESTART, 0x01, 
        WAIT4, 
        LT_RESTART, 0x00, 
        WAIT4, 
        DEC_A, 
        BRANCH_A_GT_IMM8, 0x00, 0xF6 /* loop=-10 */, 
    // done:
        LT_NOUN, 0x01, 
        WAIT2, 
        LT_PROG, 0x01, 
        WAIT2, 
        LT_VERB, 0x01, 
        WAIT2, 
        UPLINK_PROB_IMM8, 0xFF, 
        LT_TEMP, 0x01, 
        WAIT2, 
        LT_NO_ATT, 0x01, 
        LT_GIMBAL_LOCK, 0x01, 
        WAIT2, 
        LT_STBY, 0x01, 
        LT_PROG_ALRM, 0x01, 
        WAIT2, 
        LT_KEY_REL, 0x01, 
        LT_RESTART, 0x01, 
        WAIT2, 
        LT_OPR_ERR, 0x01, 
        LT_TRACKER, 0x01, 
        WAIT2, 
        LT_NA1, 0x01, 
        LT_ALT, 0x01, 
        WAIT2, 
        LT_NA2, 0x01, 
        LT_VEL, 0x01, 
        WAIT5, 
        LD_A_IMM32, 0x88, 0x88, 0xB8, 0x00, 
        CALL, 0xC5, 0x01 /* doleds=453 */, 
        WAIT5, 
        WAIT5, 
        WAIT5, 
        LT_ALL, 0x00, 
        LT_NOUN, 0x01, 
        LT_PROG, 0x01, 
        LT_VERB, 0x01, 
        UPLINK_PROB_IMM8, 0x00, 
        LT_NO_ATT, 0x01, 
        LD_A_IMM32, 0xAA, 0xAA, 0xAA, 0x00, 
        CALL, 0xC5, 0x01 /* doleds=453 */, 
    // forever:
        BRANCH, 0xFE /* forever=-2 */, 
    // doleds:
        MOV_A_R1, 
        MOV_A_R2, 
        MOV_A_R3, 
        MOV_A_NOUN, 
        MOV_A_VERB, 
        MOV_A_PROG, 
        RET, 
    // }
    //
    // Force restart (clears the Agc time since power up)
    //
    // VERB_69:
    // {
        RTC_TIMESTAMP_DIRECT, AGCINIT, 
        CALL, 0x5A, 0x01 /* VERB_36=346 */, 
    // }
    // PROG_00:
    // PROG_01:
    // PROG_02:
    // PROG_60:
    // {
        GOTO, 0xD1, 0x01 /* PROG_00=465 */, 
    // }
    // P61 playback JFK i believe
    // PROG_61:
    // {
        LD_A_IMM8, 0x03, 
        MP3_PLAY_A, 
    // f:
        BRANCH, 0xFE /* f=-2 */, 
    // }
    // P62 playback JFK We choose
    // PROG_62:
    // {
        LD_A_IMM8, 0x04, 
        MP3_PLAY_A, 
    // f:
        BRANCH, 0xFE /* f=-2 */, 
    // }
    // P68 playback A8 Genesis
    // PROG_68:
    // {
        LD_A_IMM8, 0x05, 
        MP3_PLAY_A, 
    // f:
        BRANCH, 0xFE /* f=-2 */, 
    // }
    // P69 playback A11 eagle has landed
    // PROG_69:
    // {
        LD_A_IMM8, 0x06, 
        MP3_PLAY_A, 
    // f:
        BRANCH, 0xFE /* f=-2 */, 
    // }
    // P70 playback A11 we have a problem
    // PROG_70:
    // {
        LD_A_IMM8, 0x11, 
        MP3_PLAY_A, 
    // f:
        BRANCH, 0xFE /* f=-2 */, 
    // }
    //
    // Blinky: Randomly light caution and warning lamps
    //
    // PROG_42:
    // {
        RANDOM_A, 
        LD_B_IMM8, 0x03, 
        MOD_A_B, 
    // waiting:
        WAIT4, 
        DEC_A, 
        BRANCH_A_GE_IMM8, 0x00, 0xFB /* waiting=-5 */, 
    // change lamps
        RANDOM_A, 
        LD_B_IMM8, 0x16, 
        MOD_A_B, 
        LD_B_IMM8, 0x03, 
        MUL_A_B, 
        LD_B_IMM16, 0x06, 0x02 /* LampList=518 */, 
        ADD_B_A, 
        MOV_B_C, 
        CALL_CINDIRECT, 
        GOTO, 0xED, 0x01 /* PROG_42=493 */, 
    // LampList:
    // On_NOATT:
        LT_NO_ATT, 0x01, 
        RET, 
    // On_STBY:
        LT_STBY, 0x01, 
        RET, 
    // On_NA1:
        LT_NA1, 0x01, 
        RET, 
    // On_NA2:
        LT_NA2, 0x01, 
        RET, 
    // On_TEMP:
        LT_TEMP, 0x01, 
        RET, 
    // On_GIMBAL_LOCK:
        LT_GIMBAL_LOCK, 0x01, 
        RET, 
    // On_PROG_ALRM:
        LT_PROG_ALRM, 0x01, 
        RET, 
    // On_RESTART:
        LT_RESTART, 0x01, 
        RET, 
    // On_TRACKER:
        LT_TRACKER, 0x01, 
        RET, 
    // On_ALT:
        LT_ALT, 0x01, 
        RET, 
    // On_VEL:
        LT_VEL, 0x01, 
        RET, 
    // Off_NOATT:
        LT_NO_ATT, 0x00, 
        RET, 
    // Off_STBY:
        LT_STBY, 0x00, 
        RET, 
    // Off_NA1:
        LT_NA1, 0x00, 
        RET, 
    // Off_NA2:
        LT_NA2, 0x00, 
        RET, 
    // Off_TEMP:
        LT_TEMP, 0x00, 
        RET, 
    // Off_GIMBAL_LOCK:
        LT_GIMBAL_LOCK, 0x00, 
        RET, 
    // Off_PROG_ALRM:
        LT_PROG_ALRM, 0x00, 
        RET, 
    // Off_RESTART:
        LT_RESTART, 0x00, 
        RET, 
    // Off_TRACKER:
        LT_TRACKER, 0x00, 
        RET, 
    // Off_ALT:
        LT_ALT, 0x00, 
        RET, 
    // Off_VEL:
        LT_VEL, 0x00, 
        RET, 
    // }
    // Help:
    // {
        LD_A_IMM8, 0x17, 
        CALL, 0x4E, 0x02 /* Help2=590 */, 
        RET, 
    // }
    // Help2:
    // {
        LD_B_IMM32, 0x18, 0x02, 0xFF, 0xFF, 
        CALL, 0x57, 0x02 /* Help3=599 */, 
        RET, 
    // }
    // Help3:
    // {
        LD_C_IMM32, 0xFE, 0x7E, 0x00, 0x00, 
        RET, 
    // }
    //
    // VERB 16
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
    // VERB_16:
    // {
        EMPTY_STACK, 
        UPLINK_PROB_IMM8, 0x0A, 
        COMPACTY_PROB_IMM8, 0x0A, 
        MOV_NOUN_A, 
        MOV_A_B, 
        LD_C_IMM16, 0x76, 0x02 /* NounTable=630 */, 
    // Next:
        PROG8_A_INDIRECT_C, 
        BRANCH_A_EQ_IMM8, 0xFF, 0x45 /* NotFound=+69 */, 
        INC_C, 
        ST_A_DIRECT, TMP1, 
        PROG16_A_INDIRECT_C, 
        BRANCH_B_EQ_DIRECT, TMP1, 0x35 /* Found=+53 */, 
        ADD_C_IMM8, 0x02, 
        BRANCH, 0xF1 /* Next=-15 */, 
    // NounTable:
        0x17, 
        0xB3, 0x02 /* Query_IMU_Accel=691 */, 
        0x18, 
        0xC3, 0x02 /* Query_Gyro_Accel=707 */, 
        0x19, 
        0xC3, 0x02 /* Query_RTC_DateTimeTemp=707 */, 
        0x31, 
        0xE5, 0x02 /* Query_AgcInitTime=741 */, 
        0x34, 
        0x13, 0x03 /* Query_TimeFromEvent=787 */, 
        0x35, 
        0x13, 0x03 /* Query_TimeToEvent=787 */, 
        0x36, 
        0x13, 0x03 /* Query_RTC_Time=787 */, 
        0x37, 
        0x40, 0x03 /* Query_RTC_Date=832 */, 
        0x38, 
        0x58, 0x03 /* Query_GPS_Time=856 */, 
        0x39, 
        0x58, 0x03 /* Query_GPS_Date=856 */, 
        0x43, 
        0x58, 0x03 /* Query_GPS_Coord=856 */, 
        0x44, 
        0x58, 0x03 /* Query_Orbital_Params=856 */, 
        0x65, 
        0x58, 0x03 /* Query_MET=856 */, 
        0x68, 
        0x58, 0x03 /* Query_Major_Mode=856 */, 
        0x87, 
        0x58, 0x03 /* Query_IMU_1202=856 */, 
        0x98, 
        0x58, 0x03 /* Query_AudioClipPlaying=856 */, 
        0xFF, 
    // Found:
        MOV_A_C, 
        ST_C_DIRECT, TMP1, 
    // Forever:
        LD_C_DIRECT, TMP1, 
        CALL_CINDIRECT, 
        GOTO, 0xAA, 0x02 /* Forever=682 */, 
    // NotFound:
        BLINK_OPRERR, 0x01, 
        RET, 
    // }
    // Query_IMU_Accel:
    // {
        LD_B_IMM16, 0xF4, 0xFF, 
        MOV_B_A, 
        ENCODE_A_TO_DEC, 
        MOV_A_R1, 
        INC_B, 
        MOV_B_A, 
        ENCODE_A_TO_DEC, 
        MOV_A_R2, 
        INC_B, 
        INC_B, 
        MOV_B_A, 
        ENCODE_A_TO_DEC, 
        MOV_A_R3, 
        RET, 
    // }
    // Query_Gyro_Accel:
    // {
    // }
    // Query_RTC_DateTimeTemp:
    // {
        RTC_MON_A, 
        LSHIFT_A_IMM8, 0x0C, 
        MOV_A_B, 
        RTC_DAY_A, 
        OR_B_A, 
        MOV_B_A, 
        LD_B_IMM32, 0x00, 0x0A, 0xB0, 0x00, 
        OR_A_B, 
        MOV_A_R1, 
        RTC_HH_A, 
        LSHIFT_A_IMM8, 0x08, 
        MOV_A_B, 
        RTC_MM_A, 
        OR_B_A, 
        MOV_B_A, 
        LD_B_IMM32, 0x00, 0x00, 0xB0, 0x00, 
        OR_A_B, 
        MOV_A_R2, 
        LD_A_IMM8, 0x46, 
        ENCODE_A_TO_DEC, 
        MOV_A_R3, 
        WAIT5, 
        RET, 
    // }
    // Query_AgcInitTime:
    // {
        RTC_TIMESTAMP_DIRECT, TS1, 
        TIMESTAMP_DIFF_A, TS1, AGCINIT, 
        MOV_A_B, 
        AND_A_IMM32, 0x00, 0x00, 0xFF, 0x0F, 
        RSHIFT_A_IMM8, 0x10, 
        OR_A_IMM32, 0x00, 0x00, 0xB0, 0x00, 
        MOV_A_R1, 
        MOV_B_A, 
        AND_A_IMM32, 0x00, 0xFF, 0x00, 0x00, 
        RSHIFT_A_IMM8, 0x08, 
        OR_A_IMM32, 0x00, 0x00, 0xB0, 0x00, 
        MOV_A_R2, 
        MOV_B_A, 
        AND_A_IMM32, 0xFF, 0x00, 0x00, 0x00, 
        OR_A_IMM32, 0x00, 0x00, 0xB0, 0x00, 
        MOV_A_R3, 
        RET, 
    // }
    // Query_TimeFromEvent:
    // {
    // }
    // Query_TimeToEvent:
    // {
    // }
    // Query_RTC_Time:
    // {
        RTC_HH_A, 
        LD_B_IMM8, 0x3F, 
        AND_A_B, 
        LD_B_IMM32, 0x00, 0x00, 0xB0, 0x00, 
        OR_A_B, 
        MOV_A_R1, 
        RTC_MM_A, 
        OR_A_B, 
        MOV_A_R2, 
        RTC_SS_A, 
        BRANCH_A_EQ_DIRECT, RTC_SEC, 0x07 /* skip=+7 */, 
        ST_A_DIRECT, RTC_SEC, 
        CLR_B, 
        ST_B_DIRECT, RTC_S100, 
        BRANCH, 0x06 /* over=+6 */, 
    // skip:
        LD_B_DIRECT, RTC_S100, 
        ADD_B_IMM8, 0x0A, 
        ST_B_DIRECT, RTC_S100, 
    // over:
        SWAP_A_B, 
        ENCODE_A_TO_DEC, 
        SWAP_A_B, 
        LSHIFT_A_IMM8, 0x08, 
        OR_A_B, 
        OR_A_IMM32, 0x00, 0x00, 0xB0, 0x00, 
        MOV_A_R3, 
        WAIT1, 
        RET, 
    // }
    // Query_RTC_Date:
    // {
        RTC_YEAR_A, 
        DECODE_A_FROM_DEC, 
        LD_B_IMM16, 0xD0, 0x07, 
        ADD_A_B, 
        ENCODE_A_TO_DEC, 
        MOV_A_R1, 
        RTC_MON_A, 
        OR_A_IMM32, 0x00, 0x00, 0xB0, 0x00, 
        MOV_A_R2, 
        RTC_DAY_A, 
        OR_A_IMM32, 0x00, 0x00, 0xB0, 0x00, 
        MOV_A_R3, 
        WAIT5, 
        RET, 
    // }
    // Query_GPS_Time:
    // {
    // }
    // Query_GPS_Date:
    // {
    // }
    // Query_GPS_Coord:
    // {
    // }
    // Query_Orbital_Params:
    // {
    // }
    // Query_MET:
    // {
    // }
    // Query_Major_Mode:
    // {
    // }
    // Query_IMU_1202:
    // {
    // }
    // Query_AudioClipPlaying:
    // {
    // }
    // ASM_END:
};
