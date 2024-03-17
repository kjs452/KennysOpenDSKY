
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
	CONTMODE = 15,    // DEFINE
	SELCLIP = 16,    // DEFINE
	ADJFACT = 17,    // DEFINE
	CLIP_HAS_PLAYED = 18,    // DEFINE
	ImuGyroX = 19,    // DEFINE
	ImuGyroY = 20,    // DEFINE
	ImuGyroZ = 21,    // DEFINE
	ImuAccX = 22,    // DEFINE
	ImuAccY = 23,    // DEFINE
	ImuAccZ = 24,    // DEFINE
	ImuTemp = 25,    // DEFINE
	LBL_MAIN_CPU0 = 0,    // LABEL
	LBL_MAIN_CPU1 = 3,    // LABEL
	LBL_Init = 6,    // LABEL
	LBL_VERB_37 = 13,    // LABEL
	LBL_VERB_35 = 97,    // LABEL
	LBL_VERB_01 = 175,    // LABEL
	LBL_VERB_02 = 234,    // LABEL
	LBL_VERB_03 = 273,    // LABEL
	LBL_VERB_04 = 320,    // LABEL
	LBL_Help = 344,    // LABEL
	LBL_Help2 = 350,    // LABEL
	LBL_Help3 = 359,    // LABEL
	LBL_VERB_05 = 365,    // LABEL
	LBL_VERB_36 = 385,    // LABEL
	LBL_VERB_69 = 502,    // LABEL
	LBL_PROG_00 = 508,    // LABEL
	LBL_PROG_01 = 508,    // LABEL
	LBL_PROG_02 = 508,    // LABEL
	LBL_PROG_60 = 508,    // LABEL
	LBL_PROG_11 = 510,    // LABEL
	LBL_PROG_61 = 515,    // LABEL
	LBL_PROG_62 = 520,    // LABEL
	LBL_PROG_68 = 525,    // LABEL
	LBL_PROG_69 = 530,    // LABEL
	LBL_PROG_70 = 535,    // LABEL
	LBL_PROG_42 = 540,    // LABEL
	LBL_PROG_06 = 630,    // LABEL
	LBL_VERB_06 = 798,    // LABEL
	LBL_VERB_16 = 803,    // LABEL
	LBL_VERB_06_16 = 809,    // LABEL
	LBL_Query_IMU_Accel = 912,    // LABEL
	LBL_Query_Gyro_Accel = 928,    // LABEL
	LBL_Query_RTC_DateTimeTemp = 944,    // LABEL
	LBL_Query_AgcInitTime = 981,    // LABEL
	LBL_Query_TimeFromEvent = 1027,    // LABEL
	LBL_Query_TimeToEvent = 1027,    // LABEL
	LBL_Query_RTC_Time = 1027,    // LABEL
	LBL_Query_RTC_Date = 1072,    // LABEL
	LBL_Query_GPS_Time = 1096,    // LABEL
	LBL_Query_GPS_Date = 1096,    // LABEL
	LBL_Query_GPS_Coord = 1096,    // LABEL
	LBL_Query_Orbital_Params = 1096,    // LABEL
	LBL_Query_MET = 1096,    // LABEL
	LBL_Query_Major_Mode = 1096,    // LABEL
	LBL_Query_IMU_1202 = 1096,    // LABEL
	LBL_Query_AudioClipPlaying = 1096,    // LABEL
	LBL_VERB_09 = 1122,    // LABEL
	LBL_VERB_10 = 1197,    // LABEL
	LBL_VERB_21 = 1276,    // LABEL
	LBL_VERB_22 = 1323,    // LABEL
	LBL_VERB_25 = 1370,    // LABEL
	LBL_ASM_END = 1602,    // LABEL
};

static const uint8_t Program[] PROGMEM = {
    // general variables for minor mode
    // idle loop for for cpu 0
    // MAIN_CPU0:
    // {
        CALL, 0x06, 0x00 /* Init=6 */, 
    // NOTE: this falls thru to MAIN_CPU1
    // }
    // idle loop for cpu 1	(cpu 0 comes here on VERB_36)
    // MAIN_CPU1:
    // {
        EMPTY_STACK, 
    // loop:
        BRANCH, 0xFE /* loop=-2 */, 
    // }
    //
    // Initialization stuff
    //
    // Init:
    // {
        LD_A_IMM8, 0x01, 
        ST_A_DIRECT, SELCLIP, 
        RTC_TIMESTAMP_DIRECT, AGCINIT, 
        RET, 
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
        BRANCH_A_LT_IMM8, 0x00, 0x42 /* Error=+66 */, 
        LD_B_IMM16, 0xFF, 0x00, 
        AND_A_B, 
        MOV_A_B, 
        LD_C_IMM16, 0x2F, 0x00 /* Programs=47 */, 
    // Next:
        PROG8_A_CINDIRECT, 
        BRANCH_A_EQ_IMM8, 0xFF, 0x36 /* NotFound=+54 */, 
        INC_C, 
        ST_A_DIRECT, PROG_TMP1, 
        PROG16_A_CINDIRECT, 
        ADD_C_IMM8, 0x02, 
        BRANCH_B_EQ_DIRECT, PROG_TMP1, 0x27 /* Found=+39 */, 
        BRANCH, 0xF1 /* Next=-15 */, 
    // Programs:
        0x00, 
        0xFC, 0x01 /* PROG_00=508 */, 
        0x01, 
        0xFC, 0x01 /* PROG_01=508 */, 
        0x02, 
        0xFC, 0x01 /* PROG_02=508 */, 
        0x11, 
        0xFE, 0x01 /* PROG_11=510 */, 
        0x60, 
        0xFC, 0x01 /* PROG_60=508 */, 
        0x61, 
        0x03, 0x02 /* PROG_61=515 */, 
        0x62, 
        0x08, 0x02 /* PROG_62=520 */, 
        0x68, 
        0x0D, 0x02 /* PROG_68=525 */, 
        0x69, 
        0x12, 0x02 /* PROG_69=530 */, 
        0x70, 
        0x17, 0x02 /* PROG_70=535 */, 
        0x42, 
        0x1C, 0x02 /* PROG_42=540 */, 
        0x06, 
        0x76, 0x02 /* PROG_06=630 */, 
        0xFF, 
    // Found:
        SWAP_A_B, 
    // ENCODE_A_TO_DEC
        MOV_A_PROG, 
        SWAP_A_B, 
        RUN_PROG_A, 
        BRANCH, 0x02 /* Done=+2 */, 
    // Error:
    // NotFound:
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
        BRANCH, 0xCF /* LOOP=-49 */, 
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
        BRANCH_A_LT_IMM8, 0x00, 0x04 /* bad=+4 */, 
        DECODE_A_FROM_DEC, 
        ENCODE_A_TO_UDEC, 
        BRANCH, 0x05 /* good=+5 */, 
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
        BRANCH_A_LT_IMM8, 0x00, 0x04 /* bad=+4 */, 
        DECODE_A_FROM_OCT, 
        ENCODE_A_TO_OCT, 
        BRANCH, 0x02 /* good=+2 */, 
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
        CALL, 0x58, 0x01 /* Help=344 */, 
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
    // Help:
    // {
        LD_A_IMM8, 0x17, 
        CALL, 0x5E, 0x01 /* Help2=350 */, 
        RET, 
    // }
    // Help2:
    // {
        LD_B_IMM32, 0x18, 0x02, 0xFF, 0xFF, 
        CALL, 0x67, 0x01 /* Help3=359 */, 
        RET, 
    // }
    // Help3:
    // {
        LD_C_IMM32, 0xFE, 0x7E, 0x00, 0x00, 
        RET, 
    // }
    //
    // This verb clears the foreground task and clears the DSKY
    //
    // VERB_05:
    // {
        EMPTY_STACK, 
        LD_A_IMM32, 0x00, 0x00, 0xAA, 0xAA, 
        LD_B_IMM16, 0x03, 0x00 /* MAIN_CPU1=3 */, 
        OR_A_B, 
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
        LD_A_IMM8, 0x03 /* MAIN_CPU1=3 */, 
        RUN_PROG_A, 
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
        CALL, 0xEF, 0x01 /* doleds=495 */, 
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
        CALL, 0xEF, 0x01 /* doleds=495 */, 
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
        CALL, 0x06, 0x00 /* Init=6 */, 
        CALL, 0x81, 0x01 /* VERB_36=385 */, 
    // }
    // PROG_00:
    // PROG_01:
    // PROG_02:
    // PROG_60:
    // {
        BRANCH, 0xFE /* PROG_00=-2 */, 
    // }
    // PROG_11:
    // {
    // again:
        CALL, 0x90, 0x03 /* Query_IMU_Accel=912 */, 
        BRANCH, 0xFB /* again=-5 */, 
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
        LD_B_IMM16, 0x34, 0x02 /* LampList=564 */, 
        ADD_B_A, 
        MOV_B_C, 
        CALL_CINDIRECT, 
        BRANCH, 0xE8 /* PROG_42=-24 */, 
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
    //
    // simulate Kevin Bacon / Gary Sinise Agc shutdown / Agc startup
    //
    // 1)
    //	V 50	V 25		Blinking
    //	R1 = +00062
    //	R2 = +00000
    //	R3 = +00000
    //
    // PROG_06:
    // {
    // begin standby
        LD_A_IMM16, 0x03, 0x00 /* MAIN_CPU1=3 */, 
        RUN_MINOR_A, 
        EMPTY_STACK, 
        UPLINK_PROB_IMM8, 0x00, 
        COMPACTY_PROB_IMM8, 0x00, 
        LD_A_IMM8, 0x50, 
        MOV_A_VERB, 
        LD_A_IMM8, 0x25, 
        MOV_A_NOUN, 
        BLINK_VERB, 0x01, 
        BLINK_NOUN, 0x01, 
        BLINK_PROG, 0x01, 
        LD_A_IMM32, 0x62, 0x00, 0xB0, 0x00, 
        MOV_A_R1, 
        LD_A_IMM32, 0x00, 0x00, 0xB0, 0x00, 
        MOV_A_R2, 
        LD_A_IMM32, 0x00, 0x00, 0xB0, 0x00, 
        MOV_A_R3, 
        INPUT_PROCEED, 
        BRANCH_A_LT_IMM8, 0x00, 0x64 /* cancel=+100 */, 
    // Turn everything off, except STBY light
        LT_ALL, 0x00, 
        LT_STBY, 0x01, 
        WAIT3, 
        WAIT3, 
        CALL, 0x0C, 0x03 /* clr=780 */, 
        INPUT_REQ_PROCEED, 
        LT_STBY, 0x00, 
        WAIT4, 
        UPLINK_PROB_IMM8, 0xFF, 
        WAIT4, 
        LT_TEMP, 0x01, 
        WAIT4, 
        LT_NO_ATT, 0x01, 
        WAIT4, 
        LT_PROG_ALRM, 0x01, 
        WAIT4, 
        LT_STBY, 0x01, 
        WAIT4, 
        LT_TRACKER, 0x01, 
        WAIT4, 
        LT_OPR_ERR, 0x01, 
        WAIT4, 
        WAIT4, 
        LT_ALL, 0x00, 
        UPLINK_PROB_IMM8, 0x00, 
        WAIT3, 
        LD_A_IMM8, 0x03, 
    // rloop:
        LT_RESTART, 0x01, 
        WAIT3, 
        WAIT3, 
        LT_RESTART, 0x00, 
        WAIT3, 
        WAIT3, 
        DEC_A, 
        BRANCH_A_GE_IMM8, 0x00, 0xF4 /* rloop=-12 */, 
        WAIT3, 
        LT_PROG, 0x01, 
        WAIT3, 
        LT_NOUN, 0x01, 
        WAIT3, 
        LT_VERB, 0x01, 
        WAIT3, 
        COMPACTY_PROB_IMM8, 0xFF, 
        WAIT3, 
        LD_A_IMM8, 0x16, 
        MOV_A_VERB, 
        LD_A_IMM8, 0x20, 
        MOV_A_NOUN, 
        LD_A_IMM8, 0x46, 
        MOV_A_PROG, 
        LD_A_IMM32, 0x00, 0x00, 0xC0, 0x00, 
        MOV_A_R1, 
        LD_A_IMM32, 0x80, 0x01, 0xB0, 0x00, 
        MOV_A_R2, 
        LD_A_IMM32, 0x00, 0x00, 0xB0, 0x00, 
        MOV_A_R3, 
        UPLINK_PROB_IMM8, 0x0A, 
        COMPACTY_PROB_IMM8, 0x64, 
        BRANCH, 0x05 /* forever=+5 */, 
    // cancel:
        CALL, 0x0C, 0x03 /* clr=780 */, 
        BRANCH, 0x00 /* forever=+0 */, 
    // forever:
        BRANCH, 0xFE /* forever=-2 */, 
    // clr:
        BLINK_NOUN, 0x00, 
        BLINK_VERB, 0x00, 
        BLINK_PROG, 0x00, 
        LD_A_IMM32, 0xAA, 0xAA, 0xAA, 0x00, 
        MOV_A_NOUN, 
        MOV_A_VERB, 
        MOV_A_R1, 
        MOV_A_R2, 
        MOV_A_R3, 
        MOV_A_PROG, 
        RET, 
    // }
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
    // VERB_06:
    // {
        CLR_A, 
        ST_A_DIRECT, CONTMODE, 
        BRANCH, 0x06 /* VERB_06_16=+6 */, 
    // }
    // VERB_16:
    // {
        LD_A_IMM8, 0x01, 
        ST_A_DIRECT, CONTMODE, 
        BRANCH, 0x00 /* VERB_06_16=+0 */, 
    // }
    // VERB_06_16:
    // {
        EMPTY_STACK, 
        CLR_A, 
        ST_A_DIRECT, CLIP_HAS_PLAYED, 
        UPLINK_PROB_IMM8, 0x0A, 
        COMPACTY_PROB_IMM8, 0x0A, 
        MOV_NOUN_A, 
        MOV_A_B, 
        LD_C_IMM16, 0x4A, 0x03 /* NounTable=842 */, 
    // Next:
        PROG8_A_CINDIRECT, 
        BRANCH_A_EQ_IMM8, 0xFF, 0x53 /* NotFound=+83 */, 
        AND_A_IMM32, 0xFF, 0x00, 0x00, 0x00, 
        INC_C, 
        ST_A_DIRECT, TMP1, 
        PROG16_A_CINDIRECT, 
        BRANCH_B_EQ_DIRECT, TMP1, 0x35 /* Found=+53 */, 
        ADD_C_IMM8, 0x02, 
        BRANCH, 0xEC /* Next=-20 */, 
    // NounTable:
        0x17, 
        0x90, 0x03 /* Query_IMU_Accel=912 */, 
        0x18, 
        0xA0, 0x03 /* Query_Gyro_Accel=928 */, 
        0x19, 
        0xB0, 0x03 /* Query_RTC_DateTimeTemp=944 */, 
        0x31, 
        0xD5, 0x03 /* Query_AgcInitTime=981 */, 
        0x34, 
        0x03, 0x04 /* Query_TimeFromEvent=1027 */, 
        0x35, 
        0x03, 0x04 /* Query_TimeToEvent=1027 */, 
        0x36, 
        0x03, 0x04 /* Query_RTC_Time=1027 */, 
        0x37, 
        0x30, 0x04 /* Query_RTC_Date=1072 */, 
        0x38, 
        0x48, 0x04 /* Query_GPS_Time=1096 */, 
        0x39, 
        0x48, 0x04 /* Query_GPS_Date=1096 */, 
        0x43, 
        0x48, 0x04 /* Query_GPS_Coord=1096 */, 
        0x44, 
        0x48, 0x04 /* Query_Orbital_Params=1096 */, 
        0x65, 
        0x48, 0x04 /* Query_MET=1096 */, 
        0x68, 
        0x48, 0x04 /* Query_Major_Mode=1096 */, 
        0x87, 
        0x48, 0x04 /* Query_IMU_1202=1096 */, 
        0x98, 
        0x48, 0x04 /* Query_AudioClipPlaying=1096 */, 
        0xFF, 
    // Found:
        MOV_A_C, 
        ST_C_DIRECT, TMP1, 
        LD_A_DIRECT, CONTMODE, 
        BRANCH_A_NE_IMM8, 0x00, 0x05 /* Monitor=+5 */, 
        LD_C_DIRECT, TMP1, 
        CALL_CINDIRECT, 
    // forever:
        BRANCH, 0xFE /* forever=-2 */, 
    // Monitor:
        LD_C_DIRECT, TMP1, 
        CALL_CINDIRECT, 
        BRANCH, 0xFB /* Monitor=-5 */, 
    // NotFound:
        BLINK_OPRERR, 0x01, 
        RET, 
    // }
    // Query_IMU_Accel:
    // {
        IMU_READ_DIRECT, ImuGyroX, 
        LD_A_DIRECT, ImuAccX, 
        ENCODE_A_TO_DEC, 
        MOV_A_R1, 
        LD_A_DIRECT, ImuAccY, 
        ENCODE_A_TO_DEC, 
        MOV_A_R2, 
        LD_A_DIRECT, ImuAccZ, 
        ENCODE_A_TO_DEC, 
        MOV_A_R3, 
        WAIT3, 
        RET, 
    // }
    // Query_Gyro_Accel:
    // {
        IMU_READ_DIRECT, ImuGyroX, 
        LD_A_DIRECT, ImuGyroX, 
        ENCODE_A_TO_DEC, 
        MOV_A_R1, 
        LD_A_DIRECT, ImuGyroY, 
        ENCODE_A_TO_DEC, 
        MOV_A_R2, 
        LD_A_DIRECT, ImuGyroZ, 
        ENCODE_A_TO_DEC, 
        MOV_A_R3, 
        WAIT3, 
        RET, 
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
        IMU_READ_DIRECT, ImuGyroX, 
        LD_A_DIRECT, ImuTemp, 
        ENCODE_A_TO_DEC, 
        MOV_A_R3, 
        WAIT5, 
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
        LD_A_DIRECT, CLIP_HAS_PLAYED, 
        BRANCH_A_NE_IMM8, 0x00, 0x14 /* done=+20 */, 
        INC_A, 
        ST_A_DIRECT, CLIP_HAS_PLAYED, 
        LD_A_DIRECT, SELCLIP, 
        MOV_A_B, 
        ENCODE_A_TO_DEC, 
        MOV_A_R1, 
        LD_A_DIRECT, ADJFACT, 
        ENCODE_A_TO_DEC, 
        MOV_A_R2, 
        LD_A_IMM32, 0xAA, 0xAA, 0xAA, 0x00, 
        MOV_A_R3, 
        MOV_B_A, 
        MP3_PLAY_A, 
    // done:
        RET, 
    // }
    // VERB_09:
    // {
        EMPTY_STACK, 
    // check noun
        MOV_NOUN_A, 
        DECODE_A_FROM_DEC, 
        BRANCH_A_LT_IMM8, 0x01, 0x39 /* error=+57 */, 
        BRANCH_A_GT_IMM8, 0x04, 0x36 /* error=+54 */, 
        DEC_A, 
        LSHIFT_A_IMM8, 0x01, 
        LD_B_IMM16, 0xA5, 0x04 /* Add=1189 */, 
        ADD_A_B, 
        ST_A_DIRECT, TMP3, 
        BLINK_R1, 0x01, 
        BLINK_R2, 0x01, 
        LD_A_IMM32, 0xAA, 0xAA, 0xAA, 0x00, 
        MOV_A_R1, 
        MOV_A_R2, 
        MOV_A_R3, 
        INPUT_R1, 
        BRANCH_A_LT_IMM8, 0x00, 0x1D /* error=+29 */, 
        DECODE_A_FROM_DEC, 
        ST_A_DIRECT, TMP1, 
        BLINK_R1, 0x00, 
        ENCODE_A_TO_DEC, 
        MOV_A_R1, 
        INPUT_R2, 
        BRANCH_A_LT_IMM8, 0x00, 0x12 /* error=+18 */, 
        DECODE_A_FROM_DEC, 
        ST_A_DIRECT, TMP2, 
        BLINK_R2, 0x00, 
        ENCODE_A_TO_DEC, 
        MOV_A_R2, 
        LD_A_DIRECT, TMP1, 
        LD_B_DIRECT, TMP2, 
        LD_C_DIRECT, TMP3, 
        CALL_CINDIRECT, 
        ENCODE_A_TO_DEC, 
        MOV_A_R3, 
        BRANCH, 0x02 /* done=+2 */, 
    // error:
        BLINK_OPRERR, 0x01, 
    // done:
        BRANCH, 0xFE /* done=-2 */, 
    // Add:
        ADD_A_B, 
        RET, 
    // Mul:
        MUL_A_B, 
        RET, 
    // Div:
        DIV_A_B, 
        RET, 
    // Mod:
        MOD_A_B, 
        RET, 
    // }
    // Decimal to Octal conversion N01
    // Octal to Decimal conversion N02
    //
    // VERB_10:
    // {
        EMPTY_STACK, 
        BLINK_R1, 0x01, 
        LD_A_IMM32, 0xAA, 0xAA, 0xAA, 0x00, 
        MOV_A_R1, 
        MOV_A_R2, 
        MOV_A_R3, 
        MOV_NOUN_A, 
        DECODE_A_FROM_DEC, 
        BRANCH_A_EQ_IMM8, 0x01, 0x05 /* DecimalToOctal=+5 */, 
        BRANCH_A_EQ_IMM8, 0x02, 0x29 /* OctalToDecimal=+41 */, 
        BRANCH, 0x36 /* error=+54 */, 
    // DecimalToOctal:
        INPUT_R1, 
        BLINK_R1, 0x00, 
        BRANCH_A_LT_IMM8, 0x00, 0x30 /* error=+48 */, 
        DECODE_A_FROM_DEC, 
        MOV_A_B, 
        ENCODE_A_TO_DEC, 
        MOV_A_R1, 
        MOV_B_A, 
        LD_C_IMM8, 0x00, 
        BRANCH_A_GT_IMM8, 0x00, 0x03 /* notneg=+3 */, 
        NEG_A, 
        LD_C_IMM8, 0x01, 
    // notneg:
        ENCODE_A_TO_OCT, 
        MOV_C_B, 
        BRANCH_B_NE_IMM8, 0x01, 0x0C /* over=+12 */, 
        LD_B_IMM32, 0xFF, 0xFF, 0x0F, 0x00, 
        AND_A_B, 
        LD_B_IMM32, 0x00, 0x00, 0xC0, 0x00, 
        OR_A_B, 
    // over:
        MOV_A_R2, 
        BRANCH, 0x11 /* done=+17 */, 
    // OctalToDecimal:
        INPUT_R1_OCT, 
        BLINK_R1, 0x00, 
        BRANCH_A_LT_IMM8, 0x00, 0x09 /* error=+9 */, 
        DECODE_A_FROM_OCT, 
        MOV_A_B, 
        ENCODE_A_TO_OCT, 
        MOV_A_R1, 
        MOV_B_A, 
        ENCODE_A_TO_DEC, 
        MOV_A_R2, 
        BRANCH, 0x02 /* done=+2 */, 
    // error:
        BLINK_OPRERR, 0x01, 
    // done:
        BRANCH, 0xFE /* done=-2 */, 
    // }
    //
    // Enter R1 which is a clip number.	V21 N98
    //
    // VERB_21:
    // {
        MOV_NOUN_A, 
        BRANCH_A_NE_IMM16, 0x98, 0x00, 0x26 /* error=+38 */, 
        LD_A_IMM32, 0xAA, 0xAA, 0xAA, 0x00, 
        MOV_A_R2, 
        MOV_A_R3, 
        LD_A_DIRECT, SELCLIP, 
        ENCODE_A_TO_DEC, 
        MOV_A_R1, 
        BLINK_R1, 0x01, 
        INPUT_R1, 
        BLINK_R1, 0x00, 
        BRANCH_A_LT_IMM8, 0x00, 0x13 /* error=+19 */, 
        DECODE_A_FROM_DEC, 
        MOV_A_B, 
        ENCODE_A_TO_DEC, 
        MOV_A_R1, 
        MOV_B_A, 
        BRANCH_A_LT_IMM8, 0x01, 0x0B /* error=+11 */, 
        BRANCH_A_GT_IMM8, 0x11, 0x08 /* error=+8 */, 
        ST_A_DIRECT, SELCLIP, 
        LD_A_IMM8, 0x16, 
        MOV_A_VERB, 
        GOTO, 0x23, 0x03 /* VERB_16=803 */, 
    // error:
        BLINK_OPRERR, 0x01, 
    // x:
        BRANCH, 0xFE /* x=-2 */, 
    // }
    //
    // Enter R2 which is an index adjustment factor. V21 N98
    //
    // VERB_22:
    // {
        MOV_NOUN_A, 
        BRANCH_A_NE_IMM16, 0x98, 0x00, 0x26 /* error=+38 */, 
        LD_A_IMM32, 0xAA, 0xAA, 0xAA, 0x00, 
        MOV_A_R1, 
        MOV_A_R3, 
        LD_A_DIRECT, ADJFACT, 
        ENCODE_A_TO_DEC, 
        MOV_A_R2, 
        BLINK_R2, 0x01, 
        INPUT_R2, 
        BLINK_R2, 0x00, 
        BRANCH_A_LT_IMM8, 0x00, 0x13 /* error=+19 */, 
        DECODE_A_FROM_DEC, 
        MOV_A_B, 
        ENCODE_A_TO_DEC, 
        MOV_A_R2, 
        MOV_B_A, 
        BRANCH_A_LT_IMM8, 0x00, 0x0B /* error=+11 */, 
        BRANCH_A_GT_IMM8, 0x10, 0x08 /* error=+8 */, 
        ST_A_DIRECT, ADJFACT, 
        LD_A_IMM8, 0x16, 
        MOV_A_VERB, 
        GOTO, 0x23, 0x03 /* VERB_16=803 */, 
    // error:
        BLINK_OPRERR, 0x01, 
    // x:
        BRANCH, 0xFE /* x=-2 */, 
    // }
    //
    // V25 N36		Set RTC Time (HH, MM, SS)
    // V25 N37		Set RTC Date (YYYY, MM, DD)
    //
    // TMP1		address of populate function
    // TMP2		address of set function
    //
    // VERB_25:
    // {
        MOV_NOUN_A, 
        BRANCH_A_EQ_IMM8, 0x36, 0x05 /* ptime=+5 */, 
        BRANCH_A_EQ_IMM8, 0x37, 0x07 /* pdate=+7 */, 
        BRANCH, 0x54 /* error=+84 */, 
    // ptime:
        CALL, 0xEF, 0x05 /* PopulateTime=1519 */, 
        BRANCH, 0x03 /* over=+3 */, 
    // pdate:
        CALL, 0x05, 0x06 /* PopulateDate=1541 */, 
    // over:
        BLINK_R1, 0x01, 
        INPUT_R1, 
        BRANCH_A_LT_IMM8, 0x00, 0x46 /* error=+70 */, 
        BLINK_R1, 0x00, 
        DECODE_A_FROM_DEC, 
        MOV_A_B, 
        ENCODE_A_TO_DEC, 
        MOV_A_R1, 
        MOV_NOUN_A, 
        BRANCH_A_EQ_IMM8, 0x36, 0x40 /* CheckHH=+64 */, 
        BRANCH_A_EQ_IMM8, 0x37, 0x56 /* CheckYYYY=+86 */, 
    // doR2:
        BLINK_R2, 0x01, 
        INPUT_R2, 
        BRANCH_A_LT_IMM8, 0x00, 0x33 /* error=+51 */, 
        BLINK_R2, 0x00, 
        DECODE_A_FROM_DEC, 
        MOV_A_B, 
        ENCODE_A_TO_DEC, 
        MOV_A_R2, 
        MOV_NOUN_A, 
        BRANCH_A_EQ_IMM8, 0x36, 0x35 /* CheckMM=+53 */, 
        BRANCH_A_EQ_IMM8, 0x37, 0x4D /* CheckMON=+77 */, 
    // doR3:
        BLINK_R3, 0x01, 
        INPUT_R3, 
        BRANCH_A_LT_IMM8, 0x00, 0x20 /* error=+32 */, 
        BLINK_R3, 0x00, 
        DECODE_A_FROM_DEC, 
        MOV_A_B, 
        ENCODE_A_TO_DEC, 
        MOV_A_R3, 
        MOV_NOUN_A, 
        BRANCH_A_EQ_IMM8, 0x36, 0x2A /* CheckSS=+42 */, 
        BRANCH_A_EQ_IMM8, 0x37, 0x42 /* CheckDAY=+66 */, 
    // set:
        MOV_NOUN_A, 
        BRANCH_A_EQ_IMM8, 0x36, 0x03 /* stime=+3 */, 
        BRANCH_A_EQ_IMM8, 0x37, 0x06 /* sdate=+6 */, 
    // stime:
        CALL, 0x2B, 0x06 /* SetTime=1579 */, 
        GOTO, 0x3B, 0x06 /* success=1595 */, 
    // sdate:
        CALL, 0x1B, 0x06 /* SetDate=1563 */, 
        GOTO, 0x3B, 0x06 /* success=1595 */, 
    // error:
        BLINK_OPRERR, 0x01, 
    // x:
        BRANCH, 0xFE /* x=-2 */, 
    // CheckHH:
        BRANCH_B_LT_IMM8, 0x00, 0xF9 /* error=-7 */, 
        BRANCH_B_GT_IMM8, 0x17, 0xF6 /* error=-10 */, 
        BRANCH, 0xBB /* doR2=-69 */, 
    // CheckMM:
        BRANCH_B_LT_IMM8, 0x00, 0xF1 /* error=-15 */, 
        BRANCH_B_GT_IMM8, 0x3B, 0xEE /* error=-18 */, 
        BRANCH, 0xC6 /* doR3=-58 */, 
    // CheckSS:
        BRANCH_B_LT_IMM8, 0x00, 0xE9 /* error=-23 */, 
        BRANCH_B_GT_IMM8, 0x3B, 0xE6 /* error=-26 */, 
        GOTO, 0xA4, 0x05 /* set=1444 */, 
    // CheckYYYY:
        BRANCH_B_LT_IMM16, 0xD0, 0x07, 0xDF /* error=-33 */, 
        BRANCH_B_GT_IMM16, 0x33, 0x08, 0xDB /* error=-37 */, 
        BRANCH, 0xA0 /* doR2=-96 */, 
    // CheckMON:
        BRANCH_B_LT_IMM8, 0x00, 0xD6 /* error=-42 */, 
        BRANCH_B_GT_IMM8, 0x0C, 0xD3 /* error=-45 */, 
        BRANCH, 0xAB /* doR3=-85 */, 
    // CheckDAY:
        BRANCH_B_LT_IMM8, 0x01, 0xCE /* error=-50 */, 
        BRANCH_B_GT_IMM8, 0x1F, 0xCB /* error=-53 */, 
        GOTO, 0xA4, 0x05 /* set=1444 */, 
    // PopulateTime:
        RTC_HH_A, 
        OR_A_IMM32, 0x00, 0x00, 0xB0, 0x00, 
        MOV_A_R1, 
        RTC_MM_A, 
        OR_A_IMM32, 0x00, 0x00, 0xB0, 0x00, 
        MOV_A_R2, 
        RTC_SS_A, 
        OR_A_IMM32, 0x00, 0x00, 0xB0, 0x00, 
        MOV_A_R3, 
        RET, 
    // PopulateDate:
        RTC_YEAR_A, 
        OR_A_IMM32, 0x00, 0x20, 0xB0, 0x00, 
        MOV_A_R1, 
        RTC_MON_A, 
        OR_A_IMM32, 0x00, 0x00, 0xB0, 0x00, 
        MOV_A_R2, 
        RTC_DAY_A, 
        OR_A_IMM32, 0x00, 0x00, 0xB0, 0x00, 
        MOV_A_R3, 
        RET, 
    // SetDate:
        LD_C_IMM8, 0x04, 
        MOV_R3_A, 
        RTC_A_MEM_CINDIRECT, 
        INC_C, 
        MOV_R2_A, 
        RTC_A_MEM_CINDIRECT, 
        INC_C, 
        MOV_R1_A, 
        AND_A_IMM32, 0xFF, 0x00, 0x00, 0x00, 
        RTC_A_MEM_CINDIRECT, 
        RET, 
    // SetTime:
    // clear seconds to avoid incrementing minutes
    // in the middle of the set operation
        CLR_C, 
        SWAP_A_B, 
        CLR_A, 
        RTC_A_MEM_CINDIRECT, 
        SWAP_A_B, 
        LD_C_IMM8, 0x02, 
        MOV_R1_A, 
        RTC_A_MEM_CINDIRECT, 
        DEC_C, 
        MOV_R2_A, 
        RTC_A_MEM_CINDIRECT, 
        DEC_C, 
        MOV_R3_A, 
        RTC_A_MEM_CINDIRECT, 
        RET, 
    // jump to the corresponding V16 N36 (time), V16 N37 (date)
    // success:
        EMPTY_STACK, 
        LD_A_IMM8, 0x16, 
        MOV_A_VERB, 
        GOTO, 0x23, 0x03 /* VERB_16=803 */, 
    // }
    // ASM_END:
};
