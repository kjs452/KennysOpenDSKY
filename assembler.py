#!/usr/bin/python3
#
# Kenny's OpenDSKY Apollo Guidance Computer Assembler
#
# The assembler syntax is pretty simple:
#
# DEFINE STUFF 5		// ram direct address
# label:
# {
#		MOVE_R1_A			// comment
#		LD_A_DIRECT	STUFF
# x:	LD_C_IMM16	123
#		GOTO done
#		BRANCH xx
#		DATA8 1 2 3 4
# done:
# }
#
# Directives:
#		DATA8	1 2 3				// encode byte values into the Program[] array
#		DATA16	34000 2000 -9000	// encode 16-bit values into the Program[] array
#
# Labels:
#
# Scoped Brackets:
#	Labels declared inside of brackets are deleted when leaving the scope bracket
#
# Numeric literals can be decimal or hexidecimal:
#	Decimal:		0, -10, 30
#	Hex:			0x004f	0x234f 0xBF
#
# Parsing:
#	* Each line is stripped of whitespace on beginning and end of the line
#	* Scope Brackets: Use curly braces to encapsulate some code. Nesting not allowed
#		all label definitions inside the scope brackets are local to that scope.
#	* Comments are indicated with the "//" characters
#	* Blank lines are ignored
# 	* do not use commas to seperate arguments
#	* Numeric constants are passed directly to the output:
#		0x2b, 000, -2, 2000
#	* 16-bit values are broken down into two 8-bit values
#	* 32-bit values are broken down into four 8-bit values
#
# Translation:
#	* Little-Endian is used by default to encode 16-bit and 32-bit values into Program[] space.
#		(Fortunately this is correct for both Linux Raspberry Pi and Arduino)
#		To switch (if assembling code for a platform which is big endian)
#		use the -b command line option (I.e., Solaris/IBM platforms).
#
import os
import sys
import re

ENDIAN='LITTLE'		# Change to 'BIG' to switch

#
# All the assembly instructions are listed in this table
# The key is the instruction mnemonic.
# The value is a string with these letters:
#
# Key:
#	D			direct addresss into RAM 8-bits
#	I			Immediate 8-bit value
#	J			Immediate 16-bit value
#	K			Immediate 32-bit value
#	R			relative branch address 8-bits	** see: label resolution
#	P			2-byte address into programming memory ** see: label resolution
#
# Label resolution:
#	for 'R', labels will be converted to 1-byte relative branches.
#	for 'P', labels will be converted to 2-byte absolute addresses.
#
# This string specifies the types and number of arguments to be expected for this instruction.
#
InstructionTable = {
	'MOV_R1_A': '',
	'MOV_R2_A': '',
	'MOV_R3_A': '',
	'MOV_A_R1': '',
	'MOV_A_R2': '',
	'MOV_A_R3': '',
	'DECODE_A_FROM_OCT': '',
	'DECODE_A_FROM_DEC': '',
	'ENCODE_A_TO_OCT': '',
	'ENCODE_A_TO_DEC': '',
	'ENCODE_A_TO_UDEC': '',
	'MOV_A_B': '',
	'MOV_A_C': '',
	'MOV_B_A': '',
	'MOV_B_C': '',
	'MOV_C_A': '',
	'MOV_C_B': '',
	'LD_A_DIRECT': 'D',
	'LD_B_DIRECT': 'D',
	'LD_C_DIRECT': 'D',
	'LD_A_IMM32': 'K',
	'LD_B_IMM32': 'K',
	'LD_C_IMM32': 'K',
	'LD_A_IMM16': 'J',
	'LD_B_IMM16': 'J',
	'LD_C_IMM16': 'J',
	'LD_A_IMM8': 'I',
	'LD_B_IMM8': 'I',
	'LD_C_IMM8': 'I',
	'LD_A_CDIRECT': 'D',
	'LD_B_CDIRECT': 'D',
	'LD_A_INDIRECT_C': '',
	'LD_B_INDIRECT_C': '',
	'ST_A_DIRECT': 'D',
	'ST_B_DIRECT': 'D',
	'ST_C_DIRECT': 'D',
	'ST_A_CDIRECT': 'D',
	'ST_B_CDIRECT': 'D',
	'ST_A_INDIRECT_C': '',
	'ST_B_INDIRECT_C': '',
	'CLR_A': '',
	'CLR_B': '',
	'CLR_C': '',
	'INC_A': '',
	'INC_B': '',
	'INC_C': '',
	'DEC_A': '',
	'DEC_B': '',
	'DEC_C': '',
	'PUSH_A': '',
	'PUSH_B': '',
	'PUSH_C': '',
	'POP_A': '',
	'POP_B': '',
	'POP_C': '',
	'CALL': 'P',
	'RET': '',
	'GOTO': 'P',
	'BRANCH': 'R',
	'BRANCH_A_GT_B': 'R',
	'BRANCH_A_GE_B': 'R',
	'BRANCH_A_LE_B': 'R',
	'BRANCH_A_LT_B': 'R',
	'BRANCH_A_EQ_B': 'R',
	'BRANCH_A_NE_B': 'R',
	'BRANCH_A_GT_DIRECT': 'DR',
	'BRANCH_A_GE_DIRECT': 'DR',
	'BRANCH_A_LE_DIRECT': 'DR',
	'BRANCH_A_LT_DIRECT': 'DR',
	'BRANCH_A_EQ_DIRECT': 'DR',
	'BRANCH_A_NE_DIRECT': 'DR',
	'BRANCH_A_GT_IMM8': 'IR',
	'BRANCH_A_GE_IMM8': 'IR',
	'BRANCH_A_LE_IMM8': 'IR',
	'BRANCH_A_LT_IMM8': 'IR',
	'BRANCH_A_EQ_IMM8': 'IR',
	'BRANCH_A_NE_IMM8': 'IR',
	'BRANCH_A_GT_IMM16': 'JR',
	'BRANCH_A_GE_IMM16': 'JR',
	'BRANCH_A_LE_IMM16': 'JR',
	'BRANCH_A_LT_IMM16': 'JR',
	'BRANCH_A_EQ_IMM16': 'JR',
	'BRANCH_A_NE_IMM16': 'JR',
	'BRANCH_B_GT_DIRECT': 'DR',
	'BRANCH_B_GE_DIRECT': 'DR',
	'BRANCH_B_LE_DIRECT': 'DR',
	'BRANCH_B_LT_DIRECT': 'DR',
	'BRANCH_B_EQ_DIRECT': 'DR',
	'BRANCH_B_NE_DIRECT': 'DR',
	'BRANCH_B_GT_IMM8': 'IR',
	'BRANCH_B_GE_IMM8': 'IR',
	'BRANCH_B_LE_IMM8': 'IR',
	'BRANCH_B_LT_IMM8': 'IR',
	'BRANCH_B_EQ_IMM8': 'IR',
	'BRANCH_B_NE_IMM8': 'IR',
	'BRANCH_B_GT_IMM16': 'JR',
	'BRANCH_B_GE_IMM16': 'JR',
	'BRANCH_B_LE_IMM16': 'JR',
	'BRANCH_B_LT_IMM16': 'JR',
	'BRANCH_B_EQ_IMM16': 'JR',
	'BRANCH_B_NE_IMM16': 'JR',
	'BRANCH_NOT_TIMER1': 'R',
	'BRANCH_NOT_TIMER2': 'R',
	'BRANCH_NOT_TIMER3': 'R',
	'BRANCH_NOT_TIMER4': 'R',
	'BRANCH_NOT_TIMER5': 'R',
	'SWAP_A_B': '',
	'SWAP_A_C': '',
	'SWAP_B_C': '',
	'ADD_A_B': '',
	'ADD_B_A': '',
	'SUB_A_B': '',
	'SUB_B_A': '',
	'MUL_A_B': '',
	'MUL_B_A': '',
	'DIV_A_B': '',
	'DIV_B_A': '',
	'MOD_A_B': '',
	'MOD_B_A': '',
	'AND_A_B': '',
	'AND_B_A': '',
	'OR_A_B': '',
	'OR_B_A': '',
	'OR_A_IMM32': 'K',
	'OR_B_IMM32': 'K',
	'AND_A_IMM32': 'K',
	'AND_B_IMM32': 'K',
	'LSHIFT_A_IMM8': 'I',
	'LSHIFT_B_IMM8': 'I',
	'RSHIFT_A_IMM8': 'I',
	'RSHIFT_B_IMM8': 'I',
	'NOT_A': '',
	'NOT_B': '',
	'NEG_A': '',
	'NEG_B': '',
	'MOV_A_VERB': '',
	'MOV_A_NOUN': '',
	'MOV_A_PROG': '',
	'MOV_NOUN_A': '',
	'MOV_VERB_A': '',
	'MOV_PROG_A': '',
	'BLINK_VERB': 'I',
	'BLINK_NOUN': 'I',
	'BLINK_PROG': 'I',
	'BLINK_KEYREL': 'I',
	'BLINK_OPRERR': 'I',
	'BLINK_R1': 'I',
	'BLINK_R2': 'I',
	'BLINK_R3': 'I',
	'LT_UPLINK_ACTY': 'I',
	'LT_NO_ATT': 'I',
	'LT_STBY': 'I',
	'LT_KEY_REL': 'I',
	'LT_OPR_ERR': 'I',
	'LT_NA1': 'I',
	'LT_NA2': 'I',
	'LT_TEMP': 'I',
	'LT_GIMBAL_LOCK': 'I',
	'LT_PROG_ALRM': 'I',
	'LT_RESTART': 'I',
	'LT_TRACKER': 'I',
	'LT_ALT': 'I',
	'LT_VEL': 'I',
	'LT_COMP_ACTY': 'I',
	'LT_VERB': 'I',
	'LT_NOUN': 'I',
	'LT_PROG': 'I',
	'LT_ALL': 'I',
	'UPLINK_PROB_IMM8': 'I',
	'COMPACTY_PROB_IMM8': 'I',
	'LT_ALL': 'I',
	'GPS_LAT_A': '',
	'GPS_LON_A': '',
	'GPS_YEAR_A': '',
	'GPS_MON_A': '',
	'GPS_DAY_A': '',
	'GPS_HH_A': '',
	'GPS_MM_A': '',
	'GPS_SS_A': '',
	'BRANCH_TIMESTAMP_LT': 'DDR',
	'TIMESTAMP_DIFF_A': 'DD',
	'RTC_TIMESTAMP_DIRECT': 'D',
	'RTC_DAY_A': '',
	'RTC_YEAR_A': '',
	'RTC_MON_A': '',
	'RTC_HH_A': '',
	'RTC_MM_A': '',
	'RTC_SS_A': '',
	'RTC_MEM_A': 'D',
	'RTC_A_MEM': 'D',
	'RTC_MEM_A_CDIRECT': 'D',
	'RTC_A_MEM_CDIRECT': 'D',
	'IMU_ACCX_A': '',
	'IMU_ACCY_A': '',
	'IMU_ACCZ_A': '',
	'IMU_PITCH_A': '',
	'IMU_ROLL_A': '',
	'IMU_YAW_A': '',
	'IMU_TEMP_A': '',
	'MP3_PLAY_A': '',
	'MP3_STOP': '',
	'EEPROM_WRITE_A_CDIRECT': '',
	'EEPROM_READ_A_CDIRECT': '',
	'WAIT1': '',
	'WAIT2': '',
	'WAIT3': '',
	'WAIT4': '',
	'WAIT5': '',
	'INPUT_NOUN': '',
	'INPUT_R1': '',
	'INPUT_R2': '',
	'INPUT_R3': '',
	'INPUT_R1_U': '',
	'INPUT_R2_U': '',
	'INPUT_R3_U': '',
	'INPUT_R1_OCT': '',
	'INPUT_R2_OCT': '',
	'INPUT_R3_OCT': '',
	'ADJUST_R1': 'DD',
	'ADJUST_R2': 'DD',
	'ADJUST_R3': 'DD',
	'ADJUST_R1_OCT': 'DD',
	'ADJUST_R2_OCT': 'DD',
	'ADJUST_R3_OCT': 'DD',
	'PROG8_A_INDIRECT_C': '',
	'PROG16_A_INDIRECT_C': '',
	'PROG32_A_INDIRECT_C': '',
	'ADD_A_IMM8': 'I',
	'ADD_B_IMM8': 'I',
	'ADD_C_IMM8': 'I',
	'EMPTY_STACK': '',
	'RUN_PROG_A': '',
	'CALL_CINDIRECT': '',
	'PUSH_DSKY': '',
	'POP_DSKY': '',
	'RANDOM_A': '',
}

#
# Bytes sizes for each type descriptor letters
#
Td_Size = {
	'D': 1,		# direct address into RAM
	'I': 1,		# Immediate value 8-bits
	'J': 2,		# Immediate value 16-bits
	'K': 4,		# Immediate value 32-bits
	'R': 1,		# Relative branch 8-bits -128/+127
	'P': 2,		# Program address 16-bits
}

# Location Counter
LC = 0

#
# Assembled code goes into this array
# This is an array of arrays of strings/numbers
# Each sub-array is a line from the input assembly source.
#
# Sub-arrays use this structure:
#	[code, lineno, LC, instruction, arg1, arg2, arg3, ...]
#
# For DATA8 or DATA16 directives this format is used:
#	['DATA8', lineno, LC, value1, value2, value3, ...]
#	['DATA16', lineno, LC, value1, value2, value3, ...]
#
# For "DEFINE symbol = value" this format is used:
#	['DEF', lineno, LC, symbol, value]
#
# Assembly instructions are stored as,
#	['INSTR', lineno, LC, "LD_A_IMM16", "-1234"]
#
# code can be:
#	'L'			label
#	'INSTR'		instruction
#	'DEF'		DEFINE directive
#	'DATA8'		DATA8 directive
#	'DATA16'	DATA16 directive
#	'SB'		Scope Begin bracket
#	'SE'		Scope End bracket
#	'C'			comment
#
#	LC - is the location counter (as integer)
#	instruction/directive is the 
#
# Arguments which have been resolved or translated into final form
# will be enclosed in curly braces.
#
Result = []

#
# Symbol table
#	An associative array.
#	- the key is string which is a symbol in the assembly
#	- the value is a structure of this form:
#
#		'symbol': { 'scoped': True, 'value': 123, 'lineno': 123, 'type': 'L' }
#
#	- lineno is the line the symbol was defined on
#	- 'type' is:
#		'L'	- label definition
#		'D'	- DEFINE definition
#	- 'scoped' is True if the symbol was declared inside of scope brackets.
#
SymbolTable = {}

#
# File name being assembled
#
Filename = ""

######################################################################
def usage(msg):
	sys.stdout.write("""\
Usage: assembler.py [-b] filename.asm
        -b = use big-endian byte ordering encoding (otherwise little-endian is used)
%s
""" % (msg))

######################################################################
def error(lineno, msg):
	sys.stderr.write("File: %s, Line: %d, ERROR: %s\n" % (Filename, lineno,msg))

######################################################################
#
# Remove all symbols from the symbol table which have 'scoped'=True
#
def purge_scoped_symbols():
	purge_list = []
	for key in SymbolTable:
		rec = SymbolTable[key]
		if rec['scoped'] == True:
			purge_list.append(key)

	for key in purge_list:
		del SymbolTable[key]

######################################################################
#
# Transform an integer into its hex representation
# A two's compliment algorithm is used to map negative integers
#
# 'width' is number of bytes
#
# From:
# https://stackoverflow.com/questions/1604464/twos-complement-in-python
#
def transform_to_hex(val, width):
	fmt = '%%0%dX' % (width*2)

	if val < 0:
		val = -val
		bits = width*8
		val = ((val & (2 ** bits) - 1) - (2 ** bits)) * -1
	result = fmt % (val)

	if ENDIAN == "BIG":
		return result
	elif ENDIAN == "LITTLE":
		little_endian_result = ""
		for i in reversed(range(0, len(result), 2)):
			little_endian_result += result[i:i+2]
		return little_endian_result
	else:
		error(-1, "ENDIAN = '%s' wrong. should be 'LITTLE' or 'BIG'" % (ENDIAN))
		return result

######################################################################
#
# Return the value (with embedded C comment) for
# the symbol 'sym'.
# 'symrec' will be of type 'L' (label).
#
#	symrec['type'] = 'L'
#	symrec['value'] = 3456
#	symrec['name'] = 'LabelName'
#
#	Returns,
#		"{0x0D, 0x80 /* LabelName=3456 */}"
#
#	type descriptor 'type' is one of:
#		I	- 8-bit Immediate value
#		J	- 16-bit Immediate value
#		K	- 32-bit Immediate value
#		R	- 8-bit relative address
#		D	- direct address to RAM (8-bit)
#
# The returned string is enclosed in curly braces, to
# indicate that it has already been translated.
#
#
def resolve_symbol(ident, lineno, LC, type, symrec):
	val = symrec['value']
	if type == 'I':
		hex = transform_to_hex(int(val), 1)
		if len(hex) > 2:
			error(lineno, "Overflow '%s' > 8-bit" % (val))
		str = '{0x%s /* %s=%d */}' % (hex[0:2], ident, val)

	elif type == 'J':
		hex = transform_to_hex(int(val), 2)
		if len(hex) > 4:
			error(lineno, "Overflow '%s' > 16-bit" % (val))
		str = '{0x%s, 0x%s /* %s=%d */}' % (hex[0:2], hex[2:4], ident, val)

	elif type == 'K':
		hex = transform_to_hex(int(val), 4)
		if len(hex) > 8:
			error(lineno, "Overflow '%s' > 32-bit" % (val))
		str = '{0x%s, 0x%s, 0x%s, 0x%s /* %s=%d */}' % (hex[0:2], hex[2:4], hex[4:6], hex[6:8], ident, val)

	elif type == 'R':
		rel = val - LC - 1
		if rel < -128 or rel > 127:
			error(lineno, "Relative offset out of range '%d' (%s=%d)" % (rel, ident, val))
		hex = transform_to_hex(int(rel), 1)
		str = '{0x%s /* %s=%+d */}' % (hex, ident, rel)

	elif type == 'D':
		hex = transform_to_hex(int(val), 1)
		if len(hex) > 2:
			error(lineno, "Overflow '%s' > 8-bit" % (val))
		str = '{0x%s /* %s=%d */}' % (hex[0:2], ident, val)

	elif type == 'P':
		hex = transform_to_hex(int(val), 2)
		if len(hex) > 4:
			error(lineno, "Overflow '%s' > 16-bit" % (val))
		str = '{0x%s, 0x%s /* %s=%d */}' % (hex[0:2], hex[2:4], ident, val)

	else:
		error(lineno, "Unknown type '%s'" % (type))
		sys.exit(1)

	return str

######################################################################
def is_translated(str):
	m = re.match(r'^[{].*[}]$', str)
	return m != None

######################################################################
def is_identifier(str):
	m = re.match(r'^[A-Za-z_]([A-Za-z0-9_])*$', str)
	return m != None

######################################################################
def is_number(str):
	# hex
	m = re.match(r'^0x[0-9A-Fa-f]+$', str)
	if m != None:
		return True

	# decimal
	m = re.match(r'^[-]?[0-9]+$', str)
	if m != None:
		return True

	return False

######################################################################
#
# This validates the symbol references and verifies the number
# constants are properly formatted
#
# If 'final' is true then generate an error if a symbol isn't defined.
#
def back_patch_record(final, rec):

	if rec[0] == 'DATA8':
		td = 'XXX' + ('I' * (len(rec)-3))
		for i in range(3, len(rec)):
			arg = rec[i]
			type = td[i]
			if is_number(arg):
				pass
			elif is_identifier(arg):
				if arg in SymbolTable:
					symrec = SymbolTable[arg]
					if symrec['type'] == 'L':
						rec[i] = resolve_symbol(arg, rec[1], rec[2], type, symrec)
				elif final:
					error(rec[1], "Undefined symbol '%s'" % (arg))
					return False
			elif is_translated(arg):
				pass
			else:
				error(rec[1], "Invalid number or identifier '%s'" % (arg))
				return False

	elif rec[0] == 'DATA16':
		td = 'XXX' + ('J' * (len(rec)-3))
		for i in range(3, len(rec)):
			arg = rec[i]
			type = td[i]
			if is_number(arg):
				pass
			elif is_identifier(arg):
				if arg in SymbolTable:
					symrec = SymbolTable[arg]
					if symrec['type'] == 'L':
						rec[i] = resolve_symbol(arg, rec[1], rec[2], type, symrec)
				elif final:
					error(rec[1], "Undefined symbol '%s'" % (arg))
					return False
			elif is_translated(arg):
				pass
			else:
				error(rec[1], "Invalid number or identifier '%s'" % (arg))
				return False

	elif rec[0] == 'INSTR':
		td = 'XXXX' + InstructionTable[rec[3]]
		offset = 0
		for i in range(4, len(rec)):
			arg = rec[i]
			type = td[i]
			offset += Td_Size[type]
			if is_number(arg):
				pass
			elif is_identifier(arg):
				if arg in SymbolTable:
					symrec = SymbolTable[arg]
					if symrec['type'] == 'L':
						rec[i] = resolve_symbol(arg, rec[1], rec[2]+offset, type, symrec)
				elif final:
					error(rec[1], "Undefined symbol '%s'" % (arg))
					return False
			elif is_translated(arg):
				pass
			else:
				error(rec[1], "Invalid number or identifier '%s'" % (arg))
				return False

	return True

######################################################################
#
# Resolve labels that are about to go out of scope
#
def back_patch():
	i = len(Result)-1
	done = False
	while not done:
		rec = Result[i]
		if rec[0] == 'SB':
			done = True
			continue

		success = back_patch_record(False, rec)
		if not success:
			return False
		i -= 1
	return True

######################################################################
#
# Same as back_patch() except it called when the whole
# assembly file has been processed at the end.
#
# Will issue error message about undefined symbols.
#
def back_patch_final():
	result = True
	for i in range( len(Result) ):
		rec = Result[i]
		success = back_patch_record(True, rec)
		if not success:
			result = False

	return result

######################################################################
#
# Translate a number found in the assembly code in 'arg'.
# This will be translated into one or more hex bytes.
#
# td will be:
#	D = 8-bit integer
#	I = 8-bit integer
#	J = 16-bit integer
#	K = 32-bit integer
#	R = 8-bit integer
#	P = 16-bit integer
#
def translate_number(arg, td):
	result = ""
	m = re.match(r'^0x([0-9A-Za-z]+)$', arg)
	if m != None:
		numstr = m.group(1)
		width = Td_Size[td]
		numstr = transform_to_hex(int(numstr,16), width)
		if len(numstr) > width*2:
			error(-1, "Numeric literal too big '%s'" % (arg))
			return arg
		rng = list(range(0, len(numstr), 2))
		for i in rng:
			if i != rng[len(rng)-1]:
				comma = ", "
			else:
				comma = ""
			result = result + "0x" + numstr[i:i+2] + comma
		return "{" + result + "}"

	m = re.match(r'^([-]?[0-9]+)$', arg)
	if m != None:
		numstr = m.group(1)
		width = Td_Size[td]
		numstr = transform_to_hex(int(numstr), width)
		if len(numstr) > width*2:
			error(-1, "Numeric literal too big '%s' (%s)" % (arg, numstr))
			return arg
		rng = list(range(0, len(numstr), 2))
		for i in rng:
			if i != rng[len(rng)-1]:
				comma = ", "
			else:
				comma = ""
			result = result + "0x" + numstr[i:i+2] + comma
		return "{" + result + "}"

	return arg

######################################################################
#
# Convert numeric constants into hex bytes. values that are bigger
# than 8-bits are converted into multiple comma seperated bytes.
#
def translate_numeric_operands():
	result = True
	for i in range( len(Result) ):
		rec = Result[i]
		if rec[0] == 'DATA8':
			for i in range(3, len(rec)):
				arg = rec[i]
				if is_number(arg):
					rec[i] = translate_number(arg, 'I')
		elif rec[0] == 'DATA16':
			for i in range(3, len(rec)):
				arg = rec[i]
				if is_number(arg):
					rec[i] = translate_number(arg, 'J')
		elif rec[0] == 'INSTR':
			td = 'XXXX' + InstructionTable[rec[3]]
			for i in range(4, len(rec)):
				arg = rec[i]
				type = td[i]
				if is_number(arg):
					rec[i] = translate_number(arg, type)

	return result

######################################################################
#
# Return 'arr' except truncate the array 'arr' at the spot where
# the first occurance of an element that begins with the comment pattern "//"
#
#	["0x900", "9", "12", "//", "this", "is", "a", "comment"]
# returns,
#	["0x900", "9", "12"]
#
def strip_comments(arr):
	result = []
	for i in range(len(arr)):
		m = re.match(r'^//.*$', arr[i])
		if m != None:
			break
		else:
			result.append(arr[i])
	return result

######################################################################
def add_symbol(lineno, scoped, type, name, value):
	if name in SymbolTable:
		rec = SymbolTable[name]
		error(lineno, "Symbol '%s' already defined on line %d.\n"
					% (name, rec['lineno']))
		return False

	if not is_identifier(name):
		error(lineno, "Invalid Symbol '%s'\n" % (name))
		return False

	SymbolTable[name] = {
			'scoped': scoped,
			'value': value,
			'lineno': lineno,
			'type': type }
	return True

######################################################################
#
# 'instr' is an array. The first element is the instruction
# mnemonic. Returns the number of bytes this instruction uses.
#
def parse_instruction(lineno, instr):
	if instr[0] not in InstructionTable:
		if instr[0] == "DEFINE":
			error(lineno, "invalid syntax '%s' (DEFINE cannot have a label)" % (instr[0]))
		else:
			error(lineno, "Invalid instruction '%s'" % (instr[0]))
		return -1

	argstr = InstructionTable[ instr[0] ]

	if len(instr)-1 > len(argstr):
		error(lineno, "Too many arguments for instruction '%s'" % (instr[0]))
		return -1
	elif len(instr)-1 < len(argstr):
		error(lineno, "Not enough arguments for instruction '%s'" % (instr[0]))
		return -1

	bytes = 1
	for ch in argstr:
		bytes += Td_Size[ch]
	return bytes

######################################################################
def assemble(inf):
	global LC
	global Result

	inside_scope = False
	LC = 0
	lineno = 0
	for line in inf:
		line = line.strip()
		lineno += 1

		if len(line) == 0:
			continue

		# comment
		m = re.match(r'^//.*$', line)
		if m != None:
			Result.append(['C', lineno, LC, line])
			continue

		# stand alone label
		m = re.match(r'^(\w+)(\s*):(\s*)([/][/].*)?$', line)
		if m != None:
			label = m.group(1)
			Result.append(['L', lineno, LC, label])
			success = add_symbol(lineno, inside_scope, 'L', label, LC)
			if not success:
				return False
			continue

		# scope begin
		m = re.match(r'^{(\s*)(//.*)?$', line)
		if m != None:
			if inside_scope:
				error(lineno, "Already inside a scope")
				return False
			inside_scope = True
			Result.append(['SB', lineno, LC, line])
			continue

		# scope end
		m = re.match(r'^}(\s*)(//.*)?$', line)
		if m != None:
			if not inside_scope:
				error(lineno, "Not inside a scope")
				return False
			inside_scope = False
			Result.append(['SE', lineno, LC, line])
			success = back_patch()
			if not success:
				return False
			purge_scoped_symbols()
			continue

		# DEFINE directive
		m = re.match(r'^DEFINE(\s+)(\w+)(\s*)=(\s*)(\w+)(\s*)(//.*)?$', line)
		if m != None:
			ident = m.group(2)
			value = m.group(5)
			if inside_scope:
				error(lineno, "DEFINE '%s' not allowed inside of scope brackets { }" % (ident))
				return False
			Result.append(['DEF', ident, value])
			success = add_symbol(lineno, inside_scope, 'D', ident, value)
			if not success:
				return False
			continue

		# DATA8 directive (without label)
		m = re.match(r'^DATA8(\s+)(.+)$', line)
		if m != None:
			data = m.group(2)
			data = data.split()
			data = strip_comments(data)
			rec = ['DATA8', lineno, LC] + data
			Result.append(rec)
			LC += len(data)
			continue

		# DATA8 directive with label
		m = re.match(r'^(\w+)(\s*):(\s*)DATA8(\s+)(.+)$', line)
		if m != None:
			label = m.group(1)
			data = m.group(5)
			data = data.split()
			data = strip_comments(data)
			success = add_symbol(lineno, inside_scope, 'L', label, LC)
			if not success:
				return False
			rec = ['L', lineno, LC, label]
			Result.append(rec)
			rec = ['DATA8', lineno, LC] + data
			Result.append(rec)
			LC += len(data)
			continue

		# DATA16 directive (without label)
		m = re.match(r'^DATA16(\s+)(.+)$', line)
		if m != None:
			data = m.group(2)
			data = data.split()
			data = strip_comments(data)
			rec = ['DATA16', lineno, LC] + data
			Result.append(rec)
			LC += len(data)*2
			continue

		# DATA16 directive with label
		m = re.match(r'^(\w+)(\s*):(\s*)DATA16(\s+)(.+)$', line)
		if m != None:
			label = m.group(1)
			data = m.group(5)
			data = data.split()
			data = strip_comments(data)
			success = add_symbol(lineno, inside_scope, 'L', label, LC)
			if not success:
				return False
			rec = ['L', lineno, LC, label]
			Result.append(rec)
			rec = ['DATA16', lineno, LC] + data
			Result.append(rec)
			LC += len(data)*2
			continue

		# Instructions with label
		m = re.match(r'^(\w+)(\s*):(.+)$', line)
		if m != None:
			label = m.group(1)
			instr = m.group(3)
			instr = instr.split()
			instr = strip_comments(instr)
			num_bytes = parse_instruction(lineno, instr)
			if num_bytes == -1:
				return False
			success = add_symbol(lineno, inside_scope, 'L', label, LC)
			if not success:
				return False
			rec = ['L', lineno, LC, label]
			Result.append(rec)
			rec = ['INSTR', lineno, LC] + instr
			Result.append(rec)
			LC += num_bytes
			continue

		# Instructions (without label)
		m = re.match(r'^(.+)$', line)
		if m != None:
			instr = m.group(1)
			instr = instr.split()
			instr = strip_comments(instr)
			num_bytes = parse_instruction(lineno, instr)
			if num_bytes == -1:
				return False
			rec = ['INSTR', lineno, LC] + instr
			Result.append(rec)
			LC += num_bytes
			continue

		error(lineno, "Syntax error")
		return False

	return True

######################################################################
def dump_item(item):
	m = re.match(r'^[{](.*)[}]', item)
	if m != None:
		return m.group(1)
	else:
		return item

######################################################################
def dump(outf):
	outf.write("""
//////////////////////////////////////////////////////////////////////
//
// This file automatically generated by 'assembler.py'
// Input File: '%s'
// Command: $ ./assembler.py %s
//
// This is a C source code header file which defines an enum and a program array.
// The enum defines global labels and defined symbols.
// The program array is called 'Program[]', it contains the byte codes that
// the Apollo Guidance Computer virtual machine will execute.
//
//
""" % (Filename, Filename) )

	outf.write("\n")
	outf.write("enum ASM_SYMBOLS {\n")

	for key in SymbolTable:
		rec = SymbolTable[key]
		value = rec['value']
		type = rec['type']
		if type == 'D':
			outf.write("\t%s = %s,    // %s\n" % (key, value, "DEFINE" if type == 'D' else "LABEL"))
		else:
			outf.write("\tLBL_%s = %s,    // %s\n" % (key, value, "DEFINE" if type == 'D' else "LABEL"))

	outf.write("};\n\n")

	outf.write("static const uint8_t Program[] PROGMEM = {\n")

	for rec in Result:
		if rec[0] == 'L':
			outf.write("    // %s:\n" % (rec[3]))

		elif rec[0] == 'C':
			outf.write("    %s\n" % (rec[3]))

		elif rec[0] == 'INSTR':
			outf.write("        ")
			for i in range(3, len(rec)):
				outf.write(dump_item(rec[i]) + ", ")
			outf.write("\n")
			pass

		elif rec[0] == 'DATA8':
			outf.write("        ")
			for i in range(3, len(rec)):
				outf.write(dump_item(rec[i]) + ", ")
			outf.write("\n")

		elif rec[0] == 'DATA16':
			outf.write("        ")
			for i in range(3, len(rec)):
				outf.write(dump_item(rec[i]) + ", ")
			outf.write("\n")

		elif rec[0] == 'SB':
			outf.write("    // {\n")

		elif rec[0] == 'SE':
			outf.write("    // }\n")

	outf.write("};\n")

######################################################################
def main():
	global Filename
	global ENDIAN

	if len(sys.argv) > 2:
		if sys.argv[1] == "-b":
			ENDIAN='BIG'
			sys.argv.pop(0)
		elif sys.argv[1][0] == "-":
			usage("Invalid option %s" % (sys.argv[1]))
			sys.exit(1)

	if len(sys.argv) < 2:
		usage("No file argument given")
		sys.exit(1)
	elif len(sys.argv) > 2:
		usage("Too many file arguments")
		sys.exit(1)

	Filename = sys.argv[1]

	try:
		inf = open(Filename, 'r')
	except:
		usage("Unable to open %s" % (Filename))
		sys.exit(1)

	m = re.match("(.*).asm", Filename)
	if m == None:
		usage("Bad filename extension, must be .asm")
		sys.exit(1)

	outfilename = m.group(1) + ".h"

	try:
		outf = open(outfilename, 'w')
	except:
		usage("Unable to open output file %s" % (outfilename))
		sys.exit(1)

	sys.stdout.write("Kenny's OpenDSKY Apollo Guidance Computer Assembler\n")
	if ENDIAN == 'BIG':
		sys.stdout.write("Big-endian encoding will be used.\n")
	else:
		sys.stdout.write("Little-endian encoding will be used.\n")
	sys.stdout.write("Assembling '%s'.\n" % (Filename))

	success = assemble(inf)
	if not success:
		inf.close()
		outf.close()
		return

	back_patch_final()
	translate_numeric_operands()
	dump(outf)

	sys.stdout.write("Finished assembling '%s' -> '%s'.\n" % (Filename, outfilename))

	inf.close()
	outf.close()

######################################################################
main()
