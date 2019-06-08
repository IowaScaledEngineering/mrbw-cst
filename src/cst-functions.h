#ifndef _CST_FUNCTIONS_H_
#define _CST_FUNCTIONS_H_

// These are the ProtoThrottle "functions" to which DCC (or other) function outputs can be assigned
typedef enum
{
	HORN_FN = 0,
	BELL_FN,
	BRAKE_FN,
	BRAKE_OFF_FN,
	AUX_FN,
	ENGINE_ON_FN,
	ENGINE_OFF_FN,
	THR_UNLOCK_FN,
	REV_SWAP_FN,
	COMPRESSOR_FN,
	BRAKE_TEST_FN,
	FRONT_HEADLIGHT_FN,
	FRONT_DITCH_FN,
	FRONT_DIM1_FN,
	FRONT_DIM2_FN,
	REAR_HEADLIGHT_FN,
	REAR_DITCH_FN,
	REAR_DIM1_FN,
	REAR_DIM2_FN,
	UP_FN,
	DOWN_FN,
	LAST_FN,
} Functions;

// These are the values a ProtoThrottle "function" can take.  Some are DCC function numbers, others are special functions
// Define these values explicitly, for EEPROM backwards compatibility, keeping the bottom 5 bits as the actual function number
typedef enum
{
	F00_MOM = 0x00, F10_MOM = 0x0A, F20_MOM = 0x14,
	F01_MOM = 0x01, F11_MOM = 0x0B, F21_MOM = 0x15,
	F02_MOM = 0x02, F12_MOM = 0x0C, F22_MOM = 0x16,
	F03_MOM = 0x03, F13_MOM = 0x0D, F23_MOM = 0x17,
	F04_MOM = 0x04, F14_MOM = 0x0E, F24_MOM = 0x18,
	F05_MOM = 0x05, F15_MOM = 0x0F, F25_MOM = 0x19,
	F06_MOM = 0x06, F16_MOM = 0x10, F26_MOM = 0x1A,
	F07_MOM = 0x07, F17_MOM = 0x11, F27_MOM = 0x1B,
	F08_MOM = 0x08, F18_MOM = 0x12, F28_MOM = 0x1C,
	F09_MOM = 0x09, F19_MOM = 0x13,
	F00_LAT = 0x40, F10_LAT = 0x4A, F20_LAT = 0x54,
	F01_LAT = 0x41, F11_LAT = 0x4B, F21_LAT = 0x55,
	F02_LAT = 0x42, F12_LAT = 0x4C, F22_LAT = 0x56,
	F03_LAT = 0x43, F13_LAT = 0x4D, F23_LAT = 0x57,
	F04_LAT = 0x44, F14_LAT = 0x4E, F24_LAT = 0x58,
	F05_LAT = 0x45, F15_LAT = 0x4F, F25_LAT = 0x59,
	F06_LAT = 0x46, F16_LAT = 0x50, F26_LAT = 0x5A,
	F07_LAT = 0x47, F17_LAT = 0x51, F27_LAT = 0x5B,
	F08_LAT = 0x48, F18_LAT = 0x52, F28_LAT = 0x5C,
	F09_LAT = 0x49, F19_LAT = 0x53,
	FN_OFF  = 0x80,
	FN_EMRG = 0x81,
} FunctionValues;

void printCurrentFunctionName(void);
void printCurrentFunctionValue(void);
void advanceCurrentFunction(void);
void resetCurrentFunction(void);
void incrementCurrentFunctionValue(void);
void decrementCurrentFunctionValue(void);
void readFunctionConfiguration(void);
void writeFunctionConfiguration(void);
uint8_t isFunctionOff(Functions functionName);
uint8_t isFunctionEstop(Functions functionName);
uint8_t isFunctionLatching(Functions functionName);
uint32_t getFunctionMask(Functions functionName);
void resetFunctionConfiguration(void);

#endif

