#ifndef _CST_FUNCTIONS_H_
#define _CST_FUNCTIONS_H_

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

void printCurrentFunctionName(void);
void printCurrentFunctionFunction(void);
void advanceCurrentFunction(void);
void resetCurrentFunction(void);
void incrementCurrentFunctionFunction(void);
void decrementCurrentFunctionFunction(void);
void readFunctionConfiguration(void);
void writeFunctionConfiguration(void);
void resetFunctionConfiguration(void);

#endif

