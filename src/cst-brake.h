#ifndef _CST_BRAKE_H_
#define _CST_BRAKE_H_

typedef enum
{
	BRAKE_LOW_BEGIN,
	BRAKE_LOW_WAIT,
	BRAKE_THR1_BEGIN,
	BRAKE_THR1_WAIT,
	BRAKE_THR2_BEGIN,
	BRAKE_THR2_WAIT,
	BRAKE_THR3_BEGIN,
	BRAKE_THR3_WAIT,
	BRAKE_THR4_BEGIN,
	BRAKE_THR4_WAIT,
	BRAKE_FULL_BEGIN,
	BRAKE_FULL_WAIT,
} BrakeStates;

void processBrake(uint8_t brakePosition, uint8_t brakeThreshold, uint8_t brakeLowThreshold, uint8_t brakeHighThreshold);
uint8_t getBrakePercentage(void);
BrakeStates getBrakeState(void);
uint8_t getBrake(void);
uint8_t getBrakeRelease(void);
uint8_t getEmergencyBrake(void);

#endif
