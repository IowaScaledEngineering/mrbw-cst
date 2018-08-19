/*************************************************************************
Title:    Brake support for Control Stand Throttle
Authors:  Michael D. Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     cst-brake.c
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2018 Michael Petersen & Nathan Holmes
    
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*************************************************************************/

#include <stdlib.h>
#include <stdint.h>

#include <avr/io.h>

#include "cst-common.h"
#include "cst-brake.h"
#include "cst-config.h"

static uint8_t brakePcnt = 0;
static uint8_t brakeStatus = 0;

BrakeStates brakeState = BRAKE_LOW_BEGIN;

#define BRAKE            0x01
#define BRAKE_RELEASE    0x02
#define BRAKE_EMERGENCY  0x04

#define BRAKE_HYSTERESIS  5

static inline void setBrake()
{
	brakeStatus |= BRAKE;
}

static inline void clearBrake()
{
	brakeStatus &= ~BRAKE;
}

uint8_t getBrake(void)
{
	if(brakeStatus & BRAKE)
		return 1;
	else
		return 0;
}

static inline void setBrakeRelease()
{
	brakeStatus |= BRAKE_RELEASE;
}

static inline void clearBrakeRelease()
{
	brakeStatus &= ~BRAKE_RELEASE;
}

uint8_t getBrakeRelease(void)
{
	if(brakeStatus & BRAKE_RELEASE)
		return 1;
	else
		return 0;
}

static inline void setEmergencyBrake()
{
	brakeStatus |= BRAKE_EMERGENCY;
}

static inline void clearEmergencyBrake()
{
	brakeStatus &= ~BRAKE_EMERGENCY;
}

uint8_t getEmergencyBrake(void)
{
	if(brakeStatus & BRAKE_EMERGENCY)
		return 1;
	else
		return 0;
}

uint8_t getBrakePercentage(void)
{
	return brakePcnt;
}

BrakeStates getBrakeState(void)
{
	return brakeState;
}

void processBrake(uint8_t brakePosition, uint8_t brakeThreshold, uint8_t brakeLowThreshold, uint8_t brakeHighThreshold)
{
	// Sanity check brake position and calculate percentage
	if(brakePosition < brakeLowThreshold)
		brakePcnt = 0;
	else
		brakePcnt = 100 * (brakePosition - brakeLowThreshold) / (brakeHighThreshold - brakeLowThreshold);

	// Handle emergency on brake control.  Do this outside the main brake state machine so the effect is immediate
	if(optionBits & _BV(OPTIONBITS_ESTOP_ON_BRAKE))
	{
		if(brakePosition < brakeLowThreshold)
			clearEmergencyBrake();
		if(brakePosition > brakeHighThreshold)
			setEmergencyBrake();
	}
	else
	{
		clearEmergencyBrake();
	}
	
	// Handle brake
	if( (optionBits & _BV(OPTIONBITS_VARIABLE_BRAKE)) && (optionBits & _BV(OPTIONBITS_STEPPED_BRAKE)) )
	{
		// This state machine handles the variable (stepped) brake.
		switch(brakeState)
		{
			case BRAKE_LOW_BEGIN:
				setBrakeRelease();  // Pulse the "brake off" function
				brakeState = BRAKE_LOW_WAIT;
				break;
			case BRAKE_LOW_WAIT:
				clearBrakeRelease();
				if(brakePcnt >= 20)
					brakeState = BRAKE_THR1_BEGIN;
				break;

			case BRAKE_THR1_BEGIN:
				setBrake();  // Pulse the "brake on" function
				brakeState = BRAKE_THR1_WAIT;
				break;
			case BRAKE_THR1_WAIT:
				clearBrake();
				if(brakePosition < brakeLowThreshold)
					brakeState = BRAKE_LOW_BEGIN;
				else if(brakePcnt >= 40)
					brakeState = BRAKE_THR2_BEGIN;
				break;

			case BRAKE_THR2_BEGIN:
				setBrake();  // Pulse the "brake on" function
				brakeState = BRAKE_THR2_WAIT;
				break;
			case BRAKE_THR2_WAIT:
				clearBrake();
				if(brakePosition < brakeLowThreshold)
					brakeState = BRAKE_LOW_BEGIN;
				else if(brakePcnt >= 60)
					brakeState = BRAKE_THR3_BEGIN;
				break;

			case BRAKE_THR3_BEGIN:
				setBrake();  // Pulse the "brake on" function
				brakeState = BRAKE_THR3_WAIT;
				break;
			case BRAKE_THR3_WAIT:
				clearBrake();
				if(brakePosition < brakeLowThreshold)
					brakeState = BRAKE_LOW_BEGIN;
				else if(brakePcnt >= 80)
					brakeState = BRAKE_THR4_BEGIN;
				break;

			case BRAKE_THR4_BEGIN:
				setBrake();  // Pulse the "brake on" function
				brakeState = BRAKE_THR4_WAIT;
				break;
			case BRAKE_THR4_WAIT:
				clearBrake();
				if(brakePosition < brakeLowThreshold)
					brakeState = BRAKE_LOW_BEGIN;
				else if(brakePosition > brakeHighThreshold)
					brakeState = BRAKE_FULL_BEGIN;
				break;

			case BRAKE_FULL_BEGIN:
				setBrake();  // Pulse the "brake on" function
				brakeState = BRAKE_FULL_WAIT;
				break;
			case BRAKE_FULL_WAIT:
				clearBrake();
				if(brakePosition < brakeLowThreshold)
					brakeState = BRAKE_LOW_BEGIN;
				break;
		}
	}
	else if( (optionBits & _BV(OPTIONBITS_VARIABLE_BRAKE)) && !(optionBits & _BV(OPTIONBITS_STEPPED_BRAKE)) )
	{
		// This state machine handles the variable (pulse) brake.
		switch(brakeState)
		{
			// These two states get "brake off" set by first making sure "brake on" is clear (TCS decoders don't like these changing at the same time)
			case BRAKE_LOW_BEGIN:
				clearBrake();
				brakeState = BRAKE_LOW_WAIT;
				break;
			case BRAKE_LOW_WAIT:
				setBrakeRelease();
				brakeState = BRAKE_THR1_BEGIN;
				break;

			// These states represent the pulse "brake off" period
			case BRAKE_THR1_BEGIN:
			case BRAKE_THR1_WAIT:
			case BRAKE_THR2_BEGIN:
			case BRAKE_THR2_WAIT:
				if( brakePcnt >= (((brakeCounter / brakePulseWidth)+1)*20) )
					brakeState = BRAKE_FULL_BEGIN;
				break;

			// These states represent the pulse "brake on" period
			case BRAKE_THR3_BEGIN:
			case BRAKE_THR3_WAIT:
			case BRAKE_THR4_BEGIN:
			case BRAKE_THR4_WAIT:
				if( brakePcnt < (((brakeCounter / brakePulseWidth)+1)*20) )
					brakeState = BRAKE_LOW_BEGIN;
				break;

			// These two states get "brake on" set by first making sure "brake off" is clear (TCS decoders don't like these changing at the same time)
			case BRAKE_FULL_BEGIN:
				clearBrakeRelease();
				brakeState = BRAKE_FULL_WAIT;
				break;
			case BRAKE_FULL_WAIT:
				setBrake();
				brakeState = BRAKE_THR3_BEGIN;
				break;
		}
	}
	else
	{
		// This state machine handles the basic on/off brake.  The "brake off" control is set when the handle is fully left.  The
		// "brake on" control is set when the handle is above the defined brake threshold.  Transitions always go through a middle
		// state where both controls are cleared.  This is because TCS decoders don't like these functions changing at the same
		// time in the same packet - one of the transitions is ignored.  The middle state forces the active function off before
		// turning on the other function.
		switch(brakeState)
		{
			case BRAKE_LOW_BEGIN:
			case BRAKE_LOW_WAIT:
				// Set "brake off" when below the low threshold
				setBrakeRelease();
				// Escape logic:
				//    Go to the middle state if above the brakeLowThreshold
				if(brakePosition >= brakeLowThreshold)
					brakeState = BRAKE_THR1_BEGIN;
				break;
			case BRAKE_THR1_BEGIN:
			case BRAKE_THR1_WAIT:
			case BRAKE_THR2_BEGIN:
			case BRAKE_THR2_WAIT:
			case BRAKE_THR3_BEGIN:
			case BRAKE_THR3_WAIT:
			case BRAKE_THR4_BEGIN:
			case BRAKE_THR4_WAIT:
				// Disable both "brake on" and "brake off" when between thresholds
				clearBrake();
				clearBrakeRelease();
				if(brakePosition < brakeLowThreshold)
					brakeState = BRAKE_LOW_BEGIN;
				else if(brakePosition >= brakeThreshold)
					brakeState = BRAKE_FULL_BEGIN;
				break;
			case BRAKE_FULL_BEGIN:
			case BRAKE_FULL_WAIT:
				// Set "brake on" when above the brake threshold
				setBrake();
				// Escape logic:
				//    Limit (brakeThreshold - BRAKE_HYSTERESIS) to non-negative values.  Compare the brake setting to the higher of
				//    the limited (brakeThreshold - BRAKE_HYSTERESIS) or brakeLowThreshold.  If below, go to the middle state.
				if(brakePosition < max( ((brakeThreshold > BRAKE_HYSTERESIS)?(brakeThreshold - BRAKE_HYSTERESIS):0), brakeLowThreshold ) )
					brakeState = BRAKE_THR1_BEGIN;
				break;
		}
	}		
}

