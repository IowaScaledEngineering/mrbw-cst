/*************************************************************************
Title:    DCC function handling code
Authors:  Michael D. Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     cst-functions.c
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

#include "lcd.h"
#include "cst-functions.h"

// Define these values explicitly, for EEPROM backwards compatibility
// Bottom 5 bits are the actual function number
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
} Functions;

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
} Controls;

typedef struct
{
	const char *name;
	Functions fn;
	uint8_t latching;
} ControlsData;

static ControlsData controls[] = {
	[HORN_FN]                = {.name = "HORN"},
	[BELL_FN]                = {.name = "BELL"},
	[BRAKE_FN]               = {.name = "BRAKE"},
	[BRAKE_OFF_FN]           = {.name = "BRK OFF"},
	[AUX_FN]                 = {.name = "AUX"},
	[ENGINE_ON_FN]           = {.name = "ENG ON"},
	[ENGINE_OFF_FN]          = {.name = "ENG STOP"},
	[THR_UNLOCK_FN]          = {.name = "THR UNLK"},
	[REV_SWAP_FN]            = {.name = "REV SWAP"},
	[FRONT_DIM1_FN]          = {.name = "F.DIM #1"},
	[FRONT_DIM2_FN]          = {.name = "F.DIM #2"},
	[FRONT_HEADLIGHT_FN]     = {.name = "F.HEAD"},
	[FRONT_DITCH_FN]         = {.name = "F.DITCH"},
	[REAR_DIM1_FN]           = {.name = "R.DIM #1"},
	[REAR_DIM2_FN]           = {.name = "R.DIM #2"},
	[REAR_HEADLIGHT_FN]      = {.name = "R.HEAD"},
	[REAR_DITCH_FN]          = {.name = "R.DITCH"},
	[UP_FN]                  = {.name = "UP BTN", .latching = 1},
	[DOWN_FN]                = {.name = "DOWN BTN", .latching = 1},
};

static Controls currentControl = 0;

void printCurrentControlName(void)
{
	lcd_puts(controls[currentControl].name);
}

void printCurrentControlFunction(void)
{
	switch(controls[currentControl].fn)
	{
		case FN_OFF:
			lcd_puts("F--     ");
			break;
		case FN_EMRG:
			lcd_puts("EMRG BRK");
			break;
		case F00_MOM: case F10_MOM: case F20_MOM:
		case F01_MOM: case F11_MOM: case F21_MOM:
		case F02_MOM: case F12_MOM: case F22_MOM:
		case F03_MOM: case F13_MOM: case F23_MOM:  
		case F04_MOM: case F14_MOM: case F24_MOM:
		case F05_MOM: case F15_MOM: case F25_MOM:
		case F06_MOM: case F16_MOM: case F26_MOM:
		case F07_MOM: case F17_MOM: case F27_MOM:
		case F08_MOM: case F18_MOM: case F28_MOM:
		case F09_MOM: case F19_MOM:
			lcd_putc('F');
			printDec2DigWZero((controls[currentControl].fn) & 0x1F);
			if(controls[currentControl].latching)
				lcd_puts("  MOM");
			else
				lcd_puts("     ");
			break;
		case F00_LAT: case F10_LAT: case F20_LAT:
		case F01_LAT: case F11_LAT: case F21_LAT:
		case F02_LAT: case F12_LAT: case F22_LAT:
		case F03_LAT: case F13_LAT: case F23_LAT:  
		case F04_LAT: case F14_LAT: case F24_LAT:
		case F05_LAT: case F15_LAT: case F25_LAT:
		case F06_LAT: case F16_LAT: case F26_LAT:
		case F07_LAT: case F17_LAT: case F27_LAT:
		case F08_LAT: case F18_LAT: case F28_LAT:
		case F09_LAT: case F19_LAT:
			lcd_putc('F');
			printDec2DigWZero((controls[currentControl].fn) & 0x1F);
			if(controls[currentControl].latching)
				lcd_puts("  LAT");
			else
				lcd_puts("     ");
			break;
	}
}

void advanceCurrentControl(void)
{
	currentControl++;
	if(currentControl >= LAST_FN)
		currentControl = 0;
}

void incrementCurrentControlFunction(void)
{
	if(FN_OFF == controls[currentControl].fn)
	{
		controls[currentControl].fn = F00_MOM;
	}
	else if(FN_EMRG == controls[currentControl].fn)
	{
		// Do nothing
	}
	else if(F28_MOM == controls[currentControl].fn)
	{
		if(controls[currentControl].latching)
		{
			controls[currentControl].fn = F00_LAT;
		}
		else
		{
			controls[currentControl].fn = FN_EMRG;
		}
	}
	else if(F28_LAT == controls[currentControl].fn)
	{
		controls[currentControl].fn = FN_EMRG;
	}
	else
	{
		// Take advantage of the bottom 5 bits being the function number
		controls[currentControl].fn++;
	}
	
/*	if((*functionPtr) & OFF_FUNCTION)*/
/*	{*/
/*		(*functionPtr) = 0;  // Turn on*/
/*	}*/
/*	else*/
/*	{*/
/*		if(((*functionPtr) & 0x1F) < 28)*/
/*			(*functionPtr)++;       // Increment*/
/*		else if(allowLatch && !((*functionPtr) & LATCH_FUNCTION))*/
/*			(*functionPtr) = LATCH_FUNCTION;  // Set latch bit, reset function number to zero*/
/*		else*/
/*			(*functionPtr) = ((*functionPtr) & LATCH_FUNCTION) + 28;    // Saturate, preserving latch bit*/
}

void decrementCurrentControlFunction(void)
{
	if(FN_OFF == controls[currentControl].fn)
	{
		// Do nothing
	}
	else if(FN_EMRG == controls[currentControl].fn)
	{
		if(controls[currentControl].latching)
		{
			controls[currentControl].fn = F28_LAT;
		}
		else
		{
			controls[currentControl].fn = F28_MOM;
		}
	}
	else if(F00_LAT == controls[currentControl].fn)
	{
		controls[currentControl].fn = F03_MOM;
	}
	else if(F00_MOM == controls[currentControl].fn)
	{
		controls[currentControl].fn = FN_OFF;
	}
	else
	{
		// Take advantage of the bottom 5 bits being the function number
		controls[currentControl].fn--;
	}

/*	if(~((*functionPtr) & OFF_FUNCTION))*/
/*	{*/
/*		// Not OFF...*/
/*		if(((*functionPtr) & 0x1F) > 0)*/
/*			(*functionPtr)--;       // Decrement*/
/*		else if(allowLatch && ((*functionPtr) & LATCH_FUNCTION))*/
/*			(*functionPtr) = 28;    // Unset latch bit, reset function number to 28*/
/*		else*/
/*			(*functionPtr) = OFF_FUNCTION;  // Turn off*/
/*	}*/
}

