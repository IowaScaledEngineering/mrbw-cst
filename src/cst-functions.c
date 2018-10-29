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

#include <avr/eeprom.h>

#include "lcd.h"
#include "cst-eeprom.h"
#include "cst-functions.h"

typedef struct
{
	const char *name;
	FunctionValues fn;
	uint16_t eeAddr;
	uint8_t latching;
} FunctionData;

static FunctionData functions[] = {
	[HORN_FN]                = {.name = "HORN", .eeAddr = EE_HORN_FUNCTION},
	[BELL_FN]                = {.name = "BELL", .eeAddr = EE_BELL_FUNCTION},
	[BRAKE_FN]               = {.name = "BRAKE", .eeAddr = EE_BRAKE_FUNCTION},
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

static Functions currentFunction = 0;

void printCurrentFunctionName(void)
{
	lcd_puts(functions[currentFunction].name);
}

void printCurrentFunctionFunction(void)
{
	switch(functions[currentFunction].fn)
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
			printDec2DigWZero((functions[currentFunction].fn) & 0x1F);
			if(functions[currentFunction].latching)
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
			printDec2DigWZero((functions[currentFunction].fn) & 0x1F);
			if(functions[currentFunction].latching)
				lcd_puts("  LAT");
			else
				lcd_puts("     ");
			break;
	}
}

void advanceCurrentFunction(void)
{
	currentFunction++;
	if(currentFunction >= LAST_FN)
		currentFunction = 0;
}

void resetCurrentFunction(void)
{
	currentFunction = 0;
}

void incrementCurrentFunctionFunction(void)
{
	if(FN_OFF == functions[currentFunction].fn)
	{
		functions[currentFunction].fn = F00_MOM;
	}
	else if(FN_EMRG == functions[currentFunction].fn)
	{
		// Do nothing
	}
	else if(F28_MOM == functions[currentFunction].fn)
	{
		if(functions[currentFunction].latching)
		{
			functions[currentFunction].fn = F00_LAT;
		}
		else
		{
			functions[currentFunction].fn = FN_EMRG;
		}
	}
	else if(F28_LAT == functions[currentFunction].fn)
	{
		functions[currentFunction].fn = FN_EMRG;
	}
	else
	{
		// Take advantage of the bottom 5 bits being the function number
		functions[currentFunction].fn++;
	}
}

void decrementCurrentFunctionFunction(void)
{
	if(FN_OFF == functions[currentFunction].fn)
	{
		// Do nothing
	}
	else if(FN_EMRG == functions[currentFunction].fn)
	{
		if(functions[currentFunction].latching)
		{
			functions[currentFunction].fn = F28_LAT;
		}
		else
		{
			functions[currentFunction].fn = F28_MOM;
		}
	}
	else if(F00_LAT == functions[currentFunction].fn)
	{
		functions[currentFunction].fn = F28_MOM;
	}
	else if(F00_MOM == functions[currentFunction].fn)
	{
		functions[currentFunction].fn = FN_OFF;
	}
	else
	{
		// Take advantage of the bottom 5 bits being the function number
		functions[currentFunction].fn--;
	}
}

void readFunctionConfiguration(void)
{
	functions[HORN_FN].fn = eeprom_read_byte((uint8_t*)EE_HORN_FUNCTION);
	functions[BELL_FN].fn = eeprom_read_byte((uint8_t*)EE_BELL_FUNCTION);
	functions[FRONT_DIM1_FN].fn = eeprom_read_byte((uint8_t*)EE_FRONT_DIM1_FUNCTION);
	functions[FRONT_DIM2_FN].fn = eeprom_read_byte((uint8_t*)EE_FRONT_DIM2_FUNCTION);
	functions[FRONT_HEADLIGHT_FN].fn = eeprom_read_byte((uint8_t*)EE_FRONT_HEADLIGHT_FUNCTION);
	functions[FRONT_DITCH_FN].fn = eeprom_read_byte((uint8_t*)EE_FRONT_DITCH_FUNCTION);
	functions[REAR_DIM1_FN].fn = eeprom_read_byte((uint8_t*)EE_REAR_DIM1_FUNCTION);
	functions[REAR_DIM2_FN].fn = eeprom_read_byte((uint8_t*)EE_REAR_DIM2_FUNCTION);
	functions[REAR_HEADLIGHT_FN].fn = eeprom_read_byte((uint8_t*)EE_REAR_HEADLIGHT_FUNCTION);
	functions[REAR_DITCH_FN].fn = eeprom_read_byte((uint8_t*)EE_REAR_DITCH_FUNCTION);
	functions[BRAKE_FN].fn = eeprom_read_byte((uint8_t*)EE_BRAKE_FUNCTION);
	functions[BRAKE_OFF_FN].fn = eeprom_read_byte((uint8_t*)EE_BRAKE_OFF_FUNCTION);
	functions[AUX_FN].fn = eeprom_read_byte((uint8_t*)EE_AUX_FUNCTION);
	functions[ENGINE_ON_FN].fn = eeprom_read_byte((uint8_t*)EE_ENGINE_ON_FUNCTION);
	functions[ENGINE_OFF_FN].fn = eeprom_read_byte((uint8_t*)EE_ENGINE_OFF_FUNCTION);
	functions[UP_FN].fn = eeprom_read_byte((uint8_t*)EE_UP_BUTTON_FUNCTION);
	functions[DOWN_FN].fn = eeprom_read_byte((uint8_t*)EE_DOWN_BUTTON_FUNCTION);
	functions[THR_UNLOCK_FN].fn = eeprom_read_byte((uint8_t*)EE_THR_UNLOCK_FUNCTION);
	functions[REV_SWAP_FN].fn = eeprom_read_byte((uint8_t*)EE_REV_SWAP_FUNCTION);
}

void writeFunctionConfiguration(void)
{
	eeprom_write_byte((uint8_t*)EE_HORN_FUNCTION, functions[HORN_FN].fn);
	eeprom_write_byte((uint8_t*)EE_BELL_FUNCTION, functions[BELL_FN].fn);
	eeprom_write_byte((uint8_t*)EE_FRONT_DIM1_FUNCTION, functions[FRONT_DIM1_FN].fn);
	eeprom_write_byte((uint8_t*)EE_FRONT_DIM2_FUNCTION, functions[FRONT_DIM2_FN].fn);
	eeprom_write_byte((uint8_t*)EE_FRONT_HEADLIGHT_FUNCTION, functions[FRONT_HEADLIGHT_FN].fn);
	eeprom_write_byte((uint8_t*)EE_FRONT_DITCH_FUNCTION, functions[FRONT_DITCH_FN].fn);
	eeprom_write_byte((uint8_t*)EE_REAR_DIM1_FUNCTION, functions[REAR_DIM1_FN].fn);
	eeprom_write_byte((uint8_t*)EE_REAR_DIM2_FUNCTION, functions[REAR_DIM2_FN].fn);
	eeprom_write_byte((uint8_t*)EE_REAR_HEADLIGHT_FUNCTION, functions[REAR_HEADLIGHT_FN].fn);
	eeprom_write_byte((uint8_t*)EE_REAR_DITCH_FUNCTION, functions[REAR_DITCH_FN].fn);
	eeprom_write_byte((uint8_t*)EE_BRAKE_FUNCTION, functions[BRAKE_FN].fn);
	eeprom_write_byte((uint8_t*)EE_BRAKE_OFF_FUNCTION, functions[BRAKE_OFF_FN].fn);
	eeprom_write_byte((uint8_t*)EE_AUX_FUNCTION, functions[AUX_FN].fn);
	eeprom_write_byte((uint8_t*)EE_ENGINE_ON_FUNCTION, functions[ENGINE_ON_FN].fn);
	eeprom_write_byte((uint8_t*)EE_ENGINE_OFF_FUNCTION, functions[ENGINE_OFF_FN].fn);
	eeprom_write_byte((uint8_t*)EE_UP_BUTTON_FUNCTION, functions[UP_FN].fn);
	eeprom_write_byte((uint8_t*)EE_DOWN_BUTTON_FUNCTION, functions[DOWN_FN].fn);
	eeprom_write_byte((uint8_t*)EE_THR_UNLOCK_FUNCTION, functions[THR_UNLOCK_FN].fn);
	eeprom_write_byte((uint8_t*)EE_REV_SWAP_FUNCTION, functions[REV_SWAP_FN].fn);
}

void resetFunctionConfiguration(void)
{
	functions[HORN_FN].fn = F02_MOM;
	functions[BELL_FN].fn = F01_MOM;
	functions[FRONT_DIM1_FN].fn = FN_OFF;
	functions[FRONT_DIM2_FN].fn = FN_OFF;
	functions[FRONT_HEADLIGHT_FN].fn = F00_MOM;
	functions[FRONT_DITCH_FN].fn = FN_OFF;
	functions[REAR_DIM1_FN].fn = FN_OFF;
	functions[REAR_DIM2_FN].fn = FN_OFF;
	functions[REAR_HEADLIGHT_FN].fn = F00_MOM;
	functions[REAR_DITCH_FN].fn = FN_OFF;
	functions[BRAKE_FN].fn = F10_MOM;
	functions[BRAKE_OFF_FN].fn = FN_OFF;
	functions[AUX_FN].fn = F09_MOM;
	functions[ENGINE_ON_FN].fn = F08_MOM;
	functions[ENGINE_OFF_FN].fn = FN_OFF;
	functions[UP_FN].fn = F05_MOM;
	functions[DOWN_FN].fn = F06_MOM;
	functions[THR_UNLOCK_FN].fn = FN_OFF;
	functions[REV_SWAP_FN].fn = FN_OFF;
}

