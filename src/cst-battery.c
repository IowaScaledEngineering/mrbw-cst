/*************************************************************************
Title:    Battery support for Control Stand Throttle
Authors:  Michael D. Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     cst-battery.c
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2017 Michael Petersen & Nathan Holmes
    
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

#include "lcd.h"
#include "cst-common.h"
#include "cst-battery.h"

// VBATT_OKAY is the battery voltage (in centivolts) above which the batteries are considered "fine"
#define VBATT_OKAY  220 
// VBATT_WARN is the battery voltage (in centivolts) above which the batteries are considered a warning, but not
//  in critical shape.  This should *always* be less than VBATT_OKAY
#define VBATT_WARN  200

const uint8_t BatteryFull[8] =
{
	0b00001110,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00000000
};

const uint8_t BatteryHalf[8] =
{
	0b00001110,
	0b00011011,
	0b00010001,
	0b00010001,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00000000
};
const uint8_t BatteryEmpty[8] =
{
	0b00001110,
	0b00011011,
	0b00010001,
	0b00010001,
	0b00010001,
	0b00010001,
	0b00011111,
	0b00000000
};

BatteryState batteryState = FULL;
BatteryState lastBatteryState = FULL;
uint16_t batteryVoltage = 0;

void setupBatteryChar(void)
{
	switch(batteryState)
	{
		case FULL:
			lcd_setup_custom(BATTERY_CHAR, BatteryFull);
			break;
		case HALF:
			lcd_setup_custom(BATTERY_CHAR, BatteryHalf);
			break;
		case EMPTY:
			lcd_setup_custom(BATTERY_CHAR, BatteryEmpty);
			break;
	}
}

uint8_t getBatteryVoltage(void)
{
	return (batteryVoltage >> 8);
}

#define BATTERY_FILTER_COEF 16

void setBatteryVoltage(uint8_t voltage)
{
	uint16_t voltage16 = (uint16_t)voltage << 8;

	// Filter the voltage
	if(voltage16 > batteryVoltage)
		batteryVoltage += (voltage16 - batteryVoltage) / BATTERY_FILTER_COEF;
	else if(voltage16 < batteryVoltage)
		batteryVoltage -= (batteryVoltage - voltage16) / BATTERY_FILTER_COEF;
	
	if (getBatteryVoltage() >= (VBATT_OKAY/2))  // Divide by 2 since batteryVoltage LSB = 20mV
		batteryState = FULL;
	else if (getBatteryVoltage() >= (VBATT_WARN/2))  // Divide by 2 since batteryVoltage LSB = 20mV
		batteryState = HALF;
	else
		batteryState = EMPTY;
}

void printBattery(void)
{
	if(batteryState != lastBatteryState)
	{
		setupBatteryChar();
		lastBatteryState = batteryState;
	}

	lcd_gotoxy(0,0);
	lcd_putc(BATTERY_CHAR);
}

