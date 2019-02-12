/*************************************************************************
Title:    Air pressure support for Control Stand Throttle
Authors:  Michael D. Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     cst-pressure.c
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2019 Michael Petersen & Nathan Holmes
    
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
#include "cst-pressure.h"

const uint8_t Pressure00[8] =
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

typedef enum
{
	PRESSURE_OFF = 0,
	PRESSURE_1,
	PRESSURE_2,
	PRESSURE_3,
	PRESSURE_4,
	PRESSURE_5,
	PRESSURE_6,
	PRESSURE_7,
	PRESSURE_8,
	PRESSURE_DONE
} PressureState;

PressureState pressureState;

void setupPressureChars(void)
{
	switch(pressureState)
	{
		case PRESSURE_OFF:
			lcd_setup_custom(PRESSURE_OFF, Pressure00);
			break;
	}
}

uint8_t getBatteryVoltage(void)
{
	return (batteryVoltageFilter >> 8);
}

BatteryState getBatteryState(void)
{
	return batteryState;
}

#define BATTERY_FILTER_COEF 16

void setBatteryVoltage(uint8_t voltage)
{
	uint16_t voltage16 = (uint16_t)voltage << 8;

	// Filter the voltage
	if(0 == batteryVoltageFilter)
		batteryVoltageFilter = voltage16;
	else if(voltage16 > batteryVoltageFilter)
		batteryVoltageFilter += (voltage16 - batteryVoltageFilter) / BATTERY_FILTER_COEF;
	else if(voltage16 < batteryVoltageFilter)
		batteryVoltageFilter -= (batteryVoltageFilter - voltage16) / BATTERY_FILTER_COEF;

	// Multiply by 5 since getBatteryVoltage LSB = 20mV, threshold LSB = 100mV
	// Ternary statement adds 100mV hysteresis if currently in next lower state - prevents jittering
	if (getBatteryVoltage() >= (5*batteryOkay + ((HALF == batteryState)?5:0)))
		batteryState = FULL;
	else if (getBatteryVoltage() >= (5*batteryWarn + ((EMPTY == batteryState)?5:0)))
		batteryState = HALF;
	else if ((getBatteryVoltage() >= (5*batteryCritical + ((CRITICAL == batteryState)?5:0))) || (0 == batteryCritical))
		batteryState = EMPTY;
	else  // Only if critical level is non-zero
		batteryState = CRITICAL;
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

uint8_t getBatteryOkay(void)
{
	return batteryOkay;
}

uint8_t getBatteryWarn(void)
{
	return batteryWarn;
}

uint8_t getBatteryCritical(void)
{
	return batteryCritical;
}

void setBatteryLevels(uint8_t decivoltsOkay, uint8_t decivoltsWarn, uint8_t decivoltsCritical)
{
	if(0xFF == decivoltsOkay)
		batteryOkay = VBATT_OKAY_DEFAULT;
	else
		batteryOkay = decivoltsOkay;

	if(0xFF == decivoltsWarn)
		batteryWarn = VBATT_WARN_DEFAULT;
	else
		batteryWarn = decivoltsWarn;

	if(0xFF == decivoltsCritical)
		batteryCritical = VBATT_CRITICAL_DEFAULT;
	else
		batteryCritical = decivoltsCritical;
	
	// Do some basic range checking
	if(batteryOkay > VBATT_OKAY_MAX)
		batteryOkay = VBATT_OKAY_MAX;
	else if(batteryOkay < VBATT_OKAY_MIN)
		batteryOkay = VBATT_OKAY_MIN;

	if(batteryWarn > VBATT_WARN_MAX)
		batteryWarn = VBATT_WARN_MAX;
	else if(batteryWarn < VBATT_WARN_MIN)
		batteryWarn = VBATT_WARN_MIN;
	
	if(batteryCritical > VBATT_CRITICAL_MAX)
		batteryCritical = VBATT_CRITICAL_MAX;
	else if(batteryCritical < VBATT_CRITICAL_MIN)
		batteryCritical = VBATT_CRITICAL_MIN;

	// Check relative levels.  Can push the next lower one down, but cannot push the next higher one up
	if((batteryOkay <= batteryWarn) && (batteryWarn > 0))
		batteryWarn = batteryOkay - 1;
	if((batteryWarn <= batteryCritical) && (batteryCritical > 0))
		batteryCritical = batteryWarn - 1;
}

