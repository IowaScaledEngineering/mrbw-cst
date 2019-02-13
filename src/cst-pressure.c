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

const uint8_t PressureA0[8] =
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
			lcd_setup_custom(PRESSURE_CHAR_A0, PressureA0);
			break;
	}
}

void printPressure(void)
{
	lcd_gotoxy(0,0);
	lcd_putc(PRESSURE_CHAR_A0);
	lcd_putc(PRESSURE_CHAR_A1);
	lcd_putc(PRESSURE_CHAR_A2);
	lcd_putc(PRESSURE_CHAR_A3);
	lcd_puts("BRK");
	lcd_putc(0x7E);

	lcd_gotoxy(0,1);
	lcd_putc(PRESSURE_CHAR_B0);
	lcd_putc(PRESSURE_CHAR_B1);
	lcd_putc(PRESSURE_CHAR_B2);
	lcd_putc(PRESSURE_CHAR_B3);
	lcd_puts("PIPE");
}

