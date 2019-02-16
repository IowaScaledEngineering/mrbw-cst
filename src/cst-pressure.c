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
#include "cst-math.h"
#include "cst-pressure.h"

const uint8_t PressureA0[8] =
{
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000001,
	0b00000011,
	0b00000010,
	0b00000010,
	0b00000010
};

const uint8_t PressureA1[8] =
{
	0b00000111,
	0b00001000,
	0b00011000,
	0b00000100,
	0b00000000,
	0b00010000,
	0b00000000,
	0b00000000
};

const uint8_t PressureA2[8] =
{
	0b00011110,
	0b00010001,
	0b00010001,
	0b00000010,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000
};

const uint8_t PressureA3[8] =
{
	0b00000000,
	0b00000000,
	0b00010000,
	0b00001000,
	0b00001100,
	0b00010100,
	0b00000100,
	0b00000100
};

const uint8_t PressureB0[8] =
{
	0b00000011,
	0b00000010,
	0b00000010,
	0b00000010,
	0b00000001,
	0b00000000,
	0b00000000,
	0b00000000
};

const uint8_t PressureB1[8] =
{
	0b00010000,
	0b00000000,
	0b00000000,
	0b00001000,
	0b00010000,
	0b00010010,
	0b00001100,
	0b00000111
};

const uint8_t PressureB2[8] =
{
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000001,
	0b00000000,
	0b00000100,
	0b00000011,
	0b00011110
};

const uint8_t PressureB3[8] =
{
	0b00011100,
	0b00000100,
	0b00000100,
	0b00000100,
	0b00011000,
	0b00010000,
	0b00000000,
	0b00000000
};

uint32_t milliPressure = 0;

void updatePressure10Hz(void)
{
	milliPressure += 1000;
	if(milliPressure > 360000)
		milliPressure = 0;
}

void setupPressureChars(void)
{
	lcd_setup_custom(PRESSURE_CHAR_A0, PressureA0);
	lcd_setup_custom(PRESSURE_CHAR_A1, PressureA1);
	lcd_setup_custom(PRESSURE_CHAR_A2, PressureA2);
	lcd_setup_custom(PRESSURE_CHAR_A3, PressureA3);
	lcd_setup_custom(PRESSURE_CHAR_B0, PressureB0);
	lcd_setup_custom(PRESSURE_CHAR_B1, PressureB1);
	lcd_setup_custom(PRESSURE_CHAR_B2, PressureB2);
	lcd_setup_custom(PRESSURE_CHAR_B3, PressureB3);
}

void printPressure(void)
{
	lcd_gotoxy(0,0);
	lcd_putc(PRESSURE_CHAR_A0);
	lcd_putc(PRESSURE_CHAR_A1);
	lcd_putc(PRESSURE_CHAR_A2);
	lcd_putc(PRESSURE_CHAR_A3);
//	lcd_puts("BRK");

	lcd_gotoxy(0,1);
	lcd_putc(PRESSURE_CHAR_B0);
	lcd_putc(PRESSURE_CHAR_B1);
	lcd_putc(PRESSURE_CHAR_B2);
	lcd_putc(PRESSURE_CHAR_B3);
//	lcd_puts("PIPE");

	const uint8_t length = 8;
	float radians = (milliPressure / 1000.0) * PI / 180.0;

	int8_t x = round(cos_32(radians)*length);
	int8_t y = round(sin_32(radians)*length);

	lcd_gotoxy(5,0);
	if(x > 0)
	{
		lcd_putc(' ');
		printDec2DigWZero(x);
	}
	else
	{
		lcd_putc('-');
		printDec2DigWZero(-1*x);
	}

	lcd_gotoxy(5,1);
	if(y > 0)
	{
		lcd_putc(' ');
		printDec2DigWZero(y);
	}
	else
	{
		lcd_putc('-');
		printDec2DigWZero(-1*y);
	}

}

