/*************************************************************************
Title:    LCD support for Control Stand Throttle
Authors:  Michael D. Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     cst-lcd.c
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
#include "cst-hardware.h"
#include "cst-lcd.h"
#include "cst-hardware.h"
#include "cst-time.h"
#include "cst-tonnage.h"

const uint8_t Bell[8] =
{
	0b00000100,
	0b00001110,
	0b00001110,
	0b00001110,
	0b00011111,
	0b00000000,
	0b00000100,
	0b00000000
};

const uint8_t Horn[8] =
{
	0b00000000,
	0b00000001,
	0b00010011,
	0b00011111,
	0b00010011,
	0b00000001,
	0b00000000,
	0b00000000
};

const uint8_t SoftkeyInactive[8] =
{
	0b00000000,
	0b00001110,
	0b00010001,
	0b00010001,
	0b00010001,
	0b00001110,
	0b00000000,
	0b00000000
};
const uint8_t SoftkeyActive[8] =
{
	0b00000000,
	0b00001110,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00001110,
	0b00000000,
	0b00000000
};

void setupDiagChars(void)
{
	lcd_setup_custom(BELL_CHAR, Bell);
	lcd_setup_custom(HORN_CHAR, Horn);
}

void setupSoftkeyChars(void)
{
	lcd_setup_custom(FUNCTION_INACTIVE_CHAR, SoftkeyInactive);
	lcd_setup_custom(FUNCTION_ACTIVE_CHAR, SoftkeyActive);
}

// Splash Screen Characters
const uint8_t Splash1[8] =
{
	0b00011100,
	0b00010010,
	0b00010010,
	0b00011100,
	0b00010000,
	0b00010000,
	0b00000000,
	0b00000000
};

const uint8_t Splash2[8] =
{
	0b00011100,
	0b00010010,
	0b00010010,
	0b00011100,
	0b00010010,
	0b00010010,
	0b00000000,
	0b00000000
};

const uint8_t Splash3[8] =
{
	0b00001100,
	0b00010010,
	0b00010010,
	0b00010010,
	0b00010010,
	0b00001100,
	0b00000000,
	0b00000000
};

const uint8_t Splash4[8] =
{
	0b00011100,
	0b00001001,
	0b00001001,
	0b00001001,
	0b00001001,
	0b00001000,
	0b00000000,
	0b00000000
};

const uint8_t Splash5A[8] =
{
	0b00011001,
	0b00000101,
	0b00000101,
	0b00000101,
	0b00000101,
	0b00011000,
	0b00000000,
	0b00000000
};

const uint8_t Splash5C[8] =
{
	0b00011001,
	0b00000101,
	0b00000100,
	0b00000101,
	0b00000111,
	0b00011100,
	0b00000100,
	0b00000011
};

const uint8_t Splash6A[8] =
{
	0b00011111,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00011111,
	0b00000000,
	0b00000000,
	0b00000000
};

const uint8_t Splash6B[8] =
{
	0b00011111,
	0b00000001,
	0b00000001,
	0b00000001,
	0b00011101,
	0b00000001,
	0b00000001,
	0b00000000
};

const uint8_t Splash6C[8] =
{
	0b00011111,
	0b00001111,
	0b00011110,
	0b00011101,
	0b00011010,
	0b00010100,
	0b00011000,
	0b00010000
};

const uint8_t Splash7A[8] =
{
	0b00011111,
	0b00011111,
	0b00001111,
	0b00000111,
	0b00011011,
	0b00000001,
	0b00000000,
	0b00000000
};

const uint8_t Splash7B[8] =
{
	0b00011111,
	0b00011100,
	0b00011100,
	0b00011100,
	0b00011101,
	0b00000100,
	0b00000100,
	0b00011000
};

const uint8_t Splash7C[8] =
{
	0b00011111,
	0b00001000,
	0b00010000,
	0b00000000,
	0b00011111,
	0b00000000,
	0b00000000,
	0b00000000
};

const uint8_t Splash8A[8] =
{
	0b00011100,
	0b00010100,
	0b00011000,
	0b00011100,
	0b00011110,
	0b00011001,
	0b00011001,
	0b00001110
};

const uint8_t Splash8B[8] =
{
	0b00011100,
	0b00000100,
	0b00000100,
	0b00000100,
	0b00011100,
	0b00000000,
	0b00000000,
	0b00000000
};

void displaySplashScreen(void)
{
	uint8_t i;
	
	lcd_clrscr();

	lcd_setup_custom(0, Splash1);
	lcd_setup_custom(1, Splash2);
	lcd_setup_custom(2, Splash3);
	lcd_setup_custom(3, Splash4);
	lcd_setup_custom(4, Splash5A);
	lcd_setup_custom(5, Splash6A);
	lcd_setup_custom(6, Splash7A);
	lcd_setup_custom(7, Splash8A);

	lcd_gotoxy(0,0);
	for(i=0; i<8; i++)
		lcd_putc(i);
	
	lcd_gotoxy(0,1);
	lcd_puts("THROTTLE");

	wait100ms(8);

	lcd_setup_custom(5, Splash6B);
	lcd_setup_custom(6, Splash7B);
	lcd_setup_custom(7, Splash8B);

	wait100ms(4);

	lcd_setup_custom(4, Splash5C);
	lcd_setup_custom(5, Splash6C);
	lcd_setup_custom(6, Splash7C);

	wait100ms(15);
}

void initLCD(void)
{
	lcd_init(LCD_DISP_ON);
	enableLCDBacklight();

	displaySplashScreen();

	lcd_clrscr();

	// Reload the LCD characters
	setupBatteryChar();
	setupSoftkeyChars();
	setupTonnageChars();
	setupClockChars();
}


