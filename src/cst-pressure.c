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
#include <string.h>

#include <util/delay.h>
#include <avr/wdt.h>

#include "lcd.h"
#include "cst-common.h"
#include "cst-math.h"
#include "cst-pressure.h"

#define CANVAS_ROWS       16
#define CANVAS_COLS       20
#define ROWS_PER_CHAR      8
#define COLS_PER_CHAR      5

#define MAX_PRESSURE         90
#define RESET_PRESSURE       65
#define BRAKE_TEST_DELTA     30

// East = 0deg, South = 90deg, West = 180deg, North = 270deg
#define MIN_ANGLE        112
#define MAX_ANGLE        300

typedef enum
{
	IDLE,
	BRAKE_TEST,
	PUMPING_WAIT,
	PUMPING,
	DONE
} PumpState;

static PumpState pumpState = IDLE;

static uint8_t brakeTest = 0;

static uint8_t canvas[CANVAS_ROWS / ROWS_PER_CHAR][CANVAS_COLS / COLS_PER_CHAR][ROWS_PER_CHAR];

static uint8_t pressureConfig = 0;

static uint32_t milliPressure = (uint32_t)RESET_PRESSURE * 1000;
static uint32_t maxMilliPressure = (uint32_t)MAX_PRESSURE * 1000;
static uint32_t milliPressureReservoir = (uint32_t)MAX_PRESSURE * 1000;

static volatile uint8_t compressorStopTimer = 0;

const uint8_t Gauge[CANVAS_ROWS / ROWS_PER_CHAR][CANVAS_COLS / COLS_PER_CHAR][ROWS_PER_CHAR] = 
{
	{
		{
			0b00000000,
			0b00000000,
			0b00000000,
			0b00000001,
			0b00000011,
			0b00000010,
			0b00000010,
			0b00000010
		},
		{
			0b00000111,
			0b00001000,
			0b00011000,
			0b00000100,
			0b00000000,
			0b00010000,
			0b00000000,
			0b00000000
		},
		{
			0b00011110,
			0b00010001,
			0b00010001,
			0b00000010,
			0b00000000,
			0b00000000,
			0b00000000,
			0b00000000
		},
		{
			0b00000000,
			0b00000000,
			0b00010000,
			0b00001000,
			0b00001100,
			0b00010100,
			0b00000100,
			0b00000100
		}
	},
	{
		{
			0b00000011,
			0b00000010,
			0b00000010,
			0b00000010,
			0b00000001,
			0b00000000,
			0b00000000,
			0b00000000
		},
		{
			0b00010000,
			0b00000000,
			0b00000000,
			0b00001000,
			0b00010000,
			0b00010010,
			0b00001100,
			0b00000111
		},
		{
			0b00000000,
			0b00000000,
			0b00000000,
			0b00000000,
			0b00000000,
			0b00000000,
			0b00000001,
			0b00011110
		},
		{
			0b00011100,
			0b00000100,
			0b00000100,
			0b00000100,
			0b00001000,
			0b00010000,
			0b00000000,
			0b00000000
		}
	}
};

void updatePressure10Hz(void)
{
	if(PUMPING == pumpState)
		milliPressure += ((maxMilliPressure - milliPressure) / ((uint16_t)16 << getPumpRate())) + 10;  // + to keep it going when the first part reaches zero
	
	if(compressorStopTimer)
		compressorStopTimer--;
}

void plot(uint8_t x, uint8_t y)
{
	uint8_t row = y / ROWS_PER_CHAR;
	if(row >= (CANVAS_ROWS / ROWS_PER_CHAR))
		return;
	uint8_t col = x / COLS_PER_CHAR;
	if(col >= (CANVAS_COLS / COLS_PER_CHAR))
		return;
	canvas[row][col][y % ROWS_PER_CHAR] |= 1 << ((COLS_PER_CHAR - 1) - (x % COLS_PER_CHAR));
}

/* https://en.wikipedia.org/wiki/Bresenham's_line_algorithm */
void plotLineLow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
	int8_t dx = x1 - x0;
	int8_t dy = y1 - y0;
	int8_t yi = 1;
	if(dy < 0)
	{
		yi = -1;
		dy = -dy;
	}
	int8_t D = 2*dy - dx;
	int8_t y = y0;
	int8_t x;

	for(x=x0; x<=x1; x++)
	{
		plot(x,y);
		if(D > 0)
		{
			y = y + yi;
			D = D - 2*dx;
		}
		D = D + 2*dy;
	}
}

/* https://en.wikipedia.org/wiki/Bresenham's_line_algorithm */
void plotLineHigh(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
	int8_t dx = x1 - x0;
	int8_t dy = y1 - y0;
	int8_t xi = 1;
	if(dx < 0)
	{
		xi = -1;
		dx = -dx;
	}
	int8_t D = 2*dx - dy;
	int8_t x = x0;
	int8_t y;
	
	for(y=y0; y<=y1; y++)
	{
		plot(x,y);
		if(D > 0)
		{
			x = x + xi;
			D = D - 2*dy;
		}
		D = D + 2*dx;
	}
}

/* https://en.wikipedia.org/wiki/Bresenham's_line_algorithm */
void plotLine(int8_t x0, int8_t y0, int8_t x1, int8_t y1)
{
	//FIXME: do the comparison below differently to handle unsigned int?
	if(abs(y1 - y0) < abs(x1 - x0))
	{
		if(x0 > x1)
			plotLineLow(x1, y1, x0, y0);
		else
			plotLineLow(x0, y0, x1, y1);
	}
	else
	{
		if(y0 > y1)
			plotLineHigh(x1, y1, x0, y0);
		else
			plotLineHigh(x0, y0, x1, y1);
	}
}

#define ORIGIN_X  10
#define ORIGIN_Y   8
#define LENGTH     7

void setupPressureChars(void)
{
	float degrees = ((MAX_ANGLE - MIN_ANGLE) * ((milliPressure / 1000.0) / MAX_PRESSURE)) + MIN_ANGLE;
	float radians = degrees * PI / 180.0;

	int8_t x = round(cos_32(radians)*LENGTH);
	int8_t y = round(sin_32(radians)*LENGTH);

	memcpy(canvas, Gauge, sizeof(canvas));

	plotLine(ORIGIN_X, ORIGIN_Y, ORIGIN_X + x, ORIGIN_Y + y);

	lcd_setup_custom(PRESSURE_CHAR_A0, canvas[0][0]);
	lcd_setup_custom(PRESSURE_CHAR_A1, canvas[0][1]);
	lcd_setup_custom(PRESSURE_CHAR_A2, canvas[0][2]);
	lcd_setup_custom(PRESSURE_CHAR_A3, canvas[0][3]);
	lcd_setup_custom(PRESSURE_CHAR_B0, canvas[1][0]);
	lcd_setup_custom(PRESSURE_CHAR_B1, canvas[1][1]);
	lcd_setup_custom(PRESSURE_CHAR_B2, canvas[1][2]);
	lcd_setup_custom(PRESSURE_CHAR_B3, canvas[1][3]);
}

void processPressure(uint8_t brakePcnt)
{
	switch(pumpState)
	{
		case IDLE:
			if(brakePcnt > 10)
				pumpState = PUMPING_WAIT;  // Wait for brake to be released
			else
				pumpState = PUMPING;
			break;
		case PUMPING_WAIT:
			if(brakePcnt < 10)
			{
				pumpState = PUMPING;
			}
			break;
		case PUMPING:
			if(milliPressure >= maxMilliPressure)
			{
				milliPressure = maxMilliPressure;
				pumpState = DONE;
			}
			// Fall through to process brakePcnt in both cases
		case DONE:
			if(brakePcnt > 10)
			{
				milliPressureReservoir = milliPressure;  // Save current pressure
				pumpState = BRAKE_TEST;
				compressorStopTimer = 10;  // 1 second
			}
			break;
		case BRAKE_TEST:
			if(brakePcnt < 10)
			{
				pumpState = PUMPING;
			}
			else
			{
				uint32_t milliPressureDelta = ((uint32_t)BRAKE_TEST_DELTA * 1000) * brakePcnt / 100;
				if(milliPressureDelta > milliPressureReservoir)
					milliPressure = 0;
				milliPressure = min(milliPressure,  milliPressureReservoir - milliPressureDelta);
			}
			break;
	}
}

void printPressure(void)
{
	setupPressureChars();

	lcd_gotoxy(0,0);
	lcd_putc(PRESSURE_CHAR_A0);
	lcd_putc(PRESSURE_CHAR_A1);
	lcd_putc(PRESSURE_CHAR_A2);
	lcd_putc(PRESSURE_CHAR_A3);

	lcd_gotoxy(0,1);
	lcd_putc(PRESSURE_CHAR_B0);
	lcd_putc(PRESSURE_CHAR_B1);
	lcd_putc(PRESSURE_CHAR_B2);
	lcd_putc(PRESSURE_CHAR_B3);

	lcd_gotoxy(4,0);
	lcd_putc(' ');
	printDec3Dig(milliPressure/1000);
	lcd_gotoxy(4,1);
	lcd_puts(" PSI");
}

void resetPressure(void)
{
	uint8_t randNum;
	randNum = (uint8_t)rand();  // Get random number between 0 to 255

	milliPressure = (uint32_t)(RESET_PRESSURE + (randNum&0x0F) - 7) * 1000;  // Randomize the starting pressure, -7 to +8
	maxMilliPressure = (uint32_t)(MAX_PRESSURE + (randNum&0x03) - 1) * 1000;  // Randomize the max pressure, -1 to +2

	pumpState = IDLE;
	brakeTest = 0;
}

uint8_t isCompressorRunning(uint8_t compressorStop)
{
	// Compressor active when:
	//    Pumping
	//    Compressor Stop OFF:
	//       Brake Test
	//       Done
	//    Compressor Stop ON:
	//       Brake Test and timer done

	return( (PUMPING == pumpState) || 
	         (!compressorStop && (BRAKE_TEST == pumpState)) ||
	         (!compressorStop && (DONE == pumpState)) ||
	         (compressorStop && !compressorStopTimer && (BRAKE_TEST == pumpState))
	       );
}

uint8_t isPressureIdle(void)
{
	return(IDLE == pumpState);
}

uint8_t isBrakeTestActive(void)
{
	return(BRAKE_TEST == pumpState);
}

uint8_t setPumpRate(uint8_t pumpRate)
{
	pressureConfig &= ~(0x7);
	pressureConfig |= (pumpRate & 0x07);

	return (pressureConfig & 0x7);
}

uint8_t getPumpRate(void)
{
	return (pressureConfig & 0x7);
}

uint8_t incrementPumpRate(void)
{
	if(getPumpRate() < 7)
		setPumpRate(getPumpRate() + 1);
	return getPumpRate();
}

uint8_t decrementPumpRate(void)
{
	if(getPumpRate() > 0)
		setPumpRate(getPumpRate() - 1);
	return getPumpRate();
}

uint8_t setPressureConfig(uint8_t c)
{
	pressureConfig = c;

	return pressureConfig;
}

uint8_t getPressureConfig(void)
{
	return pressureConfig;
}

