/*************************************************************************
Title:    Fast time support for Control Stand Throttle
Authors:  Michael D. Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     cst-time.c
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
#include <util/atomic.h>

#include "mrbee.h"

#include "lcd.h"

#include "cst-common.h"
#include "cst-time.h"

static uint16_t timeScaleFactor = 10;
static uint8_t timeFlags = 0;

static TimeData realTime;
static TimeData fastTime;

static volatile uint16_t fastDecisecs = 0;
static volatile uint8_t scaleTenthsAccum = 0;
static uint8_t maxDeadReckoningTime = 150;
static uint16_t deadReckoningTime = 0;

const uint8_t ClockAM[8] =
{
	0b00001000,
	0b00010100,
	0b00011100,
	0b00010100,
	0b00010100,
	0b00000000,
	0b00000000,
	0b00000000
};

const uint8_t ClockPM[8] =
{
	0b00000000,
	0b00000000,
	0b00011100,
	0b00010100,
	0b00011100,
	0b00010000,
	0b00010000,
	0b00000000
};

void setupClockChars(void)
{
	lcd_setup_custom(AM_CHAR, ClockAM);
	lcd_setup_custom(PM_CHAR, ClockPM);
}

void incrementTime(TimeData* t, uint8_t incSeconds)
{
	uint16_t i = t->seconds + incSeconds;

	while(i >= 60)
	{
		t->minutes++;
		i -= 60;
	}
	t->seconds = (uint8_t)i;
	
	while(t->minutes >= 60)
	{
		t->hours++;
		t->minutes -= 60;
	}
	
	if (t->hours >= 24)
		t->hours %= 24;
}

void displayTime(TimeData* time, uint8_t ampm)
{
	uint8_t i=0;
	uint8_t displayCharacters[6];

	displayCharacters[4] = '0' + (time->minutes % 10);
	displayCharacters[3] = '0' + ((time->minutes / 10) % 10);
	displayCharacters[2] = ':';

	if (ampm)
	{
		// If 12 hour mode
		if (time->hours == 0)
		{
			displayCharacters[0] = '1';
			displayCharacters[1] = '2';
		}
		else 
		{
			uint8_t hrs = time->hours;
			if (hrs > 12)
				hrs -= 12;
			
			i = (hrs / 10) % 10;
			displayCharacters[0] = (0==i) ? ' ' : '0' + i;
			displayCharacters[1] = '0' + (hrs % 10);
		}

		if (time->hours >= 12)
			displayCharacters[5] = PM_CHAR;
		else	
			displayCharacters[5] = AM_CHAR;

	}
	else
	{
		// 24 hour mode
		i = (time->hours / 10) % 10;
		displayCharacters[1] = '0' + (time->hours % 10);
		displayCharacters[0] = '0' + i;
		displayCharacters[5] = ' ';
	}
	
	for(i=0; i<6; i++)
	{
		lcd_putc(displayCharacters[i]);
	}
}

void printTime(void)
{
	if(0 == deadReckoningTime)
	{
		lcd_puts("--:-- ");
	}
	else if ((TIME_FLAGS_DISP_FAST | TIME_FLAGS_DISP_FAST_HOLD) == (timeFlags & (TIME_FLAGS_DISP_FAST | TIME_FLAGS_DISP_FAST_HOLD)) )  // Hold state
	{
		lcd_puts(" HOLD ");
	}
	else if (timeFlags & TIME_FLAGS_DISP_FAST)
		displayTime(&fastTime, timeFlags & TIME_FLAGS_DISP_FAST_AMPM);
	else
		displayTime(&realTime, timeFlags & TIME_FLAGS_DISP_REAL_AMPM);
}

uint16_t convertMaxDeadReckoningToDecisecs(void)
{
	if(maxDeadReckoningTime <= 250)
		return maxDeadReckoningTime;
	else
	{
		switch(maxDeadReckoningTime)
		{
			case 251:
				return 300;
				break;
			case 252:
				return 600;
				break;
			case 253:
				return 900;
				break;
			case 254:
				return 1200;
				break;
			default:  // Never should happen
				return 0;
				break;
		}
	}
}

void updateTime10Hz(void)
{
	// Called from a 10Hz timer ISR
	if (deadReckoningTime)
		deadReckoningTime--;
	if (TIME_FLAGS_DISP_FAST == (timeFlags & (TIME_FLAGS_DISP_FAST | TIME_FLAGS_DISP_FAST_HOLD)))
	{
		fastDecisecs += timeScaleFactor / 10;
		scaleTenthsAccum += timeScaleFactor % 10;
		if (scaleTenthsAccum > 10)
		{
			fastDecisecs++;
			scaleTenthsAccum -= 10;
		}		
	}
}

void updateTime(void)
{
	// Call periodically to dead reckon
	if ((timeFlags & TIME_FLAGS_DISP_FAST) && !(timeFlags & TIME_FLAGS_DISP_FAST_HOLD) && fastDecisecs >= 10)
	{
		uint8_t fastTimeSecs;
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			fastTimeSecs = fastDecisecs / 10;
			fastDecisecs -= fastTimeSecs * 10;
		}
		incrementTime(&fastTime, fastTimeSecs);
	}
}

void processTimePacket(uint8_t* pkt)
{
	realTime.hours = pkt[6];
	realTime.minutes = pkt[7];
	realTime.seconds = pkt[8];
	timeFlags = pkt[9];
	// Time source packets aren't required to have a fast section
	// Doesn't really make sense outside model railroading applications, so...
	if (pkt[MRBUS_PKT_LEN] >= 14)
	{
		fastTime.hours =  pkt[10];
		fastTime.minutes =  pkt[11];
		fastTime.seconds = pkt[12];
		timeScaleFactor = (((uint16_t)pkt[13])<<8) + (uint16_t)pkt[14];
	}		
	// If we got a packet, there's no dead reckoning time anymore
	fastDecisecs = 0;
	scaleTenthsAccum = 0;
	deadReckoningTime = convertMaxDeadReckoningToDecisecs();
}

uint8_t incrementMaxDeadReckoningTime()
{
	if(maxDeadReckoningTime < 250)
	{
		uint16_t tmpMaxDeadReckoningTime = maxDeadReckoningTime + 10;
		maxDeadReckoningTime = (tmpMaxDeadReckoningTime / 10) * 10;  // Make sure it is a multiple of 10
	}
	else if(maxDeadReckoningTime < 254)
	{
		maxDeadReckoningTime++;
	}
	return maxDeadReckoningTime;
}

uint8_t decrementMaxDeadReckoningTime()
{
	if(maxDeadReckoningTime > 250)
	{
		maxDeadReckoningTime--;
	}
	else if(maxDeadReckoningTime > 19)
	{
		maxDeadReckoningTime -= 10;
		maxDeadReckoningTime = (maxDeadReckoningTime / 10) * 10;  // Make sure it is a multiple of 10
	}
	return maxDeadReckoningTime;
}

uint8_t setMaxDeadReckoningTime(uint8_t decisecs)
{
	if(decisecs <= 250)
		decisecs = (decisecs / 10) * 10;  // Make sure it is a multiple of 10

	if(decisecs < DEAD_RECKONING_TIME_MIN)
	{
		maxDeadReckoningTime = DEAD_RECKONING_TIME_MIN;
	}
	else if(decisecs == 0xFF)
	{
		maxDeadReckoningTime = DEAD_RECKONING_TIME_DEFAULT;  // Default for unprogrammed EEPROM
	}
	else
	{
		maxDeadReckoningTime = decisecs;
	}
	return maxDeadReckoningTime;
}

uint8_t getMaxDeadReckoningTime(void)
{
	return maxDeadReckoningTime;
}

uint16_t getTimeScaleFactor(void)
{
	return timeScaleFactor;
}

void clearDeadReckoningTime(void)
{
	deadReckoningTime = 0;
}

