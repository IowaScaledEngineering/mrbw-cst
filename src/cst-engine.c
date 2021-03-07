/*************************************************************************
Title:    Engine (Prime Mover) support for Control Stand Throttle
Authors:  Michael D. Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     cst-engine.c
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2020 Michael Petersen & Nathan Holmes
    
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*************************************************************************/

#include <stdint.h>

#include "lcd.h"

#include "cst-engine.h"

typedef struct
{
	uint16_t addr;
	EngineState state;
} LocoEngineState;

static LocoEngineState locoEngineStates[ENGINE_STATE_QUEUE_SIZE];
static uint8_t engineQueueIndex = 0;

void engineStatesQueueInitialize(void)
{
	uint8_t i;
	
	engineQueueIndex = 0;
	for(i=0; i<ENGINE_STATE_QUEUE_SIZE; i++)
	{
		locoEngineStates[i].addr = 0;
		locoEngineStates[i].state = ENGINE_NOT_INITIALIZED;
	}
}

void engineStatesQueueUpdate(uint16_t locoAddr, EngineState engineState)
{
	uint8_t i;

	// Check for existing locomotive address
	for(i=0; i<ENGINE_STATE_QUEUE_SIZE; i++)
	{
		if(locoEngineStates[i].addr == locoAddr)
		{
			locoEngineStates[i].state = engineState;
			return;
		}
	}
	
	// We didn't find an existing locomotive address, so add it
	locoEngineStates[engineQueueIndex].addr = locoAddr;
	locoEngineStates[engineQueueIndex].state = engineState;
	if(++engineQueueIndex >= ENGINE_STATE_QUEUE_SIZE)
		engineQueueIndex = 0;
}

EngineState engineStatesQueueGetState(uint16_t locoAddr)
{
	uint8_t i;

	for(i=0; i<ENGINE_STATE_QUEUE_SIZE; i++)
	{
		if(locoEngineStates[i].addr == locoAddr)
		{
			return(locoEngineStates[i].state);
		}
	}

	// Not found
	return(ENGINE_NOT_INITIALIZED);
}

uint16_t engineStatesQueuePeekLocoAddress(uint8_t index)
{
	return locoEngineStates[index].addr;
}

EngineState engineStatesQueuePeekState(uint8_t index)
{
	return locoEngineStates[index].state;
}

void printEngineState(EngineState engineState)
{
	switch(engineState)
	{
		case ENGINE_OFF:
			lcd_puts("   OFF  ");
			break;
		case ENGINE_ON:
		case ENGINE_RUNNING:
			lcd_puts("   ON   ");
			break;
		case ENGINE_NOT_IDLE:
			lcd_puts("NOT IDLE");
			break;
		case ENGINE_START:
			lcd_puts("STARTING");
			break;
		case ENGINE_STOP:
			lcd_puts("STOPPING");
			break;
		case ENGINE_NOT_INITIALIZED:
			lcd_puts("UNKNOWN!");
			break;
	}
}

