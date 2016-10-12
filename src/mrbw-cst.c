/*************************************************************************
Title:    MRBW-CST Control Stand Throttle
Authors:  Nathan D. Holmes <maverick@drgw.net>
          Michael D. Petersen <railfan@drgw.net>
File:     $Id: $
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2014 Nathan Holmes
    
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
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include "lcd.h"
#include "cst-hardware.h"
#include "mrbee.h"

#define VERSION_STRING "v0.2"

#define LONG_PRESS_10MS_TICKS             100
#define BUTTON_AUTOINCREMENT_10MS_TICKS    50
#define BUTTON_AUTOINCREMENT_ACCEL         10
#define BUTTON_AUTOINCREMENT_MINIMUM        5

#define MRBUS_TX_BUFFER_DEPTH 4
#define MRBUS_RX_BUFFER_DEPTH 4

#define XBEE_SLEEP_DDR  DDRA
#define XBEE_SLEEP_PORT PORTA
#define XBEE_SLEEP      4

#define BUTTON_DYNAMIC 0x02
#define BUTTON_BELL    0x04
#define BUTTON_HORN    0x01
#define BUTTON_MENU    0x20
#define BUTTON_SELECT  0x10
#define BUTTON_UP      0x40
#define BUTTON_DOWN    0x80

#define FUNC_HORN      0x01
#define FUNC_BELL      0x02
#define FUNC_DYNAMIC   0x04

MRBusPacket mrbusTxPktBufferArray[MRBUS_TX_BUFFER_DEPTH];
MRBusPacket mrbusRxPktBufferArray[MRBUS_RX_BUFFER_DEPTH];

uint8_t mrbus_dev_addr = 0;

#define STATUS_READ_SWITCHES          0x01

volatile uint16_t button_autoincrement_10ms_ticks = BUTTON_AUTOINCREMENT_10MS_TICKS;
volatile uint16_t ticks_autoincrement = BUTTON_AUTOINCREMENT_10MS_TICKS;

volatile uint8_t ticks;
volatile uint16_t decisecs = 0;
volatile uint16_t update_decisecs = 10;
volatile uint8_t status = 0;

// Define the menu screens and menu order
// Must end with LAST_SCREEN
typedef enum
{
	MAIN_SCREEN = 0,
	LOCO_SCREEN,
	MRBUS_SCREEN,
	DEBUG_SCREEN,
	VBAT_SCREEN,
	LAST_SCREEN  // Must be the last screen
} Screens;

typedef enum
{
	NO_BUTTON = 0,
	MENU_BUTTON,
	SELECT_BUTTON,
	UP_BUTTON,
	UP_SELECT_BUTTON,
	DOWN_BUTTON,
	DOWN_SELECT_BUTTON
} Buttons;

Buttons button = NO_BUTTON;
Buttons previousButton = NO_BUTTON;
uint8_t buttonCount = 0;

uint8_t functions = 0;


uint8_t debounce(uint8_t debouncedState, uint8_t newInputs)
{
	static uint8_t clock_A=0, clock_B=0;
	uint8_t delta = newInputs ^ debouncedState;   //Find all of the changes
	uint8_t changes;

	clock_A ^= clock_B;                     //Increment the counters
	clock_B  = ~clock_B;

	clock_A &= delta;                       //Reset the counters if no changes
	clock_B &= delta;                       //were detected.

	changes = ~((~delta) | clock_A | clock_B);
	debouncedState ^= changes;
	return(debouncedState);
}

uint8_t throttleDebounce(uint8_t debouncedState, uint8_t newInputs)
{
	static uint8_t clock_A=0, clock_B=0;
	uint8_t delta = newInputs ^ debouncedState;   //Find all of the changes
	uint8_t changes;

	clock_A ^= clock_B;                     //Increment the counters
	clock_B  = ~clock_B;

	clock_A &= delta;                       //Reset the counters if no changes
	clock_B &= delta;                       //were detected.

	changes = ~((~delta) | clock_A | clock_B);
	debouncedState ^= changes;
	return(debouncedState);
}


void createVersionPacket(uint8_t destAddr, uint8_t *buf)
{
	buf[MRBUS_PKT_DEST] = destAddr;
	buf[MRBUS_PKT_SRC] = mrbus_dev_addr;
	buf[MRBUS_PKT_LEN] = 15;
	buf[MRBUS_PKT_TYPE] = 'v';
	buf[6]  = MRBUS_VERSION_WIRELESS;
	// Software Revision
	buf[7]  = 0xFF & ((uint32_t)(GIT_REV))>>16; // Software Revision
	buf[8]  = 0xFF & ((uint32_t)(GIT_REV))>>8; // Software Revision
	buf[9]  = 0xFF & (GIT_REV); // Software Revision
	buf[10]  = HWREV_MAJOR; // Hardware Major Revision
	buf[11]  = HWREV_MINOR; // Hardware Minor Revision
	buf[12] = 'C';
	buf[13] = 'S';
	buf[14] = 'T';
}


void setXbeeSleep()
{
	XBEE_SLEEP_PORT |= _BV(XBEE_SLEEP);
}

void setXbeeActive()
{
	// Unsleep the XBee
	XBEE_SLEEP_PORT &= ~_BV(XBEE_SLEEP);
}


void setActivePortDirections()
{
	// Set XBee sleep control as output
	XBEE_SLEEP_DDR |= _BV(XBEE_SLEEP);
}


void processFuncButtons(uint8_t funcButtons)
{
	// Called every 10ms
	if(funcButtons & BUTTON_DYNAMIC)
		functions &= ~(FUNC_DYNAMIC);
	else
		functions |= FUNC_DYNAMIC;

	if(funcButtons & BUTTON_BELL)
		functions &= ~(FUNC_BELL);
	else
		functions |= FUNC_BELL;

	if(funcButtons & BUTTON_HORN)
		functions &= ~(FUNC_HORN);
	else
		functions |= FUNC_HORN;
	
	if(!(funcButtons & BUTTON_MENU))
	{
		button = MENU_BUTTON;
	}
	else if(!(funcButtons & BUTTON_SELECT) && (funcButtons & BUTTON_UP) && (funcButtons & BUTTON_DOWN))
	{
		button = SELECT_BUTTON;
	}
	else if(!(funcButtons & BUTTON_SELECT) && !(funcButtons & BUTTON_UP))
	{
		button = UP_SELECT_BUTTON;
	}
	else if(!(funcButtons & BUTTON_SELECT) && !(funcButtons & BUTTON_DOWN))
	{
		button = DOWN_SELECT_BUTTON;
	}
	else if(!(funcButtons & BUTTON_UP))
	{
		button = UP_BUTTON;
	}
	else if(!(funcButtons & BUTTON_DOWN))
	{
		button = DOWN_BUTTON;
	}
	else
	{
		button = NO_BUTTON;
	}

	if((previousButton == button) && (NO_BUTTON != button))
	{
		if(buttonCount > LONG_PRESS_10MS_TICKS)  // Use greater than, so a long press can be caught as a single event in the menu handler code
		{
			if(button_autoincrement_10ms_ticks >= BUTTON_AUTOINCREMENT_ACCEL)
				button_autoincrement_10ms_ticks -= BUTTON_AUTOINCREMENT_ACCEL;  // Progressively reduce the time delay
			if(button_autoincrement_10ms_ticks < BUTTON_AUTOINCREMENT_MINIMUM)
				button_autoincrement_10ms_ticks = BUTTON_AUTOINCREMENT_MINIMUM; // Clamp to some minimum value, otherwise it goes too fast to control
			buttonCount = 0;
		}
		else
		{
			buttonCount++;
		}
	}
	else
	{
		// Reset the counters
		button_autoincrement_10ms_ticks = BUTTON_AUTOINCREMENT_10MS_TICKS;
		ticks_autoincrement = button_autoincrement_10ms_ticks;
	}
}


void initialize100HzTimer(void)
{
	// Set up timer 0 for 100Hz interrupts
	TCNT0 = 0;
	OCR0A = 0x6C;
	ticks = 0;
	decisecs = 0;
	TCCR0A = _BV(WGM01);
	TCCR0B = _BV(CS02) | _BV(CS00);
	TIMSK0 |= _BV(OCIE0A);
}

volatile uint8_t throttlePosition = 0;
volatile uint8_t throttleQuadrature = 0;

ISR(TIMER0_COMPA_vect)
{
	static uint16_t internalDeciSecs = 0;
	static uint8_t throttleQuadrature;
	static uint8_t initialized = 0;
	uint8_t newQuadrature;
	
	if (!initialized)
	{
		enableThrottle();	
		initialized = 1;
		throttleQuadrature = (PIND & (_BV(PD2) | _BV(PD3)))>>2;

	}

	status |= STATUS_READ_SWITCHES;

	if (++ticks >= 10)  // 100ms
	{
		ticks = 0;
		decisecs++;
		
//		if (pktTimeout)
	//		pktTimeout--;
		
		if (internalDeciSecs >= 600)
		{
			// Things that happen on minutes
//			if (sleepTimer != 0)
//				sleepTimer--;
			internalDeciSecs -= 600;
		}

		ledUpdate();
	}

	if(ticks_autoincrement < button_autoincrement_10ms_ticks)
			ticks_autoincrement++;

	newQuadrature = (PIND & (_BV(PD2) | _BV(PD3)))>>2;

	uint8_t quadratureUp[] = {1, 3, 0, 2};
	uint8_t quadratureDown[] = {2, 0, 3, 1};

	if (newQuadrature != throttleQuadrature)
	{

		if (newQuadrature == quadratureUp[throttleQuadrature & 0x03])
		{
			if (throttlePosition > 0)
				throttlePosition--;
		}
		else if (newQuadrature == quadratureDown[throttleQuadrature & 0x03])
		{
			if (throttlePosition < 8)
				throttlePosition++;
		}
		else
			throttlePosition = 0;	
	}
	throttleQuadrature = newQuadrature & 0x03;	
}


void wait100ms(uint16_t loops)
{
	uint16_t i;
	for(i=0; i<loops; i++)
	{
		wdt_reset();
		_delay_ms(100);
	}
}


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


void init(void)
{
	// Clear watchdog (in the case of an 'X' packet reset)
	MCUSR = 0;
#ifdef ENABLE_WATCHDOG
	// If you don't want the watchdog to do system reset, remove this chunk of code
	wdt_reset();
	WDTCSR |= _BV(WDE) | _BV(WDCE);
	WDTCSR = _BV(WDE) | _BV(WDP2) | _BV(WDP1); // Set the WDT to system reset and 1s timeout
	wdt_reset();
#else
	wdt_reset();
	wdt_disable();
#endif	

	// Initialize MRBus address from EEPROM
	mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
	// Bogus addresses, fix to default address
	if (0xFF == mrbus_dev_addr || 0x00 == mrbus_dev_addr)
	{
		mrbus_dev_addr = 0x30;
		eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR, mrbus_dev_addr);
	}

	initPorts();
	initADC();
//	initThrottle();
	enableThrottle();
	initialize100HzTimer();
}


int main(void)
{
	uint8_t funcButtons = 0;
	uint8_t txBuffer[MRBUS_BUFFER_SIZE];
	uint8_t controlsChanged = 0;

	uint8_t lastFunctions = functions;
	uint8_t lastThrottlePosition = throttlePosition;
	ReverserPosition lastReverserPosition = reverserPosition;
	uint8_t lastBrakePosition = brakePosition;
	LightPosition lastFrontLight = frontLight;
	LightPosition lastRearLight = rearLight;
	
	uint16_t locoAddress = 250;
	uint16_t newLocoAddress = locoAddress;
	uint8_t newDevAddr = mrbus_dev_addr;
	
	uint8_t backlight = 0;
	
	Screens screenState = LAST_SCREEN;  // Initialize to the last one, since that's the only state guaranteed to be present
	
	init();

	setXbeeActive();
	
	lcdEnable();
	lcdBacklightEnable();

	wdt_reset();

	// Initialize MRBus core
	mrbusPktQueueInitialize(&mrbeeTxQueue, mrbusTxPktBufferArray, MRBUS_TX_BUFFER_DEPTH);
	mrbusPktQueueInitialize(&mrbeeRxQueue, mrbusRxPktBufferArray, MRBUS_RX_BUFFER_DEPTH);
	mrbeeInit();

	sei();	

	// Fire off initial reset version packet
	createVersionPacket(0xFF, txBuffer);
	mrbusPktQueuePush(&mrbeeTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);

	led = LED_OFF;
	lcd_init(LCD_DISP_ON);
	wdt_reset();	

	lcd_gotoxy(0, 0);
	lcd_puts("MRBW-CST");
	lcd_gotoxy(2, 1);
	lcd_puts(VERSION_STRING);

	lcd_setup_custom(0, Bell);
	lcd_setup_custom(1, Horn);

	// Initialize the buttons so there are no startup artifacts when we actually use them
	funcButtons = PINB & (0xF7);

	wait100ms(20);

	lcd_clrscr();
	buttonsEnable();

	while(1)
	{
		led = LED_GREEN;
		wdt_reset();

		if (status & STATUS_READ_SWITCHES)
		{
			// Read switches every 10ms
			status &= ~STATUS_READ_SWITCHES;
			funcButtons = debounce(funcButtons, (PINB & (0xF7)));
			processFuncButtons(funcButtons);
		}

		processADC();

		switch(screenState)
		{
			case MAIN_SCREEN:
				if(backlight)
					lcdBacklightEnable();
				else
					lcdBacklightDisable();
				lcd_gotoxy(2,0);
				printDec4DigWZero(locoAddress);
				if((SELECT_BUTTON == button) && (SELECT_BUTTON != previousButton))
				{
					if(backlight)
						backlight = 0;
					else
						backlight = 1;
				}
				break;

			case LOCO_SCREEN:
				lcdBacklightEnable();
				lcd_gotoxy(0,0);
				lcd_puts("GET LOCO");
				lcd_gotoxy(2,1);
				printDec4DigWZero(newLocoAddress);
				switch(button)
				{
					case UP_BUTTON:
						if(ticks_autoincrement >= button_autoincrement_10ms_ticks)
						{
							if(newLocoAddress < 9999)
								newLocoAddress++;
							ticks_autoincrement = 0;
						}
						break;
					case UP_SELECT_BUTTON:  // Fast
						if(newLocoAddress < 9999)
							newLocoAddress++;
						break;
					case DOWN_BUTTON:
						if(ticks_autoincrement >= button_autoincrement_10ms_ticks)
						{
							if(newLocoAddress > 0)
								newLocoAddress--;
							ticks_autoincrement = 0;
						}
						break;
					case DOWN_SELECT_BUTTON:  // Fast
						if(newLocoAddress > 0)
							newLocoAddress--;
						break;
					case SELECT_BUTTON:
						// FIXME: This should really send a packet to request a new locomotive address and locoAddress is only updated once confirmation received
						locoAddress = newLocoAddress;
						lcd_clrscr();
						lcd_gotoxy(0,0);
						lcd_puts("REQUEST");
						lcd_gotoxy(0,1);
						lcd_puts("SENT");
						wait100ms(7);
						screenState = LAST_SCREEN;
						break;
					case MENU_BUTTON:
						newLocoAddress = locoAddress;  // Reset newLocoAddress since no changes were commited
						break;
					case NO_BUTTON:
						break;
				}
				break;

			case MRBUS_SCREEN:
				lcdBacklightEnable();
				lcd_gotoxy(0,0);
				lcd_puts("MRB ADR");
				lcd_gotoxy(2,1);
				lcd_puts("0x");
				printHex(newDevAddr);
				switch(button)
				{
					case UP_BUTTON:
						if(ticks_autoincrement >= button_autoincrement_10ms_ticks)
						{
							if(newDevAddr < 0xFE)
								newDevAddr++;
							ticks_autoincrement = 0;
						}
						break;
					case UP_SELECT_BUTTON:  // Fast
						if(newDevAddr < 0xFE)
							newDevAddr++;
						break;
					case DOWN_BUTTON:
						if(ticks_autoincrement >= button_autoincrement_10ms_ticks)
						{
							if(newDevAddr > 0)
								newDevAddr--;
							ticks_autoincrement = 0;
						}
						break;
					case DOWN_SELECT_BUTTON:  // Fast
						if(newDevAddr > 0)
							newDevAddr--;
						break;
					case SELECT_BUTTON:
						mrbus_dev_addr = newDevAddr;
						// FIXME: write to EEPROM
						lcd_clrscr();
						lcd_gotoxy(1,0);
						lcd_puts("SAVED!");
						wait100ms(7);
						screenState = LAST_SCREEN;
						break;
					case MENU_BUTTON:
						newDevAddr = mrbus_dev_addr;  // Reset newDevAddr since no changes were commited
						break;
					case NO_BUTTON:
						break;
				}
				break;

			case DEBUG_SCREEN:
				lcdBacklightEnable();
				lcd_gotoxy(0,0);
				if(0 == throttlePosition)
				{
					lcd_putc('I');
				}
				else
				{
					lcd_putc('0' + throttlePosition);
				}
				lcd_putc(' ');		
		
				lcd_gotoxy(2,0);
				if(brakePosition & 0x80)
				{
					lcd_putc('E');
					lcd_putc('M');
					lcd_putc('R');
					lcd_putc('G');
				}
				else
				{
					uint8_t brakePcnt = 100 * brakePosition / 128;
					lcd_putc(' ');
					printDec2Dig(brakePcnt);
					lcd_putc('%');
				}

				lcd_gotoxy(7,0);
				switch(reverserPosition)
				{
					case FORWARD:
						lcd_putc('F');
						break;
					case NEUTRAL:
						lcd_putc('N');
						break;
					case REVERSE:
						lcd_putc('R');
						break;
				}

				lcd_gotoxy(2, 1);
				lcd_puts((functions & FUNC_DYNAMIC) ? "DB":"  ");
				lcd_gotoxy(4, 1);
				lcd_putc((functions & FUNC_BELL) ? 0 : ' ');
				lcd_gotoxy(5, 1);
				lcd_putc((functions & FUNC_HORN) ? 1 : ' ');

				lcd_gotoxy(7, 1);
				switch(frontLight)
				{
					default:
					case LIGHT_OFF:
						lcd_putc('-');
						break;
					case LIGHT_DIM:
						lcd_putc('D');
						break;
					case LIGHT_BRIGHT:
						lcd_putc('B');
						break;
					case LIGHT_BRIGHT_DITCH:
						lcd_putc('*');
						break;
				}

				lcd_gotoxy(0, 1);
				switch(rearLight)
				{
					case LIGHT_OFF:
						lcd_putc('-');
						break;
					case LIGHT_DIM:
						lcd_putc('D');
						break;
					case LIGHT_BRIGHT:
						lcd_putc('B');
						break;
					case LIGHT_BRIGHT_DITCH:
						lcd_putc('*');
						break;
				}

				break;

			case VBAT_SCREEN:
				lcdBacklightEnable();
				lcd_gotoxy(0,0);
				lcd_puts("BATTERY");
				lcd_gotoxy(1,1);
				lcd_putc('0' + (((batteryVoltage*2)/100)%10));
				lcd_putc('.');
				lcd_putc('0' + (((batteryVoltage*2)/10)%10));
				lcd_putc('0' + ((batteryVoltage*2)%10));
				lcd_putc('V');
				break;

			case LAST_SCREEN:
				// Clean up and reset
				lcd_clrscr();
				screenState = 0;
				break;
		}

		// Process Menu button
		// Do this after main screen loop so screens can also do cleanup when menu is pressed
		if(MENU_BUTTON == button)
		{
			if(MENU_BUTTON != previousButton)
			{
				// Menu pressed, advance menu
				lcd_clrscr();
				screenState++;  // No range checking needed since LAST_SCREEN will reset the counter
				ticks_autoincrement = 0;
			}
			if(ticks_autoincrement >= button_autoincrement_10ms_ticks)
			{
				// Reset menu on long press
				screenState = LAST_SCREEN;
			}
		}

		previousButton = button;

		// Transmission criteria...
		// > 2 decisec from last transmission and...
		// current throttle reading is not the same as before and the direction is non-idle
		// the function buttons changed
		// *****  or *****
		// it's been more than the transmission timeout
		
		wdt_reset();
		controlsChanged =	(reverserPosition != lastReverserPosition) ||
							(throttlePosition != lastThrottlePosition) ||
							(brakePosition != lastBrakePosition) ||
							(functions != lastFunctions) ||
							(frontLight != lastFrontLight) ||
							(rearLight != lastRearLight);
		
		if (( (controlsChanged && decisecs > 1) || (decisecs >= update_decisecs))
				&& !(mrbusPktQueueFull(&mrbeeTxQueue)))
		{
			controlsChanged = 0;
			lastReverserPosition = reverserPosition;
			lastThrottlePosition = throttlePosition;
			lastBrakePosition = brakePosition;
			lastFunctions = functions;
			lastFrontLight = frontLight;
			lastRearLight = rearLight;
			
			txBuffer[MRBUS_PKT_DEST] = 0xFF;
			txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
			txBuffer[MRBUS_PKT_LEN] = 16;
			txBuffer[5] = 'S';

			switch(reverserPosition)
			{
				case FORWARD:
					txBuffer[6] = 0x02;
					break;
				case NEUTRAL:
					txBuffer[6] = 0x00;
					break;
				case REVERSE:
					txBuffer[6] = 0x03;
					break;
			}

			// FIXME: need dynamic brake logic

			uint16_t throttleTemp = throttlePosition * 32;
			txBuffer[7] = (throttleTemp > 255) ? 255 : throttleTemp;

			txBuffer[8] = 0x7F & brakePosition;
			txBuffer[9] = brakePosition;
			
			txBuffer[10] = 0;
			txBuffer[10] |= (functions & FUNC_HORN) ? 0x80 : 0x00;

			txBuffer[11] = 0;
			txBuffer[11] |= (functions & FUNC_BELL) ? 0x01 : 0x00;

			txBuffer[12] = 0;

			switch(frontLight)
			{
				case LIGHT_OFF:
					txBuffer[13] = 0x00;
					break;
				case LIGHT_DIM:
					txBuffer[13] = 0x01;
					break;
				case LIGHT_BRIGHT:
					txBuffer[13] = 0x02;
					break;
				case LIGHT_BRIGHT_DITCH:
					txBuffer[13] = 0x82;
					break;
			}

			switch(rearLight)
			{
				case LIGHT_OFF:
					txBuffer[14] = 0x00;
					break;
				case LIGHT_DIM:
					txBuffer[14] = 0x01;
					break;
				case LIGHT_BRIGHT:
					txBuffer[14] = 0x02;
					break;
				case LIGHT_BRIGHT_DITCH:
					txBuffer[14] = 0x82;
					break;
			}

			txBuffer[15] = batteryVoltage;	
			mrbusPktQueuePush(&mrbeeTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
			decisecs = 0;
		}

		if (mrbusPktQueueDepth(&mrbeeTxQueue))
		{
			wdt_reset();
			mrbeeTransmit();
		}


	}

}



