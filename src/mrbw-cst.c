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

#define MRBUS_TX_BUFFER_DEPTH 4
#define MRBUS_RX_BUFFER_DEPTH 4

MRBusPacket mrbusTxPktBufferArray[MRBUS_TX_BUFFER_DEPTH];
MRBusPacket mrbusRxPktBufferArray[MRBUS_RX_BUFFER_DEPTH];

#define STATUS_READ_SWITCHES 0x01
volatile uint8_t ticks;
volatile uint16_t decisecs=0;
volatile uint16_t update_decisecs=10;
volatile uint8_t status = 0;


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


void initialize100HzTimer(void)
{
	// Set up timer 0 for 100Hz interrupts
	TCNT0 = 0;
//	OCR0A = 0x6C;
	OCR0A = 0x0C;
	ticks = 0;
	decisecs = 0;
	TCCR0A = _BV(WGM01);
	TCCR0B = _BV(CS02) | _BV(CS00);
	TIMSK0 |= _BV(OCIE0A);
}

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

	initPorts();
	initADC();
//	initThrottle();
	enableThrottle();
	initialize100HzTimer();
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

	if (++ticks >= 10)
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


int main(void)
{
	uint8_t funcButtons = 0;
	uint8_t throttleButtons = 0;
	uint8_t i;

	init();
	
	lcdEnable();
	lcdBacklightEnable();

	wdt_reset();

	sei();
	led = LED_RED_SLOWBLINK;	
	lcd_init(LCD_DISP_ON);
	wdt_reset();	

	lcd_gotoxy(0, 0);
	lcd_puts("MRBW-CST");
	lcd_gotoxy(0, 1);
	lcd_puts("TESTWARE");

	lcd_setup_custom(0, Bell);
	lcd_setup_custom(1, Horn);

	for(i=0; i<5; i++)
	{
		wdt_reset();
		_delay_ms(200);
	}


	lcd_clrscr();
	buttonsEnable();
	while(1)
	{
		led = LED_OFF;
		wdt_reset();
		if (status & STATUS_READ_SWITCHES)
		{
			status &= ~STATUS_READ_SWITCHES;
			funcButtons = debounce(funcButtons, (PINB & (0xF7)));
		}


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
		
//		lcd_putc((PIND & _BV(PD3))?'1':'0');
//		lcd_putc((PIND & _BV(PD2))?'1':'0');

		lcd_gotoxy(2,0);
		if(brakePot >= 160)
		{
			lcd_putc('E');
			lcd_putc('M');
			lcd_putc('R');
			lcd_putc('G');
		}
		else
		{
			int brakePcnt;
			if(brakePot < 96)
				brakePcnt = 0;
			else
				brakePcnt = 100 * (brakePot - 96) / 64;
			lcd_putc(' ');
			printDec2Dig(brakePcnt);
//			lcd_putc('0' + brakePcnt / 10);
//			lcd_putc('0' + brakePcnt % 10);
			lcd_putc('%');
		}
//		printHex(brakePot);

		lcd_gotoxy(7,0);
//		printHex(reverserPot);
		if(reverserPot < 0x30)
		{
			lcd_putc('R');
		}
		else if(reverserPot < 0x70)
		{
			lcd_putc('N');
		}
		else
		{
			lcd_putc('F');
		}



		lcd_gotoxy(0, 1);
//		printHex(frontLightPot);
		if(frontLightPot < 20)
		{
			lcd_putc('*');
		}
		else if(frontLightPot < 60)
		{
			lcd_putc('B');
		}
		else if(frontLightPot < 106)
		{
			lcd_putc('D');
		}
		else
		{
			lcd_putc('-');
		}
		/*
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
		}*/
		

		lcd_gotoxy(2, 1);
		lcd_putc((funcButtons & 0x02)?' ':'D');
		lcd_putc((funcButtons & 0x04)?' ': 0);
		lcd_putc((funcButtons & 0x01)?' ': 1);

		lcd_gotoxy(5, 1);
		if (!(funcButtons & 0x10))
			lcd_putc('1');
		else if (!(funcButtons & 0x20))
			lcd_putc('2');
		else if (!(funcButtons & 0x40))
			lcd_putc('3');
		else if (!(funcButtons & 0x80))
			lcd_putc('4');
		else
			lcd_putc(' ');			

		
		lcd_gotoxy(7, 1);
//		printHex(rearLightPot);
		if(rearLightPot < 20)
		{
			lcd_putc('*');
		}
		else if(rearLightPot < 60)
		{
			lcd_putc('B');
		}
		else if(rearLightPot < 106)
		{
			lcd_putc('D');
		}
		else
		{
			lcd_putc('-');
		}

/*		switch(rearLight)
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
		}*/
	}
	
	
	/*
	// Application initialization
	setActivePortDirections();
	enableAddressSwitch();
	init();
	setXbeeActive();
	sleepTimer = sleep_tmr_reset_value;

	led = LED_OFF;

	// Initialize a 100 Hz timer.
	initialize100HzTimer();

	// Initialize MRBus core
	mrbusPktQueueInitialize(&mrbeeTxQueue, mrbusTxPktBufferArray, MRBUS_TX_BUFFER_DEPTH);
	mrbusPktQueueInitialize(&mrbeeRxQueue, mrbusRxPktBufferArray, MRBUS_RX_BUFFER_DEPTH);
	mrbeeInit();

	sei();	

	{
		// Fire off initial reset version packet
		uint8_t txBuffer[MRBUS_BUFFER_SIZE];
		createVersionPacket(0xFF, txBuffer);
		mrbusPktQueuePush(&mrbeeTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
	}



	while (1)
	{
		wdt_reset();

		if (status & STATUS_READ_SWITCHES)
		{
			status &= ~STATUS_READ_SWITCHES;
			funcButtons = debounce(funcButtons, PINC & (THROTTLE_DIR_MASK | _BV(THROTTLE_AUX_BUTTON_PC)));
		}

		switch (funcButtons & THROTTLE_DIR_MASK)
		{
			case THROTTLE_DIR_FORWARD:
				dir = 1;
				sleepTimer = sleep_tmr_reset_value;
				break;
			case THROTTLE_DIR_REVERSE:
				dir = 2;
				sleepTimer = sleep_tmr_reset_value;
				break;
			case THROTTLE_DIR_IDLE:
			default:
				dir = 0;
				break;
		}
		
		// Handle any packets that may have come in
		if (mrbusPktQueueDepth(&mrbeeRxQueue))
		{
			pktTimeout = 100;
			PktHandler();
		}
		
		if (batteryVoltage >= (VBATT_OKAY/2))
		{
			if (0 == pktTimeout)
				led = LED_GREEN;
			else
				led = LED_GREEN_FASTBLINK;
		}
		else if (batteryVoltage >= (VBATT_WARN/2))
			led = LED_RED_FASTBLINK;
		else
			led = LED_RED;

		
		// Transmission criteria...
		// > 2 decisec from last transmission and...
		// current throttle reading is not the same as before and the direction is non-idle
		// the function buttons changed
		// *****  or *****
		// it's been more than the transmission timeout
		
		if ((((((throttlePot != lastThrottlePot) && (0 != dir)) || funcButtons != lastFuncButtons) && decisecs > 1) || (decisecs >= update_decisecs))
				&& !(mrbusPktQueueFull(&mrbeeTxQueue)))
		{
			uint8_t txBuffer[MRBUS_BUFFER_SIZE];

			lastThrottlePot = throttlePot;
			lastFuncButtons = funcButtons;
			txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
			txBuffer[MRBUS_PKT_DEST] = 0xFF;
			txBuffer[MRBUS_PKT_LEN] = 10;
			txBuffer[5] = 'C';
			txBuffer[6] = dir;
			// Don't send a speed if we're in neutral
			if (0 == dir)
				txBuffer[7] = 0;
			else
				txBuffer[7] = lastThrottlePot;
			txBuffer[8] = ((funcButtons>>1) & 0x01) ^ 0x01;
			txBuffer[9] = batteryVoltage;	
			mrbusPktQueuePush(&mrbeeTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
			decisecs = 0;
		}
			
		if (mrbusPktQueueDepth(&mrbeeTxQueue))
		{
			mrbeeTransmit();
		}

		while (0 == sleepTimer)
		{
			// Time to nod off
			led = LED_OFF;
			// Disable internal power-sucking peripherals
			ADCSRA &= ~_BV(ADEN);
	
			setXbeeSleep();
			setSleepPortDirections();

			while (THROTTLE_DIR_IDLE == (PINC & THROTTLE_DIR_MASK))
				system_sleep(10);

			sleepTimer = sleep_tmr_reset_value;
			
			// Re-enable chip internal bits (ADC, etc.)
			ADCSRA |= _BV(ADEN) | _BV(ADSC) | _BV(ADIE) | _BV(ADIF);

			setActivePortDirections();
			setXbeeActive();
		}
	}*/
}



