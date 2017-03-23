/*************************************************************************
Title:    MRBW-CST Control Stand Throttle
Authors:  Michael D. Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     $Id: $
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
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <util/atomic.h>
#include "lcd.h"
#include "cst-lcd.h"
#include "cst-hardware.h"
#include "mrbee.h"

#define VERSION_STRING "0.55"

#define FAST_SLEEP

#define LONG_PRESS_10MS_TICKS             100
#define BUTTON_AUTOINCREMENT_10MS_TICKS    50
#define BUTTON_AUTOINCREMENT_ACCEL         10
#define BUTTON_AUTOINCREMENT_MINIMUM        5

#define PKT_TIMEOUT_DECISECS   100
volatile uint8_t pktTimeout = 0;

// VBATT_OKAY is the battery voltage (in centivolts) above which the batteries are considered "fine"
#define VBATT_OKAY  220 
// VBATT_WARN is the battery voltage (in centivolts) above which the batteries are considered a warning, but not
//  in critical shape.  This should *always* be less than VBATT_OKAY
#define VBATT_WARN  200

#define MRBUS_TX_BUFFER_DEPTH 4
#define MRBUS_RX_BUFFER_DEPTH 4

#define XBEE_SLEEP_DDR  DDRB
#define XBEE_SLEEP_PORT PORTB
#define XBEE_SLEEP      0

#define STATUS_READ_SWITCHES          0x01

#define HORN_CONTROL      0x01
#define BELL_CONTROL      0x02
#define DYNAMIC_CONTROL   0x04

#define OFF_FUNCTION      0x80
#define LATCH_FUNCTION    0x40

// Set EEPROM locations
#define EE_DEVICE_SLEEP_TIMEOUT       0x10
#define EE_LOCO_ADDRESS               0x11

#define EE_BASE_ADDR                  0x1F

#define EE_HORN_FUNCTION              0x20
#define EE_BELL_FUNCTION              0x21
#define EE_FRONT_DIM1_FUNCTION        0x22
#define EE_FRONT_DIM2_FUNCTION        0x23
#define EE_FRONT_HEADLIGHT_FUNCTION   0x24
#define EE_FRONT_DITCH_FUNCTION       0x25
#define EE_REAR_DIM1_FUNCTION         0x26
#define EE_REAR_DIM2_FUNCTION         0x27
#define EE_REAR_HEADLIGHT_FUNCTION    0x28
#define EE_REAR_DITCH_FUNCTION        0x29
#define EE_DYNAMIC_FUNCTION           0x2A
#define EE_ENGINE_FUNCTION            0x2B
#define EE_UP_BUTTON_FUNCTION         0x2C
#define EE_DOWN_BUTTON_FUNCTION       0x2D
#define EE_FUNC_FORCE_ON              0x30
#define EE_FUNC_FORCE_OFF             0x34

MRBusPacket mrbusTxPktBufferArray[MRBUS_TX_BUFFER_DEPTH];
MRBusPacket mrbusRxPktBufferArray[MRBUS_RX_BUFFER_DEPTH];

uint8_t mrbus_dev_addr = 0;
uint8_t mrbus_base_addr = 0;
uint16_t locoAddress = 0;

uint8_t hornFunction = 2;
uint8_t bellFunction = 7;
uint8_t frontDim1Function = 3, frontDim2Function = OFF_FUNCTION, frontHeadlightFunction = 0, frontDitchFunction = 3;
uint8_t rearDim1Function = 6, rearDim2Function = OFF_FUNCTION, rearHeadlightFunction = 5, rearDitchFunction = 6;
uint8_t dynamicFunction = OFF_FUNCTION;
uint8_t engineFunction = 8;
uint8_t upButtonFunction = OFF_FUNCTION, downButtonFunction = OFF_FUNCTION;

volatile uint16_t button_autoincrement_10ms_ticks = BUTTON_AUTOINCREMENT_10MS_TICKS;
volatile uint16_t ticks_autoincrement = BUTTON_AUTOINCREMENT_10MS_TICKS;

volatile uint8_t ticks;
volatile uint16_t decisecs = 0;
volatile uint16_t sleepTimeout_decisecs = 0;
volatile uint16_t update_decisecs = 10;
volatile uint8_t status = 0;

uint16_t sleep_tmr_reset_value;

// Define the menu screens and menu order
// Must end with LAST_SCREEN
typedef enum
{
	MAIN_SCREEN = 0,
//	DEBUG2_SCREEN,
	ENGINE_SCREEN,
	LOCO_SCREEN,
	TONNAGE_SCREEN,
	FUNC_SET_SCREEN,
	FUNC_CONFIG_SCREEN,
	COMM_SCREEN,
	PREFS_SCREEN,
	DIAG_SCREEN,
	LAST_SCREEN  // Must be the last screen
} Screens;

typedef enum
{
	NO_BUTTON = 0,
	MENU_BUTTON,
	SELECT_BUTTON,
	UP_BUTTON,
	DOWN_BUTTON,
} Buttons;

Buttons button = NO_BUTTON;
Buttons previousButton = NO_BUTTON;
uint8_t buttonCount = 0;

typedef enum
{
	HORN_FN = 0,
	BELL_FN,
	DYNAMIC_FN,
	ENGINE_FN,
	FRONT_HEADLIGHT_FN,
	FRONT_DITCH_FN,
	FRONT_DIM1_FN,
	FRONT_DIM2_FN,
	REAR_HEADLIGHT_FN,
	REAR_DITCH_FN,
	REAR_DIM1_FN,
	REAR_DIM2_FN,
	UP_FN,
	DOWN_FN,
	LAST_FN,
} Functions;

Functions functionSetting = HORN_FN;

uint32_t functionForceOn  = 0;
uint32_t functionForceOff = 0;

#define UP_OPTION_BUTTON   0x01
#define DOWN_OPTION_BUTTON 0x02

uint8_t controls = 0;

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

void PktHandler(void)
{
	uint16_t crc = 0;
	uint8_t i;
	uint8_t rxBuffer[MRBUS_BUFFER_SIZE];
	uint8_t txBuffer[MRBUS_BUFFER_SIZE];

	if (0 == mrbusPktQueuePop(&mrbeeRxQueue, rxBuffer, sizeof(rxBuffer)))
		return;


	//*************** PACKET FILTER ***************
	// Loopback Test - did we send it?  If so, we probably want to ignore it
	if (rxBuffer[MRBUS_PKT_SRC] == mrbus_dev_addr) 
		goto	PktIgnore;

	// Destination Test - is this for us or broadcast?  If not, ignore
	if (0xFF != rxBuffer[MRBUS_PKT_DEST] && mrbus_dev_addr != rxBuffer[MRBUS_PKT_DEST]) 
		goto	PktIgnore;
	
	// CRC16 Test - is the packet intact?
	for(i=0; i<rxBuffer[MRBUS_PKT_LEN]; i++)
	{
		if ((i != MRBUS_PKT_CRC_H) && (i != MRBUS_PKT_CRC_L)) 
			crc = mrbusCRC16Update(crc, rxBuffer[i]);
	}
	if ((UINT16_HIGH_BYTE(crc) != rxBuffer[MRBUS_PKT_CRC_H]) || (UINT16_LOW_BYTE(crc) != rxBuffer[MRBUS_PKT_CRC_L]))
		goto	PktIgnore;
		
	//*************** END PACKET FILTER ***************


	//*************** PACKET HANDLER - PROCESS HERE ***************

	// Just smash the transmit buffer if we happen to see a packet directed to us
	// that requires an immediate response
	//
	// If we're in here, then either we're transmitting, then we can't be 
	// receiving from someone else, or we failed to transmit whatever we were sending
	// and we're waiting to try again.  Either way, we're not going to corrupt an
	// in-progress transmission.
	//
	// All other non-immediate transmissions (such as scheduled status updates)
	// should be sent out of the main loop so that they don't step on things in
	// the transmit buffer
	
	if ('A' == rxBuffer[MRBUS_PKT_TYPE])
	{
		// PING packet
		txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
		txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		txBuffer[MRBUS_PKT_LEN] = 6;
		txBuffer[MRBUS_PKT_TYPE] = 'a';
		mrbusPktQueuePush(&mrbeeTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		goto PktIgnore;
	} 
	else if ('W' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// EEPROM WRITE Packet
		txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
		txBuffer[MRBUS_PKT_LEN] = 8;			
		txBuffer[MRBUS_PKT_TYPE] = 'w';
		eeprom_write_byte((uint8_t*)(uint16_t)rxBuffer[6], rxBuffer[7]);
		txBuffer[6] = rxBuffer[6];
		txBuffer[7] = rxBuffer[7];
		if (MRBUS_EE_DEVICE_ADDR == rxBuffer[6])
			mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
		txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		mrbusPktQueuePush(&mrbeeTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		goto PktIgnore;	
	}
	else if ('R' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// EEPROM READ Packet
		txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
		txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		txBuffer[MRBUS_PKT_LEN] = 8;			
		txBuffer[MRBUS_PKT_TYPE] = 'r';
		txBuffer[6] = rxBuffer[6];
		txBuffer[7] = eeprom_read_byte((uint8_t*)(uint16_t)rxBuffer[6]);			
		mrbusPktQueuePush(&mrbeeTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		goto PktIgnore;
	}
	else if ('V' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// Version
		createVersionPacket(rxBuffer[MRBUS_PKT_SRC], txBuffer);
		mrbusPktQueuePush(&mrbeeTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		goto PktIgnore;
	}
	else if ('X' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// Reset
		cli();
		wdt_reset();
		MCUSR &= ~(_BV(WDRF));
		WDTCSR |= _BV(WDE) | _BV(WDCE);
		WDTCSR = _BV(WDE);
		while(1);  // Force a watchdog reset
		sei();
	}
	//*************** END PACKET HANDLER  ***************

	
	//*************** RECEIVE CLEANUP ***************
PktIgnore:
	// Yes, I hate gotos as well, but sometimes they're a really handy and efficient
	// way to jump to a common block of cleanup code at the end of a function 

	// This section resets anything that needs to be reset in order to allow us to receive
	// another packet.  Typically, that's just clearing the MRBUS_RX_PKT_READY flag to 
	// indicate to the core library that the mrbus_rx_buffer is clear.
	return;	
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

volatile uint8_t wdt_tripped=0;

ISR(WDT_vect) 
{
	wdt_tripped=1;  // set global volatile variable
}

uint16_t system_sleep(uint16_t sleep_decisecs)
{
	uint16_t slept = 0;

	while(slept < sleep_decisecs)
	{
		uint16_t remaining_sleep = sleep_decisecs - slept;
		uint8_t planned_sleep = 80;
		uint8_t wdtcsr_bits = _BV(WDIF) | _BV(WDIE);

		if (remaining_sleep == 1)
		{
			wdtcsr_bits |= _BV(WDP1) | _BV(WDP0);
			planned_sleep = 1;
		}
		else if (remaining_sleep <= 3)
		{
			wdtcsr_bits |= _BV(WDP2);
			planned_sleep = 3;
		}
		else if (remaining_sleep <= 5)
		{
			wdtcsr_bits |= _BV(WDP2) | _BV(WDP0);
			planned_sleep = 5;
		}
		else if (remaining_sleep <= 10)
		{
			wdtcsr_bits |= _BV(WDP2) | _BV(WDP1);
			planned_sleep = 10;
		}
		else if (remaining_sleep <= 20)
		{
			wdtcsr_bits |= _BV(WDP2) | _BV(WDP1) | _BV(WDP0);
			planned_sleep = 20;
		}
		else if (remaining_sleep <= 40)
		{
			wdtcsr_bits |= _BV(WDP3);
			planned_sleep = 40;
		}
		else
		{
			wdtcsr_bits |= _BV(WDP3) | _BV(WDP0);
			planned_sleep = 80;
		}

		// Procedure to reset watchdog and set it into interrupt mode only

		cli();
		set_sleep_mode(SLEEP_MODE_PWR_DOWN);      // set the type of sleep mode to use
		sleep_enable();                           // enable sleep mode
		wdt_reset();
		MCUSR &= ~(_BV(WDRF));
		WDTCSR |= _BV(WDE) | _BV(WDCE);
		WDTCSR = wdtcsr_bits;

		sei();

		wdt_tripped = 0;
		// Wrap this in a loop, so we go back to sleep unless the WDT woke us up
		while (0 == wdt_tripped)
			sleep_cpu();

		wdt_reset();
		WDTCSR |= _BV(WDIE); // Restore WDT interrupt mode
		slept += planned_sleep;
	}

	sleep_disable();

#ifdef ENABLE_WATCHDOG
	// If you don't want the watchdog to do system reset, remove this chunk of code
	wdt_reset();
	MCUSR &= ~(_BV(WDRF));
	WDTCSR |= _BV(WDE) | _BV(WDCE);
	WDTCSR = _BV(WDE) | _BV(WDP2) | _BV(WDP1); // Set the WDT to system reset and 1s timeout
	wdt_reset();
#else
	wdt_reset();
	wdt_disable();
#endif

	return(slept);
}


void processButtons(uint8_t inputButtons)
{
	if(!(inputButtons & _BV(MENU_PIN)))
	{
		button = MENU_BUTTON;
	}
	else if(!(inputButtons & _BV(SELECT_PIN))) // && (inputButtons & UP_PIN) && (inputButtons & DOWN_PIN))
	{
		button = SELECT_BUTTON;
	}
/*	else if(!(inputButtons & SELECT_PIN) && !(inputButtons & UP_PIN))*/
/*	{*/
/*		button = UP_SELECT_BUTTON;*/
/*	}*/
/*	else if(!(inputButtons & SELECT_PIN) && !(inputButtons & DOWN_PIN))*/
/*	{*/
/*		button = DOWN_SELECT_BUTTON;*/
/*	}*/
	else if(!(inputButtons & _BV(UP_PIN)))
	{
		button = UP_BUTTON;
	}
	else if(!(inputButtons & _BV(DOWN_PIN)))
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

void processSwitches(uint8_t inputButtons)
{
	// Called every 10ms
	if(inputButtons & _BV(DYNAMIC_PIN))
		controls &= ~(DYNAMIC_CONTROL);
	else
		controls |= DYNAMIC_CONTROL;

	if(inputButtons & _BV(BELL_PIN))
		controls &= ~(BELL_CONTROL);
	else
		controls |= BELL_CONTROL;
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

ISR(TIMER0_COMPA_vect)
{
	status |= STATUS_READ_SWITCHES;

	if (++ticks >= 10)  // 100ms
	{
		ticks = 0;
		decisecs++;
		
		if (pktTimeout)
			pktTimeout--;
		
		if(sleepTimeout_decisecs)
		{
#ifdef FAST_SLEEP
			sleepTimeout_decisecs-=10;
#else
			sleepTimeout_decisecs--;
#endif
		}

		ledUpdate();
	}

	if(ticks_autoincrement < button_autoincrement_10ms_ticks)
			ticks_autoincrement++;
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


void readConfig(void)
{
	// Read the number of minutes before sleeping from EEP and store it.
	// If it's not in range, clamp it.
	// Abuse sleep_tmr_reset_value to read the EEPROM value in minutes before converting to decisecs
	sleep_tmr_reset_value = eeprom_read_byte((uint8_t*)EE_DEVICE_SLEEP_TIMEOUT);
	if(0 == sleep_tmr_reset_value)
	{
		sleep_tmr_reset_value = 1;
		eeprom_write_byte((uint8_t*)EE_DEVICE_SLEEP_TIMEOUT, sleep_tmr_reset_value);
	}
	else if(0xFF == sleep_tmr_reset_value)
	{
		sleep_tmr_reset_value = 5;  // Default for unprogrammed EEPROM
		eeprom_write_byte((uint8_t*)EE_DEVICE_SLEEP_TIMEOUT, sleep_tmr_reset_value);
	}
	else if(sleep_tmr_reset_value > 99)
	{
		sleep_tmr_reset_value = 99;
		eeprom_write_byte((uint8_t*)EE_DEVICE_SLEEP_TIMEOUT, sleep_tmr_reset_value);
	}
	sleep_tmr_reset_value *= 600;  // Convert to decisecs

	// Initialize MRBus address from EEPROM
	mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
	// Bogus addresses, fix to default address
	if (0xFF == mrbus_dev_addr || 0x00 == mrbus_dev_addr)
	{
		mrbus_dev_addr = 0x30;
		eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR, mrbus_dev_addr);
	}

	mrbus_base_addr = eeprom_read_byte((uint8_t*)EE_BASE_ADDR);
	// Bogus addresses, fix to default address
	if (0xFF == mrbus_base_addr || 0x00 == mrbus_base_addr)
	{
		mrbus_base_addr = 0xF0;
		eeprom_write_byte((uint8_t*)EE_BASE_ADDR, mrbus_base_addr);
	}
	
	// Locomotive Address
	locoAddress = eeprom_read_word((uint16_t*)EE_LOCO_ADDRESS);

	// Function configs
	hornFunction = eeprom_read_byte((uint8_t*)EE_HORN_FUNCTION);
	bellFunction = eeprom_read_byte((uint8_t*)EE_BELL_FUNCTION);
	frontDim1Function = eeprom_read_byte((uint8_t*)EE_FRONT_DIM1_FUNCTION);
	frontDim2Function = eeprom_read_byte((uint8_t*)EE_FRONT_DIM2_FUNCTION);
	frontHeadlightFunction = eeprom_read_byte((uint8_t*)EE_FRONT_HEADLIGHT_FUNCTION);
	frontDitchFunction = eeprom_read_byte((uint8_t*)EE_FRONT_DITCH_FUNCTION);
	rearDim1Function = eeprom_read_byte((uint8_t*)EE_REAR_DIM1_FUNCTION);
	rearDim2Function = eeprom_read_byte((uint8_t*)EE_REAR_DIM2_FUNCTION);
	rearHeadlightFunction = eeprom_read_byte((uint8_t*)EE_REAR_HEADLIGHT_FUNCTION);
	rearDitchFunction = eeprom_read_byte((uint8_t*)EE_REAR_DITCH_FUNCTION);
	dynamicFunction = eeprom_read_byte((uint8_t*)EE_DYNAMIC_FUNCTION);
	engineFunction = eeprom_read_byte((uint8_t*)EE_ENGINE_FUNCTION);
	upButtonFunction = eeprom_read_byte((uint8_t*)EE_UP_BUTTON_FUNCTION);
	downButtonFunction = eeprom_read_byte((uint8_t*)EE_DOWN_BUTTON_FUNCTION);
	
	functionForceOn = eeprom_read_dword((uint32_t*)EE_FUNC_FORCE_ON);
	functionForceOff = eeprom_read_dword((uint32_t*)EE_FUNC_FORCE_OFF);
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

	pktTimeout = PKT_TIMEOUT_DECISECS;

	readConfig();

	initPorts();
	initADC();
	enableThrottle();
	initialize100HzTimer();
}

void initLCD(void)
{
	lcd_init(LCD_DISP_ON);
	lcdBacklightEnable();

	wdt_reset();	

	lcd_gotoxy(1, 0);
	lcd_puts("Proto");
	lcd_gotoxy(0, 1);
	lcd_puts("Throttle");

	wdt_reset();

	wait100ms(20);
	lcd_clrscr();
}

int main(void)
{
	uint16_t decisecs_tmp;
	uint8_t i;

	uint8_t inputButtons = 0;
	uint8_t txBuffer[MRBUS_BUFFER_SIZE];

	uint8_t lastThrottlePosition = throttlePosition;
	ReverserPosition lastReverserPosition = reverserPosition;
	uint8_t optionButtonState = 0;

	uint8_t tonnage = 0;
	
	uint8_t backlight = 0;
	
	Screens screenState = LAST_SCREEN;  // Initialize to the last one, since that's the only state guaranteed to be present
	uint8_t subscreenStatus = 0;

	uint8_t allowLatch;
	
	uint8_t decimalNumberIndex = 0;
	uint8_t decimalNumber[4];

	uint8_t *functionPtr = &hornFunction;
	uint8_t functionNumber = 0;

	uint8_t engineState = 0;

	uint32_t functionMask = 0;
	uint32_t lastFunctionMask = 0;
	
	ReverserPosition direction = FORWARD;

	init();

	// Assign after init() so values are read from EEPROM first
	uint16_t newLocoAddress = locoAddress;
	uint8_t newDevAddr = mrbus_dev_addr;
	uint8_t newBaseAddr = mrbus_base_addr;
	uint8_t newSleepTimeout = sleep_tmr_reset_value / 600;

	uint8_t *addrPtr = &newDevAddr;

	setXbeeActive();

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		sleepTimeout_decisecs = sleep_tmr_reset_value;
	}
	
	lcdEnable();

	wdt_reset();

	// Initialize MRBus core
	mrbusPktQueueInitialize(&mrbeeTxQueue, mrbusTxPktBufferArray, MRBUS_TX_BUFFER_DEPTH);
	mrbusPktQueueInitialize(&mrbeeRxQueue, mrbusRxPktBufferArray, MRBUS_RX_BUFFER_DEPTH);
	mrbeeInit();

	sei();	

	wdt_reset();

	// Fire off initial reset version packet
	createVersionPacket(0xFF, txBuffer);
	mrbusPktQueuePush(&mrbeeTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);

	wdt_reset();

	led = LED_OFF;

	initLCD();

	// Initialize the buttons so there are no startup artifacts when we actually use them
	inputButtons = PINB & (0xF6);

	buttonsEnable();
	switchesEnable();

	while(1)
	{
		wdt_reset();

		if (status & STATUS_READ_SWITCHES)
		{
			// Read switches every 10ms
			status &= ~STATUS_READ_SWITCHES;
			inputButtons = debounce(inputButtons, (PINB & (0xF7)));
			processButtons(inputButtons);
			processSwitches(inputButtons);
		}

		processADC();

		// Convert horn to on/off control
		// FIXME: eventually add analog horn functionality
		if(hornPosition > 127)
		{
			controls &= ~(HORN_CONTROL);
		}
		else
		{
			controls |= HORN_CONTROL;
		}

		switch(screenState)
		{
			case MAIN_SCREEN:
				if(backlight)
					lcdBacklightEnable();
				else
					lcdBacklightDisable();
				lcd_gotoxy(2,0);
				printDec4DigWZero(locoAddress);

				lcd_gotoxy(1,1);
				printTime();
				
				if (batteryVoltage >= (VBATT_OKAY/2))  // Divide by 2 since batteryVoltage LSB = 20mV
					setupBatteryChar(BATTERY_FULL);
				else if (batteryVoltage >= (VBATT_WARN/2))  // Divide by 2 since batteryVoltage LSB = 20mV
					setupBatteryChar(BATTERY_HALF);
				else
					setupBatteryChar(BATTERY_EMPTY);

				lcd_gotoxy(0,0);
				lcd_putc(BATTERY_CHAR);

				if((OFF_FUNCTION & upButtonFunction) && (OFF_FUNCTION & downButtonFunction))
				{
					setupSoftkeyChars(BARGRAPH);
					printTonnage(tonnage);
				}
				else
				{
					setupSoftkeyChars(FUNCTION_KEYS);
					lcd_gotoxy(7,0);
					lcd_putc((optionButtonState & UP_OPTION_BUTTON) && !(upButtonFunction & OFF_FUNCTION) ? FUNCTION_ACTIVE_CHAR : FUNCTION_INACTIVE_CHAR);
					lcd_gotoxy(7,1);
					lcd_putc((optionButtonState & DOWN_OPTION_BUTTON) && !(downButtonFunction & OFF_FUNCTION) ? FUNCTION_ACTIVE_CHAR : FUNCTION_INACTIVE_CHAR);
				}

				switch(button)
				{
					case UP_BUTTON:
						if(UP_BUTTON != previousButton)
						{
							if((OFF_FUNCTION & upButtonFunction) && (OFF_FUNCTION & downButtonFunction))
							{
								if(tonnage >= 3)
									tonnage = 3;
								else
									tonnage++;
							}
							else
							{
								if(upButtonFunction & LATCH_FUNCTION)
									optionButtonState ^= UP_OPTION_BUTTON;  // Toggle
								else
									optionButtonState |= UP_OPTION_BUTTON;  // Momentary on
							}
						}
						break;
					case DOWN_BUTTON:
						if(DOWN_BUTTON != previousButton)
						{
							if((OFF_FUNCTION & upButtonFunction) && (OFF_FUNCTION & downButtonFunction))
							{
								if((0 == tonnage) || (tonnage > 3))
									tonnage = 0;
								else
									tonnage--;
							}
							else
							{
								if(downButtonFunction & LATCH_FUNCTION)
									optionButtonState ^= DOWN_OPTION_BUTTON;  // Toggle
								else
									optionButtonState |= DOWN_OPTION_BUTTON;  // Momentary on
							}
						}
						break;
					case SELECT_BUTTON:
						if(SELECT_BUTTON != previousButton)
						{
							if(backlight)
								backlight = 0;
							else
								backlight = 1;
						}
						break;
					case MENU_BUTTON:
					case NO_BUTTON:
						// Release buttons if momentary
						if(!(upButtonFunction & LATCH_FUNCTION))
							optionButtonState &= ~UP_OPTION_BUTTON;
						if(!(downButtonFunction & LATCH_FUNCTION))
							optionButtonState &= ~DOWN_OPTION_BUTTON;
						break;
				}
				break;

			case ENGINE_SCREEN:
				lcdBacklightEnable();
				lcd_gotoxy(0,0);
				lcd_puts(" ENGINE");
				lcd_gotoxy(2,1);
				if(engineState)
					lcd_puts(" ON");
				else
					lcd_puts("OFF");
				switch(button)
				{
					case UP_BUTTON:
						engineState = 1;
						break;
					case DOWN_BUTTON:
						engineState = 0;
						break;
					case SELECT_BUTTON:
							screenState = LAST_SCREEN;
							break;
					case MENU_BUTTON:
					case NO_BUTTON:
						break;
				}
				break;

			case LOCO_SCREEN:
				lcdBacklightEnable();
				if(!subscreenStatus)
				{
					lcd_gotoxy(0,0);
					lcd_puts(" GET NEW");
					lcd_gotoxy(0,1);
					lcd_puts("<-- LOCO");
					switch(button)
					{
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								decimalNumber[0] = (newLocoAddress / 1000) % 10;
								decimalNumber[1] = (newLocoAddress / 100) % 10;
								decimalNumber[2] = (newLocoAddress / 10) % 10;
								decimalNumber[3] = (newLocoAddress) % 10;
								subscreenStatus = 1;
								lcd_clrscr();
							}
							break;
						case MENU_BUTTON:
						case UP_BUTTON:
						case DOWN_BUTTON:
						case NO_BUTTON:
							break;
					}
				}
				else
				{
					for(i=0; i<4; i++)
					{
						lcd_gotoxy(2+i,0);
						lcd_putc('0' + decimalNumber[i]);
						lcd_gotoxy(2+i,1);
						if(i == decimalNumberIndex)
						{
							lcd_putc('^');
						}
						else
						{
							lcd_putc(' ');
						}
					}
					switch(button)
					{
						case UP_BUTTON:
							if(ticks_autoincrement >= button_autoincrement_10ms_ticks)
							{
								if(decimalNumber[decimalNumberIndex] < 9)
									decimalNumber[decimalNumberIndex]++;
								ticks_autoincrement = 0;
							}
							break;
						case DOWN_BUTTON:
							if(ticks_autoincrement >= button_autoincrement_10ms_ticks)
							{
								if(decimalNumber[decimalNumberIndex] > 0)
									decimalNumber[decimalNumberIndex]--;
								ticks_autoincrement = 0;
							}
							break;
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								// FIXME: This should really send a packet to request a new locomotive address and locoAddress is only updated once confirmation received
								newLocoAddress = (decimalNumber[0] * 1000) + (decimalNumber[1] * 100) + (decimalNumber[2] * 10) + decimalNumber[3];
								eeprom_write_word((uint16_t*)EE_LOCO_ADDRESS, newLocoAddress);
								locoAddress = eeprom_read_word((uint16_t*)EE_LOCO_ADDRESS);
								lcd_clrscr();
								lcd_gotoxy(0,0);
								lcd_puts("REQUEST");
								lcd_gotoxy(0,1);
								lcd_puts("SENT");
								wait100ms(10);
								decimalNumberIndex = 0;
								subscreenStatus = 0;
								screenState = LAST_SCREEN;
							}
							break;
						case MENU_BUTTON:
							if(MENU_BUTTON != previousButton)
							{
								decimalNumberIndex++;
								if(decimalNumberIndex > 3)
									decimalNumberIndex = 0;
							}
							break;
						case NO_BUTTON:
							break;
					}
				}
				break;

			case TONNAGE_SCREEN:
				lcdBacklightEnable();
				lcd_gotoxy(0,0);
				switch(tonnage)
				{
					case 0:
						lcd_puts("LIGHT ");
						break;
					case 1:
						lcd_puts("LOW   ");
						break;
					case 2:
						lcd_puts("MEDIUM");
						break;
					case 3:
						lcd_puts("HEAVY ");
						break;
				}
				lcd_gotoxy(0,1);
				switch(tonnage)
				{
					case 0:
						lcd_puts("ENGINE");
						break;
					case 1:
					case 2:
					case 3:
						lcd_puts("WEIGHT");
						break;
				}
				setupSoftkeyChars(BARGRAPH);
				printTonnage(tonnage);
				switch(button)
				{
					case UP_BUTTON:
						if(UP_BUTTON != previousButton)
						{
							if(tonnage >= 3)
								tonnage = 3;
							else
								tonnage++;
						}
						break;
					case DOWN_BUTTON:
						if(DOWN_BUTTON != previousButton)
						{
							if((0 == tonnage) || (tonnage > 3))
								tonnage = 0;
							else
								tonnage--;
						}
						break;
					case SELECT_BUTTON:
							screenState = LAST_SCREEN;
							break;
					case MENU_BUTTON:
					case NO_BUTTON:
						break;
				}
				break;

			case FUNC_SET_SCREEN:
				lcdBacklightEnable();
				if(!subscreenStatus)
				{
					lcd_gotoxy(5,0);
					lcd_puts("SET");
					lcd_gotoxy(0,1);
					lcd_puts("<-- FUNC");
					switch(button)
					{
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								subscreenStatus = 1;
								functionNumber = 0;
								lcd_clrscr();
							}
							break;
						case MENU_BUTTON:
						case UP_BUTTON:
						case DOWN_BUTTON:
						case NO_BUTTON:
							break;
					}
				}
				else
				{
					// Note: _BV() macro doesn't work on 32-bit numbers.  Thus why (1 << functionNumber) is used
					lcd_gotoxy(0,0);
					lcd_puts("F");
					printDec2DigWZero(functionNumber);
					lcd_gotoxy(5,0);
					if(functionForceOn & ((uint32_t)1 << functionNumber))
						lcd_puts(" ON");
					else if(functionForceOff & ((uint32_t)1 << functionNumber))
						lcd_puts("OFF");
					else
						lcd_puts("---");
					switch(button)
					{
						case UP_BUTTON:
							if(ticks_autoincrement >= button_autoincrement_10ms_ticks)
							{
								if( (functionForceOn & ((uint32_t)1 << functionNumber)) || (functionForceOff & ((uint32_t)1 << functionNumber)) )
								{
									// Function turned on, change to turned off (or already turned off)
									functionForceOn &= ~((uint32_t)1 << functionNumber);
									functionForceOff |= ((uint32_t)1 << functionNumber);
								}
								else
								{
									// Function disabled, turn on
									functionForceOff &= ~((uint32_t)1 << functionNumber);
									functionForceOn |= ((uint32_t)1 << functionNumber);
								}
								ticks_autoincrement = 0;
							}
							break;
						case DOWN_BUTTON:
							if(ticks_autoincrement >= button_autoincrement_10ms_ticks)
							{
								if(functionForceOff & ((uint32_t)1 << functionNumber))
								{
									// Function turned off, change to turned on
									functionForceOff &= ~((uint32_t)1 << functionNumber);
									functionForceOn |= ((uint32_t)1 << functionNumber);
								}
								else
								{
									// Function turned on or disabled, disable
									functionForceOff &= ~((uint32_t)1 << functionNumber);
									functionForceOn &= ~((uint32_t)1 << functionNumber);
								}
								ticks_autoincrement = 0;
							}
							break;
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								eeprom_write_dword((uint32_t*)EE_FUNC_FORCE_ON, functionForceOn);
								eeprom_write_dword((uint32_t*)EE_FUNC_FORCE_OFF, functionForceOff);
								lcd_clrscr();
								lcd_gotoxy(1,0);
								lcd_puts("SAVED!");
								wait100ms(7);
								subscreenStatus = 0;  // Escape submenu
								lcd_clrscr();
							}
							break;
						case MENU_BUTTON:
							if(MENU_BUTTON != previousButton)
							{
								// Advance through function settings
								if(++functionNumber > 28)
									functionNumber = 0;
								ticks_autoincrement = 0;
								lcd_clrscr();
							}
							break;
						case NO_BUTTON:
							break;
					}
				}
				break;

			case FUNC_CONFIG_SCREEN:
				lcdBacklightEnable();
				if(!subscreenStatus)
				{
					lcd_gotoxy(2,0);
					lcd_puts("CONFIG");
					lcd_gotoxy(0,1);
					lcd_puts("<-- FUNC");
					switch(button)
					{
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								subscreenStatus = 1;
								functionSetting = 0;
								lcd_clrscr();
							}
							break;
						case MENU_BUTTON:
						case UP_BUTTON:
						case DOWN_BUTTON:
						case NO_BUTTON:
							break;
					}
				}
				else
				{
					allowLatch = 0;
					switch(functionSetting)
					{
						case HORN_FN:
							lcd_gotoxy(0,0);
							lcd_puts("HORN");
							functionPtr = &hornFunction;
							break;
						case BELL_FN:
							lcd_gotoxy(0,0);
							lcd_puts("BELL");
							functionPtr = &bellFunction;
							break;
						case DYNAMIC_FN:
							lcd_gotoxy(0,0);
							lcd_puts("D.BRAKE");
							functionPtr = &dynamicFunction;
							break;
						case ENGINE_FN:
							lcd_gotoxy(0,0);
							lcd_puts("ENGINE");
							functionPtr = &engineFunction;
							break;
						case FRONT_DIM1_FN:
							lcd_gotoxy(0,0);
							lcd_puts("F.DIM #1");
							functionPtr = &frontDim1Function;
							break;
						case FRONT_DIM2_FN:
							lcd_gotoxy(0,0);
							lcd_puts("F.DIM #2");
							functionPtr = &frontDim2Function;
							break;
						case FRONT_HEADLIGHT_FN:
							lcd_gotoxy(0,0);
							lcd_puts("F.HEAD");
							functionPtr = &frontHeadlightFunction;
							break;
						case FRONT_DITCH_FN:
							lcd_gotoxy(0,0);
							lcd_puts("F.DITCH");
							functionPtr = &frontDitchFunction;
							break;
						case REAR_DIM1_FN:
							lcd_gotoxy(0,0);
							lcd_puts("R.DIM #1");
							functionPtr = &rearDim1Function;
							break;
						case REAR_DIM2_FN:
							lcd_gotoxy(0,0);
							lcd_puts("R.DIM #2");
							functionPtr = &rearDim2Function;
							break;
						case REAR_HEADLIGHT_FN:
							lcd_gotoxy(0,0);
							lcd_puts("R.HEAD");
							functionPtr = &rearHeadlightFunction;
							break;
						case REAR_DITCH_FN:
							lcd_gotoxy(0,0);
							lcd_puts("R.DITCH");
							functionPtr = &rearDitchFunction;
							break;
						case UP_FN:
							lcd_gotoxy(0,0);
							lcd_puts("UP BTN");
							functionPtr = &upButtonFunction;
							allowLatch = 1;
							break;
						case DOWN_FN:
							lcd_gotoxy(0,0);
							lcd_puts("DOWN BTN");
							functionPtr = &downButtonFunction;
							allowLatch = 1;
							break;
						case LAST_FN:
							// Should never get here...
							break;
					}

					lcd_gotoxy(0,1);
					lcd_puts("F");
					lcd_gotoxy(1,1);
					if((*functionPtr) & OFF_FUNCTION)
						lcd_puts("--     ");
					else
					{
						printDec2DigWZero((*functionPtr) & 0x1F);
						if(allowLatch)
						{
							lcd_gotoxy(5,1);
							if((*functionPtr) & LATCH_FUNCTION)
								lcd_puts("LAT");
							else
								lcd_puts("MOM");
						}
					}

					switch(button)
					{
						//  |off|latch|0|Func[4:0]|
						case UP_BUTTON:
							if(ticks_autoincrement >= button_autoincrement_10ms_ticks)
							{
								if((*functionPtr) & OFF_FUNCTION)
								{
									(*functionPtr) = 0;  // Turn on
								}
								else
								{
									if(((*functionPtr) & 0x1F) < 28)
										(*functionPtr)++;       // Increment
									else if(allowLatch && !((*functionPtr) & LATCH_FUNCTION))
										(*functionPtr) = LATCH_FUNCTION;  // Set latch bit, reset function number to zero
									else
										(*functionPtr) = ((*functionPtr) & LATCH_FUNCTION) + 28;    // Saturate, preserving latch bit
								}
								ticks_autoincrement = 0;
							}
							break;
						case DOWN_BUTTON:
							if(ticks_autoincrement >= button_autoincrement_10ms_ticks)
							{
								if(~((*functionPtr) & OFF_FUNCTION))
								{
									// Not OFF...
									if(((*functionPtr) & 0x1F) > 0)
										(*functionPtr)--;       // Decrement
									else if(allowLatch && ((*functionPtr) & LATCH_FUNCTION))
										(*functionPtr) = 28;    // Unset latch bit, reset function number to 28
									else
										(*functionPtr) = OFF_FUNCTION;  // Turn off
								}
								ticks_autoincrement = 0;
							}
							break;
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								eeprom_write_byte((uint8_t*)EE_HORN_FUNCTION, hornFunction);
								eeprom_write_byte((uint8_t*)EE_BELL_FUNCTION, bellFunction);
								eeprom_write_byte((uint8_t*)EE_FRONT_DIM1_FUNCTION, frontDim1Function);
								eeprom_write_byte((uint8_t*)EE_FRONT_DIM2_FUNCTION, frontDim2Function);
								eeprom_write_byte((uint8_t*)EE_FRONT_HEADLIGHT_FUNCTION, frontHeadlightFunction);
								eeprom_write_byte((uint8_t*)EE_FRONT_DITCH_FUNCTION, frontDitchFunction);
								eeprom_write_byte((uint8_t*)EE_REAR_DIM1_FUNCTION, rearDim1Function);
								eeprom_write_byte((uint8_t*)EE_REAR_DIM2_FUNCTION, rearDim2Function);
								eeprom_write_byte((uint8_t*)EE_REAR_HEADLIGHT_FUNCTION, rearHeadlightFunction);
								eeprom_write_byte((uint8_t*)EE_REAR_DITCH_FUNCTION, rearDitchFunction);
								eeprom_write_byte((uint8_t*)EE_DYNAMIC_FUNCTION, dynamicFunction);
								eeprom_write_byte((uint8_t*)EE_ENGINE_FUNCTION, engineFunction);
								eeprom_write_byte((uint8_t*)EE_UP_BUTTON_FUNCTION, upButtonFunction);
								eeprom_write_byte((uint8_t*)EE_DOWN_BUTTON_FUNCTION, downButtonFunction);
								lcd_clrscr();
								lcd_gotoxy(1,0);
								lcd_puts("SAVED!");
								wait100ms(7);
								subscreenStatus = 0;  // Escape submenu
								lcd_clrscr();
							}
							break;
						case MENU_BUTTON:
							if(MENU_BUTTON != previousButton)
							{
								// Advance through function settings
								if(++functionSetting >= LAST_FN)
									functionSetting = 0;
								ticks_autoincrement = 0;
								lcd_clrscr();
							}
							break;
						case NO_BUTTON:
							break;
					}
				}
				break;

			case COMM_SCREEN:
				lcdBacklightEnable();
				if(!subscreenStatus)
				{
					lcd_gotoxy(4,0);
					lcd_puts("COMM");
					lcd_gotoxy(0,1);
					lcd_puts("<--  CFG");
					switch(button)
					{
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								subscreenStatus = 1;
								lcd_clrscr();
							}
							break;
						case MENU_BUTTON:
						case UP_BUTTON:
						case DOWN_BUTTON:
						case NO_BUTTON:
							break;
					}
				}
				else
				{
					lcdBacklightEnable();
					lcd_gotoxy(0,0);
					if(1 == subscreenStatus)
					{
						lcd_puts("THR ADR");
						addrPtr = &newDevAddr;
					}
					else
					{
						lcd_puts("BASE ADR");
						addrPtr = &newBaseAddr;
					}
					lcd_gotoxy(2,1);
					if( (addrPtr == &newBaseAddr) && (0xFF == *addrPtr) )
					{
						lcd_puts("ALL ");
					}
					else
					{
						lcd_puts("0x");
						printHex(*addrPtr);
					}
					switch(button)
					{
						case UP_BUTTON:
							if(ticks_autoincrement >= button_autoincrement_10ms_ticks)
							{
								if(*addrPtr < 0xFE)
									(*addrPtr)++;
								if( (0xFF == newDevAddr) )
									newDevAddr = 0;  // Don't allow 0xFF for device address
								ticks_autoincrement = 0;
							}
							break;
						case DOWN_BUTTON:
							if(ticks_autoincrement >= button_autoincrement_10ms_ticks)
							{
								if(*addrPtr > 0)
									(*addrPtr)--;
								ticks_autoincrement = 0;
							}
							break;
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR, newDevAddr);
								eeprom_write_byte((uint8_t*)EE_BASE_ADDR, newBaseAddr);
								mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
								mrbus_base_addr = eeprom_read_byte((uint8_t*)EE_BASE_ADDR);
								lcd_clrscr();
								lcd_gotoxy(1,0);
								lcd_puts("SAVED!");
								wait100ms(7);
								subscreenStatus = 0;  // Escape submenu
								lcd_clrscr();
							}
							break;
						case MENU_BUTTON:
							if(MENU_BUTTON != previousButton)
							{
								// Menu pressed, advance menu
								subscreenStatus++;
								if(subscreenStatus > 2)
									subscreenStatus = 1;
								lcd_clrscr();
							}
							break;
						case NO_BUTTON:
							break;
					}
				}
				break;

			case PREFS_SCREEN:
				lcdBacklightEnable();
				if(!subscreenStatus)
				{
					lcd_gotoxy(3,0);
					lcd_puts("PREFS");
					lcd_gotoxy(0,1);
					lcd_puts("<--");
					switch(button)
					{
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								subscreenStatus = 1;
								lcd_clrscr();
							}
							break;
						case MENU_BUTTON:
						case UP_BUTTON:
						case DOWN_BUTTON:
						case NO_BUTTON:
							break;
					}
				}
				else
				{
					lcdBacklightEnable();
					lcd_gotoxy(0,0);
					lcd_puts("SLEEP");
					lcd_gotoxy(0,1);
					lcd_puts("DLY: ");
					printDec2Dig(newSleepTimeout);
					lcd_puts("M");
					switch(button)
					{
						case UP_BUTTON:
							if(ticks_autoincrement >= button_autoincrement_10ms_ticks)
							{
								if(newSleepTimeout < 99)
									newSleepTimeout++;
								else
									newSleepTimeout = 99;
								ticks_autoincrement = 0;
							}
							break;
						case DOWN_BUTTON:
							if(ticks_autoincrement >= button_autoincrement_10ms_ticks)
							{
								if(newSleepTimeout > 1)
									newSleepTimeout--;
								else
									newSleepTimeout = 1;
								ticks_autoincrement = 0;
							}
							break;
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								eeprom_write_byte((uint8_t*)EE_DEVICE_SLEEP_TIMEOUT, newSleepTimeout);
								sleep_tmr_reset_value = eeprom_read_byte((uint8_t*)EE_DEVICE_SLEEP_TIMEOUT) * 600;
								lcd_clrscr();
								lcd_gotoxy(1,0);
								lcd_puts("SAVED!");
								wait100ms(7);
								subscreenStatus = 0;  // Escape submenu
								lcd_clrscr();
							}
							break;
						case MENU_BUTTON:
							if(MENU_BUTTON != previousButton)
							{
								// Menu pressed, advance menu
								subscreenStatus++;
								if(subscreenStatus > 1)
									subscreenStatus = 1;
								lcd_clrscr();
							}
							break;
						case NO_BUTTON:
							break;
					}
				}
				break;

			case DIAG_SCREEN:
				lcdBacklightEnable();
				if(!subscreenStatus)
				{
					lcd_gotoxy(3,0);
					lcd_puts("DIAGS");
					lcd_gotoxy(0,1);
					lcd_puts("<--");
					switch(button)
					{
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								subscreenStatus = 1;
								setupDiagChars();
								lcd_clrscr();
							}
							break;
						case MENU_BUTTON:
						case UP_BUTTON:
						case DOWN_BUTTON:
						case NO_BUTTON:
							break;
					}
				}
				else
				{
					if(1 == subscreenStatus)
					{
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
						lcd_puts((controls & DYNAMIC_CONTROL) ? "DB":"  ");
						lcd_gotoxy(4, 1);
						lcd_putc((controls & BELL_CONTROL) ? BELL_CHAR : ' ');
						lcd_gotoxy(5, 1);
						lcd_putc((controls & HORN_CONTROL) ? HORN_CHAR : ' ');

						lcd_gotoxy(7, 1);
						switch(frontLight)
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
					}
					else if(2 == subscreenStatus)
					{
						lcdBacklightEnable();
						lcd_gotoxy(0,0);
						lcd_puts("SLEEP");
						lcd_gotoxy(0,1);
						ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
						{
							decisecs_tmp = sleepTimeout_decisecs;
						}
						printDec4DigWZero(decisecs_tmp);
					}
					else if(3 == subscreenStatus)
					{
						lcdBacklightEnable();
						lcd_gotoxy(0,0);
						lcd_puts("PKT TIME");
						lcd_gotoxy(0,1);
						printDec4DigWZero(pktTimeout);
					}
					else if(4 == subscreenStatus)
					{
						lcdBacklightEnable();
						lcd_gotoxy(0,0);
						lcd_puts("BATTERY");
						lcd_gotoxy(1,1);
						lcd_putc('0' + (((batteryVoltage*2)/100)%10));
						lcd_putc('.');
						lcd_putc('0' + (((batteryVoltage*2)/10)%10));
						lcd_putc('0' + ((batteryVoltage*2)%10));
						lcd_putc('V');
					}
					else if(5 == subscreenStatus)
					{
						lcdBacklightEnable();
						lcd_gotoxy(0,0);
						lcd_puts("VERSION");
						lcd_gotoxy(1,1);
						lcd_puts(VERSION_STRING);
					}
					switch(button)
					{
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								subscreenStatus = 0;  // Escape submenu
								lcd_clrscr();
							}
							break;
						case MENU_BUTTON:
							if(MENU_BUTTON != previousButton)
							{
								// Menu pressed, advance menu
								subscreenStatus++;
								if(subscreenStatus > 5)
									subscreenStatus = 1;
								lcd_clrscr();
							}
							break;
						case UP_BUTTON:
						case DOWN_BUTTON:
						case NO_BUTTON:
							break;
					}
				}
				break;

			case LAST_SCREEN:
				// Clean up and reset
				lcd_clrscr();
				screenState = 0;
				break;
		}
		// Process Menu button, but only if not in a subscreen
		// Do this after main screen loop so screens can also do cleanup when menu is pressed
		if(!subscreenStatus)
		{
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
		}

		previousButton = button;

		wdt_reset();

		// Handle any packets that may have come in
		if (mrbusPktQueueDepth(&mrbeeRxQueue))
		{
			pktTimeout = PKT_TIMEOUT_DECISECS;
			PktHandler();
		}

		wdt_reset();

		if (0 == pktTimeout)
			led = LED_RED_FASTBLINK;
		else
			led = LED_GREEN;
		

		functionMask = 0;
		if((controls & HORN_CONTROL) && !(hornFunction & OFF_FUNCTION))
			functionMask |= (uint32_t)1 << (hornFunction & 0x1F);
		if((controls & BELL_CONTROL) && !(bellFunction & OFF_FUNCTION))
			functionMask |= (uint32_t)1 << (bellFunction & 0x1F);
		if((controls & DYNAMIC_CONTROL) && !(dynamicFunction & OFF_FUNCTION))
			functionMask |= (uint32_t)1 << (dynamicFunction & 0x1F);
		if(engineState && !(engineFunction & OFF_FUNCTION))
			functionMask |= (uint32_t)1 << (engineFunction & 0x1F);
		if((optionButtonState & UP_OPTION_BUTTON) && !(upButtonFunction & OFF_FUNCTION))
			functionMask |= (uint32_t)1 << (upButtonFunction & 0x1F);
		if((optionButtonState & DOWN_OPTION_BUTTON) && !(downButtonFunction & OFF_FUNCTION))
			functionMask |= (uint32_t)1 << (downButtonFunction & 0x1F);

		wdt_reset();

		switch(frontLight)
		{
			case LIGHT_OFF:
				break;
			case LIGHT_DIM:
				if(!(frontDim1Function & OFF_FUNCTION))
					functionMask |= (uint32_t)1 << (frontDim1Function & 0x1F);
				if(!(frontDim2Function & OFF_FUNCTION))
					functionMask |= (uint32_t)1 << (frontDim2Function & 0x1F);
				break;
			case LIGHT_BRIGHT:
				if(!(frontHeadlightFunction & OFF_FUNCTION))
					functionMask |= (uint32_t)1 << (frontHeadlightFunction & 0x1F);
				break;
			case LIGHT_BRIGHT_DITCH:
				if(!(frontHeadlightFunction & OFF_FUNCTION))
					functionMask |= (uint32_t)1 << (frontHeadlightFunction & 0x1F);
				if(!(frontDitchFunction & OFF_FUNCTION))
					functionMask |= (uint32_t)1 << (frontDitchFunction & 0x1F);
				break;
		}

		wdt_reset();

		switch(rearLight)
		{
			case LIGHT_OFF:
				break;
			case LIGHT_DIM:
				if(!(rearDim1Function & OFF_FUNCTION))
					functionMask |= (uint32_t)1 << (rearDim1Function & 0x1F);
				if(!(rearDim2Function & OFF_FUNCTION))
					functionMask |= (uint32_t)1 << (rearDim2Function & 0x1F);
				break;
			case LIGHT_BRIGHT:
				if(!(rearHeadlightFunction & OFF_FUNCTION))
					functionMask |= (uint32_t)1 << (rearHeadlightFunction & 0x1F);
				break;
			case LIGHT_BRIGHT_DITCH:
				if(!(rearHeadlightFunction & OFF_FUNCTION))
					functionMask |= (uint32_t)1 << (rearHeadlightFunction & 0x1F);
				if(!(rearDitchFunction & OFF_FUNCTION))
					functionMask |= (uint32_t)1 << (rearDitchFunction & 0x1F);
				break;
		}

		functionMask |= functionForceOn;
		functionMask &= ~functionForceOff;

		uint8_t inputsChanged =	(reverserPosition != lastReverserPosition) ||
									(throttlePosition != lastThrottlePosition) ||
									(functionMask != lastFunctionMask);

		// Reset sleep timer
		if( 
			(NO_BUTTON != button) || 
			(FORWARD == reverserPosition) || 
			(REVERSE == reverserPosition) ||
			inputsChanged
			)
		{
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
			{
				sleepTimeout_decisecs = sleep_tmr_reset_value;
			}
		}

		wdt_reset();
		
		// Transmission criteria...
		// > 1 decisec from last transmission and...
		// stuff changed
		// *****  or *****
		// it's been more than the transmission timeout
		
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			decisecs_tmp = decisecs;
		}
		if (
				adcLoopInitialized() &&
				((inputsChanged && decisecs_tmp > 1) || (decisecs_tmp >= update_decisecs)) &&
				!(mrbusPktQueueFull(&mrbeeTxQueue))
			)
		{
			inputsChanged = 0;
			lastReverserPosition = reverserPosition;
			lastThrottlePosition = throttlePosition;
			lastFunctionMask = functionMask;
			
			txBuffer[MRBUS_PKT_DEST] = mrbus_base_addr;
			txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
			txBuffer[MRBUS_PKT_LEN] = 14;
			txBuffer[5] = 'S';

			txBuffer[6] = locoAddress >> 8;
			txBuffer[7] = locoAddress & 0xFF;
			
			uint8_t throttleTemp = throttlePosition * 16;
			txBuffer[8] = (throttleTemp > 127) ? 127 : throttleTemp;
			
			switch(reverserPosition)
			{
				case FORWARD:
					direction = FORWARD;
					break;
				case REVERSE:
					direction = REVERSE;
					break;
				case NEUTRAL:
					// Preserve previous direction
					break;
			}
			
			if(FORWARD == direction)
				txBuffer[8] |= 0x80;
			
			txBuffer[9]  = functionMask >> 24;
			txBuffer[10] = functionMask >> 16;
			txBuffer[11] = functionMask >> 8;
			txBuffer[12] = functionMask & 0xFF;

			txBuffer[13] = batteryVoltage;	
			mrbusPktQueuePush(&mrbeeTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
			{
				decisecs = 0;
			}
		}

		if (mrbusPktQueueDepth(&mrbeeTxQueue))
		{
			wdt_reset();
			mrbeeTransmit();
		}

		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			decisecs_tmp = sleepTimeout_decisecs;
		}
		if (0 == decisecs_tmp)
		{
			wdt_reset();

			// Time to nod off
			led = LED_OFF;

			// Disable internal power-sucking peripherals (need to be enabled when awake)
			ADCSRA &= ~_BV(ADEN);
			setXbeeSleep();
			lcdDisable();
			lcdBacklightDisable();
			TIMSK0 &= ~_BV(OCIE0A);  // Disable 100Hz timer (to prevent LEDs from blinking on right before sleeping)
			ledGreenOff();
			ledRedOff();
			switchesDisable();  // Don't disable buttons
			disableThrottle();
			
			// Reinforce that these are off
			disablePots();
			disableReverser();
			disableLightSwitches();

			// Wake up from sleep on a button press or taking the reverser out of neutral
			while(
				(NO_BUTTON == button) &&
				(FORWARD != reverserPosition) &&
				(REVERSE != reverserPosition)
				)
			{
				system_sleep(10);
				// Quick read buttons
				inputButtons = PINB & (0xF7);
				processButtons(inputButtons);
				// FIXME: quick read reverser
			}
			
			// Re-enable chip internal bits (ADC, pots, reverser, light switches done in main loop)
			setXbeeActive();
			lcdEnable();
			initLCD();
			initialize100HzTimer();
			switchesEnable();
			enableThrottle();

			// Initialize the buttons so there are no startup artifacts when we actually use them
			inputButtons = debounce(inputButtons, (PINB & (0xF7)));
			inputButtons = debounce(inputButtons, (PINB & (0xF7)));
			inputButtons = debounce(inputButtons, (PINB & (0xF7)));
			inputButtons = debounce(inputButtons, (PINB & (0xF7)));
			processButtons(inputButtons);
			processSwitches(inputButtons);
			previousButton = button;  // Prevent extraneous menu advances

			ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
			{
				sleepTimeout_decisecs = sleep_tmr_reset_value;
			}
		}

	}

}


