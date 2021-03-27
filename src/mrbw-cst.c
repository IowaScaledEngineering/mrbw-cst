/*************************************************************************
Title:    Control Stand Throttle
Authors:  Michael D. Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     mrbw-cst.c
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
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <util/atomic.h>

#include "mrbee.h"

#include "lcd.h"

#include "cst-common.h"
#include "cst-lcd.h"
#include "cst-hardware.h"
#include "cst-eeprom.h"
#include "cst-functions.h"
#include "cst-battery.h"
#include "cst-engine.h"
#include "cst-pressure.h"
#include "cst-tonnage.h"
#include "cst-time.h"
#include "cst-math.h"

//#define FAST_SLEEP
#ifdef FAST_SLEEP
#warning "Fast Sleep Enabled!"
#endif


#define LONG_PRESS_10MS_TICKS             100
#define BUTTON_AUTOINCREMENT_10MS_TICKS    50
#define BUTTON_AUTOINCREMENT_ACCEL         10
#define BUTTON_AUTOINCREMENT_MINIMUM        5

#define SLEEP_TMR_RESET_VALUE_MIN           1
#define SLEEP_TMR_RESET_VALUE_DEFAULT       5
#define SLEEP_TMR_RESET_VALUE_MAX          99

// Alerter timer in units of 1/4 minute (15 seconds), zero is off
#define ALERTER_TMR_RESET_VALUE_MIN         0
#define ALERTER_TMR_RESET_VALUE_DEFAULT     0
#define ALERTER_TMR_RESET_VALUE_MAX        60

#define IS_ALERTER_ENABLED                 (0 != alerter_tmr_reset_value)

#define TX_HOLDOFF_MIN                     10
#define TX_HOLDOFF_DEFAULT                 15

#define UPDATE_DECISECS_MIN                10
#define UPDATE_DECISECS_DEFAULT            10
#define UPDATE_DECISECS_MAX                100

#define MRBUS_DEV_ADDR_MIN                 0x30
#define MRBUS_DEV_ADDR_DEFAULT             0x30
#define MRBUS_DEV_ADDR_MAX                 0x49

#define MRBUS_BASE_ADDR_MIN                0xD0
#define MRBUS_BASE_ADDR_DEFAULT            0xD0
#define MRBUS_BASE_ADDR_MAX                0xEF

#define TIME_SOURCE_ADDRESS_DEFAULT        0x00

#define RESET_COUNTER_RESET_VALUE   5
uint8_t resetCounter = RESET_COUNTER_RESET_VALUE;

// 5 sec timeout for packets from base, base transmits every 1 sec
#define PKT_TIMEOUT_DECISECS   50
volatile uint8_t pktTimeout = 0;

uint32_t baseVersion;
char baseString[9];

#define STATUS_READ_SWITCHES          0x01

#define HORN_CONTROL      0x01
#define BELL_CONTROL      0x02
#define AUX_CONTROL       0x04
#define BRAKE_CONTROL     0x08
#define BRAKE_OFF_CONTROL 0x10
#define THR_UNLK_CONTROL  0x80

#define HORN_HYSTERESIS   5
#define BRAKE_HYSTERESIS  5

// BRAKE_PULSE_WIDTH is in decisecs
// It is the minimum on time for the pulsed brake
#define BRAKE_PULSE_WIDTH_MIN       2
#define BRAKE_PULSE_WIDTH_DEFAULT   5
#define BRAKE_PULSE_WIDTH_MAX      10

uint8_t brakePulseWidth = BRAKE_PULSE_WIDTH_DEFAULT;

// Boolean config bits (EEPROM, global)
#define CONFIGBITS_LED_BLINK         0
#define CONFIGBITS_REVERSER_LOCK     4
#define CONFIGBITS_STRICT_SLEEP      5

#define CONFIGBITS_DEFAULT                 (_BV(CONFIGBITS_LED_BLINK) | _BV(CONFIGBITS_REVERSER_LOCK) | _BV(CONFIGBITS_STRICT_SLEEP))
uint8_t configBits = CONFIGBITS_DEFAULT;

// Boolean option bits (EEPROM, per config)
#define OPTIONBITS_ESTOP_ON_BRAKE    0
#define OPTIONBITS_REVERSER_SWAP     1
#define OPTIONBITS_VARIABLE_BRAKE    2
#define OPTIONBITS_STEPPED_BRAKE     3

#define OPTIONBITS_DEFAULT                 (_BV(OPTIONBITS_ESTOP_ON_BRAKE))
uint8_t optionBits = OPTIONBITS_DEFAULT;

// Boolean system bits (volatile, global)
#define SYSTEMBITS_MENU_LOCK         0
#define SYSTEMBITS_ADV_FUNC          1

#define SYSTEMBITS_DEFAULT                 0x00
uint8_t systemBits = SYSTEMBITS_DEFAULT;


#define MRBUS_TX_BUFFER_DEPTH 16
#define MRBUS_RX_BUFFER_DEPTH 8

MRBusPacket mrbusTxPktBufferArray[MRBUS_TX_BUFFER_DEPTH];
MRBusPacket mrbusRxPktBufferArray[MRBUS_RX_BUFFER_DEPTH];

uint8_t mrbus_dev_addr = 0;
uint8_t mrbus_base_addr = 0;

uint8_t lastRSSI = 0xFF;

uint16_t locoAddress = 0;

#define BRAKE_DEAD_ZONE 5

uint8_t hornThreshold;
uint8_t brakeThreshold;
uint8_t brakeLowThreshold;
uint8_t brakeHighThreshold;

volatile uint8_t brakeCounter;

uint8_t notchSpeedStep[8];

volatile uint16_t button_autoincrement_10ms_ticks = BUTTON_AUTOINCREMENT_10MS_TICKS;
volatile uint16_t ticks_autoincrement = BUTTON_AUTOINCREMENT_10MS_TICKS;

volatile uint8_t ticks;
volatile uint16_t decisecs = 0;
volatile uint16_t sleepTimeout_decisecs = 0;
volatile uint16_t alerterTimeout_decisecs = 0;
volatile uint8_t txHoldoff = 0;
volatile uint8_t status = 0;

uint16_t update_decisecs = UPDATE_DECISECS_DEFAULT;
uint8_t txHoldoff_centisecs = TX_HOLDOFF_DEFAULT;

static uint8_t timeSourceAddress = 0xFF;

#define THROTTLE_STATUS_SLEEP           0x80
#define THROTTLE_STATUS_ALERTER         0x40
#define THROTTLE_STATUS_ALL_STOP        0x02
#define THROTTLE_STATUS_EMERGENCY       0x01

volatile uint8_t throttleStatus = 0;

uint8_t estopStatus = 0;

#define ESTOP_BRAKE    0x01
#define ESTOP_BUTTON   0x02
#define ESTOP_ALERTER  0x04

uint16_t sleep_tmr_reset_value;
uint16_t alerter_tmr_reset_value;

// Define the menu screens and menu order
// Must end with LAST_SCREEN
typedef enum
{
	MAIN_SCREEN = 0,
	ENGINE_SCREEN,
	SPECFN_SCREEN,
	LOAD_CONFIG_SCREEN,
	SAVE_CONFIG_SCREEN,
	LOCO_SCREEN,
	FORCE_FUNC_SCREEN,
	CONFIG_FUNC_SCREEN,
	NOTCH_CONFIG_SCREEN,
	OPTION_SCREEN,
	SYSTEM_SCREEN,
	COMM_SCREEN,
	PREFS_SCREEN,
	THRESHOLD_CAL_SCREEN,
	DIAG_SCREEN,
	LAST_SCREEN  // Must be the last screen
} Screens;

enum
{
	SPECFN_SUBSCREEN_PRESSURE = 1,
//	SPECFN_SUBSCREEN_TONNAGE
};

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
	BRAKE_LOW_BEGIN,
	BRAKE_LOW_WAIT,
	BRAKE_20PCNT_BEGIN,
	BRAKE_20PCNT_WAIT,
	BRAKE_40PCNT_BEGIN,
	BRAKE_40PCNT_WAIT,
	BRAKE_60PCNT_BEGIN,
	BRAKE_60PCNT_WAIT,
	BRAKE_80PCNT_BEGIN,
	BRAKE_80PCNT_WAIT,
	BRAKE_FULL_BEGIN,
	BRAKE_FULL_WAIT,
} BrakeStates;

uint32_t functionForceOn  = 0;
uint32_t functionForceOff = 0;

#define UP_OPTION_BUTTON   0x01
#define DOWN_OPTION_BUTTON 0x02

uint8_t controls = 0;

#define ENGINE_TIMER_DECISECS      20
volatile uint8_t engineTimer = 0;

#define ENGINE_SCREEN_TIMER       100
volatile uint8_t engineScreenTimer = 0;

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
	buf[10] = HWREV_MAJOR; // Hardware Major Revision
	buf[11] = HWREV_MINOR; // Hardware Minor Revision
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
	uint8_t rssi;

	if (0 == mrbeePktQueuePop(&mrbeeRxQueue, rxBuffer, sizeof(rxBuffer), &rssi))
		return;

	if(mrbus_base_addr == rxBuffer[MRBUS_PKT_SRC])
		pktTimeout = PKT_TIMEOUT_DECISECS;

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
	else if ('P' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// EEPROM Extended WRITE Packet
		// [dest][src][len][crcL][crcH]['P'] [addrL][addrH] [data0] ... [dataN]
		//  - Silent, no response packet
		// Write up to 12 bytes to EEPROM starting at {addrH,addrL}
		// Number of bytes actually written determined by len
		// Background write, values are not automatically reloaded
		uint8_t pktPtr;
		uint16_t eepAddr = ((uint16_t)rxBuffer[7] * 256) + rxBuffer[6];
		for(pktPtr = 0; pktPtr < (rxBuffer[2] - 8); pktPtr++)
		{
			eeprom_write_byte((uint8_t*)(eepAddr + pktPtr), rxBuffer[8+pktPtr]);
		}
		goto PktIgnore;	
	}
	else if ('Q' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
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
		txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
		txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		if(7 == rxBuffer[MRBUS_PKT_LEN])
		{
			// EEPROM READ Packet
			// [dest][src][len][crcL][crcH]['R'] [addrL]
			// [dest][src][len][crcL][crcH]['r'] [addrL][rdData0]
			txBuffer[MRBUS_PKT_LEN] = 8;			
			txBuffer[MRBUS_PKT_TYPE] = 'r';
			txBuffer[6] = rxBuffer[6];
			txBuffer[7] = eeprom_read_byte((uint8_t*)(uint16_t)rxBuffer[6]);			
			mrbusPktQueuePush(&mrbeeTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
			goto PktIgnore;
		}
		else
		{
			// EEPROM Extended READ Packet
			// [dest][src][len][crcL][crcH]['Q'] [addrL][addrH] [bytes]
			// [dest][src][len][crcL][crcH]['q'] [addrL][addrH] [rdData0] ... [rdDataN]
			uint8_t pktPtr;
			uint8_t bytesToRead = rxBuffer[8];
			if(bytesToRead > (MRBUS_BUFFER_SIZE - 8))
				bytesToRead = MRBUS_BUFFER_SIZE - 8;
			txBuffer[MRBUS_PKT_LEN] = 8 + bytesToRead;
			txBuffer[MRBUS_PKT_TYPE] = 'q';
			txBuffer[6] = rxBuffer[6];  // Reflect starting addr
			txBuffer[7] = rxBuffer[7];
			uint16_t eepAddr = ((uint16_t)rxBuffer[7] * 256) + rxBuffer[6];
			for(pktPtr = 0; pktPtr < bytesToRead; pktPtr++)
			{
				txBuffer[8+pktPtr] = eeprom_read_byte((uint8_t*)(eepAddr + pktPtr));
			}
			mrbusPktQueuePush(&mrbeeTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
			goto PktIgnore;
		}
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
	else if ('T' == rxBuffer[MRBUS_PKT_TYPE] &&
				( (0xFF == timeSourceAddress) || 
				  (rxBuffer[MRBUS_PKT_SRC] == timeSourceAddress) ||
				  ((rxBuffer[MRBUS_PKT_SRC] == mrbus_base_addr) && (0 == timeSourceAddress)) )
		)
	{
		// It's a time packet from our time reference source
		processTimePacket(rxBuffer);
	}
	else if ( ('V' == (rxBuffer[MRBUS_PKT_TYPE] & 0xDF)) &&
		(mrbus_base_addr == rxBuffer[MRBUS_PKT_SRC]) )
	{
		// It's a version packet from our assigned base station
		lastRSSI = rssi;
		baseVersion = ((uint32_t)rxBuffer[7] << 16) | ((uint16_t)rxBuffer[8] << 8) | rxBuffer[9];
		memset(baseString,' ', 8);  // Fill with spaces before copying string
		baseString[8] = 0;  // NULL terminate
		memcpy(baseString, &rxBuffer[12], min(rxBuffer[MRBUS_PKT_LEN]-12, 8));
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


ISR(PCINT1_vect) { }  // Used for waking from sleep


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
		ticks_autoincrement = 0;
	}
}

void processSwitches(uint8_t inputButtons)
{
	// Called every 10ms
	if(inputButtons & _BV(AUX_PIN))
		controls &= ~(AUX_CONTROL);
	else
		controls |= AUX_CONTROL;

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
	enableTimer();
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

		if(alerterTimeout_decisecs)
		{
#ifdef FAST_SLEEP
			alerterTimeout_decisecs-=10;
#else
			alerterTimeout_decisecs--;
#endif
		}

		ledUpdate();

		if (engineTimer)
			engineTimer--;
		
		if (engineScreenTimer)
			engineScreenTimer++;
		
		brakeCounter++;
		if(brakeCounter >= (4*brakePulseWidth))
			brakeCounter = 0;
		
		updateTime10Hz();
		updatePressure10Hz();
	}

	if(txHoldoff)
		txHoldoff--;

	if(ticks_autoincrement < button_autoincrement_10ms_ticks)
			ticks_autoincrement++;
}

void readConfig(void)
{
	uint8_t i;
	
	// Parse version string
	char version_string[] = VERSION_STRING;
	char *ptr = version_string;
	uint8_t version = 0;
	while(('.' != *ptr) && ('\0' != *ptr))
	{
		version *= 10;
		version += *ptr - '0';
		ptr++;
	}
	uint8_t ee_version = eeprom_read_byte((uint8_t*)EE_VERSION_MAJOR);
	if(ee_version != version)
	{
		eeprom_write_byte((uint8_t*)EE_VERSION_MAJOR, version);
	}

	if('.' == *ptr)
	{
		ptr++;
	}

	version = 0;
	while(('.' != *ptr) && ('\0' != *ptr))
	{
		version *= 10;
		version += *ptr - '0';
		ptr++;
	}
	ee_version = eeprom_read_byte((uint8_t*)EE_VERSION_MINOR);
	if(ee_version != version)
	{
		eeprom_write_byte((uint8_t*)EE_VERSION_MINOR, version);
	}


	update_decisecs = (uint16_t)eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_L) | (((uint16_t)eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_H)) << 8);
	if(update_decisecs < UPDATE_DECISECS_MIN)
	{
		update_decisecs = UPDATE_DECISECS_MIN;
		eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_H, update_decisecs >> 8);
		eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_L, update_decisecs & 0xFF);
	}
	else if(update_decisecs > UPDATE_DECISECS_MAX)
	{
		update_decisecs = UPDATE_DECISECS_MAX;
		eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_H, update_decisecs >> 8);
		eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_L, update_decisecs & 0xFF);
	}
	
	txHoldoff_centisecs = eeprom_read_byte((uint8_t*)EE_TX_HOLDOFF);
	if(txHoldoff_centisecs < TX_HOLDOFF_MIN)
	{
		txHoldoff_centisecs = TX_HOLDOFF_MIN;
		eeprom_write_byte((uint8_t*)EE_TX_HOLDOFF, txHoldoff_centisecs);
	}

	// Battery stuff
	uint8_t decivoltsOkay = eeprom_read_byte((uint8_t*)EE_BATTERY_OKAY);
	uint8_t decivoltsWarn = eeprom_read_byte((uint8_t*)EE_BATTERY_WARN);
	uint8_t decivoltsCritical = eeprom_read_byte((uint8_t*)EE_BATTERY_CRITICAL);
	setBatteryLevels(decivoltsOkay, decivoltsWarn, decivoltsCritical);
	if(getBatteryOkay() != decivoltsOkay)
		eeprom_write_byte((uint8_t*)EE_BATTERY_OKAY, getBatteryOkay());
	if(getBatteryWarn() != decivoltsWarn)
		eeprom_write_byte((uint8_t*)EE_BATTERY_WARN, getBatteryWarn());
	if(getBatteryCritical() != decivoltsCritical)
		eeprom_write_byte((uint8_t*)EE_BATTERY_CRITICAL, getBatteryCritical());

	// Read the number of minutes before sleeping from EEP and store it.
	// If it's not in range, clamp it.
	// Abuse sleep_tmr_reset_value to read the EEPROM value in minutes before converting to decisecs
	sleep_tmr_reset_value = eeprom_read_byte((uint8_t*)EE_DEVICE_SLEEP_TIMEOUT);
	if(sleep_tmr_reset_value < SLEEP_TMR_RESET_VALUE_MIN)
	{
		sleep_tmr_reset_value = SLEEP_TMR_RESET_VALUE_MIN;
		eeprom_write_byte((uint8_t*)EE_DEVICE_SLEEP_TIMEOUT, sleep_tmr_reset_value);
	}
	else if(0xFF == sleep_tmr_reset_value)
	{
		sleep_tmr_reset_value = SLEEP_TMR_RESET_VALUE_DEFAULT;  // Default for unprogrammed EEPROM
		eeprom_write_byte((uint8_t*)EE_DEVICE_SLEEP_TIMEOUT, sleep_tmr_reset_value);
	}
	else if(sleep_tmr_reset_value > SLEEP_TMR_RESET_VALUE_MAX)
	{
		sleep_tmr_reset_value = SLEEP_TMR_RESET_VALUE_MAX;
		eeprom_write_byte((uint8_t*)EE_DEVICE_SLEEP_TIMEOUT, sleep_tmr_reset_value);
	}
	sleep_tmr_reset_value *= 600;  // Convert to decisecs

	// Read the number of minutes before sleeping from EEP and store it.
	// If it's not in range, clamp it.
	// Abuse alerter_tmr_reset_value to read the EEPROM value in minutes before converting to decisecs
	alerter_tmr_reset_value = eeprom_read_byte((uint8_t*)EE_ALERTER_TIMEOUT);
	if(alerter_tmr_reset_value < ALERTER_TMR_RESET_VALUE_MIN)
	{
		alerter_tmr_reset_value = ALERTER_TMR_RESET_VALUE_MIN;
		eeprom_write_byte((uint8_t*)EE_ALERTER_TIMEOUT, alerter_tmr_reset_value);
	}
	else if(0xFF == alerter_tmr_reset_value)
	{
		alerter_tmr_reset_value = ALERTER_TMR_RESET_VALUE_DEFAULT;  // Default for unprogrammed EEPROM
		eeprom_write_byte((uint8_t*)EE_ALERTER_TIMEOUT, alerter_tmr_reset_value);
	}
	else if(alerter_tmr_reset_value > ALERTER_TMR_RESET_VALUE_MAX)
	{
		alerter_tmr_reset_value = ALERTER_TMR_RESET_VALUE_MAX;
		eeprom_write_byte((uint8_t*)EE_ALERTER_TIMEOUT, alerter_tmr_reset_value);
	}
	alerter_tmr_reset_value *= 600/4;  // Convert to decisecs

	// Fast clock
	uint8_t maxDeadReckoningTime = eeprom_read_byte((uint8_t*)EE_DEAD_RECKONING_TIME);
	setMaxDeadReckoningTime(maxDeadReckoningTime);
	if(getMaxDeadReckoningTime() != maxDeadReckoningTime)
		eeprom_write_byte((uint8_t*)EE_DEAD_RECKONING_TIME, getMaxDeadReckoningTime());

	timeSourceAddress = eeprom_read_byte((uint8_t*)EE_TIME_SOURCE_ADDRESS);

	// Pressure
	uint8_t pressureConfig = eeprom_read_byte((uint8_t*)EE_PRESSURE_CONFIG);
	setPressureConfig(pressureConfig);
	if(getPressureConfig() != pressureConfig)
		eeprom_write_byte((uint8_t*)EE_PRESSURE_CONFIG, getPressureConfig());

	configBits = eeprom_read_byte((uint8_t*)EE_CONFIGBITS);

	// Initialize MRBus address from EEPROM
	mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
	// Fix bogus addresses
	if(mrbus_dev_addr < MRBUS_DEV_ADDR_MIN)
	{
		mrbus_dev_addr = MRBUS_DEV_ADDR_MIN;
		eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR, mrbus_dev_addr);
	}
	else if(mrbus_dev_addr > MRBUS_DEV_ADDR_MAX)
	{
		mrbus_dev_addr = MRBUS_DEV_ADDR_MAX;
		eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR, mrbus_dev_addr);
	}

	mrbus_base_addr = eeprom_read_byte((uint8_t*)EE_BASE_ADDR);
	// Fix bogus addresses
	if(mrbus_base_addr < MRBUS_BASE_ADDR_MIN)
	{
		mrbus_base_addr = MRBUS_BASE_ADDR_MIN;
		eeprom_write_byte((uint8_t*)EE_BASE_ADDR, mrbus_base_addr);
	}
	else if(mrbus_base_addr > MRBUS_BASE_ADDR_MAX)
	{
		mrbus_base_addr = MRBUS_BASE_ADDR_MAX;
		eeprom_write_byte((uint8_t*)EE_BASE_ADDR, mrbus_base_addr);
	}

	// Locomotive Address
	locoAddress = eeprom_read_word((uint16_t*)EE_LOCO_ADDRESS);
	if(locoAddress & LOCO_ADDRESS_SHORT)
	{
		if((locoAddress & ~(LOCO_ADDRESS_SHORT)) > 127)
		{
			// Invalid Short Address, reset to a sane value
			locoAddress = 127;
			eeprom_write_word((uint16_t*)EE_LOCO_ADDRESS, locoAddress);
		}
	}
	else
	{
		if(locoAddress > 9999)
		{
			// Invalid Long Address, reset to a sane value
			locoAddress = 9999;
			eeprom_write_word((uint16_t*)EE_LOCO_ADDRESS, locoAddress);
		}
	}

	// Function configs
	readFunctionConfiguration();
	
	functionForceOn = eeprom_read_dword((uint32_t*)EE_FORCE_FUNC_ON);
	functionForceOff = eeprom_read_dword((uint32_t*)EE_FORCE_FUNC_OFF);

	// Thresholds
	hornThreshold = eeprom_read_byte((uint8_t*)EE_HORN_THRESHOLD);
	brakeThreshold = eeprom_read_byte((uint8_t*)EE_BRAKE_THRESHOLD);
	brakeLowThreshold = eeprom_read_byte((uint8_t*)EE_BRAKE_LOW_THRESHOLD);
	brakeHighThreshold = eeprom_read_byte((uint8_t*)EE_BRAKE_HIGH_THRESHOLD);
	
	// Options
	optionBits = eeprom_read_byte((uint8_t*)EE_OPTIONBITS);

	brakePulseWidth = eeprom_read_byte((uint8_t*)EE_BRAKE_PULSE_WIDTH);
	if(brakePulseWidth < BRAKE_PULSE_WIDTH_MIN)
	{
		brakePulseWidth = BRAKE_PULSE_WIDTH_MIN;
		eeprom_write_byte((uint8_t*)EE_BRAKE_PULSE_WIDTH, brakePulseWidth);
	}
	else if(brakePulseWidth > BRAKE_PULSE_WIDTH_MAX)
	{
		brakePulseWidth = BRAKE_PULSE_WIDTH_MAX;
		eeprom_write_byte((uint8_t*)EE_BRAKE_PULSE_WIDTH, brakePulseWidth);
	}

	// Notches
	eeprom_read_block((void *)notchSpeedStep, (void *)EE_NOTCH_SPEEDSTEP, 8);
	for(i=0; i<8; i++)
	{
		if(notchSpeedStep[i] > 126)
			notchSpeedStep[i] = 126;
		if(notchSpeedStep[i] < 1)
			notchSpeedStep[i] = 1;
	}
}

void copyConfig(uint8_t srcConfig, uint8_t destConfig)
{
	uint8_t configTemp[CONFIG_SIZE];
	eeprom_read_block((void *)configTemp, (void *)CONFIG_OFFSET(srcConfig), CONFIG_SIZE);
	eeprom_write_block((void *)configTemp, (void *)CONFIG_OFFSET(destConfig), CONFIG_SIZE);
}

void resetConfig(void)
{
	uint8_t i;

	wdt_reset();

	eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_H, UPDATE_DECISECS_DEFAULT >> 8);
	eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_L, UPDATE_DECISECS_DEFAULT & 0xFF);
	eeprom_write_byte((uint8_t*)EE_TX_HOLDOFF, TX_HOLDOFF_DEFAULT);

	eeprom_write_byte((uint8_t*)EE_DEVICE_SLEEP_TIMEOUT, SLEEP_TMR_RESET_VALUE_DEFAULT);
	eeprom_write_byte((uint8_t*)EE_ALERTER_TIMEOUT, ALERTER_TMR_RESET_VALUE_DEFAULT);
	eeprom_write_byte((uint8_t*)EE_DEAD_RECKONING_TIME, DEAD_RECKONING_TIME_DEFAULT);
	eeprom_write_byte((uint8_t*)EE_PRESSURE_CONFIG, PRESSURE_CONFIG_DEFAULT);
	eeprom_write_byte((uint8_t*)EE_CONFIGBITS, CONFIGBITS_DEFAULT);

	eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR, MRBUS_DEV_ADDR_DEFAULT);
	eeprom_write_byte((uint8_t*)EE_BASE_ADDR, MRBUS_BASE_ADDR_DEFAULT);
	eeprom_write_byte((uint8_t*)EE_TIME_SOURCE_ADDRESS, TIME_SOURCE_ADDRESS_DEFAULT);

	setBatteryLevels(0xFF, 0xFF, 0xFF);  // Set to unprogrammed value, will be reset to default by setBatteryLevels
	eeprom_write_byte((uint8_t*)EE_BATTERY_OKAY, getBatteryOkay());
	eeprom_write_byte((uint8_t*)EE_BATTERY_WARN, getBatteryWarn());
	eeprom_write_byte((uint8_t*)EE_BATTERY_CRITICAL, getBatteryCritical());

	// Skip the following, since these are specific to each physical device:
	//    EE_HORN_THRESHOLD
	//    EE_BRAKE_THRESHOLD
	//    EE_BRAKE_LOW_THRESHOLD
	//    EE_BRAKE_HIGH_THRESHOLD

	// Write working config first, then copy to all the others
	wdt_reset();
	eeprom_write_word((uint16_t*)EE_LOCO_ADDRESS, 0x0003 | LOCO_ADDRESS_SHORT);
	resetFunctionConfiguration();
	writeFunctionConfiguration();
	eeprom_write_dword((uint32_t*)EE_FORCE_FUNC_ON, 0);
	eeprom_write_dword((uint32_t*)EE_FORCE_FUNC_OFF, 0);
	eeprom_write_byte((uint8_t*)EE_BRAKE_PULSE_WIDTH, BRAKE_PULSE_WIDTH_DEFAULT);
	eeprom_write_byte((uint8_t*)EE_OPTIONBITS, OPTIONBITS_DEFAULT);
	notchSpeedStep[0] = 7;
	notchSpeedStep[1] = 23;
	notchSpeedStep[2] = 39;
	notchSpeedStep[3] = 55;
	notchSpeedStep[4] = 71;
	notchSpeedStep[5] = 87;
	notchSpeedStep[6] = 103;
	notchSpeedStep[7] = 119;
	eeprom_write_block((void *)notchSpeedStep, (void *)EE_NOTCH_SPEEDSTEP, 8);

	for (i=1; i<=MAX_CONFIGS; i++)
	{
		wdt_reset();
		lcd_gotoxy(3,1);
		printDec2Dig(MAX_CONFIGS-i+1);
		copyConfig(WORKING_CONFIG, i);
	}
	
	wdt_reset();
	
	// Read everything again
	readConfig();
}


void init(void)
{
	// Clear watchdog (in the case of an 'X' packet reset)
	MCUSR = 0;
#ifdef ENABLE_WATCHDOG
	// If you don't want the watchdog to do system reset, remove this chunk of code
	wdt_reset();
	wdt_enable(WATCHDOG_TIMEOUT);
	wdt_reset();
#else
	wdt_reset();
	wdt_disable();
#endif

	pktTimeout = 0;  // Assume no base unit until we hear from one
	lastRSSI = 0xFF;

	clearDeadReckoningTime();
	
	readConfig();
	systemBits = SYSTEMBITS_DEFAULT;

	initPorts();
	initADC();
	enableThrottle();
	initialize100HzTimer();

	engineStatesQueueInitialize();
	resetPressure();

	DDRB |= _BV(PB3);
}

int main(void)
{
	uint16_t decisecs_tmp;
	uint8_t i;

	uint8_t inputButtons = 0;
	uint8_t txBuffer[MRBUS_BUFFER_SIZE];

	uint8_t activeThrottleSetting = throttlePosition;
	uint8_t lastActiveThrottleSetting = activeThrottleSetting;

	ReverserPosition activeReverserSetting = reverserPosition;
	ReverserPosition lastActiveReverserSetting = activeReverserSetting;
	
	uint8_t lastThrottleStatus = throttleStatus;
	
	uint8_t brakePcnt = 0;

	uint8_t optionButtonState = 0;

	uint8_t backlight = 0;
	
	Screens screenState = LAST_SCREEN;  // Initialize to the last one, since that's the only state guaranteed to be present
	uint8_t subscreenState = 0;
	uint8_t subscreenCount = 0;

	BrakeStates brakeState = BRAKE_LOW_BEGIN;

	uint8_t decimalNumberIndex = 0;
	uint8_t decimalNumber[4];

	uint8_t functionNumber = 0;

	EngineState engineState = ENGINE_OFF;

	uint32_t functionMask = 0;
	uint32_t lastFunctionMask = 0;

	ReverserPosition direction = FORWARD;

	// Check if the EEPROM is initialized, check first config for all 0xFF
	for(i=0; i< 0x80; i++)
	{
		if(0xFF != eeprom_read_byte((uint8_t *)CONFIG_OFFSET(1) + i) )
			break;
	}
	if(0x80 == i)
	{
		screenState = DIAG_SCREEN;
		subscreenState = 12;
	}
	
	init();

	// Assign after init() so values are read from EEPROM first
	uint8_t newConfigNumber = 1;
	uint16_t newLocoAddress = locoAddress;
	uint8_t newDevAddr = mrbus_dev_addr;
	uint8_t newBaseAddr = mrbus_base_addr;
	uint8_t newTimeAddr = timeSourceAddress;
	uint8_t newSleepTimeout = sleep_tmr_reset_value / 600;
	uint8_t newAlerterTimeout = alerter_tmr_reset_value / 150;
	uint8_t newUpdate_seconds = update_decisecs / 10;

	uint8_t *prefsPtr = &newSleepTimeout;
	uint8_t *optionsPtr = &optionBits;

	setXbeeActive();

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		sleepTimeout_decisecs = sleep_tmr_reset_value;
		alerterTimeout_decisecs = alerter_tmr_reset_value;
	}
	
	enableLCD();

	wdt_reset();

	// Initialize MRBus core
	mrbusPktQueueInitialize(&mrbeeTxQueue, mrbusTxPktBufferArray, MRBUS_TX_BUFFER_DEPTH);
	mrbusPktQueueInitialize(&mrbeeRxQueue, mrbusRxPktBufferArray, MRBUS_RX_BUFFER_DEPTH);
	mrbeeInit();

	sei();	

	wdt_reset();

	// Queue up initial reset version packet (not sent until end of first main loop)
	createVersionPacket(0xFF, txBuffer);
	mrbusPktQueuePush(&mrbeeTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);

	wdt_reset();

	led = LED_OFF;

	enableButtons();
	enableSwitches();

	initLCD();

	// Initialize the buttons so there are no startup artifacts when we actually use them
	inputButtons = PINB & (0xF6);

	BatteryState lastBatteryState = getBatteryState();

	while(1)
	{
		wdt_reset();

		// Heartbeat (or loop timer)
		PORTB |= _BV(PB3);
		PORTB &= ~_BV(PB3);

		if (status & STATUS_READ_SWITCHES)
		{
			// Read switches every 10ms
			status &= ~STATUS_READ_SWITCHES;
			inputButtons = debounce(inputButtons, (PINB & (0xF6)));
			processButtons(inputButtons);
			processSwitches(inputButtons);
		}

		processADC();

		// Check for critical battery level
		if(CRITICAL == getBatteryState())
		{
			if(CRITICAL != lastBatteryState)
				lcd_clrscr();  // Clear if entering critical state
			led = LED_OFF;
			setXbeeSleep();
			disableLCDBacklight();
			disableSwitches();
			disableButtons();
			disableThrottle();
			lcd_gotoxy(2,0);
			lcd_puts("LOW");
			lcd_gotoxy(0,1);
			lcd_puts("BATTERY");
			lastBatteryState = getBatteryState();  // Do this here since we are going to skip the rest of the loop
			continue;
		}
		else if(lastBatteryState == CRITICAL)
		{
			lcd_clrscr();  // Clear if leaving critical state
			setXbeeActive();
			enableSwitches();
			enableButtons();
			enableThrottle();
			// Initialize the buttons so there are no startup artifacts when we actually use them
			inputButtons = debounce(inputButtons, (PINB & (0xF6)));
			inputButtons = debounce(inputButtons, (PINB & (0xF6)));
			inputButtons = debounce(inputButtons, (PINB & (0xF6)));
			inputButtons = debounce(inputButtons, (PINB & (0xF6)));
			processButtons(inputButtons);
			processSwitches(inputButtons);
			previousButton = button;  // Prevent extraneous menu advances
			clearDeadReckoningTime();
		}
		lastBatteryState = getBatteryState();

		// Convert horn to on/off control
		if(hornPosition <= (hornThreshold - HORN_HYSTERESIS))
		{
			controls &= ~(HORN_CONTROL);
		}
		else if(hornPosition >= hornThreshold)
		{
			controls |= HORN_CONTROL;
		}

		// Sanity check brake position and calculate percentage
		if(brakePosition < brakeLowThreshold)
			brakePcnt = 0;
		else
			brakePcnt = 100 * (brakePosition - brakeLowThreshold) / (brakeHighThreshold - brakeLowThreshold);

		// Handle emergency on brake control.  Do this outside the main brake state machine so the effect is immediate
		if(optionBits & _BV(OPTIONBITS_ESTOP_ON_BRAKE))
		{
			if(brakePosition < brakeLowThreshold)
				estopStatus &= ~ESTOP_BRAKE;
			if(brakePosition > brakeHighThreshold)
				estopStatus |= ESTOP_BRAKE;
		}
		else
		{
			estopStatus &= ~ESTOP_BRAKE;
		}
		
		// Handle brake
		if( (optionBits & _BV(OPTIONBITS_VARIABLE_BRAKE)) && (optionBits & _BV(OPTIONBITS_STEPPED_BRAKE)) )
		{
			// This state machine handles the variable (stepped) brake.
			switch(brakeState)
			{
				case BRAKE_LOW_BEGIN:
					controls |= BRAKE_OFF_CONTROL;  // Pulse the "brake off" control
					brakeState = BRAKE_LOW_WAIT;
					break;
				case BRAKE_LOW_WAIT:
					controls &= ~(BRAKE_OFF_CONTROL);
					if(brakePcnt >= 20)
						brakeState = BRAKE_20PCNT_BEGIN;
					break;

				case BRAKE_20PCNT_BEGIN:
					controls |= BRAKE_CONTROL;  // Pulse the "brake on" control
					brakeState = BRAKE_20PCNT_WAIT;
					break;
				case BRAKE_20PCNT_WAIT:
					controls &= ~(BRAKE_CONTROL);
					if(brakePosition < brakeLowThreshold)
						brakeState = BRAKE_LOW_BEGIN;
					else if(brakePcnt >= 40)
						brakeState = BRAKE_40PCNT_BEGIN;
					break;

				case BRAKE_40PCNT_BEGIN:
					controls |= BRAKE_CONTROL;  // Pulse the "brake on" control
					brakeState = BRAKE_40PCNT_WAIT;
					break;
				case BRAKE_40PCNT_WAIT:
					controls &= ~(BRAKE_CONTROL);
					if(brakePosition < brakeLowThreshold)
						brakeState = BRAKE_LOW_BEGIN;
					else if(brakePcnt >= 60)
						brakeState = BRAKE_60PCNT_BEGIN;
					break;

				case BRAKE_60PCNT_BEGIN:
					controls |= BRAKE_CONTROL;  // Pulse the "brake on" control
					brakeState = BRAKE_60PCNT_WAIT;
					break;
				case BRAKE_60PCNT_WAIT:
					controls &= ~(BRAKE_CONTROL);
					if(brakePosition < brakeLowThreshold)
						brakeState = BRAKE_LOW_BEGIN;
					else if(brakePcnt >= 80)
						brakeState = BRAKE_80PCNT_BEGIN;
					break;

				case BRAKE_80PCNT_BEGIN:
					controls |= BRAKE_CONTROL;  // Pulse the "brake on" control
					brakeState = BRAKE_80PCNT_WAIT;
					break;
				case BRAKE_80PCNT_WAIT:
					controls &= ~(BRAKE_CONTROL);
					if(brakePosition < brakeLowThreshold)
						brakeState = BRAKE_LOW_BEGIN;
					else if(brakePosition > brakeHighThreshold)
						brakeState = BRAKE_FULL_BEGIN;
					break;

				case BRAKE_FULL_BEGIN:
					controls |= BRAKE_CONTROL;  // Pulse the "brake on" control
					brakeState = BRAKE_FULL_WAIT;
					break;
				case BRAKE_FULL_WAIT:
					controls &= ~(BRAKE_CONTROL);
					if(brakePosition < brakeLowThreshold)
						brakeState = BRAKE_LOW_BEGIN;
					break;
			}
		}
		else if( (optionBits & _BV(OPTIONBITS_VARIABLE_BRAKE)) && !(optionBits & _BV(OPTIONBITS_STEPPED_BRAKE)) )
		{
			// This state machine handles the variable (pulse) brake.
			switch(brakeState)
			{
				// These two states get "brake off" set by first making sure "brake on" is clear (TCS decoders don't like these changing at the same time)
				case BRAKE_LOW_BEGIN:
					controls &= ~(BRAKE_CONTROL);
					brakeState = BRAKE_LOW_WAIT;
					break;
				case BRAKE_LOW_WAIT:
					controls |= BRAKE_OFF_CONTROL;
					brakeState = BRAKE_20PCNT_BEGIN;
					break;

				// These states represent the pulse "brake off" period
				case BRAKE_20PCNT_BEGIN:
				case BRAKE_20PCNT_WAIT:
				case BRAKE_40PCNT_BEGIN:
				case BRAKE_40PCNT_WAIT:
					if( brakePcnt >= (((brakeCounter / brakePulseWidth)+1)*20) )
						brakeState = BRAKE_FULL_BEGIN;
					break;

				// These states represent the pulse "brake on" period
				case BRAKE_60PCNT_BEGIN:
				case BRAKE_60PCNT_WAIT:
				case BRAKE_80PCNT_BEGIN:
				case BRAKE_80PCNT_WAIT:
					if( brakePcnt < (((brakeCounter / brakePulseWidth)+1)*20) )
						brakeState = BRAKE_LOW_BEGIN;
					break;

				// These two states get "brake on" set by first making sure "brake off" is clear (TCS decoders don't like these changing at the same time)
				case BRAKE_FULL_BEGIN:
					controls &= ~(BRAKE_OFF_CONTROL);
					brakeState = BRAKE_FULL_WAIT;
					break;
				case BRAKE_FULL_WAIT:
					controls |= BRAKE_CONTROL;
					brakeState = BRAKE_60PCNT_BEGIN;
					break;
			}
		}
		else
		{
			// This state machine handles the basic on/off brake.  The "brake off" control is set when the handle is fully left.  The
			// "brake on" control is set when the handle is above the defined brake threshold.  Transitions always go through a middle
			// state where both controls are cleared.  This is because TCS decoders don't like these functions changing at the same
			// time in the same packet - one of the transitions is ignored.  The middle state forces the active function off before
			// turning on the other function.
			switch(brakeState)
			{
				case BRAKE_LOW_BEGIN:
				case BRAKE_LOW_WAIT:
					// Set "brake off" when below the low threshold
					controls |= BRAKE_OFF_CONTROL;
					// Escape logic:
					//    Go to the middle state if above the brakeLowThreshold
					if(brakePosition >= brakeLowThreshold)
						brakeState = BRAKE_20PCNT_BEGIN;
					break;
				case BRAKE_20PCNT_BEGIN:
				case BRAKE_20PCNT_WAIT:
				case BRAKE_40PCNT_BEGIN:
				case BRAKE_40PCNT_WAIT:
				case BRAKE_60PCNT_BEGIN:
				case BRAKE_60PCNT_WAIT:
				case BRAKE_80PCNT_BEGIN:
				case BRAKE_80PCNT_WAIT:
					// Disable both "brake on" and "brake off" when between thresholds
					controls &= ~(BRAKE_CONTROL);
					controls &= ~(BRAKE_OFF_CONTROL);
					if(brakePosition < brakeLowThreshold)
						brakeState = BRAKE_LOW_BEGIN;
					else if(brakePosition >= brakeThreshold)
						brakeState = BRAKE_FULL_BEGIN;
					break;
				case BRAKE_FULL_BEGIN:
				case BRAKE_FULL_WAIT:
					// Set "brake on" when above the brake threshold
					controls |= BRAKE_CONTROL;
					// Escape logic:
					//    Limit (brakeThreshold - BRAKE_HYSTERESIS) to non-negative values.  Compare the brake setting to the higher of
					//    the limited (brakeThreshold - BRAKE_HYSTERESIS) or brakeLowThreshold.  If below, go to the middle state.
					if(brakePosition < max( ((brakeThreshold > BRAKE_HYSTERESIS)?(brakeThreshold - BRAKE_HYSTERESIS):0), brakeLowThreshold ) )
						brakeState = BRAKE_20PCNT_BEGIN;
					break;
			}
		}		


		// Swap reverser if configured to do so
		ReverserPosition reverserPosition_tmp = reverserPosition;
		if( (optionBits & _BV(OPTIONBITS_REVERSER_SWAP)) || (functionMask & getFunctionMask(REV_SWAP_FN)) )
		{
			switch(reverserPosition_tmp)
			{
				case FORWARD:
					reverserPosition_tmp = REVERSE;
					break;
				case REVERSE:
					reverserPosition_tmp = FORWARD;
					break;
				case NEUTRAL:
					reverserPosition_tmp = NEUTRAL;
					break;
			}
		}
		
		// Calculate active reverser setting
		if(activeReverserSetting != reverserPosition_tmp)
		{
			if( !(configBits & _BV(CONFIGBITS_REVERSER_LOCK)) || (0 == throttlePosition) )
			// Only allow reverser to change when reverser lock disabled, or when throttle is in idle
			activeReverserSetting = reverserPosition_tmp;
		}
		
		// Calculate active throttle setting
		if(NEUTRAL == activeReverserSetting)
		{
			if( (functionMask & getFunctionMask(THR_UNLOCK_FN)) )
			{
				// If a function is assigned to the throttle unlock (e.g. Drive Hold) and the function is active, allow the throttle to change
				activeThrottleSetting = throttlePosition;
			}
			else
			{
				// Otherwise force the throttle to idle to prevent movement
				activeThrottleSetting = 0;
			}
		}
		else
		{
			activeThrottleSetting = throttlePosition;
		}

		if((ENGINE_START == engineState) && !engineTimer)
		{
			// START --> RUNNING
			engineState = ENGINE_RUNNING;
		}
		else if((ENGINE_NOT_IDLE == engineState) && (0 == activeThrottleSetting))
		{
			// NOT_IDLE --> RUNNING
			engineState = ENGINE_RUNNING;
		}
		else if((ENGINE_STOP == engineState) && !engineTimer)
		{
			// STOP --> OFF
			engineState = ENGINE_OFF;
		}
		
		updateTime();

		switch(screenState)
		{
			case MAIN_SCREEN:
				if(!subscreenState)
				{
					lcd_gotoxy(2,0);
					if(throttleStatus & THROTTLE_STATUS_EMERGENCY)
					{
						lcd_puts("EMRG");
						enableLCDBacklight();
					}
					else if(throttleStatus & THROTTLE_STATUS_ALERTER)
					{
						lcd_puts("ALRT");
						ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
						{
							decisecs_tmp = alerterTimeout_decisecs;
						}
						if((decisecs_tmp/1) % 2)
							disableLCDBacklight();
						else
							enableLCDBacklight();
					}
					else if(activeReverserSetting != reverserPosition_tmp)
					{
						lcd_puts("REV!");
						enableLCDBacklight();
					}
					else
					{
						printLocomotiveAddress(locoAddress);
						if(backlight)
							enableLCDBacklight();
						else
							disableLCDBacklight();
					}

					lcd_gotoxy(1,1);
					printTime();
					printBattery();
				
					lcd_gotoxy(7,0);
					lcd_putc((optionButtonState & UP_OPTION_BUTTON) && !(isFunctionOff(UP_FN)) ? FUNCTION_ACTIVE_CHAR : FUNCTION_INACTIVE_CHAR);
					lcd_gotoxy(7,1);
					lcd_putc((optionButtonState & DOWN_OPTION_BUTTON) && !(isFunctionOff(DOWN_FN)) ? FUNCTION_ACTIVE_CHAR : FUNCTION_INACTIVE_CHAR);

					lcd_gotoxy(0,1);
					if(controls & AUX_CONTROL)
						lcd_putc(AUX_CHAR);
					else
						lcd_putc(' ');
					switch(button)
					{
						case UP_BUTTON:
							if(UP_BUTTON != previousButton)
							{
								if(isFunctionLatching(UP_FN))
									optionButtonState ^= UP_OPTION_BUTTON;  // Toggle
								else
									optionButtonState |= UP_OPTION_BUTTON;  // Momentary on
						}
							break;
						case DOWN_BUTTON:
							if(DOWN_BUTTON != previousButton)
							{
								if(isFunctionLatching(DOWN_FN))
									optionButtonState ^= DOWN_OPTION_BUTTON;  // Toggle
								else
									optionButtonState |= DOWN_OPTION_BUTTON;  // Momentary on
							}
							break;
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								if(backlight)
									backlight = 0;
								else
									backlight = 1;
								ticks_autoincrement = 0;  // Reset to zero so a long press can be detected
							}
							if(ticks_autoincrement >= button_autoincrement_10ms_ticks)
							{
								// Trigger power down menu on long press
								subscreenState = 1;
								lcd_clrscr();
							}
							// break;  // Roll through the other cases for cleanup
						case MENU_BUTTON:
						case NO_BUTTON:
							// Release buttons if momentary
							if(!(isFunctionLatching(UP_FN)))
								optionButtonState &= ~UP_OPTION_BUTTON;
							if(!(isFunctionLatching(DOWN_FN)))
								optionButtonState &= ~DOWN_OPTION_BUTTON;
							break;
					}
					if(optionButtonState & UP_OPTION_BUTTON)
					{
						if(isFunctionBrakeTest(UP_FN))
						{
							screenState = SPECFN_SCREEN;
							subscreenState = SPECFN_SUBSCREEN_PRESSURE;
							lcd_clrscr();
						}
					}
					if(optionButtonState & DOWN_OPTION_BUTTON)
					{
						if(isFunctionBrakeTest(DOWN_FN))
						{
							screenState = SPECFN_SCREEN;
							subscreenState = SPECFN_SUBSCREEN_PRESSURE;
							lcd_clrscr();
						}
					}
				}
				else
				{
					enableLCDBacklight();
					lcd_gotoxy(0,0);
					lcd_puts("POWER");
					lcd_gotoxy(0,1);
					lcd_puts("DOWN  -");
					lcd_putc(0x7E);
					switch(button)
					{
						case NO_BUTTON:
							if(DOWN_BUTTON == previousButton)  // Power off
							{
								// Force sleep.  Do this on the trailing edge so the release doesn't wake the throttle from sleep
								ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
								{
									throttleStatus |= THROTTLE_STATUS_SLEEP;
								}
								// Reset menu so things are clean when returning from sleep
								subscreenState = 0;
								screenState = LAST_SCREEN;
							}
							break;
						case MENU_BUTTON:
							// Escape power down menu and return to main screen
							screenState--;  // Back up one screen.  It will increment in the global MENU button handling code since we just pressed MENU.
							                // Yes, this may underflow, but there's a corresponding ++ in the MENU code.
							subscreenState = 0;
							backlight = 0;
							lcd_clrscr();
							break;
						case SELECT_BUTTON:
						case UP_BUTTON:
						case DOWN_BUTTON:
							break;
					}
				}
				break;

			case ENGINE_SCREEN:
				enableLCDBacklight();
				lcd_gotoxy(0,0);
				lcd_puts(" ENGINE");
				lcd_gotoxy(0,1);
				printEngineState(engineState);
				switch(button)
				{
					case UP_BUTTON:
						if(isFunctionOff(ENGINE_OFF_FN))
						{
							// Level based start/stop
							engineState = ENGINE_ON;
						}
						else
						{
							// Edge triggered start/stop
							// Only send start pulse if in the off state
							if((ENGINE_OFF == engineState) || (ENGINE_RUNNING == engineState))
							{
								engineState = ENGINE_START;
								engineTimer = ENGINE_TIMER_DECISECS;
							}
						}
						engineScreenTimer = 1;
						break;
					case DOWN_BUTTON:
						if(isFunctionOff(ENGINE_OFF_FN))
						{
							// Level based start/stop
							engineState = ENGINE_OFF;
						}
						else
						{
							// Edge triggered start/stop
							// Always allow stop to be sent so we can resync the state with the locomotive if they ever get out of sync
							// But only if the locomotive is in idle
							if(0 == activeThrottleSetting)
							{
								engineState = ENGINE_STOP;
								engineTimer = ENGINE_TIMER_DECISECS;
							}
							else
							{
								// Go to special state if not in idle
								// Main loop will set engineState back to RUNNING once throttle goes to idle
								engineState = ENGINE_NOT_IDLE;
							}
						}
						engineScreenTimer = 1;
						break;
					case SELECT_BUTTON:
						// Do cleanup below
					case MENU_BUTTON:
						// Do cleanup below
					case NO_BUTTON:
						break;
				}

				if((SELECT_BUTTON == button) || (MENU_BUTTON == button) || (engineScreenTimer > ENGINE_SCREEN_TIMER))
				{
					// Clean up
					if(ENGINE_NOT_IDLE == engineState)
					{
						// Clear Not Idle state if exiting the menu
						engineState = ENGINE_RUNNING;
					}
					engineScreenTimer = 0;
					
					if(MENU_BUTTON != button)
					{
						// Exit to main screen
						screenState = LAST_SCREEN;
					}
				}

				break;




			case SPECFN_SCREEN:
				enableLCDBacklight();
				if(!subscreenState)
				{
					lcd_gotoxy(1,0);
					lcd_puts("SPECIAL");
					lcd_gotoxy(0,1);
					lcd_putc(0x7F);
					lcd_puts("- FUNCS");
					switch(button)
					{
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								subscreenState = 1;
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
					enableLCDBacklight();
					lcd_gotoxy(0,0);
					if(SPECFN_SUBSCREEN_PRESSURE == subscreenState)
					{
						if(isPressureIdle())
						{
							lcd_puts("BRAKE");
							lcd_gotoxy(0,1);
							lcd_puts("TEST  -");
							lcd_putc(0x7E);
						}
						else
						{
							setupLCD(LCD_PRESSURE);
							enableLCDBacklight();
							processPressure(min(brakePcnt,100));
							printPressure();
							// Disable normal brake functions to they don't interfere with the brake test sounds as we move the brake lever
							controls &= ~(BRAKE_OFF_CONTROL);
							controls &= ~(BRAKE_CONTROL);
						}
						switch(button)
						{
							case DOWN_BUTTON:
								if(DOWN_BUTTON != previousButton)
									processPressure(min(brakePcnt,100));  // Start pumping
								break;
							case UP_BUTTON:
							case SELECT_BUTTON:
							case MENU_BUTTON:
							case NO_BUTTON:
								break;
						}
					}
/*
					else if(SPECFN_SUBSCREEN_TONNAGE == subscreenState)
					{
						setupLCD(LCD_TONNAGE);
						enableLCDBacklight();
						printTonnage();
						switch(button)
						{
							case UP_BUTTON:
								if(UP_BUTTON != previousButton)
								{
									incrementTonnage();
								}
								break;
							case DOWN_BUTTON:
								if(DOWN_BUTTON != previousButton)
								{
									decrementTonnage();
								}
								break;
							case SELECT_BUTTON:
							case MENU_BUTTON:
							case NO_BUTTON:
								break;
						}
					}
*/
					else
					{
						subscreenState = 1;
					}
					
					switch(button)
					{
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								// Escape menu system
								subscreenState = 0;
								screenState = LAST_SCREEN;
								setupLCD(LCD_DEFAULT);
								resetPressure();
							}
							break;
						case MENU_BUTTON:
							if(MENU_BUTTON != previousButton)
							{
								// Menu pressed, advance menu
								subscreenState = (subscreenState & 0x7F) + 1;
								lcd_clrscr();
								resetPressure();
							}
							break;
						case UP_BUTTON:
						case DOWN_BUTTON:
						case NO_BUTTON:
							break;
					}
				}
				break;


			case LOAD_CONFIG_SCREEN:
			case SAVE_CONFIG_SCREEN:
				if(!subscreenState)
				{
					enableLCDBacklight();
					lcd_gotoxy(0,0);
					if(LOAD_CONFIG_SCREEN == screenState)
						lcd_puts("LOAD");
					else
						lcd_puts("SAVE");
					lcd_puts(" CNF");
					lcd_gotoxy(0,1);
					printDec2DigWZero(newConfigNumber);
					lcd_puts(": ");
					{
						uint16_t eepromAddressDelta = CONFIG_OFFSET(WORKING_CONFIG) - CONFIG_OFFSET(newConfigNumber);
						uint16_t tmpLocoAddress = eeprom_read_word((uint16_t*)(EE_LOCO_ADDRESS - eepromAddressDelta));  // Read loco address of newConfigNumber
						if(tmpLocoAddress & LOCO_ADDRESS_SHORT)
						{
							if((tmpLocoAddress & ~(LOCO_ADDRESS_SHORT)) > 127)
							{
								// Invalid Short Address, reset to a sane value
								tmpLocoAddress = 127 | LOCO_ADDRESS_SHORT;
							}
						}
						else
						{
							if(tmpLocoAddress > 9999)
							{
								// Invalid Long Address, reset to a sane value
								tmpLocoAddress = 9999;
							}
						}
						printLocomotiveAddress(tmpLocoAddress);
					}
					switch(button)
					{
						case UP_BUTTON:
							if((UP_BUTTON != previousButton) && (newConfigNumber < MAX_CONFIGS))
							{
								newConfigNumber++;
							}
							break;
						case DOWN_BUTTON:
							if((DOWN_BUTTON != previousButton) && (newConfigNumber > 1))
							{
								newConfigNumber--;
							}
							break;
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								subscreenState = 1;
								lcd_clrscr();
							}
							break;
						case MENU_BUTTON:
						case NO_BUTTON:
							break;
					}
				}
				else
				{
					enableLCDBacklight();
					lcd_gotoxy(0,0);
					lcd_puts("CONFIRM");
					lcd_gotoxy(0,1);
					if(LOAD_CONFIG_SCREEN == screenState)
						lcd_puts("LOAD? -");
					else
						lcd_puts("SAVE? -");
					lcd_putc(0x7E);
					switch(button)
					{
						case DOWN_BUTTON:
							if(DOWN_BUTTON != previousButton)
							{
								lcd_clrscr();
								lcd_gotoxy(0,0);

								// Copy selected config into working config
								if(LOAD_CONFIG_SCREEN == screenState)
								{
									lcd_puts("LOADING");
									EngineState tmpEngineState = engineState;
									// Get new engine state before potentially bumping it off the queue when we save the old one
									uint16_t eepromAddressDelta = CONFIG_OFFSET(WORKING_CONFIG) - CONFIG_OFFSET(newConfigNumber);
									uint16_t tmpLocoAddress = eeprom_read_word((uint16_t*)(EE_LOCO_ADDRESS - eepromAddressDelta));  // Read loco address of newConfigNumber
									engineState = engineStatesQueueGetState(tmpLocoAddress);
									if(ENGINE_NOT_INITIALIZED == engineState)
										engineState = tmpEngineState;  // Restore old state if new locomotive not found
									engineStatesQueueUpdate(locoAddress, tmpEngineState);  // Save current engine state
									copyConfig(newConfigNumber, WORKING_CONFIG);
								}
								else
								{
									lcd_puts("SAVING");
									copyConfig(WORKING_CONFIG, newConfigNumber);
								}

								// Refresh.  Needed for load, not for save
								readConfig();

								lcd_gotoxy(0,1);
								for(i=0; i<8; i++)
								{
									// Do something to make it look active
									wait100ms(1);
									lcd_putc('.');
								}
								wait100ms(3);
								screenState = LAST_SCREEN;
								subscreenState = 0;  // Escape submenu
								lcd_clrscr();
							}
							break;
						case MENU_BUTTON:
							screenState--;  // Back up one screen.  It will increment in the global MENU button handling code since we just pressed MENU.
							subscreenState = 0;  // Escape submenu
							lcd_clrscr();
							break;
						case SELECT_BUTTON:
						case UP_BUTTON:
						case NO_BUTTON:
							break;
					}
				}
				break;

			case LOCO_SCREEN:
				// A little explanation...  We store the information about a short address in the first digit (decimalNumber[0])
				//    If 0-9, then it's a long address.  If >9, and more specifically, ('s'-'0'), the it's a short address
				enableLCDBacklight();
				if(!subscreenState)
				{
					lcd_gotoxy(5,0);
					lcd_puts("SET");
					lcd_gotoxy(0,1);
					lcd_putc(0x7F);
					lcd_puts("-  LOCO");
					switch(button)
					{
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								newLocoAddress = locoAddress;
								decimalNumberIndex = 0;
								if(newLocoAddress & LOCO_ADDRESS_SHORT)
								{
									decimalNumber[0] = 's' - '0';
									decimalNumber[1] = ((newLocoAddress & ~(LOCO_ADDRESS_SHORT)) / 100) % 10;
									decimalNumber[2] = ((newLocoAddress & ~(LOCO_ADDRESS_SHORT)) / 10) % 10;
									decimalNumber[3] = (newLocoAddress & ~(LOCO_ADDRESS_SHORT)) % 10;
								}
								else
								{
									decimalNumber[0] = (newLocoAddress / 1000) % 10;
									decimalNumber[1] = (newLocoAddress / 100) % 10;
									decimalNumber[2] = (newLocoAddress / 10) % 10;
									decimalNumber[3] = (newLocoAddress) % 10;
								}
								subscreenState = 1;
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
							if((UP_BUTTON != previousButton) || (ticks_autoincrement >= button_autoincrement_10ms_ticks))
							{
								if(0 == decimalNumberIndex)
								{
									// First digit
									if(decimalNumber[decimalNumberIndex] > 9)
										decimalNumber[decimalNumberIndex] = 0;       // short to long
									else if(9 == decimalNumber[decimalNumberIndex])
									{
										// Check if valid short address
										if( ((decimalNumber[1] * 100) + (decimalNumber[2] * 10) + decimalNumber[3]) < 128)
										{
											decimalNumber[decimalNumberIndex] = 's' - '0';   // Change to short
										}
										else
										{
											decimalNumber[decimalNumberIndex] = 0;
										}
									}
									else
										decimalNumber[decimalNumberIndex]++;
								}
								else if(decimalNumber[decimalNumberIndex] < 9)
									decimalNumber[decimalNumberIndex]++;
								else
									decimalNumber[decimalNumberIndex] = 0;
								
								ticks_autoincrement = 0;
							}
							break;
						case DOWN_BUTTON:
							if((DOWN_BUTTON != previousButton) || (ticks_autoincrement >= button_autoincrement_10ms_ticks))
							{
								if(0 == decimalNumberIndex)
								{
									// First digit
									if(decimalNumber[decimalNumberIndex] > 9)
										decimalNumber[decimalNumberIndex] = 9;       // short to long
									else if(0 == decimalNumber[decimalNumberIndex])
									{
										// Check if valid short address
										if( ((decimalNumber[1] * 100) + (decimalNumber[2] * 10) + decimalNumber[3]) < 128)
										{
											decimalNumber[decimalNumberIndex] = 's' - '0';   // Change to short
										}
										else
										{
											decimalNumber[decimalNumberIndex] = 9;
										}
									}
									else
										decimalNumber[decimalNumberIndex]--;
								}
								else if(decimalNumber[decimalNumberIndex] > 0)
									decimalNumber[decimalNumberIndex]--;
								else
									decimalNumber[decimalNumberIndex] = 9;

								ticks_autoincrement = 0;
							}
							break;
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								if(decimalNumber[0] > 9)
									newLocoAddress = ((decimalNumber[1] * 100) + (decimalNumber[2] * 10) + decimalNumber[3]) | LOCO_ADDRESS_SHORT;
								else
									newLocoAddress = (decimalNumber[0] * 1000) + (decimalNumber[1] * 100) + (decimalNumber[2] * 10) + decimalNumber[3];

								EngineState tmpEngineState = engineState;
								// Get new engine state before potentially bumping it off the queue when we save the old one
								engineState = engineStatesQueueGetState(newLocoAddress);
								if(ENGINE_NOT_INITIALIZED == engineState)
									engineState = tmpEngineState;  // Restore old state if new locomotive not found
								engineStatesQueueUpdate(locoAddress, tmpEngineState);  // Save current engine state

								eeprom_write_word((uint16_t*)EE_LOCO_ADDRESS, newLocoAddress);
								readConfig();
								newLocoAddress = locoAddress;
								lcd_clrscr();
								lcd_gotoxy(1,0);
								lcd_puts("SAVED!");
								wait100ms(7);
								subscreenState = 0;  // Escape submenu
								lcd_clrscr();
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

			case FORCE_FUNC_SCREEN:
				enableLCDBacklight();
				if(!subscreenState)
				{
					lcd_gotoxy(3,0);
					lcd_puts("FORCE");
					lcd_gotoxy(0,1);
					lcd_putc(0x7F);
					lcd_puts("-  FUNC");
					switch(button)
					{
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								subscreenState = 1;
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
							if(UP_BUTTON != previousButton) 
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
							}
							break;
						case DOWN_BUTTON:
							if(DOWN_BUTTON != previousButton)
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
							}
							break;
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								eeprom_write_dword((uint32_t*)EE_FORCE_FUNC_ON, functionForceOn);
								eeprom_write_dword((uint32_t*)EE_FORCE_FUNC_OFF, functionForceOff);
								readConfig();
								lcd_clrscr();
								lcd_gotoxy(1,0);
								lcd_puts("SAVED!");
								wait100ms(7);
								subscreenState = 0;  // Escape submenu
								lcd_clrscr();
							}
							break;
						case MENU_BUTTON:
							if(MENU_BUTTON != previousButton)
							{
								// Advance through function settings
								if(++functionNumber > 28)
									functionNumber = 0;
								lcd_clrscr();
							}
							break;
						case NO_BUTTON:
							break;
					}
				}
				break;

			case CONFIG_FUNC_SCREEN:
				enableLCDBacklight();
				if(!subscreenState)
				{
					lcd_gotoxy(2,0);
					lcd_puts("CONFIG");
					lcd_gotoxy(0,1);
					lcd_putc(0x7F);
					lcd_puts("-  FUNC");
					switch(button)
					{
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								subscreenState = 1;
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

					lcd_gotoxy(0,0);
					printCurrentFunctionName();
					lcd_gotoxy(0,1);
					printCurrentFunctionValue();
					
					switch(button)
					{
						//  |off|latch|0|Func[4:0]|
						case UP_BUTTON:
							if((UP_BUTTON != previousButton) || (ticks_autoincrement >= button_autoincrement_10ms_ticks))
							{
								incrementCurrentFunctionValue();
								ticks_autoincrement = 0;
							}
							break;
						case DOWN_BUTTON:
							if((DOWN_BUTTON != previousButton) || (ticks_autoincrement >= button_autoincrement_10ms_ticks))
							{
								decrementCurrentFunctionValue();
								ticks_autoincrement = 0;
							}
							break;
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								writeFunctionConfiguration();
								readConfig();
								lcd_clrscr();
								lcd_gotoxy(1,0);
								lcd_puts("SAVED!");
								wait100ms(7);
								resetCurrentFunction();
								subscreenState = 0;  // Escape submenu
								lcd_clrscr();
							}
							break;
						case MENU_BUTTON:
							if(MENU_BUTTON != previousButton)
							{
								advanceCurrentFunction();
								lcd_clrscr();
							}
							break;
						case NO_BUTTON:
							break;
					}
				}
				break;

			case NOTCH_CONFIG_SCREEN:
				enableLCDBacklight();
				if(!subscreenState)
				{
					lcd_gotoxy(3,0);
					lcd_puts("NOTCH");
					lcd_gotoxy(0,1);
					lcd_putc(0x7F);
					lcd_puts("-   CFG");
					switch(button)
					{
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								subscreenState = 1;
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
					enableLCDBacklight();
					lcd_gotoxy(0,0);
					lcd_puts("NOTCH ");
					uint8_t notch = subscreenState;
					lcd_putc('0' + notch);
					lcd_gotoxy(0,1);
					printDec4Dig(notchSpeedStep[notch-1]);
					switch(button)
					{
						case UP_BUTTON:
							if((UP_BUTTON != previousButton) || (ticks_autoincrement >= button_autoincrement_10ms_ticks))
							{
								if(notchSpeedStep[notch-1] < 126)
									notchSpeedStep[notch-1]++;
								else
									notchSpeedStep[notch-1] = 126;
								ticks_autoincrement = 0;
							}
							break;
						case DOWN_BUTTON:
							if((DOWN_BUTTON != previousButton) || (ticks_autoincrement >= button_autoincrement_10ms_ticks))
							{
								if(notchSpeedStep[notch-1] > 1)
									notchSpeedStep[notch-1]--;
								else
									notchSpeedStep[notch-1] = 1;
								ticks_autoincrement = 0;
							}
							break;
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								eeprom_write_block((void *)notchSpeedStep, (void *)EE_NOTCH_SPEEDSTEP, 8);
								readConfig();
								lcd_clrscr();
								lcd_gotoxy(1,0);
								lcd_puts("SAVED!");
								wait100ms(7);
								subscreenState = 0;  // Escape submenu
								lcd_clrscr();
							}
							break;
						case MENU_BUTTON:
							if(MENU_BUTTON != previousButton)
							{
								// Menu pressed, advance menu
								subscreenState++;
								if(subscreenState > 8)
									subscreenState = 1;
								lcd_clrscr();
							}
							break;
						case NO_BUTTON:
							break;
					}
				}
				break;

			case OPTION_SCREEN:
				enableLCDBacklight();
				if(!subscreenState)
				{
					lcd_gotoxy(1,0);
					lcd_puts("OPTIONS");
					lcd_gotoxy(0,1);
					lcd_putc(0x7F);
					lcd_puts("-");
					switch(button)
					{
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								subscreenState = 1;
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
					uint8_t bitPosition = 0xFF;  // <8 means boolean
					enableLCDBacklight();
					lcd_gotoxy(0,0);
					if(1 == subscreenState)
					{
						lcd_puts("VAR BRK");
						bitPosition = OPTIONBITS_VARIABLE_BRAKE;
						optionsPtr = &optionBits;
					}
					else if(2 == subscreenState)
					{
						lcd_puts("BRK TYPE");
						bitPosition = OPTIONBITS_STEPPED_BRAKE;
						optionsPtr = &optionBits;
					}
					else if(3 == subscreenState)
					{
						lcd_puts("BRK RATE");
						lcd_gotoxy(7,1);
						lcd_puts("s");
						bitPosition = 0xFF;
						optionsPtr = &brakePulseWidth;
					}
					else if(4 == subscreenState)
					{
						lcd_puts("BRK ESTP");
						bitPosition = OPTIONBITS_ESTOP_ON_BRAKE;
						optionsPtr = &optionBits;
					}
					else if(5 == subscreenState)
					{
						lcd_puts("REV SWAP");
						bitPosition = OPTIONBITS_REVERSER_SWAP;
						optionsPtr = &optionBits;
					}
					else
					{
						bitPosition = 8;
						subscreenState = 1;
					}

					if(bitPosition < 8)
					{
						if(OPTIONBITS_STEPPED_BRAKE == bitPosition)
						{
							// Special case for brake type
							lcd_gotoxy(3,1);
							if(*optionsPtr & _BV(bitPosition))
								lcd_puts(" STEP");
							else
								lcd_puts("PULSE");
						}
						else
						{
							lcd_gotoxy(5,1);
							if(*optionsPtr & _BV(bitPosition))
								lcd_puts("ON ");
							else
								lcd_puts("OFF");
						}
					}
					else if(8 == bitPosition)
					{
						// Do nothing
					}
					else if(optionsPtr == &brakePulseWidth)
					{
						lcd_gotoxy(4,1);
						lcd_putc('0' + (*optionsPtr) / 10);
						lcd_putc('.');
						lcd_putc('0' + (*optionsPtr) % 10);
					}
					else
					{
						lcd_gotoxy(4,1);
						printDec3Dig(*optionsPtr);
					}

					
					switch(button)
					{
						case UP_BUTTON:
							if((UP_BUTTON != previousButton) || (ticks_autoincrement >= button_autoincrement_10ms_ticks))
							{
								if(bitPosition < 8)
								{
									*optionsPtr |= _BV(bitPosition);
								}
								else
								{
									if(*optionsPtr < 0xFF)
										(*optionsPtr)++;
									if(brakePulseWidth > BRAKE_PULSE_WIDTH_MAX)
										brakePulseWidth = BRAKE_PULSE_WIDTH_MAX;
									ticks_autoincrement = 0;
								}
							}
							break;
						case DOWN_BUTTON:
							if((DOWN_BUTTON != previousButton) || (ticks_autoincrement >= button_autoincrement_10ms_ticks))
							{
								if(bitPosition < 8)
								{
									*optionsPtr &= ~_BV(bitPosition);
								}
								else
								{
									if(*optionsPtr > 1)
										(*optionsPtr)--;
									if(brakePulseWidth < BRAKE_PULSE_WIDTH_MIN)
										brakePulseWidth = BRAKE_PULSE_WIDTH_MIN;
									ticks_autoincrement = 0;
								}
							}
							break;
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								eeprom_write_byte((uint8_t*)EE_BRAKE_PULSE_WIDTH, brakePulseWidth);
								eeprom_write_byte((uint8_t*)EE_OPTIONBITS, optionBits);
								readConfig();
								lcd_clrscr();
								lcd_gotoxy(1,0);
								lcd_puts("SAVED!");
								wait100ms(7);
								subscreenState = 0;  // Escape submenu
								lcd_clrscr();
							}
							break;
						case MENU_BUTTON:
							if(MENU_BUTTON != previousButton)
							{
								// Menu pressed, advance menu
								subscreenState++;

								// Conditionally skip menus if they don't apply
								while(	((2 == subscreenState) && !(optionBits & _BV(OPTIONBITS_VARIABLE_BRAKE))) ||  // Skip brake type when variable brake disabled
										((3 == subscreenState) && !(optionBits & _BV(OPTIONBITS_VARIABLE_BRAKE))) ||  // Skip pulse width when variable brake disabled
										((3 == subscreenState) &&  (optionBits & _BV(OPTIONBITS_STEPPED_BRAKE)))      // Skip pulse width when brake type = stepped
										)
								{
									subscreenState++;
								}

								lcd_clrscr();
							}
							break;
						case NO_BUTTON:
							break;
					}
				}
				break;

			case THRESHOLD_CAL_SCREEN:
				enableLCDBacklight();
				if(!subscreenState)
				{
					lcd_gotoxy(0,0);
					lcd_puts("THRSHOLD");
					lcd_gotoxy(0,1);
					lcd_putc(0x7F);
					lcd_puts("-   CAL");
					switch(button)
					{
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								subscreenState = 1;
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
					uint8_t *positionPtr = &hornPosition;
					uint8_t *thresholdPtr = &hornThreshold;
					enableLCDBacklight();
					lcd_gotoxy(0,0);
					if(1 == subscreenState)
					{
						lcd_puts("HORN");
						positionPtr = &hornPosition;
						thresholdPtr = &hornThreshold;
					}
					else if(2 == subscreenState)
					{
						lcd_puts("BRAKE");
						positionPtr = &brakePosition;
						thresholdPtr = &brakeThreshold;
					}
					else if(3 == subscreenState)
					{
						lcd_puts("BRAKE");
						lcd_gotoxy(0,1);
						lcd_puts("LOW");
						positionPtr = &brakePosition;
						thresholdPtr = &brakeLowThreshold;
					}
					else if(4 == subscreenState)
					{
						lcd_puts("BRAKE");
						lcd_gotoxy(0,1);
						lcd_puts("HIGH");
						positionPtr = &brakePosition;
						thresholdPtr = &brakeHighThreshold;
					}
					else
					{
						subscreenState = 1;
					}
					lcd_gotoxy(7,0);
					if(0xFF == *thresholdPtr)
						lcd_putc('-');
					else
						lcd_putc((*positionPtr >= *thresholdPtr) ? FUNCTION_ACTIVE_CHAR : FUNCTION_INACTIVE_CHAR);
					lcd_gotoxy(5,1);
					printDec3DigWZero(*positionPtr);
					switch(button)
					{
						case UP_BUTTON:
							if(UP_BUTTON != previousButton)
							{
								// Update threshold to current position
								if(thresholdPtr == &brakeLowThreshold)
								{
									*thresholdPtr = *positionPtr + BRAKE_DEAD_ZONE;
								}
								else if(thresholdPtr == &brakeHighThreshold)
								{
									*thresholdPtr = *positionPtr - BRAKE_DEAD_ZONE;
								}
								else
								{
									*thresholdPtr = *positionPtr;
								}
							}
							break;
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								eeprom_write_byte((uint8_t*)EE_HORN_THRESHOLD, hornThreshold);
								eeprom_write_byte((uint8_t*)EE_BRAKE_THRESHOLD, brakeThreshold);
								eeprom_write_byte((uint8_t*)EE_BRAKE_LOW_THRESHOLD, brakeLowThreshold);
								eeprom_write_byte((uint8_t*)EE_BRAKE_HIGH_THRESHOLD, brakeHighThreshold);
								readConfig();
								lcd_clrscr();
								lcd_gotoxy(1,0);
								lcd_puts("SAVED!");
								wait100ms(7);
								subscreenState = 0;  // Escape submenu
								lcd_clrscr();
							}
							break;
						case MENU_BUTTON:
							if(MENU_BUTTON != previousButton)
							{
								// Menu pressed, advance menu
								subscreenState++;
								lcd_clrscr();
							}
							break;
						case DOWN_BUTTON:
							// DOWN does nothing
						case NO_BUTTON:
							break;
					}
				}
				break;

			case COMM_SCREEN:
				enableLCDBacklight();
				if(!subscreenState)
				{
					lcd_gotoxy(4,0);
					lcd_puts("COMM");
					lcd_gotoxy(0,1);
					lcd_putc(0x7F);
					lcd_puts("-   CFG");
					switch(button)
					{
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								subscreenState = 1;
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
					enableLCDBacklight();
					lcd_gotoxy(0,0);
					if(1 == subscreenState)
					{
						lcd_puts("THRTL ID");
						prefsPtr = &newDevAddr;
						lcd_gotoxy(4,1);
						lcd_putc('A' + (newDevAddr - MRBUS_DEV_ADDR_MIN));
					}
					else if(2 == subscreenState)
					{
						lcd_puts("BASE ADR");
						prefsPtr = &newBaseAddr;
						lcd_gotoxy(3,1);
						printDec2DigWZero(newBaseAddr - MRBUS_BASE_ADDR_MIN);
					}
					else if(3 == subscreenState)
					{
						lcd_puts("TIME ADR");
						prefsPtr = &newTimeAddr;
						if(0x00 == newTimeAddr)
						{
							lcd_gotoxy(2,1);
							lcd_puts("BASE");
						}
						else if(0xFF == newTimeAddr)
						{
							lcd_gotoxy(2,1);
							lcd_puts(" ALL");
						}
						else
						{
							lcd_gotoxy(2,1);
							lcd_puts("0x");
							printHex(newTimeAddr);
						}
					}
					else if(4 == subscreenState)
					{
						lcd_puts("TX INTVL");
						prefsPtr = &newUpdate_seconds;
						lcd_gotoxy(4,1);
						printDec3Dig(*prefsPtr);
						lcd_gotoxy(7,1);
						lcd_puts("s");
					}
					else if(5 == subscreenState)
					{
						lcd_puts("TX HLDOF");
						prefsPtr = &txHoldoff_centisecs;
						lcd_gotoxy(3,1);
						lcd_putc('0' + (*prefsPtr) / 100);
						lcd_putc('.');
						lcd_putc('0' + ((*prefsPtr)/10) % 10);
						lcd_putc('0' + (*prefsPtr) % 10);
						lcd_gotoxy(7,1);
						lcd_puts("s");
					}
					else
					{
						subscreenState = 1;
					}

					switch(button)
					{
						case UP_BUTTON:
							if((UP_BUTTON != previousButton) || (ticks_autoincrement >= button_autoincrement_10ms_ticks))
							{
								if( ((prefsPtr != &newUpdate_seconds)&&(prefsPtr != &txHoldoff_centisecs)) || (systemBits & _BV(SYSTEMBITS_ADV_FUNC)) )
								{
									if(*prefsPtr < 0xFF)
										(*prefsPtr)++;
									// Check bounds
									if(newDevAddr > MRBUS_DEV_ADDR_MAX)
										newDevAddr = MRBUS_DEV_ADDR_MAX;
									if(newBaseAddr > MRBUS_BASE_ADDR_MAX)
										newBaseAddr = MRBUS_BASE_ADDR_MAX;
									if(newUpdate_seconds > UPDATE_DECISECS_MAX / 10)
										newUpdate_seconds = UPDATE_DECISECS_MAX / 10;
								}
								ticks_autoincrement = 0;
							}
							break;
						case DOWN_BUTTON:
							if((DOWN_BUTTON != previousButton) || (ticks_autoincrement >= button_autoincrement_10ms_ticks))
							{
								if((prefsPtr == &newUpdate_seconds)||(prefsPtr == &txHoldoff_centisecs))
								{
									if( (systemBits & _BV(SYSTEMBITS_ADV_FUNC)) && (*prefsPtr > 1) )
										(*prefsPtr)--;
									// Check bounds
									if(txHoldoff_centisecs < TX_HOLDOFF_MIN)
										txHoldoff_centisecs = TX_HOLDOFF_MIN;
								}
								else
								{
									if(*prefsPtr > 0)
										(*prefsPtr)--;
									// Check bounds
									if(newDevAddr < MRBUS_DEV_ADDR_MIN)
										newDevAddr = MRBUS_DEV_ADDR_MIN;
									if(newBaseAddr < MRBUS_BASE_ADDR_MIN)
										newBaseAddr = MRBUS_BASE_ADDR_MIN;
								}
								ticks_autoincrement = 0;
							}
							break;
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR, newDevAddr);
								eeprom_write_byte((uint8_t*)EE_BASE_ADDR, newBaseAddr);
								eeprom_write_byte((uint8_t*)EE_TIME_SOURCE_ADDRESS, newTimeAddr);
								eeprom_write_byte((uint8_t*)EE_TX_HOLDOFF, txHoldoff_centisecs);
								update_decisecs = (uint16_t)newUpdate_seconds * 10;
								eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_H, update_decisecs >> 8);
								eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_L, update_decisecs & 0xFF);
								readConfig();
								newDevAddr = mrbus_dev_addr;
								newBaseAddr = mrbus_base_addr;
								newTimeAddr = timeSourceAddress;
								newUpdate_seconds = update_decisecs / 10;
								lcd_clrscr();
								lcd_gotoxy(1,0);
								lcd_puts("SAVED!");
								wait100ms(7);
								subscreenState = 0;  // Escape submenu
								lcd_clrscr();
							}
							break;
						case MENU_BUTTON:
							if(MENU_BUTTON != previousButton)
							{
								// Menu pressed, advance menu
								subscreenState++;
								lcd_clrscr();
							}
							break;
						case NO_BUTTON:
							break;
					}
				}
				break;

			case PREFS_SCREEN:
				enableLCDBacklight();
				if(!subscreenState)
				{
					lcd_gotoxy(3,0);
					lcd_puts("PREFS");
					lcd_gotoxy(0,1);
					lcd_putc(0x7F);
					lcd_puts("-");
					switch(button)
					{
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								subscreenState = 1;
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
					uint8_t bitPosition = 0xFF;  // <8 means boolean
					enableLCDBacklight();
					lcd_gotoxy(0,0);

					// FIXME: These variables serve no real purpose other than indicating which menu is active
					//         A better solution would be to name the subscreens like in Special Functions
					uint8_t maxDeadReckoningTime = getMaxDeadReckoningTime();
					uint8_t pressureCoefficients = getPressureConfig();
					
					if(1 == subscreenState)
					{
						lcd_puts("SLEEP");
						lcd_gotoxy(0,1);
						lcd_puts("DLY:");
						lcd_gotoxy(4,1);
						printDec3Dig(*prefsPtr);
						lcd_puts("M");
						bitPosition = 0xFF;
						prefsPtr = &newSleepTimeout;
					}
					else if(2 == subscreenState)
					{
						lcd_puts("ALERTER");
						lcd_gotoxy(0,1);
						lcd_puts("DLY:");
						lcd_gotoxy(4,1);
						if(newAlerterTimeout)
						{
							printDec3Dig(newAlerterTimeout * 15);
							lcd_puts("s");
						}
						else
						{
							lcd_puts(" OFF");
						}
						bitPosition = 0xFF;
						prefsPtr = &newAlerterTimeout;
					}
					else if(3 == subscreenState)
					{
						lcd_puts("TIMEOUT");
						lcd_gotoxy(0,1);
						lcd_puts("CLK:");
						lcd_gotoxy(4,1);
						printDec3Dig(convertMaxDeadReckoningToDecisecs() / 10);
						lcd_puts("s");
						bitPosition = 0xFF;
						prefsPtr = &maxDeadReckoningTime;
					}
					else if(4 == subscreenState)
					{
						lcd_puts("PUMP");
						lcd_gotoxy(0,1);
						lcd_puts("RATE:");
						lcd_gotoxy(7,1);
						lcd_putc('1' + getPumpRate());
						bitPosition = 0xFF;
						prefsPtr = &pressureCoefficients;
					}
					else if(5 == subscreenState)
					{
						lcd_puts("LED BLNK");
						bitPosition = CONFIGBITS_LED_BLINK;
						prefsPtr = &configBits;
					}
					else if(6 == subscreenState)
					{
						lcd_puts("REV LOCK");
						bitPosition = CONFIGBITS_REVERSER_LOCK;
						prefsPtr = &configBits;
					}
					else if(7 == subscreenState)
					{
						lcd_puts("STRICT");
						lcd_gotoxy(0,1);
						lcd_puts("SLP");
						bitPosition = CONFIGBITS_STRICT_SLEEP;
						prefsPtr = &configBits;
					}
					else
					{
						bitPosition = 8;
						subscreenState = 1;
					}

					if(bitPosition < 8)
					{
						lcd_gotoxy(4,1);
						if(*prefsPtr & _BV(bitPosition))
							lcd_puts(" ON ");
						else
							lcd_puts(" OFF");
					}
					else if(8 == bitPosition)
					{
						// Do nothing
					}
					

					switch(button)
					{
						case UP_BUTTON:
							if((UP_BUTTON != previousButton) || (ticks_autoincrement >= button_autoincrement_10ms_ticks))
							{
								if(bitPosition < 8)
								{
									*prefsPtr |= _BV(bitPosition);
								}
								else
								{
									if(prefsPtr == &maxDeadReckoningTime)
									{
										incrementMaxDeadReckoningTime();
									}
									if(prefsPtr == &pressureCoefficients)
									{
										incrementPumpRate();
									}
									else
									{
										if(*prefsPtr < 0xFF)
											(*prefsPtr)++;
										if(newSleepTimeout > SLEEP_TMR_RESET_VALUE_MAX)
											newSleepTimeout = SLEEP_TMR_RESET_VALUE_MAX;
										if(newAlerterTimeout > ALERTER_TMR_RESET_VALUE_MAX)
											newAlerterTimeout = ALERTER_TMR_RESET_VALUE_MAX;
									}
									ticks_autoincrement = 0;
								}
							}
							break;
						case DOWN_BUTTON:
							if((DOWN_BUTTON != previousButton) || (ticks_autoincrement >= button_autoincrement_10ms_ticks))
							{
								if(bitPosition < 8)
								{
									*prefsPtr &= ~_BV(bitPosition);
								}
								else
								{
									if(prefsPtr == &maxDeadReckoningTime)
									{
										decrementMaxDeadReckoningTime();
									}
									if(prefsPtr == &pressureCoefficients)
									{
										decrementPumpRate();
									}
									else
									{
										if(*prefsPtr > 0)
											(*prefsPtr)--;
										if(newSleepTimeout < SLEEP_TMR_RESET_VALUE_MIN)
											newSleepTimeout = SLEEP_TMR_RESET_VALUE_MIN;
										if(newAlerterTimeout < ALERTER_TMR_RESET_VALUE_MIN)
											newAlerterTimeout = ALERTER_TMR_RESET_VALUE_MIN;
									}
									ticks_autoincrement = 0;
								}
							}
							break;
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								eeprom_write_byte((uint8_t*)EE_DEVICE_SLEEP_TIMEOUT, newSleepTimeout);
								eeprom_write_byte((uint8_t*)EE_ALERTER_TIMEOUT, newAlerterTimeout);
								eeprom_write_byte((uint8_t*)EE_DEAD_RECKONING_TIME, getMaxDeadReckoningTime());
								eeprom_write_byte((uint8_t*)EE_PRESSURE_CONFIG, getPressureConfig());
								eeprom_write_byte((uint8_t*)EE_CONFIGBITS, configBits);
								readConfig();
								// The only way to escape the prefs menu is by saving the values, so the new* variables don't serve the purpose of allowing the user to cancel a change in this case.
								// new* values are used because the values used in the program are not the same format as used here.
								newSleepTimeout = sleep_tmr_reset_value / 600;
								newAlerterTimeout = alerter_tmr_reset_value / 150;
								// Reset alerter here so it doesn't trigger the alerter down below when changing from off
								ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
								{
									alerterTimeout_decisecs = alerter_tmr_reset_value;
								}
								lcd_clrscr();
								lcd_gotoxy(1,0);
								lcd_puts("SAVED!");
								wait100ms(7);
								subscreenState = 0;  // Escape submenu
								lcd_clrscr();
							}
							break;
						case MENU_BUTTON:
							if(MENU_BUTTON != previousButton)
							{
								// Menu pressed, advance menu
								subscreenState++;
								lcd_clrscr();
							}
							break;
						case NO_BUTTON:
							break;
					}
				}
				break;

			case SYSTEM_SCREEN:
				enableLCDBacklight();
				if(!subscreenState)
				{
					lcd_gotoxy(2,0);
					lcd_puts("SYSTEM");
					lcd_gotoxy(0,1);
					lcd_putc(0x7F);
					lcd_puts("-");
					switch(button)
					{
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								subscreenState = 1;
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
					uint8_t bitPosition = 0xFF;  // <8 means boolean
					enableLCDBacklight();
					lcd_gotoxy(0,0);

					uint8_t decivoltsOkay = getBatteryOkay();
					uint8_t decivoltsWarn = getBatteryWarn();
					uint8_t decivoltsCritical = getBatteryCritical();

					if(1 == subscreenState)
					{
						lcd_puts("MENU LCK");
						bitPosition = SYSTEMBITS_MENU_LOCK;
						prefsPtr = &systemBits;
					}
					else if(2 == subscreenState)
					{
						lcd_puts("ADV FUNC");
						bitPosition = SYSTEMBITS_ADV_FUNC;
						prefsPtr = &systemBits;
					}
					else if(3 == subscreenState)
					{
						lcd_puts("BAT OKAY");
						lcd_gotoxy(7,1);
						lcd_puts("V");
						bitPosition = 0xFF;
						prefsPtr = &decivoltsOkay;
					}
					else if(4 == subscreenState)
					{
						lcd_puts("BAT WARN");
						lcd_gotoxy(7,1);
						lcd_puts("V");
						bitPosition = 0xFF;
						prefsPtr = &decivoltsWarn;
					}
					else if(5 == subscreenState)
					{
						lcd_puts("BAT CRIT");
						lcd_gotoxy(7,1);
						lcd_puts("V");
						bitPosition = 0xFF;
						prefsPtr = &decivoltsCritical;
					}
					else
					{
						bitPosition = 8;
						subscreenState = 1;
					}


					if(bitPosition < 8)
					{
						lcd_gotoxy(4,1);
						if(*prefsPtr & _BV(bitPosition))
							lcd_puts(" ON ");
						else
							lcd_puts(" OFF");
					}
					else if(8 == bitPosition)
					{
						// Do nothing
					}
					else
					{
						lcd_gotoxy(4,1);
						lcd_putc('0' + (*prefsPtr) / 10);
						lcd_putc('.');
						lcd_putc('0' + (*prefsPtr) % 10);
					}


					switch(button)
					{
						case UP_BUTTON:
							if((UP_BUTTON != previousButton) || (ticks_autoincrement >= button_autoincrement_10ms_ticks))
							{
								if(bitPosition < 8)
								{
									*prefsPtr |= _BV(bitPosition);
								}
								else if( ((prefsPtr != &decivoltsOkay)&&(prefsPtr != &decivoltsWarn)&&(prefsPtr != &decivoltsCritical)) || (systemBits & _BV(SYSTEMBITS_ADV_FUNC)) )
								{
									if(*prefsPtr < 0xFF)
										(*prefsPtr)++;
									setBatteryLevels(decivoltsOkay, decivoltsWarn, decivoltsCritical);
									ticks_autoincrement = 0;
								}
							}
							break;
						case DOWN_BUTTON:
							if((DOWN_BUTTON != previousButton) || (ticks_autoincrement >= button_autoincrement_10ms_ticks))
							{
								if(bitPosition < 8)
								{
									*prefsPtr &= ~_BV(bitPosition);
								}
								else if( ((prefsPtr != &decivoltsOkay)&&(prefsPtr != &decivoltsWarn)&&(prefsPtr != &decivoltsCritical)) || (systemBits & _BV(SYSTEMBITS_ADV_FUNC)) )
								{
									if(*prefsPtr > 0)
										(*prefsPtr)--;
									setBatteryLevels(decivoltsOkay, decivoltsWarn, decivoltsCritical);
									ticks_autoincrement = 0;
								}
							}
							break;
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								eeprom_write_byte((uint8_t*)EE_BATTERY_OKAY, getBatteryOkay());
								eeprom_write_byte((uint8_t*)EE_BATTERY_WARN, getBatteryWarn());
								eeprom_write_byte((uint8_t*)EE_BATTERY_CRITICAL, getBatteryCritical());
								readConfig();
								lcd_clrscr();
								lcd_gotoxy(1,0);
								lcd_puts("SAVED!");
								wait100ms(7);
								subscreenState = 0;  // Escape submenu
								lcd_clrscr();
							}
							break;
						case MENU_BUTTON:
							if(MENU_BUTTON != previousButton)
							{
								// Menu pressed, advance menu
								subscreenState++;
								lcd_clrscr();
							}
							break;
						case NO_BUTTON:
							break;
					}
				}
				break;
				
			case DIAG_SCREEN:
				enableLCDBacklight();
				if(!subscreenState)
				{
					lcd_gotoxy(3,0);
					lcd_puts("DIAGS");
					lcd_gotoxy(0,1);
					lcd_putc(0x7F);
					lcd_putc('-');
					switch(button)
					{
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								setupLCD(LCD_DIAGS);
								subscreenState = 1;
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
					if(1 == subscreenState)
					{
						if(!subscreenCount)
						{
							enableLCDBacklight();
							lcd_gotoxy(5,0);
							if(0 == activeThrottleSetting)
							{
								lcd_putc('I');
							}
							else
							{
								lcd_putc('0' + activeThrottleSetting);
							}
							lcd_gotoxy(0,0);
							if(throttleStatus & THROTTLE_STATUS_EMERGENCY)
							{
								lcd_puts("EMRG");
							}
							else
							{
								if( (optionBits & _BV(OPTIONBITS_VARIABLE_BRAKE)) && (optionBits & _BV(OPTIONBITS_STEPPED_BRAKE)) )
								{
									switch(brakeState)
									{
										// These two states get "brake off" set by first making sure "brake on" is clear (TCS decoders don't like these changing at the same time)
										case BRAKE_LOW_BEGIN:
										case BRAKE_LOW_WAIT:
											lcd_puts("OFF ");
											break;
										case BRAKE_20PCNT_BEGIN:
										case BRAKE_20PCNT_WAIT:
											lcd_puts("BRK1");
											break;
										case BRAKE_40PCNT_BEGIN:
										case BRAKE_40PCNT_WAIT:
											lcd_puts("BRK2");
											break;
										case BRAKE_60PCNT_BEGIN:
										case BRAKE_60PCNT_WAIT:
											lcd_puts("BRK3");
											break;
										case BRAKE_80PCNT_BEGIN:
										case BRAKE_80PCNT_WAIT:
											lcd_puts("BRK4");
											break;
										case BRAKE_FULL_BEGIN:
										case BRAKE_FULL_WAIT:
											lcd_puts("BRK5");
											break;
									}
								}
								else
								{
									if( !(controls & BRAKE_CONTROL) && !(controls & BRAKE_OFF_CONTROL) )
										lcd_putc(FUNCTION_INACTIVE_CHAR);
									else if( (controls & BRAKE_CONTROL) && !(controls & BRAKE_OFF_CONTROL) )
										lcd_putc(FUNCTION_ACTIVE_CHAR);
									else if( !(controls & BRAKE_CONTROL) && (controls & BRAKE_OFF_CONTROL) )
										lcd_putc('*');
									else if( (controls & BRAKE_CONTROL) && (controls & BRAKE_OFF_CONTROL) )
										lcd_putc('!');  // Invalid condition
									printDec2Dig((brakePcnt>99)?99:brakePcnt);
									lcd_putc('%');
								}
							}
						
							lcd_gotoxy(7,0);
							switch(activeReverserSetting)
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
							lcd_putc((controls & AUX_CONTROL) ? FUNCTION_ACTIVE_CHAR : FUNCTION_INACTIVE_CHAR);
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
						else
						{
							if(subscreenCount > 3)
								subscreenCount = 3;
							enableLCDBacklight();
							lcd_gotoxy(0,0);
							lcd_puts("FN:");
							lcd_gotoxy(0,1);
							lcd_putc('0'+(subscreenCount-1));
							lcd_puts("0+");
							for(i=0; i<10; i++)
							{
								uint8_t fnum = (10*(subscreenCount-1))+i;
								lcd_gotoxy((i%5)+3, i/5);
								if(functionMask & ((uint32_t)1 << (fnum)))
								{
									lcd_putc('0' + i);
								}
								else
								{
									lcd_putc(' ');
								}
							}
						}
					}
					else if(2 == subscreenState)
					{
						enableLCDBacklight();
						lcd_gotoxy(0,0);
						lcd_puts("SLEEP");
						lcd_gotoxy(0,1);
						ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
						{
							decisecs_tmp = sleepTimeout_decisecs;
						}
						printDec4Dig((decisecs_tmp+9)/10);
						lcd_gotoxy(5,1);
						lcd_puts("sec");
					}
					else if(3 == subscreenState)
					{
						enableLCDBacklight();
						lcd_gotoxy(0,0);
						lcd_puts("ALERTER");
						lcd_gotoxy(0,1);
						ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
						{
							decisecs_tmp = alerterTimeout_decisecs;
						}
						if(IS_ALERTER_ENABLED)
						{
							printDec4Dig((decisecs_tmp+9)/10);
							lcd_gotoxy(5,1);
							lcd_puts("sec");
						}
						else
						{
							lcd_gotoxy(5,1);
							lcd_puts("OFF");
						}
					}
					else if(4 == subscreenState)
					{
						if(!subscreenCount)
						{
							enableLCDBacklight();
							lcd_gotoxy(0,0);
							lcd_puts("ENGINE");
							lcd_gotoxy(0,1);
							lcd_puts("HISTORY");
						}
						else
						{
							if(subscreenCount > ENGINE_STATE_QUEUE_SIZE)
								subscreenCount = ENGINE_STATE_QUEUE_SIZE;
							lcd_gotoxy(0,0);
							printDec2Dig(subscreenCount);
							lcd_putc(':');
							printLocomotiveAddress(engineStatesQueuePeekLocoAddress(subscreenCount-1));
							lcd_gotoxy(0,1);
							printEngineState(engineStatesQueuePeekState(subscreenCount-1));
						}
					}
					else if(5 == subscreenState)
					{
						enableLCDBacklight();
						lcd_gotoxy(0,0);
						lcd_puts("PKT TIME");
						lcd_gotoxy(0,1);
						lcd_putc('[');
						lcd_gotoxy(7,1);
						lcd_putc(']');
						lcd_gotoxy(1,1);
						uint8_t pktTimeout_tmp = (pktTimeout + 6) / 7;
						for(i = 0; i < 6; i++)
						{
							if(pktTimeout_tmp)
							{
								pktTimeout_tmp--;
								lcd_putc(0xFF);
							}
							else
								lcd_putc(' ');
						}
					}
					else if(6 == subscreenState)
					{
						enableLCDBacklight();
						lcd_gotoxy(0,0);
						lcd_puts("RSSI");
						lcd_gotoxy(0,1);
						if(lastRSSI < 255)
						{
							if(lastRSSI < 10)
							{
								lcd_puts("  -");
								lcd_putc('0' + lastRSSI);
							}
							else if(lastRSSI < 100)
							{
								lcd_puts(" -");
								printDec2Dig(lastRSSI);
							}
							else
							{
								lcd_putc('-');
								printDec3Dig(lastRSSI);
							}
							lcd_gotoxy(4,1);
							lcd_puts("dBm ");
						}
						else
						{
							lcd_gotoxy(0,1);
							lcd_puts("NO SIGNL");
						}
					}
					else if(7 == subscreenState)
					{
						enableLCDBacklight();
						lcd_gotoxy(0,0);
						lcd_puts("FT RATIO");
						lcd_gotoxy(1,1);
						uint8_t timeScaleFactor = getTimeScaleFactor();
						uint8_t timeScaleFactor_tmp = (timeScaleFactor/100)%10;
						if(timeScaleFactor_tmp)
							lcd_putc('0' + timeScaleFactor_tmp);
						else
							lcd_putc(' ');
						lcd_putc('0' + (timeScaleFactor/10)%10);
						lcd_putc('.');
						lcd_putc('0' + timeScaleFactor%10);
						lcd_puts(":1");
					}
					else if(8 == subscreenState)
					{
						enableLCDBacklight();
						lcd_gotoxy(0,0);
						lcd_puts("BATTERY");
						lcd_gotoxy(1,1);
						lcd_putc('0' + (((getBatteryVoltage()*2)/100)%10));
						lcd_putc('.');
						lcd_putc('0' + (((getBatteryVoltage()*2)/10)%10));
						lcd_putc('0' + ((getBatteryVoltage()*2)%10));
						lcd_putc('V');
					}
					else if(9 == subscreenState)
					{
						enableLCDBacklight();
						lcd_gotoxy(0,0);
						lcd_puts("VERSION");
						lcd_gotoxy(0,1);
						lcd_puts(VERSION_STRING);
					}
					else if(10 == subscreenState)
					{
						enableLCDBacklight();
						lcd_gotoxy(0,0);
						lcd_puts("GIT REV");
						lcd_gotoxy(1,1);
						printHex((GIT_REV >> 16) & 0xFF);
						printHex((GIT_REV >> 8) & 0xFF);
						printHex(GIT_REV & 0xFF);
					}
					else if(11 == subscreenState)
					{
						enableLCDBacklight();
						lcd_gotoxy(0,0);
						lcd_puts("BASE TYP");
						lcd_gotoxy(0,1);
						lcd_puts(baseString);
					}
					else if(12 == subscreenState)
					{
						enableLCDBacklight();
						lcd_gotoxy(0,0);
						lcd_puts("BASE REV");
						lcd_gotoxy(1,1);
						printHex((baseVersion >> 16) & 0xFF);
						printHex((baseVersion >> 8) & 0xFF);
						printHex(baseVersion & 0xFF);
					}
					else if(13 == subscreenState)
					{
						if(resetCounter)
						{
							enableLCDBacklight();
							lcd_gotoxy(0,0);
							lcd_puts("FACTORY");
							lcd_gotoxy(0,1);
							lcd_puts("RESET ");
							lcd_putc('0' + resetCounter);
							lcd_putc(0x7E);
						}
						else
						{
							lcd_clrscr();
							lcd_gotoxy(0,0);
							lcd_puts("RESET!!!");
							resetConfig();
							while(1);  // Force a watchdog reset
						}
					}
					else
					{
						subscreenState = 1;
					}

					switch(button)
					{
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								setupLCD(LCD_DEFAULT);   // Restore default characters
								subscreenState = 0;  // Escape submenu
								subscreenCount = 0;
								lcd_clrscr();
							}
							break;
						case MENU_BUTTON:
							if(MENU_BUTTON != previousButton)
							{
								// Menu pressed, advance menu
								subscreenState++;
								subscreenCount = 0;
								lcd_clrscr();
								resetCounter = RESET_COUNTER_RESET_VALUE;
							}
							break;
						case DOWN_BUTTON:
							if(DOWN_BUTTON != previousButton)
							{
								// Decrement here blindly, but it's only used in the reset screen
								// It will be reset anyway prior to entering reset screen
								resetCounter--;
								
								if(subscreenCount)
								{
									subscreenCount--;
									lcd_clrscr();
								}
							}
							break;
						case UP_BUTTON:
							if(UP_BUTTON != previousButton)
							{
								// Decrement here blindly, but it's only used in the reset screen
								// It will be reset anyway prior to entering reset screen
								resetCounter--;
								
								if(subscreenCount < 255)
								{
									// Will be limited where used above
									subscreenCount++;
									lcd_clrscr();
								}
							}
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
		if(!subscreenState)
		{
			if(MENU_BUTTON == button)
			{
				if(MENU_BUTTON != previousButton)
				{
					// Menu pressed, advance menu
					lcd_clrscr();
					screenState++;  // No range checking needed since LAST_SCREEN will reset the counter
					ticks_autoincrement = 0;  // Reset to zero so a long press can be detected
					
					// Check for conditional menus
					if(!(systemBits & _BV(SYSTEMBITS_ADV_FUNC)))
					{
						// Advanced functions NOT active
						if(THRESHOLD_CAL_SCREEN == screenState)
						{
							if(	(0xFF != hornThreshold) &&
								(0xFF != brakeThreshold) &&
								(0xFF != brakeLowThreshold) &&
								(0xFF != brakeHighThreshold)
								)
							{
								// Skip threshold menu, but only if already calilbrated
								screenState++;
							}
						}
					}
					
					if(systemBits & _BV(SYSTEMBITS_MENU_LOCK))
					{
						// Menu lock active
						while( 	(ENGINE_SCREEN != screenState) &&
								(SPECFN_SCREEN != screenState) &&
								(LOAD_CONFIG_SCREEN != screenState) &&
								(LOCO_SCREEN != screenState) &&
								(FORCE_FUNC_SCREEN != screenState) &&
								(SYSTEM_SCREEN != screenState) &&
								(LAST_SCREEN != screenState)
							)
						{
							// Skip menu(s)
							screenState++;
						}
					}
					
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
			PktHandler();
		}

		wdt_reset();

		if (0 == pktTimeout)
		{
			baseVersion = 0;
			lastRSSI = 0xFF;
			strcpy(baseString, "  NONE  ");
			led = LED_RED_FASTBLINK;
		}
		else if(configBits & _BV(CONFIGBITS_LED_BLINK))
		{
			//  Blink GREEN unless configred to be off
			led = LED_GREEN;
		}
		else
		{
			led = LED_OFF;
		}
		

		// Figure out which functions should be on and which should be off
		functionMask = 0;
		estopStatus &= ~ESTOP_BUTTON;
		if(controls & HORN_CONTROL)
		{
			if(FORCE_FUNC_SCREEN == screenState)
			{
				functionMask |= (uint32_t)1 << (functionNumber);
			}
			else
			{
				functionMask |= getFunctionMask(HORN_FN);
			}
		}
		if(controls & BELL_CONTROL)
			functionMask |= getFunctionMask(BELL_FN);
		if(controls & AUX_CONTROL)
		{
			functionMask |= getFunctionMask(AUX_FN);
			if(isFunctionEstop(AUX_FN))
				estopStatus |= ESTOP_BUTTON;
		}
		if(controls & BRAKE_CONTROL)
			functionMask |= getFunctionMask(BRAKE_FN);
		if(controls & BRAKE_OFF_CONTROL)
			functionMask |= getFunctionMask(BRAKE_OFF_FN);
		if((ENGINE_ON == engineState)||(ENGINE_START == engineState))
			functionMask |= getFunctionMask(ENGINE_ON_FN);
		if(ENGINE_STOP == engineState)
			functionMask |= getFunctionMask(ENGINE_OFF_FN);
		if(optionButtonState & UP_OPTION_BUTTON)
		{
			functionMask |= getFunctionMask(UP_FN);
			if(isFunctionEstop(UP_FN))
				estopStatus |= ESTOP_BUTTON;
		}
		if(optionButtonState & DOWN_OPTION_BUTTON)
		{
			functionMask |= getFunctionMask(DOWN_FN);
			if(isFunctionEstop(DOWN_FN))
				estopStatus |= ESTOP_BUTTON;
		}

		if(isCompressorRunning())
		{
#ifdef COMPRESSOR_TM
			lcd_gotoxy(4,0);
			lcd_putc('C');
#endif
			functionMask |= getFunctionMask(COMPRESSOR_FN);
		}
#ifdef COMPRESSOR_TM
		else
		{
			lcd_gotoxy(4,0);
			lcd_putc(' ');
		}
#endif
		if(isBrakeTestActive())
		{
#ifdef COMPRESSOR_TM
			lcd_gotoxy(4,1);
			lcd_putc('B');
#endif
			functionMask |= getFunctionMask(BRAKE_TEST_FN);
		}
		else
		{
#ifdef COMPRESSOR_TM
			lcd_gotoxy(4,1);
			lcd_putc(' ');
#endif
		}

		if(controls & THR_UNLK_CONTROL)
			functionMask |= getFunctionMask(THR_UNLOCK_FN);

		if(NEUTRAL == activeReverserSetting)
		{
			functionMask |= getFunctionMask(NEUTRAL_FN);
		}

		wdt_reset();

		switch(frontLight)
		{
			case LIGHT_OFF:
				break;
			case LIGHT_DIM:
				functionMask |= getFunctionMask(FRONT_DIM1_FN);
				functionMask |= getFunctionMask(FRONT_DIM2_FN);
				break;
			case LIGHT_BRIGHT:
				functionMask |= getFunctionMask(FRONT_HEADLIGHT_FN);
				break;
			case LIGHT_BRIGHT_DITCH:
				functionMask |= getFunctionMask(FRONT_HEADLIGHT_FN);
				functionMask |= getFunctionMask(FRONT_DITCH_FN);
				break;
		}

		wdt_reset();

		switch(rearLight)
		{
			case LIGHT_OFF:
				break;
			case LIGHT_DIM:
				functionMask |= getFunctionMask(REAR_DIM1_FN);
				functionMask |= getFunctionMask(REAR_DIM2_FN);
				break;
			case LIGHT_BRIGHT:
				functionMask |= getFunctionMask(REAR_HEADLIGHT_FN);
				break;
			case LIGHT_BRIGHT_DITCH:
				functionMask |= getFunctionMask(REAR_HEADLIGHT_FN);
				functionMask |= getFunctionMask(REAR_DITCH_FN);
				break;
		}

		uint16_t alerterTimeout_decisecs_tmp;
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			alerterTimeout_decisecs_tmp = alerterTimeout_decisecs;
		}
		throttleStatus &= ~THROTTLE_STATUS_ALERTER;  // Clear here, (re)set below.
		estopStatus &= ~ESTOP_ALERTER;
		if(IS_ALERTER_ENABLED)
		{
			if(alerterTimeout_decisecs_tmp < 100)
			{
				functionMask |= getFunctionMask(ALERTER_FN);
				lastFunctionMask |= getFunctionMask(ALERTER_FN);  // Fake lastFunctionMask so this doesn't trigger inputsChanged below and reset alerter timer
				throttleStatus |= THROTTLE_STATUS_ALERTER;
			}  // Fall through if...
			if(0 == alerterTimeout_decisecs_tmp)
			{
				// Throttle down to idle and apply brakes
				activeThrottleSetting = 0;
				lastActiveThrottleSetting = 0;
				functionMask |= getFunctionMask(BRAKE_FN);
				if(isFunctionEstop(ALERTER_FN))
					estopStatus |= ESTOP_ALERTER;
				lastFunctionMask |= getFunctionMask(BRAKE_FN);  // Fake lastFunctionMask so this doesn't trigger inputsChanged below and reset alerter timer
			}
		}

		// Force specific functions on or off
		functionMask |= functionForceOn;
		functionMask &= ~functionForceOff;

		wdt_reset();

		// Process various E-Stop inputs to create single status bit
		if(estopStatus)
			throttleStatus |= THROTTLE_STATUS_EMERGENCY;
		else
			throttleStatus &= ~THROTTLE_STATUS_EMERGENCY;

		uint8_t inputsChanged =	(activeReverserSetting != lastActiveReverserSetting) ||
									(activeThrottleSetting != lastActiveThrottleSetting) ||
									(functionMask != lastFunctionMask) ||
									((throttleStatus & THROTTLE_STATUS_EMERGENCY) != (lastThrottleStatus & THROTTLE_STATUS_EMERGENCY));
									// Look at just EMERG bit since other bits are used for sleep and alerter

		// Reset sleep timer
		// Using activeReverserSetting also guarantees the throttle (activeThrottleSetting) was in idle when entering sleep, so it will unsleep in the idle position.
		if( 
			(NO_BUTTON != button) || 
			inputsChanged ||
			((configBits & _BV(CONFIGBITS_STRICT_SLEEP)) && ((FORWARD == activeReverserSetting) || (REVERSE == activeReverserSetting)))
			)
		{
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
			{
				sleepTimeout_decisecs = sleep_tmr_reset_value;
			}
		}

		// Reset alerter timer
		// Using activeReverserSetting also guarantees the throttle (activeThrottleSetting) was in idle when entering sleep, so it will unsleep in the idle position.
		if( 
			(NO_BUTTON != button) || 
			inputsChanged ||
			(NEUTRAL == activeReverserSetting)
			)
		{
			if(alerterTimeout_decisecs_tmp || ((!alerterTimeout_decisecs_tmp) && (NEUTRAL == activeReverserSetting)))
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
			{
				alerterTimeout_decisecs = alerter_tmr_reset_value;
			}
		}

		wdt_reset();

		// New packet criteria: Stuff changed ...or... it's been more than the transmission timeout
		//    ...and there's room in the queue
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			decisecs_tmp = decisecs;
		}
		if (
				adcLoopInitialized() &&
				((inputsChanged) || (decisecs_tmp >= update_decisecs)) &&
				!(mrbusPktQueueFull(&mrbeeTxQueue))
			)
		{
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
			{
				decisecs = 0;
			}
			inputsChanged = 0;
			lastActiveReverserSetting = activeReverserSetting;
			lastActiveThrottleSetting = activeThrottleSetting;
			lastFunctionMask = functionMask;
			lastThrottleStatus = throttleStatus;
			
			txBuffer[MRBUS_PKT_DEST] = mrbus_base_addr;
			txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
			txBuffer[MRBUS_PKT_LEN] = 15;
			txBuffer[5] = 'S';

			txBuffer[6] = locoAddress >> 8;
			txBuffer[7] = locoAddress & 0xFF;

			if(throttleStatus & THROTTLE_STATUS_EMERGENCY)
			{
				txBuffer[8] = 1;  // E-stop				
			}
			else if(0 == activeThrottleSetting)
				txBuffer[8] = 0;
			else
				txBuffer[8] = notchSpeedStep[activeThrottleSetting-1] + 1;
			
			switch(activeReverserSetting)
			{
				case FORWARD:
					direction = FORWARD;
					break;
				case REVERSE:
					direction = REVERSE;
					break;
				case NEUTRAL:
					// Preserve previous direction for DCC purposes (mainly if auto-directional lighting is enabled)
					break;
			}
			
			if(FORWARD == direction)
				txBuffer[8] |= 0x80;
			
			txBuffer[9]  = functionMask >> 24;
			txBuffer[10] = functionMask >> 16;
			txBuffer[11] = functionMask >> 8;
			txBuffer[12] = functionMask & 0xFF;

			txBuffer[13] = throttleStatus;

			txBuffer[14] = getBatteryVoltage();	
			mrbusPktQueuePush(&mrbeeTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		}

		// Transmission criteria: something in the buffer ...and... it's been more than the minimum holdoff
		if (mrbusPktQueueDepth(&mrbeeTxQueue) && !txHoldoff)
		{
			wdt_reset();
			mrbeeTransmit();
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
			{
				txHoldoff = txHoldoff_centisecs;
			}
		}

		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			decisecs_tmp = sleepTimeout_decisecs;
		}
		if((0 == decisecs_tmp) || (throttleStatus & THROTTLE_STATUS_SLEEP))
		{
			wdt_reset();

			// Time to nod off
			led = LED_OFF;

			// Disable internal power-sucking peripherals (need to be enabled when awake)
			disableADC();
			setXbeeSleep();
			disableLCD();
			disableLCDBacklight();
			disableTimer();  // Disable 100Hz timer (to prevent LEDs from blinking on right before sleeping)
			ledGreenOff();
			ledRedOff();
			disableSwitches();  // Don't disable buttons
			disableThrottle();
			
			// Reinforce that these are off
			disablePots();
			disableReverser();
			disableLightSwitches();

			set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // set the type of sleep mode to use
			cli();
			wdt_reset();
			wdt_disable();                         // Disable watchdog so it doesn't reset us in sleep
			PCIFR |= _BV(PCIF1);                   // Clear any pending interrupts, maybe from the button release on the last sleep
			PCICR |= _BV(PCIE1);                   // Enable Pin Change Interrupt Bank 1 (softkey interrupts)
			PCMSK1 |= _BV(PCINT15) | _BV(PCINT14) | _BV(PCINT13) | _BV(PCINT12);  // Enable interrupts on softkeys
			sleep_enable();                        // Enable sleep mode
			sei();
			sleep_cpu();
			cli();
			sleep_disable();                       // Disable sleep mode
			PCICR &= ~_BV(PCIE1);                  // Disable Pin Change Interrupt Bank 1
			wdt_reset();
			wdt_enable(WATCHDOG_TIMEOUT);          // Reenable watchdog
			wdt_reset();
			sei();

			// Re-enable chip internal bits (ADC, pots, reverser, light switches done in ADC loop)
			setXbeeActive();
			enableLCD();
			initLCD();
			if(DIAG_SCREEN == screenState)
			{
				// Change LCD chars if in the DIAG screen
				setupLCD(LCD_DIAGS);
			}
			initialize100HzTimer();
			enableSwitches();
			enableThrottle();

			// Initialize the buttons so there are no startup artifacts when we actually use them
			inputButtons = debounce(inputButtons, (PINB & (0xF6)));
			inputButtons = debounce(inputButtons, (PINB & (0xF6)));
			inputButtons = debounce(inputButtons, (PINB & (0xF6)));
			inputButtons = debounce(inputButtons, (PINB & (0xF6)));
			processButtons(inputButtons);
			processSwitches(inputButtons);
			previousButton = button;  // Prevent extraneous menu advances
			clearDeadReckoningTime();

			ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
			{
				sleepTimeout_decisecs = sleep_tmr_reset_value;
				alerterTimeout_decisecs = alerter_tmr_reset_value;
			}
			throttleStatus &= ~THROTTLE_STATUS_SLEEP;
		}

	}

}


