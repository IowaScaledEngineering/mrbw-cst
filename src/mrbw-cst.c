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
#include "cst-battery.h"
#include "cst-tonnage.h"
#include "cst-time.h"

//#define FAST_SLEEP
#ifdef FAST_SLEEP
#warning "Fast Sleep Enabled!"
#endif

#define max(a,b)  ((a)>(b)?(a):(b))

#define LONG_PRESS_10MS_TICKS             100
#define BUTTON_AUTOINCREMENT_10MS_TICKS    50
#define BUTTON_AUTOINCREMENT_ACCEL         10
#define BUTTON_AUTOINCREMENT_MINIMUM        5

#define SLEEP_TMR_RESET_VALUE_MIN           1
#define SLEEP_TMR_RESET_VALUE_DEFAULT       5
#define SLEEP_TMR_RESET_VALUE_MAX          99

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
uint8_t resetCounter;

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

#define OFF_FUNCTION      0x80
#define LATCH_FUNCTION    0x40

#define HORN_HYSTERESIS   5
#define BRAKE_HYSTERESIS  5

// BRAKE_PULSE_WIDTH is in decisecs
// It is the minimum on time for the pulsed brake
#define BRAKE_PULSE_WIDTH_MIN       2
#define BRAKE_PULSE_WIDTH_DEFAULT   5
#define BRAKE_PULSE_WIDTH_MAX      10

uint8_t brakePulseWidth = BRAKE_PULSE_WIDTH_DEFAULT;

// Set EEPROM locations
#define EE_VERSION_MAJOR              0x0E
#define EE_VERSION_MINOR              0x0F

#define EE_CURRENT_CONFIG             0x10
#define EE_DEVICE_SLEEP_TIMEOUT       0x11
#define EE_DEAD_RECKONING_TIME        0x12
#define EE_CONFIGBITS                 0x13
#define EE_BATTERY_OKAY               0x14
#define EE_BATTERY_WARN               0x15
#define EE_BATTERY_CRITICAL           0x16
#define EE_TX_HOLDOFF                 0x1D
#define EE_TIME_SOURCE_ADDRESS        0x1E
#define EE_BASE_ADDR                  0x1F
#define EE_HORN_THRESHOLD             0x20
#define EE_BRAKE_THRESHOLD            0x21
#define EE_BRAKE_LOW_THRESHOLD        0x22
#define EE_BRAKE_HIGH_THRESHOLD       0x23

// 20 configs * 128 bytes = 2560 bytes
//  +128 bytes for global = 2688 bytes

//        Total available = 4096 bytes

#define MAX_CONFIGS      20

// Put working (scratchspace) config at the end of EEPROM space
#define WORKING_CONFIG   31

#define CONFIG_START                  0x80
#define CONFIG_SIZE                   0x80


// These are offsets from CONFIG_START
#define EE_LOCO_ADDRESS               (0x00 + configOffset)
//      EE_LOCO_ADDRESS                0x01
#define EE_HORN_FUNCTION              (0x02 + configOffset)
#define EE_BELL_FUNCTION              (0x03 + configOffset)
#define EE_BRAKE_FUNCTION             (0x04 + configOffset)
#define EE_AUX_FUNCTION               (0x05 + configOffset)
#define EE_ENGINE_ON_FUNCTION         (0x06 + configOffset)
#define EE_ENGINE_OFF_FUNCTION        (0x07 + configOffset)

#define EE_FRONT_DIM1_FUNCTION        (0x08 + configOffset)
#define EE_FRONT_DIM2_FUNCTION        (0x09 + configOffset)
#define EE_FRONT_HEADLIGHT_FUNCTION   (0x0A + configOffset)
#define EE_FRONT_DITCH_FUNCTION       (0x0B + configOffset)
#define EE_REAR_DIM1_FUNCTION         (0x0C + configOffset)
#define EE_REAR_DIM2_FUNCTION         (0x0D + configOffset)
#define EE_REAR_HEADLIGHT_FUNCTION    (0x0E + configOffset)
#define EE_REAR_DITCH_FUNCTION        (0x0F + configOffset)

#define EE_UP_BUTTON_FUNCTION         (0x10 + configOffset)
#define EE_DOWN_BUTTON_FUNCTION       (0x11 + configOffset)
#define EE_THR_UNLOCK_FUNCTION        (0x12 + configOffset)
#define EE_BRAKE_OFF_FUNCTION         (0x13 + configOffset)
#define EE_REV_SWAP_FUNCTION          (0x14 + configOffset)

#define EE_BRAKE_PULSE_WIDTH          (0x16 + configOffset)
#define EE_OPTIONBITS                 (0x17 + configOffset)

#define EE_FUNC_FORCE_ON              (0x18 + configOffset)
//      EE_FUNC_FORCE_ON               0x19
//      EE_FUNC_FORCE_ON               0x1A
//      EE_FUNC_FORCE_ON               0x1B
#define EE_FUNC_FORCE_OFF             (0x1C + configOffset)
//      EE_FUNC_FORCE_OFF              0x1D
//      EE_FUNC_FORCE_OFF              0x1E
//      EE_FUNC_FORCE_OFF              0x1F

#define EE_NOTCH_SPEEDSTEP                (0x20 + configOffset)
//      EE_NOTCH_SPEEDSTEP                 0x21
//      EE_NOTCH_SPEEDSTEP                 0x22
//      EE_NOTCH_SPEEDSTEP                 0x23
//      EE_NOTCH_SPEEDSTEP                 0x24
//      EE_NOTCH_SPEEDSTEP                 0x25
//      EE_NOTCH_SPEEDSTEP                 0x26
//      EE_NOTCH_SPEEDSTEP                 0x27

uint16_t configOffset;


// Boolean config bits (EEPROM, global)
#define CONFIGBITS_LED_BLINK         0
#define CONFIGBITS_REVERSER_LOCK     4

#define CONFIGBITS_DEFAULT                 (_BV(CONFIGBITS_LED_BLINK) | _BV(CONFIGBITS_REVERSER_LOCK))
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

uint8_t hornFunction = 2;
uint8_t bellFunction = 7;
uint8_t frontDim1Function = 3, frontDim2Function = OFF_FUNCTION, frontHeadlightFunction = 0, frontDitchFunction = 3;
uint8_t rearDim1Function = 6, rearDim2Function = OFF_FUNCTION, rearHeadlightFunction = 5, rearDitchFunction = 6;
uint8_t brakeFunction = OFF_FUNCTION;
uint8_t brakeOffFunction = OFF_FUNCTION;
uint8_t auxFunction = OFF_FUNCTION;
uint8_t engineOnFunction = 8;
uint8_t engineStopFunction = OFF_FUNCTION;
uint8_t upButtonFunction = OFF_FUNCTION, downButtonFunction = OFF_FUNCTION;
uint8_t throttleUnlockFunction = 9;
uint8_t reverserSwapFunction = OFF_FUNCTION;

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
volatile uint8_t txHoldoff = 0;
volatile uint8_t status = 0;

uint16_t update_decisecs = UPDATE_DECISECS_DEFAULT;
uint8_t txHoldoff_centisecs = TX_HOLDOFF_DEFAULT;

static uint8_t timeSourceAddress = 0xFF;

#define THROTTLE_STATUS_SLEEP           0x80
#define THROTTLE_STATUS_ALL_STOP        0x02
#define THROTTLE_STATUS_EMERGENCY       0x01

volatile uint8_t throttleStatus = 0;

uint16_t sleep_tmr_reset_value;

// Define the menu screens and menu order
// Must end with LAST_SCREEN
typedef enum
{
	MAIN_SCREEN = 0,
	ENGINE_SCREEN,
	TONNAGE_SCREEN,
	LOAD_CONFIG_SCREEN,
	SAVE_CONFIG_SCREEN,
	LOCO_SCREEN,
	FUNC_FORCE_SCREEN,
	FUNC_CONFIG_SCREEN,
	NOTCH_CONFIG_SCREEN,
	OPTION_SCREEN,
	SYSTEM_SCREEN,
	COMM_SCREEN,
	PREFS_SCREEN,
	THRESHOLD_CAL_SCREEN,
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

typedef enum
{
	HORN_FN = 0,
	BELL_FN,
	BRAKE_FN,
	BRAKE_OFF_FN,
	AUX_FN,
	ENGINE_ON_FN,
	ENGINE_OFF_FN,
	THR_UNLOCK_FN,
	REV_SWAP_FN,
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

typedef enum
{
	ENGINE_OFF = 0,
	ENGINE_START,
	ENGINE_ON,
	ENGINE_RUNNING,
	ENGINE_NOT_IDLE,
	ENGINE_STOP,
} EngineState;

#define ENGINE_TIMER_DECISECS      50
volatile uint8_t engineTimer = 0;

inline uint16_t calculateConfigOffset(uint8_t cfgNum)
{
	return(((cfgNum - 1) * CONFIG_SIZE) + CONFIG_START);
}

inline uint8_t calculateConfigNumber(uint16_t cfgOffset)
{
	return(((configOffset - CONFIG_START) / CONFIG_SIZE) + 1);
}

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
	buf[MRBUS_PKT_LEN] = 20;
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
	buf[15] = '*';
	buf[16] = 'E';
	buf[17] = 'M';
	buf[18] = 'C';
	buf[19] = '2';
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
		ticks_autoincrement = button_autoincrement_10ms_ticks;
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

		ledUpdate();

		if (engineTimer)
			engineTimer--;
		
		brakeCounter++;
		if(brakeCounter >= (4*brakePulseWidth))
			brakeCounter = 0;
		
		updateTime10Hz();
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

	// Fast clock
	uint8_t maxDeadReckoningTime = eeprom_read_byte((uint8_t*)EE_DEAD_RECKONING_TIME);
	uint8_t newMaxDeadReckoningTime = setMaxDeadReckoningTime(maxDeadReckoningTime);
	if(newMaxDeadReckoningTime != maxDeadReckoningTime)
		eeprom_write_byte((uint8_t*)EE_DEAD_RECKONING_TIME, newMaxDeadReckoningTime);

	timeSourceAddress = eeprom_read_byte((uint8_t*)EE_TIME_SOURCE_ADDRESS);

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

	// The following code used to calculate the active EEPROM memory space, based on the currently selected config #.
	// Now we only have one config that is used during operation (WORKING_CONFIG), so make sure this is what we use.
	configOffset = eeprom_read_byte((uint8_t*)EE_CURRENT_CONFIG);  // Abuse configOffset to read the config number
	if(configOffset != WORKING_CONFIG)
	{
		configOffset = WORKING_CONFIG;
		eeprom_write_byte((uint8_t*)EE_CURRENT_CONFIG, configOffset);
	}
	configOffset = calculateConfigOffset(configOffset);
	
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
	brakeFunction = eeprom_read_byte((uint8_t*)EE_BRAKE_FUNCTION);
	brakeOffFunction = eeprom_read_byte((uint8_t*)EE_BRAKE_OFF_FUNCTION);
	auxFunction = eeprom_read_byte((uint8_t*)EE_AUX_FUNCTION);
	engineOnFunction = eeprom_read_byte((uint8_t*)EE_ENGINE_ON_FUNCTION);
	engineStopFunction = eeprom_read_byte((uint8_t*)EE_ENGINE_OFF_FUNCTION);
	upButtonFunction = eeprom_read_byte((uint8_t*)EE_UP_BUTTON_FUNCTION);
	downButtonFunction = eeprom_read_byte((uint8_t*)EE_DOWN_BUTTON_FUNCTION);
	throttleUnlockFunction = eeprom_read_byte((uint8_t*)EE_THR_UNLOCK_FUNCTION);
	reverserSwapFunction = eeprom_read_byte((uint8_t*)EE_REV_SWAP_FUNCTION);
	
	functionForceOn = eeprom_read_dword((uint32_t*)EE_FUNC_FORCE_ON);
	functionForceOff = eeprom_read_dword((uint32_t*)EE_FUNC_FORCE_OFF);

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
	uint8_t i;
	uint16_t sourceAddr;
	uint16_t destAddr;

	sourceAddr = calculateConfigOffset(srcConfig);
	destAddr = calculateConfigOffset(destConfig);

	for(i=0; i<CONFIG_SIZE; i++)
	{
		uint8_t tmpEE = eeprom_read_byte((uint8_t*)(sourceAddr++));
		eeprom_write_byte((uint8_t*)(destAddr++), tmpEE);
	}
}

void resetConfig(void)
{
	uint8_t i;

	wdt_reset();

	eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_H, UPDATE_DECISECS_DEFAULT >> 8);
	eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_L, UPDATE_DECISECS_DEFAULT & 0xFF);
	eeprom_write_byte((uint8_t*)EE_TX_HOLDOFF, TX_HOLDOFF_DEFAULT);

	eeprom_write_byte((uint8_t*)EE_DEVICE_SLEEP_TIMEOUT, SLEEP_TMR_RESET_VALUE_DEFAULT);
	eeprom_write_byte((uint8_t*)EE_DEAD_RECKONING_TIME, DEAD_RECKONING_TIME_DEFAULT);
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

	eeprom_write_byte((uint8_t*)EE_CURRENT_CONFIG, WORKING_CONFIG);
	
	for (i=1; i<=MAX_CONFIGS; i++)
	{
		wdt_reset();
		configOffset = calculateConfigOffset(i);
		eeprom_write_word((uint16_t*)EE_LOCO_ADDRESS, 0x0003 | LOCO_ADDRESS_SHORT);
		eeprom_write_byte((uint8_t*)EE_HORN_FUNCTION, 2);
		eeprom_write_byte((uint8_t*)EE_BELL_FUNCTION, 1);
		eeprom_write_byte((uint8_t*)EE_FRONT_DIM1_FUNCTION, OFF_FUNCTION);
		eeprom_write_byte((uint8_t*)EE_FRONT_DIM2_FUNCTION, OFF_FUNCTION);
		eeprom_write_byte((uint8_t*)EE_FRONT_HEADLIGHT_FUNCTION, 0);
		eeprom_write_byte((uint8_t*)EE_FRONT_DITCH_FUNCTION, OFF_FUNCTION);
		eeprom_write_byte((uint8_t*)EE_REAR_DIM1_FUNCTION, OFF_FUNCTION);
		eeprom_write_byte((uint8_t*)EE_REAR_DIM2_FUNCTION, OFF_FUNCTION);
		eeprom_write_byte((uint8_t*)EE_REAR_HEADLIGHT_FUNCTION, 0);
		eeprom_write_byte((uint8_t*)EE_REAR_DITCH_FUNCTION, OFF_FUNCTION);
		eeprom_write_byte((uint8_t*)EE_BRAKE_FUNCTION, 10);
		eeprom_write_byte((uint8_t*)EE_BRAKE_OFF_FUNCTION, OFF_FUNCTION);
		eeprom_write_byte((uint8_t*)EE_AUX_FUNCTION, 9);
		eeprom_write_byte((uint8_t*)EE_ENGINE_ON_FUNCTION, 8);
		eeprom_write_byte((uint8_t*)EE_ENGINE_OFF_FUNCTION, OFF_FUNCTION);
		eeprom_write_byte((uint8_t*)EE_UP_BUTTON_FUNCTION, 5);
		eeprom_write_byte((uint8_t*)EE_DOWN_BUTTON_FUNCTION, 6);
		eeprom_write_byte((uint8_t*)EE_THR_UNLOCK_FUNCTION, OFF_FUNCTION);
		eeprom_write_byte((uint8_t*)EE_REV_SWAP_FUNCTION, OFF_FUNCTION);
		eeprom_write_dword((uint32_t*)EE_FUNC_FORCE_ON, 0);
		eeprom_write_dword((uint32_t*)EE_FUNC_FORCE_OFF, 0);
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
	}

	copyConfig(1, WORKING_CONFIG);  // Grab one of the configs for default
	
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

	uint8_t allowLatch;
	
	uint8_t decimalNumberIndex = 0;
	uint8_t decimalNumber[4];

	uint8_t *functionPtr = &hornFunction;
	uint8_t functionNumber = 0;

	EngineState engineState = ENGINE_OFF;

	uint32_t functionMask = 0;
	uint32_t lastFunctionMask = 0;

	ReverserPosition direction = FORWARD;

	init();

	// Assign after init() so values are read from EEPROM first
	uint8_t newConfigNumber = 1;
	uint16_t newLocoAddress = locoAddress;
	uint8_t newDevAddr = mrbus_dev_addr;
	uint8_t newBaseAddr = mrbus_base_addr;
	uint8_t newTimeAddr = timeSourceAddress;
	uint8_t newSleepTimeout = sleep_tmr_reset_value / 600;
	uint8_t newMaxDeadReckoningTime_seconds = getMaxDeadReckoningTime() / 10;
	uint8_t newUpdate_seconds = update_decisecs / 10;

	uint8_t *addrPtr = &newDevAddr;
	uint8_t *prefsPtr = &newSleepTimeout;
	uint8_t *optionsPtr = &optionBits;

	setXbeeActive();

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		sleepTimeout_decisecs = sleep_tmr_reset_value;
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
		// Keep buffer filled with version packets
		if(!(mrbusPktQueueFull(&mrbeeTxQueue)))
		{
			createVersionPacket(0xFF, txBuffer);
			mrbusPktQueuePush(&mrbeeTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		}

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
				throttleStatus &= ~THROTTLE_STATUS_EMERGENCY;
			if(brakePosition > brakeHighThreshold)
				throttleStatus |= THROTTLE_STATUS_EMERGENCY;
		}
		else
		{
			throttleStatus &= ~THROTTLE_STATUS_EMERGENCY;
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
		if( (optionBits & _BV(OPTIONBITS_REVERSER_SWAP)) ||
			((!(reverserSwapFunction & OFF_FUNCTION)) && (functionMask & ((uint32_t)1 << (reverserSwapFunction & 0x1F)))) )
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
			if( (!(throttleUnlockFunction & OFF_FUNCTION)) && (functionMask & ((uint32_t)1 << (throttleUnlockFunction & 0x1F))) )
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
				
	//				if((OFF_FUNCTION & upButtonFunction) && (OFF_FUNCTION & downButtonFunction))
					if(0)
					{
						printTonnage();
					}
					else
					{
						lcd_gotoxy(7,0);
						lcd_putc((optionButtonState & UP_OPTION_BUTTON) && !(upButtonFunction & OFF_FUNCTION) ? FUNCTION_ACTIVE_CHAR : FUNCTION_INACTIVE_CHAR);
						lcd_gotoxy(7,1);
						lcd_putc((optionButtonState & DOWN_OPTION_BUTTON) && !(downButtonFunction & OFF_FUNCTION) ? FUNCTION_ACTIVE_CHAR : FUNCTION_INACTIVE_CHAR);
					}

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
	//							if((OFF_FUNCTION & upButtonFunction) && (OFF_FUNCTION & downButtonFunction))
								if(0)
								{
									incrementTonnage();
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
	//							if((OFF_FUNCTION & upButtonFunction) && (OFF_FUNCTION & downButtonFunction))
								if(0)
								{
									decrementTonnage();
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
								ticks_autoincrement = 0;  // Reset to zero so a long press can be detected
							}
							if(ticks_autoincrement >= button_autoincrement_10ms_ticks)
							{
								// Trigger power down menu on long press
								subscreenState = 1;
								lcd_clrscr();
							}
							break;
						case NO_BUTTON:
							// Release buttons if momentary
							if(!(upButtonFunction & LATCH_FUNCTION))
								optionButtonState &= ~UP_OPTION_BUTTON;
							if(!(downButtonFunction & LATCH_FUNCTION))
								optionButtonState &= ~DOWN_OPTION_BUTTON;
						case MENU_BUTTON:
							break;
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
				}
				switch(button)
				{
					case UP_BUTTON:
						if(engineStopFunction & OFF_FUNCTION)
						{
							// Level based start/stop
							engineState = ENGINE_ON;
						}
						else
						{
							// Edge triggered start/stop
							// Only send start pulse if in the off state
							if(ENGINE_OFF == engineState)
							{
								engineState = ENGINE_START;
								engineTimer = ENGINE_TIMER_DECISECS;
							}
						}
						break;
					case DOWN_BUTTON:
						if(engineStopFunction & OFF_FUNCTION)
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
						break;
					case SELECT_BUTTON:
						screenState = LAST_SCREEN;
						// Fall through
					case MENU_BUTTON:
						if(ENGINE_NOT_IDLE == engineState)
						{
							// Clear Not Idle state if exiting the menu
							engineState = ENGINE_RUNNING;
						}
					case NO_BUTTON:
						break;
				}
				break;

			case TONNAGE_SCREEN:
				enableLCDBacklight();
				lcd_gotoxy(0,0);
				switch(getTonnage())
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
				switch(getTonnage())
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
							screenState = LAST_SCREEN;
							break;
					case MENU_BUTTON:
					case NO_BUTTON:
						break;
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
						uint16_t tmpConfigOffset = configOffset;  // Save configOffset
						configOffset = calculateConfigOffset(newConfigNumber);
						uint16_t tmpLocoAddress = eeprom_read_word((uint16_t*)EE_LOCO_ADDRESS);  // Read loco address of newConfigNumber
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
						configOffset = tmpConfigOffset;  // Restore old configOffset value
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
								if(LOAD_CONFIG_SCREEN == screenState)
									lcd_puts("LOADING");
								else
									lcd_puts("SAVING");

								// Copy selected config into working config
								if(LOAD_CONFIG_SCREEN == screenState)
								{
									copyConfig(newConfigNumber, WORKING_CONFIG);
								}
								else
								{
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
							if(ticks_autoincrement >= button_autoincrement_10ms_ticks)
							{
								if( (0 == decimalNumberIndex) && (decimalNumber[decimalNumberIndex] > 9) )
									decimalNumber[decimalNumberIndex] = 0;  // Short to Long
								else if(decimalNumber[decimalNumberIndex] < 9)
									decimalNumber[decimalNumberIndex]++;
								
								// Validate number
								if(decimalNumber[0] > 9)
								{
									// Short Address
									if( ((decimalNumber[1] * 100) + (decimalNumber[2] * 10) + decimalNumber[3]) > 127)
									{
										decimalNumber[1] = 1;
										decimalNumber[2] = 2;
										decimalNumber[3] = 7;
									}
								}
								ticks_autoincrement = 0;
							}
							break;
						case DOWN_BUTTON:
							if(ticks_autoincrement >= button_autoincrement_10ms_ticks)
							{
								if( (0 == decimalNumberIndex) && (0 == decimalNumber[decimalNumberIndex]) )
									decimalNumber[decimalNumberIndex] = 's' - '0';  // Long to Short
								else if( (decimalNumber[decimalNumberIndex] > 0) && (decimalNumber[decimalNumberIndex] <= 9))
									decimalNumber[decimalNumberIndex]--;

								// Validate number
								if(decimalNumber[0] > 9)
								{
									// Short Address
									if( ((decimalNumber[1] * 100) + (decimalNumber[2] * 10) + decimalNumber[3]) > 127)
									{
										decimalNumber[1] = 1;
										decimalNumber[2] = 2;
										decimalNumber[3] = 7;
									}
								}
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

			case FUNC_FORCE_SCREEN:
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
						case BRAKE_FN:
							lcd_gotoxy(0,0);
							lcd_puts("BRAKE");
							functionPtr = &brakeFunction;
							break;
						case BRAKE_OFF_FN:
							lcd_gotoxy(0,0);
							lcd_puts("BRK OFF");
							functionPtr = &brakeOffFunction;
							break;
						case AUX_FN:
							lcd_gotoxy(0,0);
							lcd_puts("AUX");
							functionPtr = &auxFunction;
							break;
						case ENGINE_ON_FN:
							lcd_gotoxy(0,0);
							lcd_puts("ENG ON");
							functionPtr = &engineOnFunction;
							break;
						case ENGINE_OFF_FN:
							lcd_gotoxy(0,0);
							lcd_puts("ENG STOP");
							functionPtr = &engineStopFunction;
							break;
						case THR_UNLOCK_FN:
							lcd_gotoxy(0,0);
							lcd_puts("THR UNLK");
							functionPtr = &throttleUnlockFunction;
							break;
						case REV_SWAP_FN:
							lcd_gotoxy(0,0);
							lcd_puts("REV SWAP");
							functionPtr = &reverserSwapFunction;
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
								eeprom_write_byte((uint8_t*)EE_BRAKE_FUNCTION, brakeFunction);
								eeprom_write_byte((uint8_t*)EE_BRAKE_OFF_FUNCTION, brakeOffFunction);
								eeprom_write_byte((uint8_t*)EE_AUX_FUNCTION, auxFunction);
								eeprom_write_byte((uint8_t*)EE_ENGINE_ON_FUNCTION, engineOnFunction);
								eeprom_write_byte((uint8_t*)EE_ENGINE_OFF_FUNCTION, engineStopFunction);
								eeprom_write_byte((uint8_t*)EE_UP_BUTTON_FUNCTION, upButtonFunction);
								eeprom_write_byte((uint8_t*)EE_DOWN_BUTTON_FUNCTION, downButtonFunction);
								eeprom_write_byte((uint8_t*)EE_THR_UNLOCK_FUNCTION, throttleUnlockFunction);
								eeprom_write_byte((uint8_t*)EE_REV_SWAP_FUNCTION, reverserSwapFunction);
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
							if(ticks_autoincrement >= button_autoincrement_10ms_ticks)
							{
								if(notchSpeedStep[notch-1] < 126)
									notchSpeedStep[notch-1]++;
								else
									notchSpeedStep[notch-1] = 126;
								ticks_autoincrement = 0;
							}
							break;
						case DOWN_BUTTON:
							if(ticks_autoincrement >= button_autoincrement_10ms_ticks)
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
							if(ticks_autoincrement >= button_autoincrement_10ms_ticks)
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
							if(ticks_autoincrement >= button_autoincrement_10ms_ticks)
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
						addrPtr = &newDevAddr;
						lcd_gotoxy(4,1);
						lcd_putc('A' + (newDevAddr - MRBUS_DEV_ADDR_MIN));
					}
					else if(2 == subscreenState)
					{
						lcd_puts("BASE ADR");
						addrPtr = &newBaseAddr;
						lcd_gotoxy(3,1);
						printDec2DigWZero(newBaseAddr - MRBUS_BASE_ADDR_MIN);
					}
					else if(3 == subscreenState)
					{
						lcd_puts("TIME ADR");
						addrPtr = &newTimeAddr;
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
					else
					{
						subscreenState = 1;
					}

					switch(button)
					{
						case UP_BUTTON:
							if(ticks_autoincrement >= button_autoincrement_10ms_ticks)
							{
								if(*addrPtr < 0xFF)
									(*addrPtr)++;
								// Check bounds
								if(newDevAddr > MRBUS_DEV_ADDR_MAX)
									newDevAddr = MRBUS_DEV_ADDR_MAX;
								if(newBaseAddr > MRBUS_BASE_ADDR_MAX)
									newBaseAddr = MRBUS_BASE_ADDR_MAX;
								ticks_autoincrement = 0;
							}
							break;
						case DOWN_BUTTON:
							if(ticks_autoincrement >= button_autoincrement_10ms_ticks)
							{
								if(*addrPtr > 0)
									(*addrPtr)--;
								// Check bounds
								if(newDevAddr < MRBUS_DEV_ADDR_MIN)
									newDevAddr = MRBUS_DEV_ADDR_MIN;
								if(newBaseAddr < MRBUS_BASE_ADDR_MIN)
									newBaseAddr = MRBUS_BASE_ADDR_MIN;
								ticks_autoincrement = 0;
							}
							break;
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR, newDevAddr);
								eeprom_write_byte((uint8_t*)EE_BASE_ADDR, newBaseAddr);
								eeprom_write_byte((uint8_t*)EE_TIME_SOURCE_ADDRESS, newTimeAddr);
								readConfig();
								newDevAddr = mrbus_dev_addr;
								newBaseAddr = mrbus_base_addr;
								newTimeAddr = timeSourceAddress;
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
					if(1 == subscreenState)
					{
						lcd_puts("SLEEP");
						lcd_gotoxy(0,1);
						lcd_puts("DLY:");
						lcd_gotoxy(7,1);
						lcd_puts("M");
						bitPosition = 0xFF;
						prefsPtr = &newSleepTimeout;
					}
					else if(2 == subscreenState)
					{
						lcd_puts("TIMEOUT");
						lcd_gotoxy(0,1);
						lcd_puts("CLK:");
						lcd_gotoxy(7,1);
						lcd_puts("s");
						bitPosition = 0xFF;
						prefsPtr = &newMaxDeadReckoningTime_seconds;
					}
					else if(3 == subscreenState)
					{
						lcd_puts("TX INTVL");
						lcd_gotoxy(7,1);
						lcd_puts("s");
						bitPosition = 0xFF;
						prefsPtr = &newUpdate_seconds;
					}
					else if(4 == subscreenState)
					{
						lcd_puts("TX HLDOF");
						lcd_gotoxy(7,1);
						lcd_puts("s");
						bitPosition = 0xFF;
						prefsPtr = &txHoldoff_centisecs;
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
					else if(prefsPtr == &txHoldoff_centisecs)
					{
						lcd_gotoxy(3,1);
						lcd_putc('0' + (*prefsPtr) / 100);
						lcd_putc('.');
						lcd_putc('0' + ((*prefsPtr)/10) % 10);
						lcd_putc('0' + (*prefsPtr) % 10);
					}
					else
					{
						lcd_gotoxy(4,1);
						printDec3Dig(*prefsPtr);
					}

					
					switch(button)
					{
						case UP_BUTTON:
							if(ticks_autoincrement >= button_autoincrement_10ms_ticks)
							{
								if(bitPosition < 8)
								{
									*prefsPtr |= _BV(bitPosition);
								}
								else if( ((prefsPtr != &newUpdate_seconds)&&(prefsPtr != &txHoldoff_centisecs)) || (systemBits & _BV(SYSTEMBITS_ADV_FUNC)) )
								{
									if(*prefsPtr < 0xFF)
										(*prefsPtr)++;
									if(newUpdate_seconds > UPDATE_DECISECS_MAX / 10)
										newUpdate_seconds = UPDATE_DECISECS_MAX / 10;
									if(newSleepTimeout > SLEEP_TMR_RESET_VALUE_MAX)
										newSleepTimeout = SLEEP_TMR_RESET_VALUE_MAX;
									if(newMaxDeadReckoningTime_seconds > DEAD_RECKONING_TIME_MAX / 10)
										newMaxDeadReckoningTime_seconds = DEAD_RECKONING_TIME_MAX / 10;
									ticks_autoincrement = 0;
								}
							}
							break;
						case DOWN_BUTTON:
							if(ticks_autoincrement >= button_autoincrement_10ms_ticks)
							{
								if(bitPosition < 8)
								{
									*prefsPtr &= ~_BV(bitPosition);
								}
								else if( ((prefsPtr != &newUpdate_seconds)&&(prefsPtr != &txHoldoff_centisecs)) || (systemBits & _BV(SYSTEMBITS_ADV_FUNC)) )
								{
									if(*prefsPtr > 1)
										(*prefsPtr)--;
									if(txHoldoff_centisecs < TX_HOLDOFF_MIN)
										txHoldoff_centisecs = TX_HOLDOFF_MIN;
									if(newSleepTimeout < SLEEP_TMR_RESET_VALUE_MIN)
										newSleepTimeout = SLEEP_TMR_RESET_VALUE_MIN;
									if(newMaxDeadReckoningTime_seconds < DEAD_RECKONING_TIME_MIN / 10)
										newMaxDeadReckoningTime_seconds = DEAD_RECKONING_TIME_MIN / 10;
									ticks_autoincrement = 0;
								}
							}
							break;
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								eeprom_write_byte((uint8_t*)EE_DEVICE_SLEEP_TIMEOUT, newSleepTimeout);
								eeprom_write_byte((uint8_t*)EE_DEAD_RECKONING_TIME, newMaxDeadReckoningTime_seconds * 10);
								eeprom_write_byte((uint8_t*)EE_CONFIGBITS, configBits);
								eeprom_write_byte((uint8_t*)EE_TX_HOLDOFF, txHoldoff_centisecs);
								update_decisecs = (uint16_t)newUpdate_seconds * 10;
								eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_H, update_decisecs >> 8);
								eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_L, update_decisecs & 0xFF);
								readConfig();
								// The only way to escape the prefs menu is by saving the values, so the new* variables don't serve the purpose of allowing the user to cancel a change in this case.
								// new* values are used because the values used in the program are not the same format as used here.
								newSleepTimeout = sleep_tmr_reset_value / 600;
								newMaxDeadReckoningTime_seconds = getMaxDeadReckoningTime() / 10;
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
							if(ticks_autoincrement >= button_autoincrement_10ms_ticks)
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
							if(ticks_autoincrement >= button_autoincrement_10ms_ticks)
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
								setupDiagChars();
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
						printDec4Dig(decisecs_tmp/10);
						lcd_gotoxy(5,1);
						lcd_puts("sec");
					}
					else if(3 == subscreenState)
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
					else if(4 == subscreenState)
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
					else if(5 == subscreenState)
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
					else if(6 == subscreenState)
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
					else if(7 == subscreenState)
					{
						enableLCDBacklight();
						lcd_gotoxy(0,0);
						lcd_puts("VERSION");
						lcd_gotoxy(0,1);
						lcd_puts(VERSION_STRING);
					}
					else if(8 == subscreenState)
					{
						enableLCDBacklight();
						lcd_gotoxy(0,0);
						lcd_puts("GIT REV");
						lcd_gotoxy(1,1);
						printHex((GIT_REV >> 16) & 0xFF);
						printHex((GIT_REV >> 8) & 0xFF);
						printHex(GIT_REV & 0xFF);
					}
					else if(9 == subscreenState)
					{
						enableLCDBacklight();
						lcd_gotoxy(0,0);
						lcd_puts("BASE TYP");
						lcd_gotoxy(0,1);
						lcd_puts(baseString);
					}
					else if(10 == subscreenState)
					{
						enableLCDBacklight();
						lcd_gotoxy(0,0);
						lcd_puts("BASE REV");
						lcd_gotoxy(1,1);
						printHex((baseVersion >> 16) & 0xFF);
						printHex((baseVersion >> 8) & 0xFF);
						printHex(baseVersion & 0xFF);
					}
					else if(11 == subscreenState)
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
								setupClockChars();   // Restore clock characters
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
								
								if(subscreenCount < 3)
								{
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
								(TONNAGE_SCREEN != screenState) &&
								(LOAD_CONFIG_SCREEN != screenState) &&
								(LOCO_SCREEN != screenState) &&
								(FUNC_FORCE_SCREEN != screenState) &&
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
		if((controls & HORN_CONTROL) && !(hornFunction & OFF_FUNCTION))
			functionMask |= (uint32_t)1 << (hornFunction & 0x1F);
		if((controls & BELL_CONTROL) && !(bellFunction & OFF_FUNCTION))
			functionMask |= (uint32_t)1 << (bellFunction & 0x1F);
		if((controls & AUX_CONTROL) && !(auxFunction & OFF_FUNCTION))
			functionMask |= (uint32_t)1 << (auxFunction & 0x1F);
		if((controls & BRAKE_CONTROL) && !(brakeFunction & OFF_FUNCTION))
			functionMask |= (uint32_t)1 << (brakeFunction & 0x1F);
		if((controls & BRAKE_OFF_CONTROL) && !(brakeOffFunction & OFF_FUNCTION))
			functionMask |= (uint32_t)1 << (brakeOffFunction & 0x1F);
		if(((ENGINE_ON == engineState)||(ENGINE_START) == engineState) && !(engineOnFunction & OFF_FUNCTION))
			functionMask |= (uint32_t)1 << (engineOnFunction & 0x1F);
		if((ENGINE_STOP == engineState) && !(engineStopFunction & OFF_FUNCTION))
			functionMask |= (uint32_t)1 << (engineStopFunction & 0x1F);
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

		// Force specific functions on or off
		functionMask |= functionForceOn;
		functionMask &= ~functionForceOff;

		uint8_t inputsChanged =	(activeReverserSetting != lastActiveReverserSetting) ||
									(activeThrottleSetting != lastActiveThrottleSetting) ||
									(functionMask != lastFunctionMask) ||
									(throttleStatus != lastThrottleStatus);

		// Reset sleep timer
		// Using activeReverserSetting also guarantees the throttle (activeThrottleSetting) was in idle when entering sleep, so it will unsleep in the idle position.
		if( 
			(NO_BUTTON != button) || 
			(FORWARD == activeReverserSetting) || 
			(REVERSE == activeReverserSetting) ||
			inputsChanged
			)
		{
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
			{
				sleepTimeout_decisecs = sleep_tmr_reset_value;
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
				setupDiagChars();
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
			}
			throttleStatus &= ~THROTTLE_STATUS_SLEEP;
		}

	}

}


