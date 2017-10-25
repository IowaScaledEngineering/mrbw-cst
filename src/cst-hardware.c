/*************************************************************************
Title:    Misc. hardware support for Control Stand Throttle
Authors:  Michael D. Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     cst-hardware.c
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

#include "lcd.h"

#include "cst-common.h"
#include "cst-hardware.h"
#include "cst-battery.h"

// Port and IO stuff
void initPorts()
{
	// DDR = 0 - Input / 1 - Output

	// Initialize ports 
	// Pin Assignments for PORTA/DDRA
	//  PA0 - Analog - VREV
	//  PA1 - Analog - VBRAKE
	//  PA2 - Analog - VLIGHT_R
	//  PA3 - Analog - VLIGHT_F
	//  PA4 - Analog - VHORN
	//  PA5 - Output - LCD Backlight Enable
	//  PA6 - Output - LCD Power Enable (inverted)
	//  PA7 - Analog - VBATTERY
	DDRA  = 0b01100000;
	PORTA = 0b00000000;

	// Pin Assignments for PORTB/DDRB
	//  PB0 - Output - XBee SLEEP_EN
	//  PB1 - Input  - Bell
	//  PB2 - Input  - Dynamic Brake
	//  PB3 - Output - N/C
	//  PB4 - Input  - Softkey 0
	//  PB5 - Input  - Softkey 1
	//  PB6 - Input  - Softkey 2
	//  PB7 - Input  - Softkey 3
	DDRB  = 0b00001001;
	buttonsEnable();
	enableSwitches();

	// Pin Assignments for PORTC/DDRC
	//  PC0 - Output - Reverser Enable
	//  PC1 - Output - Pots Enable
	//  PC2 - Output - LCD RS
	//  PC3 - Output - LCD E
	//  PC4 - Output - LCD D4
	//  PC5 - Output - LCD D5
	//  PC6 - Output - LCD D6
	//  PC7 - Output - LCD D7
	DDRC  = 0b11111111;
	PORTC = 0b00000000;

	// Pin Assignments for PORTC/DDRC
	//  PD0 - Input  - XBEE RX
	//  PD1 - Output - XBEE TX
	//  PD2 - Input  - Throttle A Ph
	//  PD3 - Input  - Throttle B Ph
	//  PD4 - Output - Throttle Enable
	//  PD5 - Output - Light Switches Enable
	//  PD6 - Output - LED Red
	//  PD7 - Output - LED Green
	DDRD  = 0b11110000;
	PORTD = 0b00000000;
}




// Battery stuff
const uint8_t BatteryFull[8] =
{
	0b00001110,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00000000
};

const uint8_t BatteryHalf[8] =
{
	0b00001110,
	0b00011011,
	0b00010001,
	0b00010001,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00000000
};
const uint8_t BatteryEmpty[8] =
{
	0b00001110,
	0b00011011,
	0b00010001,
	0b00010001,
	0b00010001,
	0b00010001,
	0b00011111,
	0b00000000
};

BatteryState batteryState = FULL;
BatteryState lastBatteryState = FULL;
uint8_t batteryVoltage = 0;

// VBATT_OKAY is the battery voltage (in centivolts) above which the batteries are considered "fine"
#define VBATT_OKAY  220 
// VBATT_WARN is the battery voltage (in centivolts) above which the batteries are considered a warning, but not
//  in critical shape.  This should *always* be less than VBATT_OKAY
#define VBATT_WARN  200

void setupBatteryChar(void)
{
	switch(batteryState)
	{
		case FULL:
			lcd_setup_custom(BATTERY_CHAR, BatteryFull);
			break;
		case HALF:
			lcd_setup_custom(BATTERY_CHAR, BatteryHalf);
			break;
		case EMPTY:
			lcd_setup_custom(BATTERY_CHAR, BatteryEmpty);
			break;
	}
}

uint8_t getBatteryVoltage(void)
{
	return batteryVoltage;
}

void printBattery(void)
{
	if(batteryState != lastBatteryState)
	{
		setupBatteryChar();
		lastBatteryState = batteryState;
	}

	lcd_gotoxy(0,0);
	lcd_putc(BATTERY_CHAR);
}



// LED Control
void ledUpdate()
{
	static uint8_t ledPhase = 0;
	switch(++ledPhase)
	{
		case 1:
		case 3:
			switch(led)
			{
				case LED_GREEN_FASTBLINK:
					ledGreenOn();
					break;
		
				case LED_RED_FASTBLINK:			
					ledRedOn();
					break;

				case LED_OFF:
					ledRedOff();
					ledGreenOff();
				default:
					break;
			}
			break;

		case 2:
			switch(led)
			{
				case LED_GREEN:
				case LED_GREEN_SLOWBLINK:
					ledGreenOn();
					break;
				case LED_GREEN_FASTBLINK:
					ledGreenOff();
					break;
				case LED_RED:
				case LED_RED_SLOWBLINK:
					ledRedOn();
					break;
				case LED_RED_FASTBLINK:			
					ledRedOff();
					break;
				default:
					break;

			}
			break;

		case 6:
			switch(led)
			{
				case LED_GREEN_SLOWBLINK:
					ledGreenOn();
					break;
				case LED_RED_SLOWBLINK:
					ledRedOn();
					break;
				default:
					break;

			}
			break;

		case 4:
		case 8:
			switch(led)
			{
				case LED_GREEN:
				case LED_GREEN_SLOWBLINK:
				case LED_GREEN_FASTBLINK:
					ledGreenOff();
					break;

				case LED_RED:
				case LED_RED_SLOWBLINK:
				case LED_RED_FASTBLINK:			
					ledRedOff();
					break;
				default:
					break;
					
			}
			break;

		case 10:
			ledGreenOff();
			ledRedOff();
//			ledPhase = 0;
			break;
		
		case 20:
			ledPhase = 0;
			break;

	}
}



// Throttle Stuff
volatile uint8_t throttlePosition = 0;
volatile uint8_t throttleQuadrature;

void enableThrottle(void)
{
	EIMSK = 0;  // Turn off interrupts while configuring
	EICRA |= _BV(ISC10) | _BV(ISC00);  // Enable any edge on INT0 and INT1
	PORTD |= _BV(THROTTLE_ENABLE);  // Turn on the pull-ups
	_delay_ms(1);  // Measured 20us rise time
	throttleQuadrature = (PIND & (_BV(PD2) | _BV(PD3)))>>2;  // Get an initial read
	EIMSK = _BV(INT1) | _BV(INT0);  // Enable INT0 and INT1 interrupts
}

void disableThrottle(void)
{
	EIMSK = 0;  // Turn off interrupts
	PORTD &= ~_BV(THROTTLE_ENABLE);  // Disable pull-ups
}

ISR(INT0_vect)
{
	uint8_t newQuadrature;
	
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

ISR(INT1_vect, ISR_ALIASOF(INT0_vect));




// ADC Stuff
typedef enum
{
	ADC_STATE_START_VREV = 0,
	ADC_STATE_READ_VREV,
	ADC_STATE_START_VBRAKE,
	ADC_STATE_READ_VBRAKE,
	ADC_STATE_START_VHORN,
	ADC_STATE_READ_VHORN,
	ADC_STATE_START_VLIGHT_F,
	ADC_STATE_READ_VLIGHT_F,
	ADC_STATE_START_VLIGHT_R,
	ADC_STATE_READ_VLIGHT_R,
	ADC_STATE_START_VBATT,
	ADC_STATE_READ_VBATT,
	ADC_STATE_LAST  // Must be the last state
} ADCState;

volatile LEDStatus led;

ReverserPosition reverserPosition = NEUTRAL;

uint8_t brakePosition = 0;
uint8_t hornPosition = 0;

uint8_t frontLightValue = 0;
LightPosition frontLight = LIGHT_OFF;
uint8_t rearLightValue = 0;
LightPosition rearLight = LIGHT_OFF;

uint8_t adcLoopCount = 0;

void initADC()
{
	PRR0 &= ~_BV(PRADC);

	// Disable digital buffer on analog ports
	DIDR0 = _BV(ANALOG_VREV) | _BV(ANALOG_VBRAKE) | _BV(ANALOG_VLIGHT_F) | _BV(ANALOG_VLIGHT_R) | _BV(ANALOG_VBATT);
	
	// Setup ADC
	ADMUX  = _BV(REFS0); // AVCC reference, ADC7 channel
	ADCSRA = _BV(ADATE) | _BV(ADIF) | _BV(ADPS2) | _BV(ADPS1); // 128 prescaler
	ADCSRB = 0x00;
	ADCSRA |= _BV(ADSC) | _BV(ADIE) | _BV(ADIF);
	
	adcLoopCount = 0;
}

volatile uint16_t adcAccumulator = 0;
volatile uint8_t adcCount = 0;

ISR(ADC_vect)
{
	adcAccumulator += ADC;
	adcCount++;

	if (adcCount >= 64)
	{
		// Turn off ADC after 64 samples
		disableADC();
	}
}

void startADC(uint8_t mux)
{
	ADMUX  = _BV(REFS0) | mux;
	adcAccumulator = 0;
	adcCount = 0;
	enableADC();
}

void processADC()
{
	static ADCState adcState = 0;
	
	if(!(ADCSRA & _BV(ADEN)))
	{
		// Only process ADC if ADC not running
/*		int16_t delta = 0;*/
		switch(adcState)
		{
			case ADC_STATE_START_VREV:
				enableReverser();
				startADC(ANALOG_VREV);
				adcState++;
				break;

			case ADC_STATE_READ_VREV:
				adcAccumulator >>= 8;
				if(adcAccumulator < 0x30)
				{
					reverserPosition = REVERSE;
				}
				else if(adcAccumulator < 0x70)
				{
					reverserPosition = NEUTRAL;
				}
				else
				{
					reverserPosition = FORWARD;
				}
				adcState++;
				break;

			case ADC_STATE_START_VBRAKE:
				enablePots();
				startADC(ANALOG_VBRAKE);
				adcState++;
				break;

			case ADC_STATE_READ_VBRAKE:
				brakePosition = adcAccumulator >> 8;
				adcState++;
				break;

			case ADC_STATE_START_VHORN:
				enablePots();
				startADC(ANALOG_VHORN);
				adcState++;
				break;

			case ADC_STATE_READ_VHORN:
				hornPosition = 255 - (adcAccumulator >> 8);
				adcState++;
				break;

			case ADC_STATE_START_VLIGHT_F:
				enableLightSwitches();
				startADC(ANALOG_VLIGHT_F);
				adcState++;
				break;

			case ADC_STATE_READ_VLIGHT_F:
				frontLightValue = (adcAccumulator >> 8);
				if (frontLightValue > 160)
					frontLight = LIGHT_OFF;
				else if (frontLightValue > 96)
					frontLight = LIGHT_DIM;
				else if (frontLightValue > 32)
					frontLight = LIGHT_BRIGHT;
				else 
					frontLight = LIGHT_BRIGHT_DITCH;
/*				delta = (int16_t)(adcAccumulator >> 9) - (int16_t)frontLightValue;*/
/*				if(delta > 64)  // Rate-of-change clamping determined experimentally*/
/*					delta = 64;*/
/*				else if(delta < -32)*/
/*					delta = -32;*/
/*				frontLightValue = ((delta + ((int16_t)frontLightValue << 3)) >> 3);*/
/*				if (frontLightValue > 106)*/
/*					frontLight = LIGHT_OFF;*/
/*				else if (frontLightValue > 60)*/
/*					frontLight = LIGHT_DIM;*/
/*				else if (frontLightValue > 20)*/
/*					frontLight = LIGHT_BRIGHT;*/
/*				else */
/*					frontLight = LIGHT_BRIGHT_DITCH;*/
				adcState++;
				break;

			case ADC_STATE_START_VLIGHT_R:
				enableLightSwitches();
				startADC(ANALOG_VLIGHT_R);
				adcState++;
				break;

			case ADC_STATE_READ_VLIGHT_R:
				rearLightValue = (adcAccumulator >> 8);
				if (rearLightValue > 160)
					rearLight = LIGHT_OFF;
				else if (rearLightValue > 96)
					rearLight = LIGHT_DIM;
				else if (rearLightValue > 32)
					rearLight = LIGHT_BRIGHT;
				else 
					rearLight = LIGHT_BRIGHT_DITCH;
/*				delta = (int16_t)(adcAccumulator >> 9) - (int16_t)rearLightValue;*/
/*				if(delta > 64)  // Rate-of-change clamping determined experimentally*/
/*					delta = 64;*/
/*				else if(delta < -32)*/
/*					delta = -32;*/
/*				rearLightValue = ((delta + ((int16_t)rearLightValue << 3)) >> 3);*/
/*				if (rearLightValue > 106)*/
/*					rearLight = LIGHT_OFF;*/
/*				else if (rearLightValue > 60)*/
/*					rearLight = LIGHT_DIM;*/
/*				else if (rearLightValue > 20)*/
/*					rearLight = LIGHT_BRIGHT;*/
/*				else */
/*					rearLight = LIGHT_BRIGHT_DITCH;*/
				adcState++;
				break;

			case ADC_STATE_START_VBATT:
				ADMUX  = _BV(REFS0) | ANALOG_VBATT;
				startADC(ANALOG_VBATT);
				adcState++;
				break;

			case ADC_STATE_READ_VBATT:
				// Measuring battery voltage
				// In 20mV increments
				adcAccumulator >>= 6; // Divide by 64 - get the average measurement
				batteryVoltage = (uint8_t)((adcAccumulator * 5) / 31);
				if (batteryVoltage >= (VBATT_OKAY/2))  // Divide by 2 since batteryVoltage LSB = 20mV
					batteryState = FULL;
				else if (batteryVoltage >= (VBATT_WARN/2))  // Divide by 2 since batteryVoltage LSB = 20mV
					batteryState = HALF;
				else
					batteryState = EMPTY;
				adcState++;
				break;

			case ADC_STATE_LAST:
				if(adcLoopCount < 255)
					adcLoopCount++;  // Saturate at 255
				adcState = 0;
				break;		
		}
	}
}

uint8_t adcLoopInitialized(void)
{
	// Make sure all the ADC averaging is settled
	if(adcLoopCount > 32)
		return(1);
	else
		return(0);
}


