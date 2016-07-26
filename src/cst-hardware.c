#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <util/delay.h>

#include "cst-hardware.h"

volatile uint8_t reverserPot = 0;
volatile uint8_t throttlePot = 0;
volatile uint8_t batteryVoltage = 0;
volatile uint8_t brakePot = 0;
volatile LightPosition frontLight = LIGHT_OFF;
volatile LightPosition rearLight = LIGHT_OFF;
volatile uint8_t frontLightPot = 0;
volatile uint8_t rearLightPot = 0;

volatile LEDStatus led;


void initPorts()
{
	// DDR = 0 - Input / 1 - Output

	// Initialize ports 
	// Pin Assignments for PORTA/DDRA
	//  PA0 - Analog - VREV
	//  PA1 - Analog - VBRAKE
	//  PA2 - Analog - VLIGHT_F
	//  PA3 - Analog - VLIGHT_R
	//  PA4 - Output - Xbee SLEEP_EN
	//  PA5 - Output - LCD Backlight Enable
	//  PA6 - Output - LCD Power Enable (inverted)
	//  PA7 - Analog - VBATTERY
	DDRA  = 0b01110000;
	PORTA = 0b00000000;

	// Pin Assignments for PORTB/DDRB
	//  PB0 - Input  - Horn
	//  PB1 - Input  - Dynamic Brake
	//  PB2 - Input  - Bell
	//  PB3 - Output - N/C
	//  PB4 - Input  - Softkey 0
	//  PB5 - Input  - Softkey 1
	//  PB6 - Input  - Softkey 2
	//  PB7 - Input  - Softkey 3
	DDRB  = 0b00001000;
	PORTB = 0b11110111;  // Pullups on for all buttons

	// Pin Assignments for PORTC/DDRC
	//  PC0 - Output - Reverser Enable
	//  PC1 - Output - Brake Enable
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

	DIDR0 = _BV(ANALOG_VREV) | _BV(ANALOG_VBRAKE) | _BV(ANALOG_VLIGHT_F) | _BV(ANALOG_VLIGHT_R) | _BV(ANALOG_VBATT);

}

void initADC()
{
	PRR0 &= ~_BV(PRADC);
	
	// Setup ADC
	ADMUX  = _BV(REFS0); // AVCC reference, ADC7 channel
	ADCSRA = _BV(ADATE) | _BV(ADIF) | _BV(ADPS2) | _BV(ADPS1); // 128 prescaler
	ADCSRB = 0x00;
	throttlePot = 0;
	ADCSRA |= _BV(ADEN) | _BV(ADSC) | _BV(ADIE) | _BV(ADIF);
}

void initThrottle()
{
	enableThrottle();
	EICRA = _BV(ISC10) | _BV(ISC00);
	EIMSK = _BV(INT1) | _BV(INT0);
}

typedef enum
{
	ADC_STATE_START_VREV  = 0,
	ADC_STATE_READ_VREV,
	ADC_STATE_START_VBRAKE,
	ADC_STATE_READ_VBRAKE,
	ADC_STATE_START_VLIGHT_F,
	ADC_STATE_READ_VLIGHT_F,
	ADC_STATE_START_VLIGHT_R,
	ADC_STATE_READ_VLIGHT_R,
	ADC_STATE_START_VBATT,
	ADC_STATE_READ_VBATT
} ADCState;

ISR(ADC_vect)
{
	static uint16_t accumulator=0;
	static uint8_t count=0;
	static ADCState state=ADC_STATE_START_VREV;
	uint8_t i;

	accumulator += ADC;

	switch(state)
	{
		// All of the "start" cases throw away the first conversion, as it starts
		// with the wrong mux settings and therefore will
		// actually be reading the wrong input
		case ADC_STATE_START_VREV:
			accumulator = 0;
			count = 0;
			state = ADC_STATE_READ_VREV;
			return;

		case ADC_STATE_START_VBRAKE:
			accumulator = 0;
			count = 0;
			state = ADC_STATE_READ_VBRAKE;
			return;

		case ADC_STATE_START_VLIGHT_F:
			accumulator = 0;
			count = 0;
			state = ADC_STATE_READ_VLIGHT_F;
			return;

		case ADC_STATE_START_VLIGHT_R:
			accumulator = 0;
			count = 0;
			state = ADC_STATE_READ_VLIGHT_R;
			return;

		case ADC_STATE_START_VBATT:
			accumulator = 0;
			count = 0;
			state = ADC_STATE_READ_VBATT;
			return;
		

		case ADC_STATE_READ_VREV:
		case ADC_STATE_READ_VBRAKE:
		case ADC_STATE_READ_VLIGHT_F:
		case ADC_STATE_READ_VLIGHT_R:
		case ADC_STATE_READ_VBATT:
			count++;
			break;

		default:
			// Eh, what are we doing here?
			ADMUX  = _BV(REFS0) | 0x00; // AVCC reference, ADC7 channel (throttle pot)
			count = 0;
			state = ADC_STATE_START_VREV;
			break;
	
	}

	if (count >= 64)
	{
		switch(state)
		{

			case ADC_STATE_READ_VREV:
				ADMUX  = _BV(REFS0) | ANALOG_VBRAKE; // AVCC reference, ADC6 channel (battery voltage)
				enableBrakePot();
				disableReverserPot();
				reverserPot = accumulator >> 8;
				state = ADC_STATE_START_VBRAKE;
				break;

			case ADC_STATE_READ_VBRAKE:
				ADMUX  = _BV(REFS0) | ANALOG_VLIGHT_F;
				enableLightSwitches();
				disableBrakePot();
				brakePot = accumulator >> 8;
				state = ADC_STATE_START_VLIGHT_F;
				break;

			case ADC_STATE_READ_VLIGHT_F:
				ADMUX  = _BV(REFS0) | ANALOG_VLIGHT_R;
				state = ADC_STATE_START_VLIGHT_R;
				frontLightPot = (accumulator >> 9);
				if (frontLightPot > 96)
					frontLight = LIGHT_OFF;
				else if (frontLightPot > 64)
					frontLight = LIGHT_DIM;
				else if (frontLightPot > 32)
					frontLight = LIGHT_BRIGHT;
				else 
					frontLight = LIGHT_BRIGHT_DITCH;
				
				break;

			case ADC_STATE_READ_VLIGHT_R:
				ADMUX  = _BV(REFS0) | ANALOG_VBATT;
				disableLightSwitches();
				state = ADC_STATE_START_VBATT;
				rearLightPot = (accumulator >> 9);
				if (rearLightPot > 96)
					rearLight = LIGHT_OFF;
				else if (rearLightPot > 64)
					rearLight = LIGHT_DIM;
				else if (rearLightPot > 32)
					rearLight = LIGHT_BRIGHT;
				else 
					rearLight = LIGHT_BRIGHT_DITCH;

				break;

			case ADC_STATE_READ_VBATT:
				// Measuring battery voltage
				// In 20mV increments
				accumulator >>= 6; // Divide by 64 - get the average measurement
				batteryVoltage = (uint8_t)(accumulator / 8);			

				// Intentional fallthrough
				
			case ADC_STATE_START_VREV:
			case ADC_STATE_START_VBRAKE:
			case ADC_STATE_START_VLIGHT_F:
			case ADC_STATE_START_VLIGHT_R:
			case ADC_STATE_START_VBATT:
			default:  // These are all "what the heck are we doing here? cases
				enableReverserPot();
				ADMUX  = _BV(REFS0) | ANALOG_VREV;
				state = ADC_STATE_START_VREV;
				break;		
		}

		accumulator = 0;
		count = 0;
	}
}


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
			ledPhase = 0;
			break;

	}
}





