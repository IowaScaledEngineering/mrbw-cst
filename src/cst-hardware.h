#ifndef _CST_HARDWARE_H_
#define _CST_HARDWARE_H_

#define AUX_PIN        PB2
#define BELL_PIN       PB1
#define MENU_PIN       PB5
#define SELECT_PIN     PB4
#define UP_PIN         PB6
#define DOWN_PIN       PB7

#define LCD_POWER      PA6
#define LCD_BACKLIGHT  PA5

#define REVERSER_ENABLE PC0
#define POTS_ENABLE     PC1
#define LIGHTSW_ENABLES PD5
#define THROTTLE_ENABLE PD4

#define ANALOG_VREV     PA0
#define ANALOG_VBRAKE   PA1
#define ANALOG_VHORN    PA4
#define ANALOG_VLIGHT_R PA2
#define ANALOG_VLIGHT_F PA3
#define ANALOG_VBATT    PA7

#define LED_RED_PIN     PD6
#define LED_GREEN_PIN   PD7

#define XBEE_SLEEP_DDR  DDRB
#define XBEE_SLEEP_PORT PORTB
#define XBEE_SLEEP      0

void initPorts();

typedef enum
{
	NEUTRAL = 0,
	FORWARD,
	REVERSE
} ReverserPosition;

extern ReverserPosition reverserPosition;

extern uint8_t brakePosition;
extern uint8_t hornPosition;
extern volatile uint8_t throttlePosition;

typedef enum
{
	LIGHT_OFF = 0,
	LIGHT_DIM = 1,
	LIGHT_BRIGHT = 2,	
	LIGHT_BRIGHT_DITCH = 3,	

} LightPosition;

extern uint8_t frontLightPot;
extern LightPosition frontLight;
extern uint8_t rearLightPot;
extern LightPosition rearLight;

void initADC();
void processADC();

uint8_t adcLoopInitialized(void);

typedef enum
{
	LED_OFF = 0x00,
	LED_GREEN,
	LED_RED,
	LED_YELLOW,
	LED_GREEN_SLOWBLINK,
	LED_GREEN_FASTBLINK,
	LED_RED_SLOWBLINK,
	LED_RED_FASTBLINK,
	LED_YELLOW_SLOWBLINK,
	LED_YELLOW_FASTBLINK

} LEDStatus;

extern volatile LEDStatus led;

	//  PD6 - Output - LED Red
	//  PD7 - Output - LED Green
inline void ledGreenOff()
{
	PORTD &= ~_BV(LED_GREEN_PIN);
}
inline void ledGreenOn()
{
	PORTD |= _BV(LED_GREEN_PIN);
}
inline void ledRedOff()
{
	PORTD &= ~_BV(LED_RED_PIN);
}
inline void ledRedOn()
{
	PORTD |= _BV(LED_RED_PIN);
}

inline void enableSwitches()
{
	PORTB |= 0b00000110;  // Pullups on
}

inline void disableSwitches()
{
	PORTB &= ~(0b00000110);  // Pullups off
}

inline void buttonsEnable()
{
	PORTB |= 0b11110000;  // Pullups on
}

inline void buttonsDisable()
{
	PORTB &= ~(0b11110000);  // Pullups off
}

inline void enableLCD()
{
	PORTA &= ~_BV(LCD_POWER);
}

inline void disableLCD()
{
	PORTA |= _BV(LCD_POWER);
	PORTC &= ~(0xFC);  // Set LCD lines low
}

inline void enableLCDBacklight()
{
	PORTA |= _BV(LCD_BACKLIGHT);
}

inline void disableLCDBacklight()
{
	PORTA &= ~_BV(LCD_BACKLIGHT);
}

inline void enablePots()
{
	PORTC |= _BV(POTS_ENABLE);
}

inline void disablePots()
{
	PORTC &= ~_BV(POTS_ENABLE);
}

inline void enableReverser()
{
	PORTC |= _BV(REVERSER_ENABLE);
}

inline void disableReverser()
{
	PORTC &= ~_BV(REVERSER_ENABLE);
}

inline void enableLightSwitches()
{
	PORTD |= _BV(LIGHTSW_ENABLES);
}

inline void disableLightSwitches()
{
	PORTD &= ~_BV(LIGHTSW_ENABLES);
}

inline void enableADC()
{
	ADCSRA |= _BV(ADEN) | _BV(ADSC);
}

inline void disableADC()
{
	ADCSRA &= ~_BV(ADEN);
}

inline void enableTimer()
{
	TIMSK0 |= _BV(OCIE0A);
}

inline void disableTimer()
{
	TIMSK0 &= ~_BV(OCIE0A);
}

inline void setXbeeSleep()
{
	XBEE_SLEEP_PORT |= _BV(XBEE_SLEEP);
}

inline void setXbeeActive()
{
	// Unsleep the XBee
	XBEE_SLEEP_PORT &= ~_BV(XBEE_SLEEP);
}

void enableThrottle();
void disableThrottle();

void ledUpdate();

#endif
