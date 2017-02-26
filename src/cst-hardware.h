#ifndef _CST_HARDWARE_H_
#define _CST_HARDWARE_H_

#define DYNAMIC_BIT    0x04
#define BELL_BIT       0x02
#define MENU_BIT       0x20
#define SELECT_BIT     0x10
#define UP_BIT         0x40
#define DOWN_BIT       0x80

#define LCD_POWER      PA6
#define LCD_BACKLIGHT  PA5

#define ENABLE_REVERSER PC0
#define ENABLE_POTS     PC1
#define ENABLE_LIGHTSWS PD5
#define ENABLE_THROTTLE PD4

#define ANALOG_VREV     PA0
#define ANALOG_VBRAKE   PA1
#define ANALOG_VHORN    PA4
#define ANALOG_VLIGHT_R PA2
#define ANALOG_VLIGHT_F PA3
#define ANALOG_VBATT    PA7

#define LED_RED_PIN     PD6
#define LED_GREEN_PIN   PD7


typedef enum
{
	NEUTRAL = 0,
	FORWARD,
	REVERSE
} ReverserPosition;

extern ReverserPosition reverserPosition;

extern uint8_t brakePosition;
extern uint8_t hornPosition;

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

extern uint8_t batteryVoltage;

void initADC();
void processADC();

void initPorts();

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

inline void buttonsEnable()
{
	PORTB = 0b11110110;  // Pullups on for all buttons
}

inline void buttonsDisable()
{
	PORTB = 0b00000000;  // Pullups off for all buttons
}

inline void lcdEnable()
{
	PORTA &= ~_BV(LCD_POWER);
}

inline void lcdDisable()
{
	PORTA |= ~_BV(LCD_POWER);
}

inline void lcdBacklightEnable()
{
	PORTA |= _BV(LCD_BACKLIGHT);
}

inline void lcdBacklightDisable()
{
	PORTA &= ~_BV(LCD_BACKLIGHT);
}

inline void enablePots()
{
	PORTC |= _BV(ENABLE_POTS);
}

inline void disablePots()
{
	PORTC &= ~_BV(ENABLE_POTS);
}

inline void enableReverser()
{
	PORTC |= _BV(ENABLE_REVERSER);
}

inline void disableReverser()
{
	PORTC &= ~_BV(ENABLE_REVERSER);
}

inline void enableLightSwitches()
{
	PORTD |= _BV(ENABLE_LIGHTSWS);
}

inline void disableLightSwitches()
{
	PORTD &= ~_BV(ENABLE_LIGHTSWS);
}

inline void enableThrottle()
{
	PORTD |= _BV(ENABLE_THROTTLE);
}

inline void disableThrottle()
{
	PORTD &= ~_BV(ENABLE_THROTTLE);
}


void ledUpdate();

#endif
