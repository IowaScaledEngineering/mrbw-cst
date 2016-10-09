#ifndef _CST_HARDWARE_H_
#define _CST_HARDWARE_H_

#define LCD_POWER      PA6
#define LCD_BACKLIGHT  PA5

#define ENABLE_REVERSER PC0
#define ENABLE_BRAKE    PC1
#define ENABLE_LIGHTSWS PD5
#define ENABLE_THROTTLE PD4
#define ANALOG_VREV     PA0
#define ANALOG_VBRAKE   PA1
#define ANALOG_VLIGHT_F PA2
#define ANALOG_VLIGHT_R PA3
#define ANALOG_VBATT    PA7
#define LED_RED_PIN     PD6
#define LED_GREEN_PIN   PD7

void initADC();
void initPorts();

typedef enum
{
	NEUTRAL = 0,
	FORWARD,
	REVERSE
} ReverserPosition;

typedef enum
{
	LIGHT_OFF = 0,
	LIGHT_DIM = 1,
	LIGHT_BRIGHT = 2,	
	LIGHT_BRIGHT_DITCH = 3,	

} LightPosition;

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

extern volatile uint8_t batteryVoltage;
extern volatile uint8_t brakePot;
extern volatile uint8_t reverserPot;
volatile ReverserPosition reverserPosition;
extern volatile LightPosition frontLight;
extern volatile LightPosition rearLight;
extern volatile uint8_t frontLightPot;
extern volatile uint8_t rearLightPot;
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
	PORTB = 0b11110111;  // Pullups on for all buttons
}


inline void buttonsDisable()
{
	PORTB = 0b00000000;  // Pullups on for all buttons
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

inline void	enableBrakePot()
{
	PORTC |= _BV(ENABLE_BRAKE);
}

inline void	disableBrakePot()
{
	PORTC &= ~_BV(ENABLE_BRAKE);
}


inline void enableReverserPot()
{
	PORTC |= _BV(ENABLE_REVERSER);
}

inline void disableReverserPot()
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
