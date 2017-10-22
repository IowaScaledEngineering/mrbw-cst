#ifndef _LCD_CHAR_H_
#define _LCD_CHAR_H_

#define BATTERY_CHAR            0
#define BELL_CHAR               1
#define HORN_CHAR               2
#define AM_CHAR                 1
#define PM_CHAR                 2
#define FUNCTION_INACTIVE_CHAR  3
#define FUNCTION_ACTIVE_CHAR    4
#define TONNAGE_TOP             5
#define TONNAGE_BOTTOM          6

typedef enum
{
	EMPTY = 0,
	HALF,
	FULL,
	UNKNOWN
} BatteryState;

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

const uint8_t BarGraphBottomEmpty[8] =
{
	0b00010001,
	0b00010001,
	0b00010001,
	0b00010001,
	0b00010001,
	0b00010001,
	0b00010001,
	0b00011111
};

const uint8_t BarGraphBottomHalf[8] =
{
	0b00010001,
	0b00010001,
	0b00010001,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111
};

const uint8_t BarGraphTopEmpty[8] =
{
	0b00011111,
	0b00010001,
	0b00010001,
	0b00010001,
	0b00010001,
	0b00010001,
	0b00010001,
	0b00010001
};

const uint8_t BarGraphTopHalf[8] =
{
	0b00011111,
	0b00010001,
	0b00010001,
	0b00010001,
	0b00010001,
	0b00011111,
	0b00011111,
	0b00011111
};

const uint8_t BarGraphFull[8] =
{
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111
};

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

const uint8_t SoftkeyInactive[8] =
{
	0b00000000,
	0b00001110,
	0b00010001,
	0b00010001,
	0b00010001,
	0b00001110,
	0b00000000,
	0b00000000
};
const uint8_t SoftkeyActive[8] =
{
	0b00000000,
	0b00001110,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00001110,
	0b00000000,
	0b00000000
};

const uint8_t ClockAM[8] =
{
	0b00001000,
	0b00010100,
	0b00011100,
	0b00010100,
	0b00010100,
	0b00000000,
	0b00000000,
	0b00000000
};

const uint8_t ClockPM[8] =
{
	0b00000000,
	0b00000000,
	0b00011100,
	0b00010100,
	0b00011100,
	0b00010000,
	0b00010000,
	0b00000000
};

void setupClockChars(void)
{
	lcd_setup_custom(AM_CHAR, ClockAM);
	lcd_setup_custom(PM_CHAR, ClockPM);
}

void setupDiagChars(void)
{
	lcd_setup_custom(BELL_CHAR, Bell);
	lcd_setup_custom(HORN_CHAR, Horn);
}

void setupBatteryChar(BatteryState state)
{
	switch(state)
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
		case UNKNOWN:
			break;
	}
}

void setupSoftkeyChars(void)
{
	lcd_setup_custom(FUNCTION_INACTIVE_CHAR, SoftkeyInactive);
	lcd_setup_custom(FUNCTION_ACTIVE_CHAR, SoftkeyActive);
}

void printTonnage(uint8_t tonnage)
{
	switch(tonnage)
	{
		case 0:
			lcd_setup_custom(TONNAGE_TOP, BarGraphTopEmpty);
			lcd_setup_custom(TONNAGE_BOTTOM, BarGraphBottomEmpty);
			break;
		case 1:
			lcd_setup_custom(TONNAGE_TOP, BarGraphTopEmpty);
			lcd_setup_custom(TONNAGE_BOTTOM, BarGraphBottomHalf);
			break;
		case 2:
			lcd_setup_custom(TONNAGE_TOP, BarGraphTopHalf);
			lcd_setup_custom(TONNAGE_BOTTOM, BarGraphFull);
			break;
		case 3:
			lcd_setup_custom(TONNAGE_TOP, BarGraphFull);
			lcd_setup_custom(TONNAGE_BOTTOM, BarGraphFull);
			break;
		}
	lcd_gotoxy(7,0);
	lcd_putc(TONNAGE_TOP);
	lcd_gotoxy(7,1);
	lcd_putc(TONNAGE_BOTTOM);
}

// Splash Screen Characters
const uint8_t Splash1A[8] =
{
	0b00000111,
	0b00000100,
	0b00000100,
	0b00000100,
	0b00000111,
	0b00000000,
	0b00000000,
	0b00000000
};

const uint8_t Splash1C[8] =
{
	0b00000111,
	0b00000101,
	0b00000011,
	0b00000111,
	0b00001111,
	0b00010010,
	0b00010011,
	0b00001110
};

const uint8_t Splash2A[8] =
{
	0b00011111,
	0b00000011,
	0b00000001,
	0b00000000,
	0b00011111,
	0b00000000,
	0b00000000,
	0b00000000
};

const uint8_t Splash2B[8] =
{
	0b00011111,
	0b00000111,
	0b00000111,
	0b00000111,
	0b00010111,
	0b00000100,
	0b00000100,
	0b00000011
};

const uint8_t Splash2C[8] =
{
	0b00011111,
	0b00011101,
	0b00011010,
	0b00010100,
	0b00001011,
	0b00010000,
	0b00000000,
	0b00000000
};

const uint8_t Splash3A[8] =
{
	0b00011111,
	0b00011110,
	0b00011111,
	0b00011111,
	0b00001111,
	0b00000111,
	0b00000011,
	0b00000001
};

const uint8_t Splash3B[8] =
{
	0b00011111,
	0b00010000,
	0b00010000,
	0b00010000,
	0b00010111,
	0b00010000,
	0b00010000,
	0b00000000
};

const uint8_t Splash3C[8] =
{
	0b00011111,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00011111,
	0b00000000,
	0b00000000,
	0b00000000
};

const uint8_t Splash4A[8] =
{
	0b00010111,
	0b00010100,
	0b00000100,
	0b00010111,
	0b00011100,
	0b00000100,
	0b00000100,
	0b00011000
};

const uint8_t Splash4B[8] =
{
	0b00010111,
	0b00010100,
	0b00010100,
	0b00010111,
	0b00010100,
	0b00000100,
	0b00000000,
	0b00000000
};

const uint8_t Splash5[8] =
{
	0b00000111,
	0b00010100,
	0b00010100,
	0b00000111,
	0b00000100,
	0b00000100,
	0b00000000,
	0b00000000
};

const uint8_t Splash6[8] =
{
	0b00000011,
	0b00010100,
	0b00010100,
	0b00000100,
	0b00010100,
	0b00010011,
	0b00000000,
	0b00000000
};

const uint8_t Splash7[8] =
{
	0b00000111,
	0b00010010,
	0b00010010,
	0b00010010,
	0b00010010,
	0b00000010,
	0b00000000,
	0b00000000
};

const uint8_t Splash8[8] =
{
	0b00000110,
	0b00001001,
	0b00001001,
	0b00001001,
	0b00001001,
	0b00000110,
	0b00000000,
	0b00000000
};


#endif

