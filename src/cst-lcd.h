#ifndef _LCD_CHAR_H_
#define _LCD_CHAR_H_

#define BATTERY_CHAR          0

#define BELL_CHAR             1
#define HORN_CHAR             2

#define FUNCTION_INACTIVE_CHAR 4
#define FUNCTION_ACTIVE_CHAR   5
#define BARGRAPH_BOTTOM_EMPTY 4
#define BARGRAPH_BOTTOM_HALF  5
#define BARGRAPH_TOP_EMPTY    6
#define BARGRAPH_TOP_HALF     7
#define BARGRAPH_FULL         0xFF

typedef enum
{
	BATTERY_EMPTY = 0,
	BATTERY_HALF,
	BATTERY_FULL
} BatteryState;

typedef enum
{
	FUNCTION_KEYS = 0,
	BARGRAPH
} SoftkeyState;

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


void setupDiagChars(void)
{
	lcd_setup_custom(BELL_CHAR, Bell);
	lcd_setup_custom(BELL_CHAR, Horn);
}

void setupBatteryChar(BatteryState state)
{
	switch(state)
	{
		case BATTERY_FULL:
			lcd_setup_custom(BATTERY_CHAR, BatteryFull);
			break;
		case BATTERY_HALF:
			lcd_setup_custom(BATTERY_CHAR, BatteryHalf);
			break;
		case BATTERY_EMPTY:
			lcd_setup_custom(BATTERY_CHAR, BatteryEmpty);
			break;
	}
}

void setupSoftkeyChars(SoftkeyState state)
{
	switch(state)
	{
		case FUNCTION_KEYS:
			lcd_setup_custom(FUNCTION_INACTIVE_CHAR, SoftkeyInactive);
			lcd_setup_custom(FUNCTION_ACTIVE_CHAR, SoftkeyActive);
			break;
		case BARGRAPH:
			lcd_setup_custom(BARGRAPH_BOTTOM_EMPTY, BarGraphBottomEmpty);
			lcd_setup_custom(BARGRAPH_BOTTOM_HALF, BarGraphBottomHalf);
			lcd_setup_custom(BARGRAPH_TOP_EMPTY, BarGraphTopEmpty);
			lcd_setup_custom(BARGRAPH_TOP_HALF, BarGraphTopHalf);
			break;
	}
}

void printTonnage(uint8_t tonnage)
{
	switch(tonnage)
	{
		case 0:
			lcd_gotoxy(7,0);
			lcd_putc(BARGRAPH_TOP_EMPTY);
			lcd_gotoxy(7,1);
			lcd_putc(BARGRAPH_BOTTOM_EMPTY);
			break;
		case 1:
			lcd_gotoxy(7,0);
			lcd_putc(BARGRAPH_TOP_EMPTY);
			lcd_gotoxy(7,1);
			lcd_putc(BARGRAPH_BOTTOM_HALF);
			break;
		case 2:
			lcd_gotoxy(7,0);
			lcd_putc(BARGRAPH_TOP_HALF);
			lcd_gotoxy(7,1);
			lcd_putc(BARGRAPH_FULL);
			break;
		case 3:
			lcd_gotoxy(7,0);
			lcd_putc(BARGRAPH_FULL);
			lcd_gotoxy(7,1);
			lcd_putc(BARGRAPH_FULL);
			break;
		}
}

void printTime()
{
	lcd_puts("00:00");
}

#endif

