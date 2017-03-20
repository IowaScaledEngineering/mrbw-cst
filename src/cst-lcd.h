#ifndef _LCD_CHAR_H_
#define _LCD_CHAR_H_

#define BARGRAPH_BOTTOM_EMPTY 0
#define BARGRAPH_BOTTOM_HALF  1
#define BARGRAPH_TOP_EMPTY    2
#define BARGRAPH_TOP_HALF     3
#define BARGRAPH_FULL         0xFF
#define BELL_CHAR             4
#define HORN_CHAR             0x7E
#define BATTERY_FULL          5
#define BATTERY_HALF          6
#define BATTERY_EMPTY         7

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
	0b00011111
};

const uint8_t BatteryHalf[8] =
{
	0b00001110,
	0b00010001,
	0b00010001,
	0b00010001,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111
};
const uint8_t BatteryEmpty[8] =
{
	0b00001110,
	0b00010001,
	0b00010001,
	0b00010001,
	0b00010001,
	0b00010001,
	0b00010001,
	0b00011111
};


void setupCustomChars(void)
{
	lcd_setup_custom(BELL_CHAR, Bell);
	lcd_setup_custom(BARGRAPH_BOTTOM_EMPTY, BarGraphBottomEmpty);
	lcd_setup_custom(BARGRAPH_BOTTOM_HALF, BarGraphBottomHalf);
	lcd_setup_custom(BARGRAPH_TOP_EMPTY, BarGraphTopEmpty);
	lcd_setup_custom(BARGRAPH_TOP_HALF, BarGraphTopHalf);
	lcd_setup_custom(BATTERY_FULL, BatteryFull);
	lcd_setup_custom(BATTERY_HALF, BatteryHalf);
	lcd_setup_custom(BATTERY_EMPTY, BatteryEmpty);
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
	lcd_gotoxy(0,1);
	lcd_puts("00:00");
}

#endif

