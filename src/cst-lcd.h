#ifndef _LCD_CHAR_H_
#define _LCD_CHAR_H_

#define BELL_CHAR             0
#define HORN_CHAR             1
#define BARGRAPH_BOTTOM_EMPTY 2
#define BARGRAPH_BOTTOM_HALF  3
#define BARGRAPH_TOP_EMPTY    4
#define BARGRAPH_TOP_HALF     5
#define BARGRAPH_FULL         6

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

