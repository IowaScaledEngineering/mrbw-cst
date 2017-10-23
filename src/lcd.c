/****************************************************************************
 Title	:   HD44780U LCD library
 Author:    Peter Fleury <pfleury@gmx.ch>  http://jump.to/fleury
 File:	    $Id: lcd.c,v 1.14.2.1 2006/01/29 12:16:41 peter Exp $
 Software:  AVR-GCC 3.3 
 Target:    any AVR device, memory mapped mode only for AT90S4414/8515/Mega

 DESCRIPTION
       Basic routines for interfacing a HD44780U-based text lcd display

       Originally based on Volker Oth's lcd library,
       changed lcd_init(), added additional constants for lcd_command(),
       added 4-bit I/O mode, improved and optimized code.

       Library can be operated in memory mapped mode (LCD_IO_MODE=0) or in 
       4-bit IO port mode (LCD_IO_MODE=1). 8-bit IO port mode not supported.
       
       Memory mapped mode compatible with Kanda STK200, but supports also
       generation of R/W signal through A8 address line.

 USAGE
       See the C include lcd.h file for a description of each function
       
*****************************************************************************/
#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "lcd.h"

#define LCD_ROWS    2
#define LCD_COLUMNS 8

#define lcd_e_delay()   __asm__ __volatile__( "rjmp 1f\n 1:" );
#define lcd_e_high()    LCD_E_PORT  |=  _BV(LCD_E_PIN);
#define lcd_e_low()     LCD_E_PORT  &= ~_BV(LCD_E_PIN);

#define lcd_rs_high()   LCD_RS_PORT |=  _BV(LCD_RS_PIN)
#define lcd_rs_low()    LCD_RS_PORT &= ~_BV(LCD_RS_PIN)

#define DDR(x) (*(&x - 1))      /* address of data direction register of port x */
#define PIN(x) (*(&x - 2))    /* address of input register of port x          */

#define lcd_e_delay()   __asm__ __volatile__( "rjmp 1f\n 1:" );
#define lcd_e_high()    LCD_E_PORT  |=  _BV(LCD_E_PIN);
#define lcd_e_low()     LCD_E_PORT  &= ~_BV(LCD_E_PIN);
#define lcd_rw_high()   LCD_RW_PORT |=  _BV(LCD_RW_PIN)
#define lcd_rw_low()    LCD_RW_PORT &= ~_BV(LCD_RW_PIN)
#define lcd_rs_high()   LCD_RS_PORT |=  _BV(LCD_RS_PIN)
#define lcd_rs_low()    LCD_RS_PORT &= ~_BV(LCD_RS_PIN)

#define LCD_FUNCTION_DEFAULT    LCD_FUNCTION_4BIT_2LINES 

#define delay(us) _delay_us(us);

static uint8_t lcdCache[LCD_ROWS*LCD_COLUMNS];
static uint8_t lcdIndex;

/* toggle Enable Pin to initiate write */
static void lcd_e_toggle()
{
    lcd_e_high();
    delay(1);
    lcd_e_low();
}

#define lcd_command(data) lcd_write((data), 0)
#define lcd_data(data) lcd_write((data), 1)

static void lcd_write(uint8_t data, uint8_t rs)
{
	if (rs)
		lcd_rs_high();
	else
		lcd_rs_low();
	
	lcd_e_low();
		
	PORTC &= ~(0xF0);
	PORTC |= (data & 0xF0);
	lcd_e_toggle();

	PORTC &= ~(0xF0);
	PORTC |= ((data<<4) & 0xF0);
	lcd_e_toggle();

	if (0 == rs && (0x01 == data || 0x02 == data || 0x03 == data))
		_delay_ms(2);
	else
		_delay_us(60);
}


/*************************************************************************
Set cursor to specified position
Input:    x  horizontal position  (0: left most position)
          y  vertical position    (0: first line)
Returns:  none
*************************************************************************/
void lcd_gotoxy(uint8_t x, uint8_t y)
{
	if ( y==0 ) 
		lcd_command((1<<LCD_DDRAM)+LCD_START_LINE1+x);
	else
		lcd_command((1<<LCD_DDRAM)+LCD_START_LINE2+x);
	lcdIndex = x + (y * LCD_COLUMNS);
}


/*************************************************************************
Clear display and set cursor to home position
*************************************************************************/
void lcd_clrscr(void)
{
	uint8_t i;
    lcd_command(1<<LCD_CLR);
	lcdIndex = 0;
	for(i=0; i<(LCD_ROWS*LCD_COLUMNS); i++)
	{
		lcdCache[i] = 0x20;  // Clear with spaces
	}
}


/*************************************************************************
Set cursor to home position
*************************************************************************/
void lcd_home(void)
{
    lcd_command(1<<LCD_HOME);
    lcdIndex = 0;
}


/*************************************************************************
Display character at current cursor position 
Input:    character to be displayed                                       
Returns:  none
*************************************************************************/
void lcd_putc(char c)
{
	if(c != lcdCache[lcdIndex])
	{
		lcd_gotoxy(lcdIndex % LCD_COLUMNS, lcdIndex / LCD_COLUMNS);
		lcd_data(c);
		lcdCache[lcdIndex] = c;
	}
	lcdIndex++;  // Index (cursor position) is expected to increase regardless of whether written or not
	if(lcdIndex >= (LCD_ROWS * LCD_COLUMNS))
		lcdIndex = 0;
}


/*************************************************************************
Display string without auto linefeed 
Input:    string to be displayed
Returns:  none
*************************************************************************/
void lcd_puts(const char *s)
/* print string on lcd (no auto linefeed) */
{
    register char c;

    while ( (c = *s++) ) {
        lcd_putc(c);
    }

}/* lcd_puts */


/*************************************************************************
Display string from program memory without auto linefeed 
Input:     string from program memory be be displayed                                        
Returns:   none
*************************************************************************/
void lcd_puts_p(const char *progmem_s)
/* print string from program memory on lcd (no auto linefeed) */
{
    register char c;

    while ( (c = pgm_read_byte(progmem_s++)) ) {
        lcd_putc(c);
    }

}/* lcd_puts_p */


/*************************************************************************
Initialize display and select type of cursor 
Input:    dispAttr LCD_DISP_OFF            display off
                   LCD_DISP_ON             display on, cursor off
                   LCD_DISP_ON_CURSOR      display on, cursor on
                   LCD_DISP_CURSOR_BLINK   display on, cursor on flashing
Returns:  none
*************************************************************************/






void lcd_init(uint8_t dispAttr)
{
	DDRC |= 0xFC;
	lcd_rs_low();
	_delay_ms(100);

	lcd_e_low();
	
	// For the first three calls, we're going to emit 0x30
	PORTC &= ~(0xF0);
	PORTC |= 0x30;
	lcd_e_toggle();
	_delay_ms(5);

	// repeat last command
	lcd_e_toggle();      
	_delay_us(180);

	// repeat last command
	lcd_e_toggle();      
	_delay_us(180);

	PORTC &= ~(0xF0);
	PORTC |= 0x20;
	lcd_e_toggle();      
	_delay_us(180);
	// Now in 4 bit mode


	lcd_command(LCD_FUNCTION_DEFAULT);      /* function set: display lines  */
	lcd_command(LCD_DISP_OFF);              /* display off                  */
	lcd_clrscr();                           /* display clear                */ 
	lcd_command(LCD_MODE_DEFAULT);          /* set entry mode               */
	lcd_command(dispAttr);                  /* display/cursor control       */
}

void printHex(uint8_t val)
{
	const uint8_t bin2hex[] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };
	lcd_putc(bin2hex[(val>>4) & 0x0F]);
	lcd_putc(bin2hex[val & 0x0F]);
}

void printDec3DigWZero(uint16_t val)
{
	lcd_putc('0' + ((val/100)%10));
	lcd_putc('0' + ((val/10)%10));
	lcd_putc('0' + (val%10));
}

void printDec3Dig(uint16_t val)
{
	if (val >= 100)
		lcd_putc('0' + ((val/100)%10));
	else
		lcd_putc(' ');
	
	if (val >= 10)
		lcd_putc('0' + ((val/10)%10));
	else
		lcd_putc(' ');
	lcd_putc('0' + (val%10));
}

void printDec4Dig(uint16_t val)
{
	if (val >= 1000)
		lcd_putc('0' + ((val/1000)%10));
	else
		lcd_putc(' ');

	if (val >= 100)
		lcd_putc('0' + ((val/100)%10));
	else
		lcd_putc(' ');
	
	if (val >= 10)
		lcd_putc('0' + ((val/10)%10));
	else
		lcd_putc(' ');
	lcd_putc('0' + (val%10));
}

void printDec4DigWZero(uint16_t val)
{
	lcd_putc('0' + ((val/1000)%10));
	lcd_putc('0' + ((val/100)%10));
	lcd_putc('0' + ((val/10)%10));
	lcd_putc('0' + (val%10));
}

void printDec2Dig(uint8_t val)
{
	if (val >= 10)
		lcd_putc('0' + ((val/10)%10));
	else
		lcd_putc(' ');
	lcd_putc('0' + (val%10));
}

void printDec2DigWZero(uint8_t val)
{
	lcd_putc('0' + ((val/10)%10));
	lcd_putc('0' + (val%10));
}

const uint8_t UpperLeftDiag[8] =
{
	0b00000111,
	0b00001111,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111
};

const uint8_t  UpperBlock[8] =
{
	0b00011111,
	0b00011111,
	0b00011111,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000
};

const uint8_t UpperRightDiag[8] =
{
	0b00011100,
	0b00011110,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111
};

const uint8_t LowerLeftDiag[8] =
{
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00001111,
	0b00000111
};


const uint8_t LowerBlock[8] =
{
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00011111,
	0b00011111,
	0b00011111
};

const uint8_t LowerRightDiag[8] =
{
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011110,
	0b00011100
};

const uint8_t MiddleBlock[8] =
{
	0b00011111,
	0b00011111,
	0b00011111,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00011111,
	0b00011111
};

void lcd_setup_custom(uint8_t charNum, const uint8_t* charDefinition)
{
	uint8_t i;
	lcd_command(0x40 + charNum * 8);
	for(i=0; i<8; i++)
	{
		lcd_command(0x40 + charNum * 8 + i);
		lcd_data(*charDefinition++);
	}
}

void lcd_setup_bigdigits()
{
	// assignes each segment a write number
	lcd_setup_custom(0, UpperLeftDiag);
	lcd_setup_custom(1, UpperBlock);
	lcd_setup_custom(2, UpperRightDiag);
	lcd_setup_custom(3, LowerLeftDiag);
	lcd_setup_custom(4, LowerBlock);
	lcd_setup_custom(5, LowerRightDiag);
	lcd_setup_custom(6, MiddleBlock);
	lcd_gotoxy(0,0);

}

const uint8_t bigDigits[14][6] = 
{
	// 0
	{ 0, 1, 2, 3, 4, 5},

	// 1
	{ 1, 2, ' ', 4, 255, 4},

	// 2
	{ 6, 6, 2, 3, 4, 4},

	// 3
	{ 6, 6, 2, 4, 4, 5},

	// 4
	{ 3, 4, 255, ' ', ' ', 255},
	
	// 5
	{ 3, 6, 6, 4, 4, 5},

	// 6
	{ 0, 6, 6, 3, 4, 5},

	// 7
	{ 1, 1, 2, ' ', ' ', 255},
	
	// 8
	{ 0, 6, 2, 3, 4, 5},
		
	// 9
	{ 0, 6, 2, ' ', ' ', 255},
	
	// H - 10
	{ 0, 4, 2, 3, ' ', 5},

	// l - 11
	{ 255, ' ', ' ', 3, 4, 4},

	// d - 12
	{ 4, 4, 255, 3, 4, 5},
	
	// space - 13
	{ ' ', ' ', ' ',  ' ', ' ', ' ' }
	
};

void lcd_putc_big(uint8_t position, uint8_t digit)
{
	uint8_t column;
	uint8_t i;
	if (digit > 13)
		digit = 0;
	
	switch(position)
	{
		default:
		case 0:
			column = 0;
			break;
			
		case 1:
			column = 4;
			break;
			
		case 2:
			column = 8;
			break;
			
		case 3:
			column = 12;
			break;
	}

	lcd_gotoxy(column, 0);
	for(i=0; i<3; i++)
		lcd_putc(bigDigits[digit][i]);

	lcd_gotoxy(column, 1);
	for(i=3; i<6; i++)
		lcd_putc(bigDigits[digit][i]);
}

