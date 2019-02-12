#ifndef _CST_LCD_H_
#define _CST_LCD_H_

typedef enum
{
	LCD_RESET = 0,
	LCD_DEFAULT,
	LCD_DIAGS,
	LCD_TONNAGE,
	LCD_PRESSURE
} LcdMode;

void displaySplashScreen(void);
void printLocomotiveAddress(uint16_t addr);
void setupLCD(LcdMode mode);
void initLCD(void);

#endif

