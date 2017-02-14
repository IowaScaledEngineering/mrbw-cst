#include <sys/time.h>
#include <stdio.h>
#include <unistd.h>
#include <curses.h>

#define STATUS_READ_SWITCHES          0x01

#define LONG_PRESS_10MS_TICKS             100
#define BUTTON_AUTOINCREMENT_10MS_TICKS    50
#define BUTTON_AUTOINCREMENT_ACCEL         10
#define BUTTON_AUTOINCREMENT_MINIMUM        5

#define DYNAMIC_PIN 0x02
#define BELL_PIN    0x04
#define HORN_PIN    0x01
#define MENU_PIN    0x20
#define SELECT_PIN  0x10
#define UP_PIN      0x40
#define DOWN_PIN    0x80

#define HORN_CONTROL      0x01
#define BELL_CONTROL      0x02
#define DYNAMIC_CONTROL   0x04

#define BARGRAPH_BOTTOM_EMPTY ' '
#define BARGRAPH_BOTTOM_HALF  '.'
#define BARGRAPH_TOP_EMPTY    ' '
#define BARGRAPH_TOP_HALF     '.'
#define BARGRAPH_FULL         ':'


volatile uint16_t button_autoincrement_10ms_ticks = BUTTON_AUTOINCREMENT_10MS_TICKS;
volatile uint16_t ticks_autoincrement = BUTTON_AUTOINCREMENT_10MS_TICKS;

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

ReverserPosition reverserPosition = NEUTRAL;

uint8_t brakePosition = 0;

uint8_t frontLightPot = 0;
LightPosition frontLight = LIGHT_OFF;
uint8_t rearLightPot = 0;
LightPosition rearLight = LIGHT_OFF;

uint8_t batteryVoltage = 0;

volatile uint8_t ticks;
volatile uint16_t decisecs = 0;
volatile uint16_t update_decisecs = 10;
volatile uint8_t status = 0;

// Define the menu screens and menu order
// Must end with LAST_SCREEN
typedef enum
{
	MAIN_SCREEN = 0,
	LOCO_SCREEN,
	TONNAGE_SCREEN,
	FUNC_SCREEN,
	MRBUS_SCREEN,
	DEBUG_SCREEN,
	VBAT_SCREEN,
	LAST_SCREEN  // Must be the last screen
} Screens;

typedef enum
{
	NO_BUTTON = 0,
	MENU_BUTTON,
	SELECT_BUTTON,
	UP_BUTTON,
	DOWN_BUTTON,
} Buttons;

Buttons button = NO_BUTTON;
Buttons previousButton = NO_BUTTON;
uint8_t buttonCount = 0;

typedef enum
{
	HORN_FN = 0,
	BELL_FN,
	DYNAMIC_FN,
	FRONT_HEADLIGHT_FN,
	FRONT_DITCH_FN,
	FRONT_DIM1_FN,
	FRONT_DIM2_FN,
	REAR_HEADLIGHT_FN,
	REAR_DITCH_FN,
	REAR_DIM1_FN,
	REAR_DIM2_FN,
	UP_FN,
	DOWN_FN,
	LAST_FN,
} Functions;

Functions functionSetting = HORN_FN;

#define UP_OPTION_BUTTON   0x01
#define DOWN_OPTION_BUTTON 0x02

uint8_t controls = 0;

void lcdBacklightEnable()
{
	mvprintw(4,0,"Backlight ON ");
}

void lcdBacklightDisable()
{
	mvprintw(4,0,"Backlight OFF");
}

void lcd_gotoxy(uint8_t x, uint8_t y)
{
	move(y,x);
}

void lcd_putc(char c)
{
	addch(c);
}

void lcd_puts(const char *s)
{
	register char c;

	while ( (c = *s++) ) {
		lcd_putc(c);
	}
}

void lcd_clrscr(void)
{
	clear();
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

void printHex(uint8_t val)
{
	const uint8_t bin2hex[] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };
	lcd_putc(bin2hex[(val>>4) & 0x0F]);
	lcd_putc(bin2hex[val & 0x0F]);
}

void wait100ms(uint16_t loops)
{
	uint16_t i;
	refresh();
	for(i=0; i<loops; i++)
	{
		usleep(100000);
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
	lcd_gotoxy(0,1);
	lcd_puts("00:00");
}



void processButtons(uint8_t inputButtons)
{
	// Called every 10ms
	if(inputButtons & DYNAMIC_PIN)
		controls &= ~(DYNAMIC_CONTROL);
	else
		controls |= DYNAMIC_CONTROL;

	if(inputButtons & BELL_PIN)
		controls &= ~(BELL_CONTROL);
	else
		controls |= BELL_CONTROL;

	if(inputButtons & HORN_PIN)
		controls &= ~(HORN_CONTROL);
	else
		controls |= HORN_CONTROL;
	
	if(!(inputButtons & MENU_PIN))
	{
		button = MENU_BUTTON;
	}
	else if(!(inputButtons & SELECT_PIN) && (inputButtons & UP_PIN) && (inputButtons & DOWN_PIN))
	{
		button = SELECT_BUTTON;
	}
/*	else if(!(inputButtons & SELECT_PIN) && !(inputButtons & UP_PIN))*/
/*	{*/
/*		button = UP_SELECT_BUTTON;*/
/*	}*/
/*	else if(!(inputButtons & SELECT_PIN) && !(inputButtons & DOWN_PIN))*/
/*	{*/
/*		button = DOWN_SELECT_BUTTON;*/
/*	}*/
	else if(!(inputButtons & UP_PIN))
	{
		button = UP_BUTTON;
	}
	else if(!(inputButtons & DOWN_PIN))
	{
		button = DOWN_BUTTON;
	}
	else
	{
		button = NO_BUTTON;
	}

	if((previousButton == button) && (NO_BUTTON != button))
	{
		if(buttonCount > LONG_PRESS_10MS_TICKS)  // Use greater than, so a long press can be caught as a single event in the menu handler code
		{
			if(button_autoincrement_10ms_ticks >= BUTTON_AUTOINCREMENT_ACCEL)
				button_autoincrement_10ms_ticks -= BUTTON_AUTOINCREMENT_ACCEL;  // Progressively reduce the time delay
			if(button_autoincrement_10ms_ticks < BUTTON_AUTOINCREMENT_MINIMUM)
				button_autoincrement_10ms_ticks = BUTTON_AUTOINCREMENT_MINIMUM; // Clamp to some minimum value, otherwise it goes too fast to control
			buttonCount = 0;
		}
		else
		{
			buttonCount++;
		}
	}
	else
	{
		// Reset the counters
		button_autoincrement_10ms_ticks = BUTTON_AUTOINCREMENT_10MS_TICKS;
		ticks_autoincrement = button_autoincrement_10ms_ticks;
	}
}

volatile uint8_t throttlePosition = 0;

int main(void)
{
	struct timeval startTime, endTime;
	long mtime, seconds, useconds;    

	uint8_t inputButtons = 0;

	uint8_t tonnage = 0;

	uint8_t mrbus_dev_addr = 0x03;
	
	uint16_t locoAddress = 250;
	uint16_t newLocoAddress = locoAddress;
	uint8_t newDevAddr = mrbus_dev_addr;
	
	uint8_t backlight = 0;
	
	Screens screenState = LAST_SCREEN;  // Initialize to the last one, since that's the only state guaranteed to be present
	uint8_t subscreenStatus = 0;

	uint8_t i;
	uint8_t allowLatch;
	
	uint8_t decimalNumberIndex = 0;
	uint8_t decimalNumber[4];

	uint8_t optionButtonState = 0;

	uint8_t hornFunction = 2;
	uint8_t bellFunction = 7;
	uint8_t frontDim1Function = 3, frontDim2Function = 0x80, frontHeadlightFunction = 0, frontDitchFunction = 3;
	uint8_t rearDim1Function = 6, rearDim2Function = 0x80, rearHeadlightFunction = 5, rearDitchFunction = 6;
	uint8_t dynamicFunction = 8;
	uint8_t upButtonFunction = 0x80, downButtonFunction = 0x80;

	uint8_t *functionPtr = &hornFunction;


	initscr();
	noecho();
	cbreak();
	timeout(0);

	while(1)
	{
		gettimeofday(&startTime, NULL);

		if (status & STATUS_READ_SWITCHES)
		{
			// Read switches every 10ms
			status &= ~STATUS_READ_SWITCHES;
			int c = getch();
			switch(c)
			{
				case 'q':
					endwin();
					return 0;
				case 'a':
					inputButtons = 0xDF;
					break;
				case 'z':
					inputButtons = 0xEF;
					break;
				case 'd':
					inputButtons = 0xBF;
					break;
				case 'c':
					inputButtons = 0x7F;
					break;
				default:
					inputButtons = 0xFF;
					break;
			}
			processButtons(inputButtons);
		}

/*switch(button)*/
/*{*/
/*	case MENU_BUTTON:*/
/*		mvprintw(6,0,"MENU      ");*/
/*		break;*/
/*	case SELECT_BUTTON:*/
/*		mvprintw(6,0,"SELECT    ");*/
/*		break;*/
/*	case UP_BUTTON:*/
/*		mvprintw(6,0,"UP        ");*/
/*		break;*/
/*	case DOWN_BUTTON:*/
/*		mvprintw(6,0,"DOWN      ");*/
/*		break;*/
/*	default:*/
/*		mvprintw(6,0,"          ");*/
/*		break;*/
/*}*/
/*wait100ms(5);*/

/*		processADC();*/

		switch(screenState)
		{
			case MAIN_SCREEN:
				if(backlight)
					lcdBacklightEnable();
				else
					lcdBacklightDisable();
				lcd_gotoxy(0,0);
				printDec4DigWZero(locoAddress);
				printTime();

				if((0x80 & upButtonFunction) && (0x80 & downButtonFunction))
				{
					printTonnage(tonnage);
				}
				else
				{
					lcd_gotoxy(7,0);
					lcd_putc((optionButtonState & UP_OPTION_BUTTON) && !(upButtonFunction & 0x80) ? 'o' : ' ');  // FIXME: Also try 0x7E, 0xDB
					lcd_gotoxy(7,1);
					lcd_putc((optionButtonState & DOWN_OPTION_BUTTON) && !(downButtonFunction & 0x80) ? 'o' : ' ');
				}

				switch(button)
				{
					case UP_BUTTON:
						if(UP_BUTTON != previousButton)
						{
							if((0x80 & upButtonFunction) && (0x80 & downButtonFunction))
							{
								if(tonnage >= 3)
									tonnage = 0;
								else
									tonnage++;
							}
							else
							{
								if(upButtonFunction & 0x40)
									optionButtonState ^= UP_OPTION_BUTTON;  // Toggle
								else
									optionButtonState |= UP_OPTION_BUTTON;  // Momentary on
							}
						}
						break;
					case DOWN_BUTTON:
						if(DOWN_BUTTON != previousButton)
						{
							if((0x80 & upButtonFunction) && (0x80 & downButtonFunction))
							{
								if((0 == tonnage) || (tonnage > 3))
									tonnage = 3;
								else
									tonnage--;
							}
							else
							{
								if(downButtonFunction & 0x40)
									optionButtonState ^= DOWN_OPTION_BUTTON;  // Toggle
								else
									optionButtonState |= DOWN_OPTION_BUTTON;  // Momentary on
							}
						}
						break;
					case SELECT_BUTTON:
						if(SELECT_BUTTON != previousButton)
						{
							if(backlight)
								backlight = 0;
							else
								backlight = 1;
						}
						break;
					case MENU_BUTTON:
					case NO_BUTTON:
						// Release buttons if momentary
						if(!(upButtonFunction & 0x40))
							optionButtonState &= ~UP_OPTION_BUTTON;
						if(!(downButtonFunction & 0x40))
							optionButtonState &= ~DOWN_OPTION_BUTTON;
						break;
				}
				break;

			case LOCO_SCREEN:
				lcdBacklightEnable();
				if(!subscreenStatus)
				{
					lcd_gotoxy(0,0);
					lcd_puts("GET NEW");
					lcd_gotoxy(0,1);
					lcd_puts("LOCO -->");
					switch(button)
					{
						case SELECT_BUTTON:
							decimalNumber[0] = (newLocoAddress / 1000) % 10;
							decimalNumber[1] = (newLocoAddress / 100) % 10;
							decimalNumber[2] = (newLocoAddress / 10) % 10;
							decimalNumber[3] = (newLocoAddress) % 10;
							subscreenStatus = 1;
							lcd_clrscr();
							break;
						case MENU_BUTTON:
						case UP_BUTTON:
						case DOWN_BUTTON:
						case NO_BUTTON:
							break;
					}
				}
				else
				{
					for(i=0; i<4; i++)
					{
						lcd_gotoxy(2+i,0);
						lcd_putc('0' + decimalNumber[i]);
						lcd_gotoxy(2+i,1);
						if(i == decimalNumberIndex)
						{
							lcd_putc('^');
						}
						else
						{
							lcd_putc(' ');
						}
					}
					switch(button)
					{
						case UP_BUTTON:
							if(ticks_autoincrement >= button_autoincrement_10ms_ticks)
							{
								if(decimalNumber[decimalNumberIndex] < 9)
									decimalNumber[decimalNumberIndex]++;
								ticks_autoincrement = 0;
							}
							break;
						case DOWN_BUTTON:
							if(ticks_autoincrement >= button_autoincrement_10ms_ticks)
							{
								if(decimalNumber[decimalNumberIndex] > 0)
									decimalNumber[decimalNumberIndex]--;
								ticks_autoincrement = 0;
							}
							break;
						case SELECT_BUTTON:
							// FIXME: This should really send a packet to request a new locomotive address and locoAddress is only updated once confirmation received
							newLocoAddress = (decimalNumber[0] * 1000) + (decimalNumber[1] * 100) + (decimalNumber[2] * 10) + decimalNumber[3];
							locoAddress = newLocoAddress;
							lcd_clrscr();
							lcd_gotoxy(0,0);
							lcd_puts("REQUEST");
							lcd_gotoxy(0,1);
							lcd_puts("SENT");
							wait100ms(10);
							decimalNumberIndex = 0;
							subscreenStatus = 0;
							screenState = LAST_SCREEN;
							break;
						case MENU_BUTTON:
							decimalNumberIndex++;
							if(decimalNumberIndex > 3)
								decimalNumberIndex = 0;
							break;
						case NO_BUTTON:
							break;
					}
				}
				break;

			case TONNAGE_SCREEN:
				lcdBacklightEnable();
				lcd_gotoxy(0,0);
				lcd_puts("WEIGHT");
				lcd_gotoxy(0,1);
				switch(tonnage)
				{
					case 0:
						lcd_puts("LIGHT ");
						break;
					case 1:
						lcd_puts("LOW   ");
						break;
					case 2:
						lcd_puts("MEDIUM");
						break;
					case 3:
						lcd_puts("HEAVY ");
						break;
				}
				printTonnage(tonnage);
				switch(button)
				{
					case UP_BUTTON:
						if(UP_BUTTON != previousButton)
						{
							if(tonnage >= 3)
								tonnage = 0;
							else
								tonnage++;
						}
						break;
					case DOWN_BUTTON:
						if(DOWN_BUTTON != previousButton)
						{
							if((0 == tonnage) || (tonnage > 3))
								tonnage = 3;
							else
								tonnage--;
						}
						break;
					case SELECT_BUTTON:
							screenState = LAST_SCREEN;
							break;
					case MENU_BUTTON:
					case NO_BUTTON:
						break;
				}
				break;

			case FUNC_SCREEN:
				lcdBacklightEnable();
				if(!subscreenStatus)
				{
					lcd_gotoxy(0,0);
					lcd_puts("CONFIG");
					lcd_gotoxy(0,1);
					lcd_puts("FUNC -->");
					switch(button)
					{
						case SELECT_BUTTON:
							subscreenStatus = 1;
							functionSetting = 0;
							lcd_clrscr();
							break;
						case MENU_BUTTON:
						case UP_BUTTON:
						case DOWN_BUTTON:
						case NO_BUTTON:
							break;
					}
				}
				else
				{
					allowLatch = 0;
					switch(functionSetting)
					{
						case HORN_FN:
							lcd_gotoxy(0,0);
							lcd_puts("HORN");
							functionPtr = &hornFunction;
							break;
						case BELL_FN:
							lcd_gotoxy(0,0);
							lcd_puts("BELL");
							functionPtr = &bellFunction;
							break;
						case DYNAMIC_FN:
							lcd_gotoxy(0,0);
							lcd_puts("D.BRAKE");
							functionPtr = &dynamicFunction;
							break;
						case FRONT_DIM1_FN:
							lcd_gotoxy(0,0);
							lcd_puts("F.DIM #1");
							functionPtr = &frontDim1Function;
							break;
						case FRONT_DIM2_FN:
							lcd_gotoxy(0,0);
							lcd_puts("F.DIM #2");
							functionPtr = &frontDim2Function;
							break;
						case FRONT_HEADLIGHT_FN:
							lcd_gotoxy(0,0);
							lcd_puts("F.HEAD");
							functionPtr = &frontHeadlightFunction;
							break;
						case FRONT_DITCH_FN:
							lcd_gotoxy(0,0);
							lcd_puts("F.DITCH");
							functionPtr = &frontDitchFunction;
							break;
						case REAR_DIM1_FN:
							lcd_gotoxy(0,0);
							lcd_puts("R.DIM #1");
							functionPtr = &rearDim1Function;
							break;
						case REAR_DIM2_FN:
							lcd_gotoxy(0,0);
							lcd_puts("R.DIM #2");
							functionPtr = &rearDim2Function;
							break;
						case REAR_HEADLIGHT_FN:
							lcd_gotoxy(0,0);
							lcd_puts("R.HEAD");
							functionPtr = &rearHeadlightFunction;
							break;
						case REAR_DITCH_FN:
							lcd_gotoxy(0,0);
							lcd_puts("R.DITCH");
							functionPtr = &rearDitchFunction;
							break;
						case UP_FN:
							lcd_gotoxy(0,0);
							lcd_puts("UP BTN");
							functionPtr = &upButtonFunction;
							allowLatch = 1;
							break;
						case DOWN_FN:
							lcd_gotoxy(0,0);
							lcd_puts("DOWN BTN");
							functionPtr = &downButtonFunction;
							allowLatch = 1;
							break;
						case LAST_FN:
							// Should never get here...
							break;
					}

					lcd_gotoxy(0,1);
					lcd_puts("F");
					lcd_gotoxy(1,1);
					if((*functionPtr) & 0x80)
						lcd_puts("--     ");
					else
					{
						printDec2DigWZero((*functionPtr) & 0x1F);
						if(allowLatch)
						{
							lcd_gotoxy(5,1);
							if((*functionPtr) & 0x40)
								lcd_puts("LAT");
							else
								lcd_puts("MOM");
						}
					}

					switch(button)
					{
						//  |off|latch|0|Func[4:0]|
						case UP_BUTTON:
							if(ticks_autoincrement >= button_autoincrement_10ms_ticks)
							{
								if((*functionPtr) & 0x80)
								{
									(*functionPtr) = 0;  // Turn on
								}
								else
								{
									if(((*functionPtr) & 0x1F) < 28)
										(*functionPtr)++;       // Increment
									else if(allowLatch && !((*functionPtr) & 0x40))
										(*functionPtr) = 0x40;  // Set latch bit, reset function number to zero
									else
										(*functionPtr) = ((*functionPtr) & 0x40) + 28;    // Saturate, preserving latch bit
								}
								ticks_autoincrement = 0;
							}
							break;
						case DOWN_BUTTON:
							if(ticks_autoincrement >= button_autoincrement_10ms_ticks)
							{
								if(~((*functionPtr) & 0x80))
								{
									// Not OFF...
									if(((*functionPtr) & 0x1F) > 0)
										(*functionPtr)--;       // Decrement
									else if(allowLatch && ((*functionPtr) & 0x40))
										(*functionPtr) = 28;    // Unset latch bit, reset function number to 28
									else
										(*functionPtr) = 0x80;  // Turn off
								}
								ticks_autoincrement = 0;
							}
							break;
						case SELECT_BUTTON:
							// FIXME: write to EEPROM
							lcd_clrscr();
							lcd_gotoxy(1,0);
							lcd_puts("SAVED!");
							wait100ms(7);
							subscreenStatus = 0;
							screenState = LAST_SCREEN;
							break;
						case MENU_BUTTON:
							// Advance through function settings
							lcd_clrscr();
							if(++functionSetting >= LAST_FN)
								functionSetting = 0;
							break;
						case NO_BUTTON:
							break;
					}
				}
				break;

			case MRBUS_SCREEN:
				lcdBacklightEnable();
				lcd_gotoxy(0,0);
				lcd_puts("MRB ADR");
				lcd_gotoxy(2,1);
				lcd_puts("0x");
				printHex(newDevAddr);
				switch(button)
				{
					case UP_BUTTON:
						if(ticks_autoincrement >= button_autoincrement_10ms_ticks)
						{
							if(newDevAddr < 0xFE)
								newDevAddr++;
							ticks_autoincrement = 0;
						}
						break;
					case DOWN_BUTTON:
						if(ticks_autoincrement >= button_autoincrement_10ms_ticks)
						{
							if(newDevAddr > 0)
								newDevAddr--;
							ticks_autoincrement = 0;
						}
						break;
					case SELECT_BUTTON:
						mrbus_dev_addr = newDevAddr;
						// FIXME: write to EEPROM
						lcd_clrscr();
						lcd_gotoxy(1,0);
						lcd_puts("SAVED!");
						wait100ms(7);
						screenState = LAST_SCREEN;
						break;
					case MENU_BUTTON:
						newDevAddr = mrbus_dev_addr;  // Reset newDevAddr since no changes were commited
						break;
					case NO_BUTTON:
						break;
				}
				break;

			case DEBUG_SCREEN:
				lcdBacklightEnable();
				lcd_gotoxy(0,0);
				if(0 == throttlePosition)
				{
					lcd_putc('I');
				}
				else
				{
					lcd_putc('0' + throttlePosition);
				}
				lcd_putc(' ');		
		
				lcd_gotoxy(2,0);
				if(brakePosition & 0x80)
				{
					lcd_putc('E');
					lcd_putc('M');
					lcd_putc('R');
					lcd_putc('G');
				}
				else
				{
					uint8_t brakePcnt = 100 * brakePosition / 128;
					lcd_putc(' ');
					printDec2Dig(brakePcnt);
					lcd_putc('%');
				}

				lcd_gotoxy(7,0);
				switch(reverserPosition)
				{
					case FORWARD:
						lcd_putc('F');
						break;
					case NEUTRAL:
						lcd_putc('N');
						break;
					case REVERSE:
						lcd_putc('R');
						break;
				}

				lcd_gotoxy(2, 1);
				lcd_puts((controls & DYNAMIC_CONTROL) ? "DB":"  ");
				lcd_gotoxy(4, 1);
				lcd_putc((controls & BELL_CONTROL) ? 'B' : ' ');
				lcd_gotoxy(5, 1);
				lcd_putc((controls & HORN_CONTROL) ? 'H' : ' ');

				lcd_gotoxy(7, 1);
				switch(frontLight)
				{
					default:
					case LIGHT_OFF:
						lcd_putc('-');
						break;
					case LIGHT_DIM:
						lcd_putc('D');
						break;
					case LIGHT_BRIGHT:
						lcd_putc('B');
						break;
					case LIGHT_BRIGHT_DITCH:
						lcd_putc('*');
						break;
				}

				lcd_gotoxy(0, 1);
				switch(rearLight)
				{
					case LIGHT_OFF:
						lcd_putc('-');
						break;
					case LIGHT_DIM:
						lcd_putc('D');
						break;
					case LIGHT_BRIGHT:
						lcd_putc('B');
						break;
					case LIGHT_BRIGHT_DITCH:
						lcd_putc('*');
						break;
				}

				break;

			case VBAT_SCREEN:
				lcdBacklightEnable();
				lcd_gotoxy(0,0);
				lcd_puts("BATTERY");
				lcd_gotoxy(1,1);
				lcd_putc('0' + (((batteryVoltage*2)/100)%10));
				lcd_putc('.');
				lcd_putc('0' + (((batteryVoltage*2)/10)%10));
				lcd_putc('0' + ((batteryVoltage*2)%10));
				lcd_putc('V');
				break;

			case LAST_SCREEN:
			default:
				// Clean up and reset
				lcd_clrscr();
				screenState = 0;
				break;
		}
		// Process Menu button, but only if not in a subscreen
		// Do this after main screen loop so screens can also do cleanup when menu is pressed
		if(!subscreenStatus)
		{
			if(MENU_BUTTON == button)
			{
				if(MENU_BUTTON != previousButton)
				{
					// Menu pressed, advance menu
					lcd_clrscr();
					screenState++;  // No range checking needed since LAST_SCREEN will reset the counter
					ticks_autoincrement = 0;
				}
				if(ticks_autoincrement >= button_autoincrement_10ms_ticks)
				{
					// Reset menu on long press

					screenState = LAST_SCREEN;
				}
			}
		}

		previousButton = button;

		do
		{
			gettimeofday(&endTime, NULL);
			seconds  = endTime.tv_sec  - startTime.tv_sec;
			useconds = endTime.tv_usec - startTime.tv_usec;
			mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
		} while(mtime < 10);

		// Wait 10ms...  simulate timer interrupt

		status |= STATUS_READ_SWITCHES;

		if (++ticks >= 10)  // 100ms
		{
			ticks = 0;
			decisecs++;
		}

		if(ticks_autoincrement < button_autoincrement_10ms_ticks)
				ticks_autoincrement++;

	}

	endwin();
	nocbreak();
	return 0;
}

