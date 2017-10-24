#ifndef _CST_LCD_H_
#define _CST_LCD_H_

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

extern const uint8_t Bell[8];
extern const uint8_t Horn[8];
extern const uint8_t BatteryFull[8];
extern const uint8_t BatteryHalf[8];
extern const uint8_t BatteryEmpty[8];
extern const uint8_t SoftkeyInactive[8];
extern const uint8_t SoftkeyActive[8];
extern const uint8_t BarGraphBottomEmpty[8];
extern const uint8_t BarGraphBottomHalf[8];
extern const uint8_t BarGraphTopEmpty[8];
extern const uint8_t BarGraphTopHalf[8];
extern const uint8_t BarGraphFull[8];
extern const uint8_t ClockAM[8];
extern const uint8_t ClockPM[8];

void setupClockChars(void);
void setupDiagChars(void);
void setupBatteryChar(BatteryState state);
void setupSoftkeyChars(void);

// Splash Screen Characters
extern const uint8_t Splash1[8];
extern const uint8_t Splash2[8];
extern const uint8_t Splash3[8];
extern const uint8_t Splash4[8];
extern const uint8_t Splash5A[8];
extern const uint8_t Splash5C[8];
extern const uint8_t Splash6A[8];
extern const uint8_t Splash6B[8];
extern const uint8_t Splash6C[8];
extern const uint8_t Splash7A[8];
extern const uint8_t Splash7B[8];
extern const uint8_t Splash7C[8];
extern const uint8_t Splash8A[8];
extern const uint8_t Splash8B[8];

#endif

