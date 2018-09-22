#ifndef _CST_BATTERY_H_
#define _CST_BATTERY_H_

typedef enum
{
	FULL = 0,
	HALF,
	EMPTY,
	CRITICAL
} BatteryState;

void setupBatteryChar(void);
uint8_t getBatteryVoltage(void);
BatteryState getBatteryState(void);
void setBatteryVoltage(uint8_t voltage);
void printBattery(void);

#endif
