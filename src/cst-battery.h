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
void setBatteryLevels(uint8_t centivoltsOkay, uint8_t centivoltsWarn, uint8_t centivoltsCritical);

uint8_t getBatteryOkay(void);
uint8_t getBatteryWarn(void);
uint8_t getBatteryCritical(void);

#endif
