#ifndef _CST_PRESSURE_H_
#define _CST_PRESSURE_H_

#define PRESSURE_CONFIG_DEFAULT    0x03

void updatePressure10Hz(void);

void setupPressureChars(void);
void processPressure(uint8_t brakePcnt);
void printPressure(void);
void resetPressure(void);
uint8_t isCompressorRunning(uint8_t compressorStop);
uint8_t isPressureIdle(void);
uint8_t isBrakeTestActive(void);
uint8_t setPumpRate(uint8_t pumpRate);
uint8_t getPumpRate(void);
uint8_t incrementPumpRate(void);
uint8_t decrementPumpRate(void);
uint8_t setPressureConfig(uint8_t c);
uint8_t getPressureConfig(void);

#endif
