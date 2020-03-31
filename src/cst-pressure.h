#ifndef _CST_PRESSURE_H_
#define _CST_PRESSURE_H_

#define PRESSURE_COEF_DEFAULT    2

void updatePressure10Hz(void);

void setupPressureChars(void);
void processPressure(void);
void printPressure(void);
void resetPressure(void);
void enableBrakeTest(void);
void disableBrakeTest(void);
uint8_t isPressurePumping(void);
uint8_t isBrakeTestActive(void);
uint8_t setPumpRate(uint8_t pumpRate);
uint8_t getPumpRate(void);
uint8_t incrementPumpRate(void);
uint8_t decrementPumpRate(void);
uint8_t setPressureCoefficients(uint8_t c);
uint8_t getPressureCoefficients(void);

#endif
