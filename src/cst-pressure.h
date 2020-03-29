#ifndef _CST_PRESSURE_H_
#define _CST_PRESSURE_H_

void updatePressure10Hz(void);

void setupPressureChars(void);
void processPressure(uint8_t notch);
void printPressure(void);
void resetPressure(void);
void enableBrakeTest(void);
void disableBrakeTest(void);
uint8_t isPressurePumping(void);
uint8_t isBrakeTestActive(void);

#endif
