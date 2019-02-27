#ifndef _CST_PRESSURE_H_
#define _CST_PRESSURE_H_

void updatePressure10Hz(void);

void setupPressureChars(void);
void processPressure(uint8_t notch);
void printPressure(void);
void resetPressure(void);
void toggleBrakeTest(void);

#endif
