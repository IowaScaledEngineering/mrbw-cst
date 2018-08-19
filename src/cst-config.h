#ifndef _CST_CONFIG_H_
#define _CST_CONFIG_H_

#define OPTIONBITS_ESTOP_ON_BRAKE    0
#define OPTIONBITS_REVERSER_SWAP     1
#define OPTIONBITS_VARIABLE_BRAKE    2
#define OPTIONBITS_STEPPED_BRAKE     3
#define OPTIONBITS_PARABOLIC_BRAKE   4

#define OPTIONBITS_DEFAULT                 (_BV(OPTIONBITS_ESTOP_ON_BRAKE) | _BV(OPTIONBITS_PARABOLIC_BRAKE))
extern uint8_t optionBits;

// BRAKE_PULSE_WIDTH is in decisecs
// It is the minimum on time for the pulsed brake
#define BRAKE_PULSE_WIDTH_MIN       2
#define BRAKE_PULSE_WIDTH_DEFAULT   5
#define BRAKE_PULSE_WIDTH_MAX      10

extern uint8_t brakePulseWidth;

#endif
