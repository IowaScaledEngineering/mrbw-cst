#ifndef _CST_COMMON_H_
#define _CST_COMMON_H_

#define LOCO_ADDRESS_SHORT 0x8000

// Default
#define BATTERY_CHAR            0
#define FUNCTION_INACTIVE_CHAR  1
#define FUNCTION_ACTIVE_CHAR    2
#define AM_CHAR                 3
#define PM_CHAR                 4
#define AUX_CHAR                5

// Pressure
#define PRESSURE_CHAR_A0        0
#define PRESSURE_CHAR_A1        1
#define PRESSURE_CHAR_A2        2
#define PRESSURE_CHAR_A3        3
#define PRESSURE_CHAR_B0        4
#define PRESSURE_CHAR_B1        5
#define PRESSURE_CHAR_B2        6
#define PRESSURE_CHAR_B3        7

// Tonnage
#define TONNAGE_TOP             6
#define TONNAGE_BOTTOM          7

// Diags
#define BELL_CHAR               6
#define HORN_CHAR               7

void wait100ms(uint16_t loops);

#endif
