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

// Tonnage
#define TONNAGE_TOP             6
#define TONNAGE_BOTTOM          7

// Diags
#define BELL_CHAR               6
#define HORN_CHAR               7

void wait100ms(uint16_t loops);

#endif
