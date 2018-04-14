#ifndef _CST_COMMON_H_
#define _CST_COMMON_H_

#define LOCO_ADDRESS_SHORT 0x8000

#define BATTERY_CHAR            0
#define BELL_CHAR               1
#define HORN_CHAR               2
#define AM_CHAR                 1
#define PM_CHAR                 2
#define FUNCTION_INACTIVE_CHAR  3
#define FUNCTION_ACTIVE_CHAR    4
#define TONNAGE_TOP             5
#define TONNAGE_BOTTOM          6

void wait100ms(uint16_t loops);

#endif
