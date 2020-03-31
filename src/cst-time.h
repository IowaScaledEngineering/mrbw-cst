#ifndef _CST_TIME_H_
#define _CST_TIME_H_

#define DEAD_RECKONING_TIME_MIN             10
#define DEAD_RECKONING_TIME_DEFAULT        100

#define TIME_FLAGS_DISP_FAST       0x01
#define TIME_FLAGS_DISP_FAST_HOLD  0x02
#define TIME_FLAGS_DISP_REAL_AMPM  0x04
#define TIME_FLAGS_DISP_FAST_AMPM  0x08

typedef struct
{
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t dayOfWeek;
	uint8_t day;
	uint8_t month;
	uint16_t year;
} TimeData;

void setupClockChars(void);
void incrementTime(TimeData* t, uint8_t incSeconds);
void displayTime(TimeData* time, uint8_t ampm);
void printTime(void);
void processTimePacket(uint8_t* pkt);
void updateTime10Hz(void);
void updateTime(void);
uint16_t convertMaxDeadReckoningToDecisecs(void);
uint8_t incrementMaxDeadReckoningTime(void);
uint8_t decrementMaxDeadReckoningTime(void);
uint8_t setMaxDeadReckoningTime(uint8_t t);
uint8_t getMaxDeadReckoningTime(void);
uint16_t getTimeScaleFactor(void);
void clearDeadReckoningTime(void);

#endif

