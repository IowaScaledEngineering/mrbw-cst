#ifndef _CST_ENGINE_H_
#define _CST_ENGINE_H_

#define ENGINE_STATE_QUEUE_SIZE   8

typedef enum
{
	ENGINE_OFF = 0,
	ENGINE_START,
	ENGINE_ON,
	ENGINE_RUNNING,
	ENGINE_NOT_IDLE,
	ENGINE_STOP,
	ENGINE_NOT_INITIALIZED,
} EngineState;

void engineStatesQueueInitialize(void);
void engineStatesQueueUpdate(uint16_t locoAddr, EngineState engineState);
EngineState engineStatesQueueGetState(uint16_t locoAddr);
uint16_t engineStatesQueuePeekLocoAddress(uint8_t index);
EngineState engineStatesQueuePeekState(uint8_t index);
void printEngineState(EngineState engineState);

#endif
