#ifndef __bsp_log_H
#define __bsp_log_H


#ifdef __cplusplus
extern "C"{
#endif

#include "stm32f1xx_hal.h"

struct LogInfo{
	uint32_t ID;
	uint16_t PeakCurrent;
	uint8_t  lightState;
	uint8_t  OpenSpeed;
	uint16_t MotorSpeed;
	uint8_t  MotorState;
	uint8_t  GateWay;
	uint8_t  GentleSensorState;
	uint8_t  AirSensorState;
	uint8_t  Revs[10];
}__attribute__((aligned(1)));


typedef struct LogInfo tLogInfo;

union _uLogInfo{
	tLogInfo uLogInfo;
	uint8_t uLogBuffer[24];
};

typedef union _uLogInfo uLogInfo;

extern uLogInfo gLogInfo;

void bsp_LogInit(void);
void bsp_LogWriteUpdataFlag(void);
void bsp_LogClearUpdataFlag(void);
void bsp_LogCheckUpdata(void);

#ifdef __cplusplus
}
#endif

#endif

