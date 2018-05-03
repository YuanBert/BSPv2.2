#ifndef __bsp_log_H
#define __bsp_log_H


#ifdef __cplusplus
extern "C"{
#endif

#include "stm32f1xx_hal.h"

struct LogInfo{
	uint32_t ID;
	uint16_t PeakCurrent;
	uint8_t  lightStates;
	uint8_t  OpenSpeed;
	uint16_t MotorSpeed;
	uint8_t  GateWay;
	uint8_t gen
}__attribute__((aligned(1)));




#ifdef __cplusplus
}
#endif

#endif

