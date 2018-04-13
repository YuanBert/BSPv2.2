#ifndef __bsp_digitalfloodcontrol_H
#define __bsp_digitalfloodcontrol_H

#ifdef __cplusplus
extern "C"{
#endif

#include "stm32f1xx_hal.h"
#include "bsp_common.h"

uint8_t GetThresholdValue(void);
uint16_t GetTiggerSumValue(void);

BSP_StatusTypeDef UpdateVbaseValue(void);



BSP_StatusTypeDef DigitalfloodInit(void);

BSP_StatusTypeDef SetTiggerSumValue(uint16_t value);

BSP_StatusTypeDef SetThresholdValue(uint8_t value);




BSP_StatusTypeDef CheckDigitalfloodontrol(void);


#ifdef __cplusplus
}
#endif

#endif
