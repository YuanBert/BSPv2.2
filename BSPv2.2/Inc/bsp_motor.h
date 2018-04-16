#ifndef __bsp_motor_H
#define __bsp_motor_H

#ifdef __cplusplus
extern "C"{
#endif

/************* INCLUDES ********************/
#include "bsp_common.h"
#include "bsp_dac5571.h"

/************ DEFINES *********************/
#define		UPDIR		0
#define     DOWNDIR		1


enum e_Motor_Error_Code{
		MotorOK = 0,
		MotorRunErr = 1,
		MotorVerRasterErr,
		MotorHorRasterErr,
		MotorOtherErr
};


BSP_StatusTypeDef      BSP_MotorInit(void);
BSP_StatusTypeDef      BSP_MotorOpen(void);
BSP_StatusTypeDef      BSP_MotorClose(void);
BSP_StatusTypeDef      BSP_MotorRun(uint8_t nDir);
BSP_StatusTypeDef      BSP_MotorStop(void);
BSP_StatusTypeDef      BSP_MotorSpeedSet(uint8_t vSpeed);
BSP_StatusTypeDef      BSP_MotorCheck(void);
BSP_StatusTypeDef      BSP_MotorAction(void);
BSP_StatusTypeDef      BSP_MotorCheckA(void);
BSP_StatusTypeDef      BSP_MotorActionA(void);


#ifdef __cplusplus
}
#endif

#endif
