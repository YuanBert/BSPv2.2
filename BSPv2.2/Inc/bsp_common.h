/**
  ******************************************************************************
  * File Name          : bsp_common.h
  * Description        : 
  *
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __bsp_common_H
#define __bsp_common_H
#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
//#include "bsp_DataTransmissionLayer.h"

#define __Debug__

#define RunningLED_OFF                  GPIO_PIN_SET
#define RunningLED_ON                   GPIO_PIN_RESET
#define CommunicationLED_OFF            GPIO_PIN_SET
#define CommunicationLED_ON             GPIO_PIN_RESET
#define MCUAtmosphereLEDG_ON            GPIO_PIN_SET
#define MCUAtmosphereLEDG_OFF           GPIO_PIN_RESET
#define MCUAtmosphereLEDR_ON            GPIO_PIN_SET
#define MCUAtmosphereLEDR_OFF           GPIO_PIN_RESET
  
   
   typedef struct {
	uint8_t CurrentReadVal;
	uint8_t	LastReadVal;
    uint8_t GpioState;
	uint8_t FilterCnt;
	uint8_t FilterCntSum;
    }GPIOSTRUCT, pGPIOSTRUCT;

   /** enum: BSP_StatusTypeDef
   **
   ** DESCRIPTION:
   **  --��բ�������
   **
   ** CREATED: 2017/12/7, by bert
   **
   ** FILE: 
   **
   ** AUTHOR: Bert.Zhang
   ********************************************************************************
   */
   typedef enum
   {
     BSP_OK       = 0x00U,
     BSP_ERROR    = 0x01U,
     BSP_BUSY     = 0x02U,
     BSP_TIMEOUT  = 0x03U,
     BSP_NOCMD    = 0x04U
   }BSP_StatusTypeDef;

/*******************************************************************************
** struct: sGpioStatusDetection
**
** DESCRIPTION:
**  --gpio status 
**
** CREATED: 2017/12/26, by bert
**
** FILE: 
**
** AUTHOR: Bert.Zhang
********************************************************************************
*/
struct sGpioStatusDetection
{
  uint8_t   GpioCurrentReadVal;                 //Current GPIO value
  uint8_t   GpioLastReadVal;                    //Last GPIO value
  uint8_t   GpioFilterCnt;                      //Filter times
  uint16_t  GpioFilterCntSum;
  uint8_t   GpioStatusVal;                      //GPIO true logic state
  uint8_t   GpioCheckedFlag;                    //vehicle is stilled Flag
  uint8_t   GpioSendDataFlag;                   //Send data flag,it is zero if data been sent within asingle logic ,
                                                //and set to one in the next logic
  uint32_t  GpioValidLogicTimeCnt;              //Gpio vaild logic time counter
}; 

typedef struct sGpioStatusDetection  GPIOSTATUSDETECTION, *pGPIOSTATUSDETECTION;

   typedef struct s_MotorMachine{
     uint8_t  VerticalRasterState;  
     uint8_t  HorizontalRasterState;
     uint8_t  RunningState;
     uint8_t  RunDir;
     uint16_t SetRunSpeed;
     uint16_t CurrentRunSpeed;
     uint8_t  StartFlag;
     uint8_t  VerFilterCnt;
     uint8_t  HorFilterCnt;
     uint8_t  Motor_Error;
     uint8_t  GentleSensorFlag;
     uint8_t  RadarSensorFlag;
     uint8_t  AirSensorFlag;
     uint8_t  RemoteControFlag;
     uint8_t  OpenFlag;
     uint8_t  CloseFlag;
     uint8_t  EncounteredFlag;
     uint8_t  DigitalAntiSmashingFlag;
     uint16_t  StepCnt;
   }MOTORMACHINE,*pMOTORMACHINE;

#ifdef __cplusplus
}
#endif
#endif /*__bsp_common_H */