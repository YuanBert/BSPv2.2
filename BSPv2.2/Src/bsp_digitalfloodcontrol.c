#include "bsp_digitalfloodcontrol.h"
#include "tim.h"
#include "bsp_common.h"
#include "bsp_DataTransmissionLayer.h"
#include "bsp_log.h"



uint16_t     Vthreshold;
uint16_t     Vbase;          //基础电流值
uint16_t     Vdelta;

uint16_t     Vnormalt;

uint16_t	    Vnormal;
uint16_t		TiggerTimeSum;

volatile    uint16_t	TiggerTimeCnt;
volatile    uint8_t 	gDigitalFoodFlag;


volatile uint16_t	  VbaseBuffer[128];
volatile uint16_t	  VbaseCnt;

volatile uint8_t         SettingVbaseValeFlag;


BSP_StatusTypeDef UpdateVbaseValue(void)
{
	BSP_StatusTypeDef  state = BSP_OK;
	uint16_t i;
	uint32_t sum;
	uint8_t VbaseValueBuffer[4];

	for(i = 0,sum = 0; i < 128;i++)
	{
		sum += VbaseBuffer[i];
	}
		
	Vbase = (uint16_t)(sum >> 7);  //更新Vbase值,采用的是均值滤波的方法，可以进行优化
		/* 添加日志信息,将基准电流上报 */
    gLogInfo.uLogInfo.PeakCurrent = Vbase;
	bsp_LogWriteUpdataFlag();
	
#ifdef __Debug__
	BSP_SendDataToDriverBoard((uint8_t*)VbaseBuffer,256,0xFFFF);
	VbaseValueBuffer[0] = 0xAA;
	VbaseValueBuffer[1] = Vbase >> 8;
	VbaseValueBuffer[2] = Vbase;
    VbaseValueBuffer[3] = 0xAA;
	BSP_SendDataToDriverBoard(VbaseValueBuffer,4,0xFFFF);
#endif	

	VbaseCnt = 0;
	TiggerTimeCnt = 0;
	//SettingVbaseValeFlag = 0;	//该标记位是初次使用标记的，初次上电的时候，如要进行Vbase的自动采样，此时数字防砸是禁用的
	
	return state;
}


BSP_StatusTypeDef DigitalfloodInit(void)
{
  BSP_StatusTypeDef  state = BSP_OK;
  Vthreshold = 0x20;  	//deflaut tirgger I is 0x39 -- 0.5A  0xF5 -- 2A
  TiggerTimeSum = 0x10; //默认反映精度为0.15s
  SettingVbaseValeFlag = 1;
  gDigitalFoodFlag = 0;
  VbaseCnt = 0;
  Vbase = 0;
  
  return state;  
}

BSP_StatusTypeDef SetTiggerSumValue(uint16_t value)
{
	BSP_StatusTypeDef  state = BSP_OK;
	TiggerTimeSum = value;
	return state;

}


BSP_StatusTypeDef SetThresholdValue(uint8_t value)
{
  BSP_StatusTypeDef  state = BSP_OK;
  Vthreshold = value;
  return state;
}

BSP_StatusTypeDef CheckDigitalfloodontrol(void)
{
  BSP_StatusTypeDef  state = BSP_OK;
  //判断是否处在正常关闸状态，否返回
  HAL_TIM_Base_Start_IT(&htim6);
  
  if(TiggerTimeCnt > TiggerTimeSum)
  {
	HAL_TIM_Base_Stop_IT(&htim6);
	Vnormal = 0;
	Vnormalt = 0;
	TiggerTimeCnt = 0;
	gDigitalFoodFlag = 1;
  	//电机停止转动，写遇阻标记位
	
  }


  //停止定时器，当干水平到位时立即停止定时器
  //HAL_TIM_Base_Stop_IT(&htim6);
  
  return state;
}


