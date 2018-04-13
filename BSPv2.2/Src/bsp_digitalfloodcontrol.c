#include "bsp_digitalfloodcontrol.h"
#include "tim.h"
#include "bsp_common.h"



volatile uint8_t          Vthreshold;
volatile uint16_t         Vbase;          //基础电流值
volatile uint16_t		  Vdelta;

volatile uint16_t		TiggerTimeSum;
volatile uint16_t		TiggerTimeCnt;
volatile uint16_t       Vnormal;
volatile uint16_t       Vnormalt;


uint16_t	          VbaseBuffer[512];
volatile uint16_t	  VbaseCnt;
volatile uint8_t         VbaseUpdataflag;  //在开始落闸的时候写1，如果中途遇阻，设置为0，如果中途未遇阻，则不写零，在更新数据完成后置零
volatile uint8_t         SettingVbaseValeFlag;


BSP_StatusTypeDef UpdateVbaseValue(void)
{
	BSP_StatusTypeDef  state = BSP_OK;
	uint16_t i;
	uint32_t sum;
	if(0 == VbaseUpdataflag)
	{
		return state;
	}
	VbaseUpdataflag = 0;

	for(i = 0; i < VbaseCnt;i++)
	{
		sum += VbaseBuffer[i];
	}
	
	Vbase = (uint16_t)(sum / VbaseCnt);
	VbaseCnt = 0;
        if(SettingVbaseValeFlag)
        {
          SettingVbaseValeFlag = 0;
        }
	/* 此处可以添加写日志信息，动态的上报电流值，来检测设备是否有问题 */
	
	return state;
}


BSP_StatusTypeDef DigitalfloodInit(void)
{
  BSP_StatusTypeDef  state = BSP_OK;
  Vthreshold = 0x3E;  	//deflaut tirgger I is 2A
  TiggerTimeSum = 0x19; //默认反映精度为0.25s
  SettingVbaseValeFlag = 1;
  
  
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
	Vnormal = 0;
	Vnormalt = 0;
	TiggerTimeCnt = 0;
	HAL_TIM_Base_Stop_IT(&htim6);
	
  	//电机停止转动，写遇阻标记位
	
  }


  //停止定时器，当干水平到位时立即停止定时器
  //HAL_TIM_Base_Stop_IT(&htim6);
  
  return state;
}


