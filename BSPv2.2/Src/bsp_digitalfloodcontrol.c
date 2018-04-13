#include "bsp_digitalfloodcontrol.h"
#include "tim.h"
#include "bsp_common.h"



volatile uint8_t          Vthreshold;
volatile uint16_t         Vbase;          //��������ֵ
volatile uint16_t		  Vdelta;

volatile uint16_t		TiggerTimeSum;
volatile uint16_t		TiggerTimeCnt;
volatile uint16_t       Vnormal;
volatile uint16_t       Vnormalt;


uint16_t	          VbaseBuffer[512];
volatile uint16_t	  VbaseCnt;
volatile uint8_t         VbaseUpdataflag;  //�ڿ�ʼ��բ��ʱ��д1�������;���裬����Ϊ0�������;δ���裬��д�㣬�ڸ���������ɺ�����
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
	/* �˴��������д��־��Ϣ����̬���ϱ�����ֵ��������豸�Ƿ������� */
	
	return state;
}


BSP_StatusTypeDef DigitalfloodInit(void)
{
  BSP_StatusTypeDef  state = BSP_OK;
  Vthreshold = 0x3E;  	//deflaut tirgger I is 2A
  TiggerTimeSum = 0x19; //Ĭ�Ϸ�ӳ����Ϊ0.25s
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
  //�ж��Ƿ���������բ״̬���񷵻�
  HAL_TIM_Base_Start_IT(&htim6);
  if(TiggerTimeCnt > TiggerTimeSum)
  {
	Vnormal = 0;
	Vnormalt = 0;
	TiggerTimeCnt = 0;
	HAL_TIM_Base_Stop_IT(&htim6);
	
  	//���ֹͣת����д������λ
	
  }


  //ֹͣ��ʱ��������ˮƽ��λʱ����ֹͣ��ʱ��
  //HAL_TIM_Base_Stop_IT(&htim6);
  
  return state;
}


