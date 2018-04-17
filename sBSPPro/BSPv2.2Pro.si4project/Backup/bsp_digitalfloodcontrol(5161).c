#include "bsp_digitalfloodcontrol.h"
#include "tim.h"
#include "bsp_common.h"
#include "bsp_DataTransmissionLayer.h"

#define __Debug__

uint8_t     Vthreshold;
uint16_t    Vbase;          //��������ֵ
uint16_t	Vdelta;
uint16_t    Vnormal;
uint16_t    Vnormalt;

volatile uint16_t		TiggerTimeSum;
volatile uint16_t		TiggerTimeCnt;



uint16_t	          VbaseBuffer[512];
volatile uint16_t	  VbaseCnt;
volatile uint8_t         VbaseUpdataflag;  //�ڿ�ʼ��բ��ʱ��д1�������;���裬����Ϊ0�������;δ���裬��д�㣬�ڸ���������ɺ�����
volatile uint8_t         SettingVbaseValeFlag;


BSP_StatusTypeDef UpdateVbaseValue(void)
{
	BSP_StatusTypeDef  state = BSP_OK;
	uint16_t i;
	uint32_t sum;
	uint8_t VbaseValueBuffer[2];

	for(i = 0; i < VbaseCnt;i++)
	{
		sum += VbaseBuffer[i];
	}
		
	Vbase = (uint16_t)(sum / VbaseCnt);  //����Vbaseֵ,���õ��Ǿ�ֵ�˲��ķ��������Խ����Ż�
		/* �����־��Ϣ,����׼�����ϱ� */
	
#ifdef __Debug__	
	VbaseValueBuffer[0] = Vbase >> 8;
	VbaseValueBuffer[1] = Vbase;
	BSP_SendDataToDriverBoard(VbaseValueBuffer,2,0xFFFF);	
#endif	

	VbaseCnt = 0;
	TiggerTimeCnt = 0;
	SettingVbaseValeFlag = 0;	//�ñ��λ�ǳ���ʹ�ñ�ǵģ������ϵ��ʱ����Ҫ����Vbase���Զ���������ʱ���ַ����ǽ��õ�
	
	return state;
}


BSP_StatusTypeDef DigitalfloodInit(void)
{
  BSP_StatusTypeDef  state = BSP_OK;
  Vthreshold = 0x3E;  	//deflaut tirgger I is 2A
  TiggerTimeSum = 0x19; //Ĭ�Ϸ�ӳ����Ϊ0.25s
  SettingVbaseValeFlag = 1;

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


