#include "bsp_motor.h"
#include "tim.h"
#include "adc.h"
#include "bsp_DataTransmissionLayer.h"


extern  uint32_t ADCBuffer[256];
extern 			 uint16_t		Vnormal;
extern volatile uint16_t		TiggerTimeSum;
extern volatile uint16_t		TiggerTimeCnt;
extern volatile uint8_t 	    gDigitalFoodFlag;

extern volatile uint8_t gOpenFlag;
extern MOTORMACHINE gMotorMachine;
extern GPIOSTRUCT gGentleSensorGpio;
extern GPIOSTRUCT gAirSensorGpio;

BSP_StatusTypeDef      BSP_MotorInit(void)
{
  BSP_StatusTypeDef state  = BSP_OK;
  HAL_GPIO_WritePin(MotorFRCtrl_GPIO_Port,MotorFRCtrl_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(MotorENCtrl_GPIO_Port,MotorENCtrl_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(MotorBRKCtrl_GPIO_Port,MotorBRKCtrl_Pin,GPIO_PIN_SET);

  /* �˴���ӱ��λ��ʼ������ */
  gMotorMachine.RunDir = DOWNDIR;
  gMotorMachine.RunningState = 0;
  gMotorMachine.StartFlag = 1;
  return state;
}
BSP_StatusTypeDef      BSP_MotorOpen(void)
{
  BSP_StatusTypeDef state  = BSP_OK;
  HAL_GPIO_WritePin(MotorBRKCtrl_GPIO_Port,MotorBRKCtrl_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MotorFRCtrl_GPIO_Port,MotorFRCtrl_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MotorENCtrl_GPIO_Port,MotorENCtrl_Pin,GPIO_PIN_RESET);  
  return state;	
}
BSP_StatusTypeDef      BSP_MotorClose(void)
{
	BSP_StatusTypeDef state  = BSP_OK;  
  	HAL_GPIO_WritePin(MotorBRKCtrl_GPIO_Port,MotorBRKCtrl_Pin,GPIO_PIN_RESET);
 	HAL_GPIO_WritePin(MotorFRCtrl_GPIO_Port,MotorFRCtrl_Pin, GPIO_PIN_SET);
  	HAL_GPIO_WritePin(MotorENCtrl_GPIO_Port,MotorENCtrl_Pin,GPIO_PIN_RESET);
  	return state;
}
BSP_StatusTypeDef      BSP_MotorRun(uint8_t nDir)
{
  BSP_StatusTypeDef state  = BSP_OK;
  if(UPDIR == nDir) //��ʱ��
  {
    HAL_GPIO_WritePin(MotorBRKCtrl_GPIO_Port,MotorBRKCtrl_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MotorFRCtrl_GPIO_Port,MotorFRCtrl_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MotorENCtrl_GPIO_Port,MotorENCtrl_Pin,GPIO_PIN_RESET);
	BSP_DAC5571_WriteValue(NormalOperationMode, 120);
  }
  
  if(DOWNDIR == nDir) //˳ʱ��
  {
    HAL_GPIO_WritePin(MotorBRKCtrl_GPIO_Port,MotorBRKCtrl_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MotorFRCtrl_GPIO_Port,MotorFRCtrl_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MotorENCtrl_GPIO_Port,MotorENCtrl_Pin,GPIO_PIN_RESET);    
  }
  return state;
}


BSP_StatusTypeDef      BSP_MotorStop(void)
{
  BSP_StatusTypeDef state  = BSP_OK;
  HAL_GPIO_WritePin(MotorBRKCtrl_GPIO_Port,MotorBRKCtrl_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(MotorFRCtrl_GPIO_Port,MotorFRCtrl_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MotorENCtrl_GPIO_Port,MotorENCtrl_Pin,GPIO_PIN_SET);
  return state;	
}

BSP_StatusTypeDef	   BSP_MotorSpeedSet(uint8_t vSpeed)
{
	BSP_StatusTypeDef state  = BSP_OK;
	BSP_DAC5571_WriteValue(NormalOperationMode, vSpeed);
	return state;	
}

BSP_StatusTypeDef      BSP_MotorCheck(void)
{
	BSP_StatusTypeDef state  = BSP_OK;

	//�ظд����ǲ�ִ�в���
//	if(1 == gGentleSensorGpio.GpioState)
//	{
//		/* �ϴ�����ͣ����ʱ��Ϣ�����Ը��ݴ˽�����ʾ�ͻ� */
//      
//      return state;
//	}

	
	
	if(0 == gMotorMachine.HorizontalRasterState && 0 == gMotorMachine.VerticalRasterState)
	{
		if(1 == gMotorMachine.RunningState && DOWNDIR == gMotorMachine.RunDir)
		{
			if(TiggerTimeCnt > TiggerTimeSum)
			{
				gDigitalFoodFlag = 1;
			}
			/* ���ַ��� *//* ѹ�������� */
			if(1 == gDigitalFoodFlag || (gAirSensorGpio.GpioState && 0)) //��ʱ��ѹ��������������
			{
				HAL_TIM_Base_Stop_IT(&htim6);
				gMotorMachine.RunningState = 0;
				BSP_MotorStop();
				Vnormal = 0;
				TiggerTimeCnt = 0;
				gOpenFlag = 1;
				gDigitalFoodFlag = 0;
#ifdef __Debug__
                //BSP_SendDataToDriverBoard((uint8_t*)"\r\n TiggerTimeCnt > TiggerTimeSum\r\n",35, 0xFFFF);
#endif
				/* д��־��Ϣ������������Ϣ */				
			}
			
			/* ���Ҳ��� */
			/*HAL_TIM_Base_Stop_IT(&htim6);//���跴���ǹر�ADC������Ҳ���ǹر����ַ���	*/
			/* ���ͣת���������з���ת����������״̬�ı䣬gOpenFlag = 1, */
			
			return state;
		}

		if(0 == gMotorMachine.RunningState)
		{
			gOpenFlag = 4; //ִ�����µĲ���
			
			if(1 == gMotorMachine.StartFlag)
			{
			  gMotorMachine.StartFlag = 0;
			}
			return state;
		}
	}


	return state;
}
BSP_StatusTypeDef      BSP_MotorAction(void)
{
	BSP_StatusTypeDef state  = BSP_OK;
	if(1 == gMotorMachine.RunningState)
	{
		return state;
	}
	
	if(1 == gOpenFlag)
	{
		if(1 == gMotorMachine.VerticalRasterState)
		{
			gOpenFlag = 3;
			return state;
		}
		gMotorMachine.RunningState = 1;
		gMotorMachine.RunDir = UPDIR;
		BSP_MotorRun(gMotorMachine.RunDir);
		BSP_MotorSpeedSet(100);
		gOpenFlag = 2;
		HAL_TIM_Base_Start_IT(&htim4);
		
                
#ifdef __Debug__
		//BSP_SendDataToDriverBoard((uint8_t*)"\r\n BSP_MotorAction gOpenFlag = 1\r\n",35, 0xFFFF);
#endif
	return state;

                
	}

	if(4 == gOpenFlag)
	{
		if(1 == gMotorMachine.HorizontalRasterState)
		{
			gOpenFlag = 0;
			return state;
		}
		gMotorMachine.RunningState = 1;
		gMotorMachine.RunDir = DOWNDIR;
		BSP_MotorRun(gMotorMachine.RunDir);
		BSP_MotorSpeedSet(86);
		
        HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADCBuffer, 32);
        //��TIM6��ʱ���ж�
		HAL_TIM_Base_Start_IT(&htim6);
        
		gOpenFlag = 5; //���ڹ�բ��״̬
		return state;
	}

	return state;
}
BSP_StatusTypeDef      BSP_MotorCheckA(void);
BSP_StatusTypeDef      BSP_MotorActionA(void);
