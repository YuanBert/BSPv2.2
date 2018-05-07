#include "bsp_wrieless.h"
#include "bsp_motor.h"
#include "bsp_Log.h"
#include "BSP_DAC5571.h"




uint8_t gWrielessActionFlag;
uint8_t gWrielessModeFlag;

extern MOTORMACHINE gMotorMachine;

static uint8_t wrielessCurrentReadValue;
static uint8_t wrielessLastReadValue;
static uint8_t wrielessFilterCnt;
static uint8_t wrielessFilterSum;
static uint8_t wrielessGpioState;
static uint8_t wrielessGpioRelease;


BSP_StatusTypeDef  BSP_WirelessInit(uint8_t nFilterSum)
{
	BSP_StatusTypeDef state = BSP_OK;
	wrielessFilterSum = nFilterSum;

	return state;
}

BSP_StatusTypeDef  BSP_WirelessCheck(void)
{
	BSP_StatusTypeDef state = BSP_OK;
	wrielessCurrentReadValue = HAL_GPIO_ReadPin(MCU_WIRLESS_GPIO_Port, MCU_WIRLESS_Pin);
	if(1 == wrielessCurrentReadValue && 1 == wrielessLastReadValue)
	{
		if(0 == wrielessGpioState && wrielessGpioRelease)
		{
			wrielessFilterCnt ++;
			if(wrielessFilterCnt > wrielessFilterSum)
			{
				wrielessGpioState = 1;
				wrielessFilterCnt = 0;
				wrielessGpioRelease = 0;
			}
		}
	}
	else
	{
		wrielessFilterCnt = 0;
		wrielessGpioState = 0;
		wrielessGpioRelease = 1;
	}
	wrielessLastReadValue = wrielessCurrentReadValue;

	if(0 == wrielessGpioState)
	{
		return state;
	}

	

	//��ˮƽλ��ִ�п�բ����
	if(1 == gMotorMachine.HorizontalRasterState && 0 == gMotorMachine.RunningState)
	{
		gWrielessActionFlag = 2;
        
        gLogInfo.uLogInfo.GateWay = 2;
        bsp_LogWriteUpdataFlag();
        
		gWrielessModeFlag = 1;
		wrielessGpioState = 0;
		return state;
	}

	//�ڴ�ֱλ��ִ�й�բ����
	if(1 == gMotorMachine.VerticalRasterState && 0 == gMotorMachine.RunningState)
	{
		gWrielessActionFlag = 3;
        /* �����־*/
        gLogInfo.uLogInfo.GateWay = 3;
        bsp_LogWriteUpdataFlag();
        
		gWrielessModeFlag = 1;
		wrielessGpioState = 0;
		return state;
	}

	//������ͣλ��ʱ���п�բ����
	if(1 == gWrielessActionFlag && 0 == gMotorMachine.RunningState)
	{
		gWrielessActionFlag = 2;
        
        gLogInfo.uLogInfo.GateWay = 2;
        bsp_LogWriteUpdataFlag();
        
		gWrielessModeFlag = 1;
		wrielessGpioState = 0;
		return state;
	}
	
	//�����˶�״̬ʱ������ң������ͣ
	if(1 == gMotorMachine.RunningState)
	{
		gWrielessActionFlag = 1; //��ͣ
        
        gLogInfo.uLogInfo.GateWay = 1;
        bsp_LogWriteUpdataFlag();
        
		gWrielessModeFlag = 1;	//����ң�ؿ�բģʽ
		wrielessGpioState = 0;
		return state;
	}
	return state;
}

BSP_StatusTypeDef  BSP_WirelessAction(void)
{
	BSP_StatusTypeDef state = BSP_OK;

	switch (gWrielessActionFlag)
		{
		case 1:
			BSP_MotorStop();
			gMotorMachine.RunningState = 0;
			/* �����־ */
			break;
			
		case 2:
			gMotorMachine.RunningState = 1;
			gMotorMachine.RunDir = UPDIR;
			BSP_MotorRun(UPDIR);
			BSP_DAC5571_WriteValue(NormalOperationMode, 60);
			break;

	    case 3:
			gMotorMachine.RunningState = 1;
			gMotorMachine.RunDir = DOWNDIR;
			BSP_MotorRun(DOWNDIR);
			BSP_DAC5571_WriteValue(NormalOperationMode, 60);
			//BSP_Log_UpOpenMode(3 + (gWrielessModeFlag << 4));
			break;
			
	default:
		//BSP_Log_UpOpenMode(0 + (gWrielessModeFlag << 4));
      /* �����־*/
		break;
		
		}
	return state;
}

//END OF FILE 

