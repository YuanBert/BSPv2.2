#include "bsp_log.h"
#include "bsp_DataTransmissionLayer.h"


uLogInfo gLogInfo;

static uint8_t UpdataFlag;


void bsp_LogWriteUpdataFlag(void)
{
	UpdataFlag = 1;
}
void bsp_LogClearUpdataFlag(void)
{
	UpdataFlag = 0;
}


void bsp_LogInit(void)
{
	UpdataFlag = 0;
	gLogInfo.uLogInfo.ID = 0x12345678;
	gLogInfo.uLogInfo.OpenSpeed = 2; //1.5s 
}

void bsp_LogCheckUpdata(void)
{
	if(1 == UpdataFlag)
	{
		BSP_SendDataToDriverBoard(gLogInfo.uLogBuffer,24,0xFFFF);
	}

	UpdataFlag = 0;
}




