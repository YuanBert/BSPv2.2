#include "bsp_log.h"
#include "bsp_DataTransmissionLayer.h"


uLogInfo gLogInfo;


static uint8_t UpdataFlag;

static uint8_t getXORCode(uint8_t* pData,uint16_t len)
{
  uint8_t ret;
  uint16_t i;
  ret = pData[0];
  for(i = 1; i < len; i++)
  {
    ret ^= pData[i];
  }
  return ret;
}

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
	uint8_t HeadBuffer[31];
    uint8_t i;
    if(1 == UpdataFlag)
	{
        HeadBuffer[0] = 0x5B;
        HeadBuffer[1] = 0xD2;
        HeadBuffer[2] = 0x01;
        HeadBuffer[3] = 0x00;
        HeadBuffer[4] = 0x18;
        HeadBuffer[30] = 0x5D;
        for(i = 0; i < 24;i++)
        {
          HeadBuffer[5+i] = gLogInfo.uLogBuffer[i];
        }
        HeadBuffer[29] = getXORCode(HeadBuffer+1,28);
		BSP_SendDataToDriverBoard(HeadBuffer,31,0xFFFF);
	}

	UpdataFlag = 0;
}




