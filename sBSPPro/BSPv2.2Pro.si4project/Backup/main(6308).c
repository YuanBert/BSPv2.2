
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "bsp_common.h"
#include "bsp_DataTransmissionLayer.h"
#include "bsp_ProtocolLayer.h"
#include "bsp_digitalfloodcontrol.h"
#include "bsp_motor.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
extern volatile uint16_t       TiggerTimeCnt;
extern  uint16_t       TiggerTimeSum;
extern  uint16_t       Vnormal;
extern  uint16_t       Vnormalt;
extern  uint16_t       Vthreshold;
extern  uint16_t       Vbase;          //基础电流值
extern  uint16_t       Vdelta;

extern          uint16_t      VbaseBuffer[128];
extern volatile uint16_t      VbaseCnt;
extern volatile uint8_t       VbaseUpdataflag; 
extern volatile uint8_t       SettingVbaseValeFlag;

MOTORMACHINE gMotorMachine;



//volatile uint16_t ADCSampleBuffer[512];
volatile uint32_t ADCBuffer[256];

volatile uint8_t  gLogTimerFlag;
volatile uint32_t gLogTimerCnt;

volatile uint8_t  gOpenBarTimeoutFlag;
volatile uint8_t  gOpenBarTimerFlag;
volatile uint32_t gOpenBarTimerCnt;

volatile uint8_t gOpenSpeedTimerFlag;
volatile uint8_t gOpenSpeedTimerCnt;

volatile uint8_t gBarFirstArriveOpenedPosinFlag;
volatile uint8_t gBarFirstArriveClosedPosionFlag;

volatile uint8_t gOpenFlag;
volatile uint8_t gObstructFlag;

volatile uint8_t gHorCloseFlag;

volatile uint8_t gCarEnteredFlag;
GPIOSTRUCT gGentleSensorGpio;

GPIOSTRUCT gHorGpio;
GPIOSTRUCT gVerGpio;


static uint32_t Vadcdata;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void bsp_GpioStructInit(void);
void bsp_SendCarEnterFlag(void);
void bsp_SendCarEnterTimeroutFlag(void);
void bsp_ADCCheck(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM3_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  BSP_MotorInit();
  bsp_GpioStructInit();
  DigitalfloodInit();
  
  BSP_DriverBoardProtocolInit();
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADCBuffer, 256);
  
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim5);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    /*bsp_ADCCheck();
    HAL_Delay(10);
    */
      
  
  BSP_HandingUartDataFromDriverBoard();
  BSP_HandingDriverBoardRequest();
  BSP_SendAckData();

  /* 发送日志信息 */
  if(1 == gLogTimerFlag)
  {
	gLogTimerFlag = 0;
	
  }
	/* 检测车辆是否入场或者出场信息 */
  if(1 == gCarEnteredFlag)
  {
	if(3 == gOpenFlag)
	{
		gCarEnteredFlag = 0;
		gOpenBarTimeoutFlag = 0;
		gOpenBarTimerCnt = 0;
		gOpenBarTimerFlag = 0;
		gOpenFlag = 4;
		/* 发送车辆入场或者出场信息 */
		bsp_SendCarEnterFlag();
	}
  }

  if(1 == gBarFirstArriveOpenedPosinFlag)
  {
	if(2 == gOpenFlag)
	{
		gOpenFlag = 3;
		VbaseCnt = 0;
		TiggerTimeCnt = 0;
	}
	gBarFirstArriveOpenedPosinFlag = 0;
#ifdef __Debug__
	BSP_SendDataToDriverBoard((uint8_t*)"\r\n gBarFirstArriveOpenedPosinFlag \r\n",35,0xFFFF);
#endif
  }

  if(1 == gBarFirstArriveClosedPosionFlag)
  {
	gBarFirstArriveClosedPosionFlag = 0;
	if(1 == gOpenBarTimeoutFlag)
	{
		gOpenBarTimeoutFlag = 0;
		/* 发送车辆入场或者出场超时信息 */
		bsp_SendCarEnterTimeroutFlag();
	}
	/* 更新Vbase的数据值 */
	UpdateVbaseValue();
#ifdef	__Debug__
	BSP_SendDataToDriverBoard((uint8_t*)"\r\n gBarFirstArriveClosedPosionFlag\r\n",35, 0xFFFF);
#endif
  }


  

  BSP_MotorCheck();
  BSP_MotorAction();
	
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* ADC1_2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* TIM3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* TIM4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
  /* TIM5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM5_IRQn);
  /* TIM6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM6_IRQn);
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* I2C2_EV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(I2C2_EV_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);
  /* I2C2_ER_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(I2C2_ER_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim)
{
	UNUSED(htim);
        /* 常开的定时器中断，50us进入一次，对光栅状态进行检测 */
	if(htim3.Instance == htim->Instance)
	{
	  gHorGpio.CurrentReadVal = HAL_GPIO_ReadPin(HorRasterInput_GPIO_Port,HorRasterInput_Pin);
	  gVerGpio.CurrentReadVal = HAL_GPIO_ReadPin(VerRasterInput_GPIO_Port,VerRasterInput_Pin);
	  if(0 == gHorGpio.CurrentReadVal && 0 == gHorGpio.LastReadVal)
	  {
		gHorGpio.FilterCnt++;
		if(gHorGpio.FilterCnt > gHorGpio.FilterCntSum)
		{
		  gMotorMachine.HorizontalRasterState = 1;
		  gHorGpio.GpioState = 1;
		  gHorGpio.FilterCnt = 0;	
		  if(DOWNDIR == gMotorMachine.RunDir)
		  {
			HAL_TIM_Base_Stop_IT(&htim4);
			HAL_TIM_Base_Stop_IT(&htim6);//停止定时器中断
			BSP_MotorStop(); //停止电机转动
			gOpenFlag = 0;
			gMotorMachine.RunDir = UPDIR;//修改方向标记位
			gMotorMachine.RunningState = 0;//修改运行状态
			gBarFirstArriveClosedPosionFlag = 1;//
			if(SettingVbaseValeFlag)
			{
			  gHorCloseFlag++;
			  if(gHorCloseFlag > 2)
			  {
				gHorCloseFlag = 0;
				SettingVbaseValeFlag = 0;
			  }
			}
		  }		
		}
	  }
	  else
	  {
		gMotorMachine.HorizontalRasterState = 0;
		gHorGpio.GpioState = 0;
		gHorGpio.FilterCnt = 0;
	  }
	  gHorGpio.LastReadVal = gHorGpio.CurrentReadVal;
	  if(0 == gVerGpio.CurrentReadVal && 0 == gVerGpio.LastReadVal)
	  {
		gVerGpio.FilterCnt++;
		if(gVerGpio.FilterCnt > gVerGpio.FilterCntSum)
		{
		  gVerGpio.GpioState = 1;
		  gVerGpio.FilterCnt = 0;
		  gMotorMachine.VerticalRasterState = 1;
		  if(UPDIR == gMotorMachine.RunDir)
		  {
			BSP_MotorStop();
			gMotorMachine.RunDir = DOWNDIR;//修改方向标记位
			gMotorMachine.RunningState = 0;//改变运行状态标记位
			gOpenBarTimerFlag = 1;
			gBarFirstArriveOpenedPosinFlag = 1;
		  }		
		  if(1 == gMotorMachine.StartFlag)	//如果初始位置在垂直位置，此可以进行自动复位
		  {
			gMotorMachine.StartFlag = 0;
			gOpenBarTimerFlag = 1;
			gOpenFlag = 3;	//表示处在垂直位置
			VbaseCnt = 0;   //归零计数
			TiggerTimeCnt = 0;//
		  }		
		}
	  }
	  else
	  {
		gMotorMachine.VerticalRasterState = 0;
		gVerGpio.GpioState = 0;
		gVerGpio.FilterCnt = 0;
	  }
	  gVerGpio.LastReadVal = gVerGpio.CurrentReadVal;
	  return;
	}
        
   	/* 非常开的定时器中断，该中断在接收到开闸指令后才打开，用来检测地感和
	压力波传感器信号，在检测到杆到达水平位置之后就直接关闭该中断，该中断每1ms
	进入一次，开闸的调速也将在改中断中完成*/
	if(htim4.Instance == htim->Instance)
	{
	  /* 调速 */
	  gOpenSpeedTimerCnt ++;
	  if(gOpenSpeedTimerCnt > 9)
	  {
		gOpenSpeedTimerFlag = 1;
		gOpenSpeedTimerCnt = 0;
	  }   
		/* 地感检测，起到防砸和判断车辆是否进入的作用 */
	  gGentleSensorGpio.CurrentReadVal = HAL_GPIO_ReadPin(GentleSensor_GPIO_Port,GentleSensor_Pin);
	  if(0 == gGentleSensorGpio.CurrentReadVal && 0 == gGentleSensorGpio.LastReadVal)
	  {
		if(0 == gGentleSensorGpio.GpioState)
		{
		  gGentleSensorGpio.FilterCnt++;
		  if(gGentleSensorGpio.FilterCnt > gGentleSensorGpio.FilterCntSum)
		  {
			gGentleSensorGpio.GpioState = 1;
			gGentleSensorGpio.FilterCnt = 0;
			/* 添加日志文档       */		
		  }
		}
	  }
	  else
	  {
		if(gGentleSensorGpio.GpioState)
		{
		  gCarEnteredFlag = 1;	
		  /* 添加日志文档 */
		}
		gGentleSensorGpio.GpioState = 0;
		gGentleSensorGpio.FilterCnt = 0;
		gGentleSensorGpio.LastReadVal = gGentleSensorGpio.CurrentReadVal;
		/* 空气波检测 */			
	  }
	  return;
	}
        
        
    /* 常开中断，每200ms进入一次，用以对log日志进行上报，系统自检等功能，同时
    也来兼顾来车控灯的作用 */
	if(htim5.Instance == htim->Instance)
	{
	  gLogTimerCnt++;
	  if(gLogTimerCnt > 10)
	  {
		gLogTimerFlag = 1;
		gLogTimerCnt = 0;
	  }	
	  if(1 == gOpenBarTimerFlag)
	  {
		gOpenBarTimerCnt++;
		if(gOpenBarTimerCnt > 50)	//等待时间为10秒钟，超过10秒钟认为是超时
		{
		  gOpenBarTimeoutFlag = 1;
		  gOpenBarTimerCnt = 0;
		  gOpenBarTimerFlag = 0;
		  gOpenFlag = 4;
		}
	  }
	  return;
	}
        
        
        
    /* 此定时器中断只有关闸的时候才会起作用，开启中断后每10ms进入一次 */
	if(htim6.Instance == htim->Instance)
	{
	  bsp_ADCCheck();
	  if(0 == SettingVbaseValeFlag)
	  {
		if(Vnormal > Vbase)
		{
		  Vdelta = Vnormal - Vbase;
		}
		else
		{
		  Vdelta = Vbase - Vnormal;                           
		}
		if(Vdelta > Vthreshold)
		{
		  TiggerTimeCnt++;
		}
		else
		{
		  TiggerTimeCnt = 0;
		} 
	  }
	  if(0 == TiggerTimeCnt)
	  {	
		if(VbaseCnt > 128) //如果采样超过128个点则丢弃
		{
		  VbaseCnt = 0;
		}
		VbaseBuffer[VbaseCnt++] = Vnormal;
	  }
	  else
	  {
		VbaseCnt = 0;
	  }          
	  Vnormal = 0;
	}	
}


void bsp_SendCarEnterFlag(void)
{
	uint8_t pData[7];
	pData[0] = 0x5B;
	pData[1] = 0xE3;
	pData[3] = 0x00;
	pData[4] = 0x00;
	pData[6] = 0x5D;
	pData[2] = 0x00;
	pData[5] = 0xE3;
	BSP_SendDataToDriverBoard(pData,7,0xFFFF);
	return ;
}
void bsp_SendCarEnterTimeroutFlag(void)
{
	uint8_t pData[7];
	pData[0] = 0x5B;
	pData[1] = 0xE3;
	pData[3] = 0x00;
	pData[4] = 0x00;
	pData[6] = 0x5D;
	pData[2] = 0x01;
	pData[5] = 0xE2;
	BSP_SendDataToDriverBoard(pData,7,0xFFFF);
	return;
}
void bsp_GpioStructInit(void)
{
	gVerGpio.LastReadVal = 0;
	gVerGpio.FilterCnt = 0;
	gVerGpio.GpioState = 0;
	gVerGpio.FilterCntSum = 15;


	gHorGpio.LastReadVal = 0;
	gHorGpio.FilterCnt = 0;
	gHorGpio.GpioState = 0;
	gHorGpio.FilterCntSum = 15;

	gGentleSensorGpio.LastReadVal = 0;
	gGentleSensorGpio.FilterCnt = 0;
	gGentleSensorGpio.GpioState = 0;
	gGentleSensorGpio.FilterCntSum = 15;

	gBarFirstArriveOpenedPosinFlag = 0;
	gBarFirstArriveClosedPosionFlag = 0;
}
void  bsp_ADCCheck()
{
   uint16_t i;
   uint8_t VbaseValueBuffer[4];
   char sBuffer[10];
   
  
   for(i = 0,Vadcdata = 0; i < 256;)
   {
           Vadcdata += ADCBuffer[i];
           i++;
	}
	Vnormal = (uint16_t)(Vadcdata >> 8);
    sprintf();
    
#ifdef __Debug__
   
	VbaseValueBuffer[0] = 0xBB;
	VbaseValueBuffer[1] = Vnormal >> 8;
	VbaseValueBuffer[2] = Vnormal;
	VbaseValueBuffer[3] = 0xBB;
	BSP_SendDataToDriverBoard(VbaseValueBuffer,4,0xFFFF);	
#endif	

}
/**
  * @brief  Conversion complete callback in non blocking mode 
  * @param  hadc: ADC handle
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);
  /* NOTE : This function should not be modified. When the callback is needed,
            function HAL_ADC_ConvCpltCallback must be implemented in the user file.
   */
	//bsp_ADCCheck();
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
