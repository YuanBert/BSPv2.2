
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
<<<<<<< HEAD
=======
#include "stdlib.h"
#include "stdio.h"
#include "bsp_common.h"
#include "bsp_DataTransmissionLayer.h"
#include "bsp_ProtocolLayer.h"
#include "bsp_digitalfloodcontrol.h"
#include "bsp_motor.h"

#define REDLED_ON       HAL_GPIO_WritePin(MCUAtmosphereLEDR_GPIO_Port,MCUAtmosphereLEDR_Pin,MCUAtmosphereLEDR_ON)
#define REDLED_OFF		HAL_GPIO_WritePin(MCUAtmosphereLEDR_GPIO_Port,MCUAtmosphereLEDR_Pin,MCUAtmosphereLEDR_OFF)
#define REDLED_TOGGLE   HAL_GPIO_TogglePin(MCUAtmosphereLEDR_GPIO_Port,MCUAtmosphereLEDR_Pin)

#define GREENLED_ON		HAL_GPIO_WritePin(MCUAtmosphereLEDG_GPIO_Port,MCUAtmosphereLEDG_Pin,MCUAtmosphereLEDG_ON)
#define GREENLED_OFF	HAL_GPIO_WritePin(MCUAtmosphereLEDG_GPIO_Port,MCUAtmosphereLEDG_Pin,MCUAtmosphereLEDG_OFF)
#define GREENLED_TOGGLE	HAL_GPIO_TogglePin(MCUAtmosphereLEDG_GPIO_Port,MCUAtmosphereLEDG_Pin)
>>>>>>> parent of 59fd819... Ê∑ªÂä†‰∫ÜÊó•Âøó‰ø°ÊÅØ‰∏äÊä•‰ø°ÊÅØÔºå‰ª•ÂèäÈÅ•ÊéßÂºÄÈó∏Êìç‰Ωú

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
<<<<<<< HEAD
=======
extern volatile uint16_t       TiggerTimeCnt;
extern  uint16_t       TiggerTimeSum;

extern  uint16_t       Vnormalt;
extern  uint16_t       Vthreshold;
extern  uint16_t       Vbase;          //ª˘¥°µÁ¡˜÷µ
extern  uint16_t       Vdelta;

extern volatile uint16_t      VbaseBuffer[128];
extern uint16_t      Vnormal;
extern volatile uint16_t      VbaseCnt;
extern volatile uint8_t       VbaseUpdataflag; 
extern volatile uint8_t       SettingVbaseValeFlag;
extern volatile uint8_t 	  gDigitalFoodFlag;

extern uint16_t SpeedDataBufferPtr;
extern uint8_t  SpeedDataBufferOne[300];

MOTORMACHINE gMotorMachine;



//volatile uint16_t ADCSampleBuffer[512];
uint32_t ADCBuffer[32];

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
GPIOSTRUCT gAirSensorGpio;


GPIOSTRUCT gHorGpio;
GPIOSTRUCT gVerGpio;

uint8_t gLedTimerFlag;



static uint32_t Vadcdata;
>>>>>>> parent of 59fd819... Ê∑ªÂä†‰∫ÜÊó•Âøó‰ø°ÊÅØ‰∏äÊä•‰ø°ÊÅØÔºå‰ª•ÂèäÈÅ•ÊéßÂºÄÈó∏Êìç‰Ωú

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

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
<<<<<<< HEAD

=======
  BSP_MotorInit();
  bsp_GpioStructInit();
  DigitalfloodInit();
  
  BSP_DriverBoardProtocolInit();
  //HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADCBuffer, 256);
  //HAL_ADC_Stop_DMA(&hadc1);
  
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim5);
  
>>>>>>> parent of 59fd819... Ê∑ªÂä†‰∫ÜÊó•Âøó‰ø°ÊÅØ‰∏äÊä•‰ø°ÊÅØÔºå‰ª•ÂèäÈÅ•ÊéßÂºÄÈó∏Êìç‰Ωú
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
<<<<<<< HEAD

=======
    /*bsp_ADCCheck();
    HAL_Delay(10);
    */
      
  
  BSP_HandingUartDataFromDriverBoard();
  BSP_HandingDriverBoardRequest();
  BSP_SendAckData();

  /* ∑’Œßµ∆øÿ÷∆ */
  if(1 == gLedTimerFlag)
  {
	gLedTimerFlag = 0;
	if(0 == gOpenFlag)
	{
		REDLED_ON;
		GREENLED_OFF;
	}
	if(2 == gOpenFlag)
	{
		REDLED_OFF;
		GREENLED_TOGGLE;
	}
	if(3 == gOpenFlag)
	{
		REDLED_OFF;
		GREENLED_ON;
	}
	if(5 == gOpenFlag)
	{
		REDLED_TOGGLE;
		GREENLED_OFF;
	}
  }
  

  /* ∑¢ÀÕ»’÷æ–≈œ¢ */
  if(1 == gLogTimerFlag)
  {
	gLogTimerFlag = 0;
#ifdef __Debug__
    BSP_SendDataToDriverBoard((uint8_t*)"\r\n Timelog \r\n",13,0xFFFF);
#endif
  }
  

  /* ºÏ≤‚≥µ¡æ «∑Ò»Î≥°ªÚ’ﬂ≥ˆ≥°–≈œ¢ */
  if(1 == gCarEnteredFlag)
  {
	if(3 == gOpenFlag)
	{
		gCarEnteredFlag = 0;
		gOpenBarTimeoutFlag = 0;
		gOpenBarTimerCnt = 0;
		gOpenBarTimerFlag = 0;
		gOpenFlag = 4;
		/* ∑¢ÀÕ≥µ¡æ»Î≥°ªÚ’ﬂ≥ˆ≥°–≈œ¢ */
		bsp_SendCarEnterFlag();
	}
  }

  if(1 == gBarFirstArriveOpenedPosinFlag)
  {
    gBarFirstArriveOpenedPosinFlag = 0;
	if(2 == gOpenFlag)
	{
		gOpenFlag = 3;
		VbaseCnt = 0;
		TiggerTimeCnt = 0;
	}
	SpeedDataBufferPtr = 0;
#ifdef __Debug__
	//BSP_SendDataToDriverBoard((uint8_t*)"\r\n gBarFirstArriveOpenedPosinFlag \r\n",35,0xFFFF);
#endif
  }

  if(1 == gBarFirstArriveClosedPosionFlag)
  {
	gBarFirstArriveClosedPosionFlag = 0;
	if(2 == gOpenBarTimeoutFlag)
	{
		gOpenBarTimeoutFlag = 0;
		/* ∑¢ÀÕ≥µ¡æ»Î≥°ªÚ’ﬂ≥ˆ≥°≥¨ ±–≈œ¢ */
		bsp_SendCarEnterTimeroutFlag();
	}
	/* ∏¸–¬Vbaseµƒ ˝æ›÷µ */
	UpdateVbaseValue();
	SpeedDataBufferPtr = 0;
#ifdef	__Debug__
	//BSP_SendDataToDriverBoard((uint8_t*)"\r\n gBarFirstArriveClosedPosionFlag\r\n",35, 0xFFFF);
#endif
  }


  

  BSP_MotorCheck();
  BSP_MotorAction();
  
  if(1 == gOpenSpeedTimerFlag && 2 == gOpenFlag)
  {
      gOpenSpeedTimerFlag = 0;

	  /* ÃÌº”µ˜ÀŸ¥˙¬Î */
	  BSP_MotorSpeedSet(SpeedDataBufferOne[SpeedDataBufferPtr]);
	  
#ifdef __Debug__
	  BSP_SendByteToDriverBoard(SpeedDataBufferOne[SpeedDataBufferPtr],0xFFFF);
#endif

	  SpeedDataBufferPtr++;
	  if(SpeedDataBufferPtr > 299)
	  {
		SpeedDataBufferPtr = 299;
	  } 
  }
	
>>>>>>> parent of 59fd819... Ê∑ªÂä†‰∫ÜÊó•Âøó‰ø°ÊÅØ‰∏äÊä•‰ø°ÊÅØÔºå‰ª•ÂèäÈÅ•ÊéßÂºÄÈó∏Êìç‰Ωú
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
<<<<<<< HEAD
=======
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim)
{
	UNUSED(htim);
        /* ≥£ø™µƒ∂® ±∆˜÷–∂œ£¨50usΩ¯»Î“ª¥Œ£¨∂‘π‚’§◊¥Ã¨Ω¯––ºÏ≤‚ */
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
			HAL_TIM_Base_Stop_IT(&htim6);//Õ£÷π∂® ±∆˜÷–∂œ
            HAL_ADC_Stop_DMA(&hadc1);
			BSP_MotorStop(); //Õ£÷πµÁª˙◊™∂Ø
            HAL_TIM_Base_Stop_IT(&htim4);
			gOpenFlag = 0;
			gMotorMachine.RunDir = UPDIR;//–ﬁ∏ƒ∑ΩœÚ±Íº«Œª
			gMotorMachine.RunningState = 0;//–ﬁ∏ƒ‘À––◊¥Ã¨
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
			gMotorMachine.RunDir = DOWNDIR;//–ﬁ∏ƒ∑ΩœÚ±Íº«Œª
			gMotorMachine.RunningState = 0;//∏ƒ±‰‘À––◊¥Ã¨±Íº«Œª
			gOpenBarTimerFlag = 1;
			gBarFirstArriveOpenedPosinFlag = 1;
		  }		
		  if(1 == gMotorMachine.StartFlag)	//»Áπ˚≥ı ºŒª÷√‘⁄¥π÷±Œª÷√£¨¥Àø…“‘Ω¯––◊‘∂Ø∏¥Œª
		  {
			BSP_MotorStop();
            gMotorMachine.StartFlag = 0;
			gOpenBarTimerFlag = 1;
			gOpenFlag = 3;	//±Ì æ¥¶‘⁄¥π÷±Œª÷√
			VbaseCnt = 0;   //πÈ¡„º∆ ˝
			TiggerTimeCnt = 0;//
		  }
          if(1 == gOpenBarTimeoutFlag)
          {
            gOpenFlag = 4;  
            gOpenBarTimeoutFlag = 2;
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
        
   	/* ∑«≥£ø™µƒ∂® ±∆˜÷–∂œ£¨∏√÷–∂œ‘⁄Ω” ’µΩø™’¢÷∏¡Ó∫Û≤≈¥Úø™£¨”√¿¥ºÏ≤‚µÿ∏–∫Õ
	—π¡¶≤®¥´∏–∆˜–≈∫≈£¨‘⁄ºÏ≤‚µΩ∏ÀµΩ¥ÔÀÆ∆ΩŒª÷√÷Æ∫ÛæÕ÷±Ω”πÿ±’∏√÷–∂œ£¨∏√÷–∂œ√ø1ms
	Ω¯»Î“ª¥Œ£¨ø™’¢µƒµ˜ÀŸ“≤Ω´‘⁄∏ƒ÷–∂œ÷–ÕÍ≥…*/
	if(htim4.Instance == htim->Instance)
	{
	  /* µ˜ÀŸ */
	  gOpenSpeedTimerCnt ++;
	  if(gOpenSpeedTimerCnt > 4)    //5msΩ¯»Î“ª¥Œµ˜ÀŸ÷–∂œ
	  {
		gOpenSpeedTimerFlag = 1;
		gOpenSpeedTimerCnt = 0;
	  }   
		/* µÿ∏–ºÏ≤‚£¨∆µΩ∑¿‘“∫Õ≈–∂œ≥µ¡æ «∑ÒΩ¯»Îµƒ◊˜”√ */
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
			/* ÃÌº”»’÷æŒƒµµ       */		
		  }
		}
	  }
	  else
	  {
		if(gGentleSensorGpio.GpioState)
		{
		  gCarEnteredFlag = 1;	
		  /* ÃÌº”»’÷æŒƒµµ */
		}
		gGentleSensorGpio.GpioState = 0;
		gGentleSensorGpio.FilterCnt = 0;
		gGentleSensorGpio.LastReadVal = gGentleSensorGpio.CurrentReadVal;
		/* ø’∆¯≤®ºÏ≤‚ */

		gAirSensorGpio.CurrentReadVal = HAL_GPIO_ReadPin(MCU_AIR_GPIO_Port,MCU_AIR_Pin);
		if(0 == gAirSensorGpio.CurrentReadVal && 0 == gAirSensorGpio.LastReadVal)
		{
			if(0 == gAirSensorGpio.GpioState)
			{
				gAirSensorGpio.FilterCnt++;
				if(gAirSensorGpio.FilterCnt > gAirSensorGpio.FilterCntSum)
				{
					gAirSensorGpio.GpioState = 1;
					gAirSensorGpio.FilterCnt = 0;
					/* –¥»’÷æ–≈œ¢ */
				}
			}
		}
		else
		{
			if(1 == gAirSensorGpio.GpioState)
			{
				gAirSensorGpio.GpioState = 0;
				/* –¥»’÷æ–≈œ¢ */
			}
			gAirSensorGpio.FilterCnt = 0;
		}
	  }
	  return;
	}
        
        
    /* ≥£ø™÷–∂œ£¨√ø200msΩ¯»Î“ª¥Œ£¨”√“‘∂‘log»’÷æΩ¯––…œ±®£¨œµÕ≥◊‘ºÏµ»π¶ƒ‹£¨Õ¨ ±
    “≤¿¥ºÊπÀ¿¥≥µøÿµ∆µƒ◊˜”√ */
	if(htim5.Instance == htim->Instance)
	{
	  gLedTimerFlag = 1;
	  gLogTimerCnt++;
	  if(gLogTimerCnt > 50)
	  {
		gLogTimerFlag = 1;
		gLogTimerCnt = 0;
	  }	
	  if(1 == gOpenBarTimerFlag)
	  {
		gOpenBarTimerCnt++;
		if(gOpenBarTimerCnt > 60)	//µ»¥˝ ±º‰Œ™6√Î÷”£¨≥¨π˝6√Î÷”»œŒ™ «≥¨ ±
		{
		  gOpenBarTimeoutFlag = 1;
		  gOpenBarTimerCnt = 0;
		  gOpenBarTimerFlag = 0;
		}
	  }
	  return;
	}
        
        
        
    /* ¥À∂® ±∆˜÷–∂œ÷ª”–πÿ’¢µƒ ±∫Ú≤≈ª·∆◊˜”√£¨ø™∆Ù÷–∂œ∫Û√ø10msΩ¯»Î“ª¥Œ */
	if(htim6.Instance == htim->Instance)
	{
	  bsp_ADCCheck();
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
	gVerGpio.FilterCntSum = 19;


	gHorGpio.LastReadVal = 0;
	gHorGpio.FilterCnt = 0;
	gHorGpio.GpioState = 0;
	gHorGpio.FilterCntSum = 19;

	gGentleSensorGpio.LastReadVal = 0;
	gGentleSensorGpio.FilterCnt = 0;
	gGentleSensorGpio.GpioState = 0;
	gGentleSensorGpio.FilterCntSum = 25;

	gAirSensorGpio.CurrentReadVal = 0;
	gAirSensorGpio.LastReadVal = 0;
	gAirSensorGpio.GpioState = 0;
	gAirSensorGpio.FilterCnt = 0;
	gAirSensorGpio.FilterCntSum = 10;

	gBarFirstArriveOpenedPosinFlag = 0;
	gBarFirstArriveClosedPosionFlag = 0;
}
void  bsp_ADCCheck()
{
   uint16_t i;
   //uint8_t VbaseValueBuffer[4];
   //char sBuffer[10];
   if(5!= gOpenFlag)
   {
      return;
   }
  
   for(i = 0,Vadcdata = 0; i < 32;i++)
   {
           Vadcdata += ADCBuffer[i];
	}
	Vnormal = (uint16_t)(Vadcdata >> 5);
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
	  
    if(3 > TiggerTimeCnt)
    {	
      if(VbaseCnt > 128) //»Áπ˚≤…—˘≥¨π˝128∏ˆµ„‘Ú∂™∆˙
      {
        VbaseCnt = 0;
      }
      VbaseBuffer[VbaseCnt++] = Vnormal;
    }
    else
    {
      VbaseCnt = 0;
    }              
#ifdef __Debug__
    //sprintf(sBuffer,"\r\nBB%4dBB\r\n",Vnormal);
//	VbaseValueBuffer[0] = 0xBB;
//	VbaseValueBuffer[1] = Vnormal >> 8;
//	VbaseValueBuffer[2] = Vnormal;
//	VbaseValueBuffer[3] = 0xBB;
//	BSP_SendDataToDriverBoard(VbaseValueBuffer,4,0xFFFF);
    //BSP_SendDataToDriverBoard((uint8_t*)sBuffer,sizeof(sBuffer),0xFFFF);
#endif	
  Vnormal = 0;
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

>>>>>>> parent of 59fd819... Ê∑ªÂä†‰∫ÜÊó•Âøó‰ø°ÊÅØ‰∏äÊä•‰ø°ÊÅØÔºå‰ª•ÂèäÈÅ•ÊéßÂºÄÈó∏Êìç‰Ωú

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
