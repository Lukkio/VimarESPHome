/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "AssembyCode.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void GetParamert(void);
//void GetParamert2(int32_t p1, char *p2, int32_t p3);
//int32_t GetParamert3(int32_t p1, char *p2, int32_t p3);
static void JumpToBootloader( void );
//void SwitchOnOff();
//void ExecuteFunctionTable();
//char CRC8(char* param_1, int param_2);
//int Dummy(int param_1, int param_2);

//((void(*)(void))0x1234)(int32_t p1, int32_t *p2, int32_t p3);
//int32_t (*)(void))0x08003e7c(int32_t p1, int32_t *p2, int32_t p3);
//typedef int32_t func_08003e7c(int32_t p1, char *p2, int32_t p3);
unsigned char UART[200];
char ReadTemp[]="ReadT";

//void ReadUART(void);
//void WriteUART(void);

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  WriteToUart(0);
  /* USER CODE END 2 */
  GetParamert();
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  /**USART1 GPIO Configuration
  PA9   ------> USART1_TX
  PA10   ------> USART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, 0);
  NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_SetTXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_DisableFIFO(USART1);
  LL_USART_ConfigAsyncMode(USART1);

  /* USER CODE BEGIN WKUPType USART1 */

  /* USER CODE END WKUPType USART1 */

  LL_USART_Enable(USART1);

  /* Polling USART1 initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART1))) || (!(LL_USART_IsActiveFlag_REACK(USART1))))
  {
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//void HAL_UART_IRQHandler(UART_HandleTypeDef *huart)
//{
//	ReadUART();
//	WriteUART();
//}
//
//void ReadUART(void)
//{
//	if(USART1->ISR << 0x1a < 0)
//	{
//	 UART[UART[9]] = USART1->RDR;
//	 UART[8] = ((USART1->ISR << 0x1c < 0 || USART1->ISR << 0x1d < 0) || USART1->ISR << 0x1e < 0) || USART1->ISR << 0x1f < 0;
//	 UART[0] = (UART[0] + 1) & 0xf;
//	}
//}
//
//void WriteUART(void)
//{
//	int Position = UART[32];
//
//	if(USART1->ISR << 0x19 < 0)
//	{
//		USART1->ICR = 0x40;
//		if(Position < UART[28])
//		{
//			UART[32] = Position + 1;
//			UART[40] = UART[Position + 37];
//
//		}
//		else UART[36] = 1;
//	}
//}

void GetParamert(void)
{
	//patch offset 0x80064b4 per puntare a 0x080084f8
	//funzione custom aggiunta -> offset 0x080084f8 (GetParamert())

	//	temperatura settata -> 0x20000680 (valore int)
	//	temperatura attuale -> 0x20000650 (valore int)
	//	on/OFF -> 0x2000067C (valore 0x00 / 0x01)
	//	luminosità -> 0x20000670 (valore 0x00 / 0x01 / 0x02 / 0x03)
	//	caldo/freddo -> 0x2000066C (valore 0x00 / 0x01)
	//	celsus/Fahre.. -> 0x20000671 (valore 0x00 / 0x01)
	//	display visualizzato -> 0x20000011 (3 char)

	//Commands UART:
	//"ReadA"  -> read all variables
	//Buffer	   [0][1]	    |	[2]   |	 [3]  |		[4]		 |		[5][6]		 | [7] |
	//		| ActualTemperature | HotCold | OnOff | Celsus_Fahre | SettedTemperature | \n  |

	//"Set 1"
	//"Set 2"	HotCold -> command UART to send: "Set2 [0x01,0x00]"
	//"Set 3"	Brightness -> command UART to send: "Set3 [0x01,0x02,0x03]"
	//"Set 4"	OnOff -> command UART to send: "Set4 [0x01,0x00]"
	//"Set 5"	Celsus_Fahre -> command UART to send: "Set5 [0x01,0x00]"  
	//"Set 6"	SettedTemperature2 -> command UART to send: "Set6 [0xff,0xff]"
	//"Set 7"	OnOff -> command UART to send: "Set7 [OnOff(0x01),HotCold(0x00)]"
	//"Set 8 BA" button software set test
	//"Set 9"	OnOff -> command UART to send: "Set9 [0x01,0x00]" + 12c memory write
	//"BOOTL"	jump to bootloader

	//func_08003e7c* f_08003e7c = (func_08003e7c*)0x08003e7c;
	//char * uartBuffer[12] = (void*)0x20000558;

	//Load UART buffer (max 12 byte)
	char * U1 = (void*)0x20000558;
	char * U2 = (void*)0x20000559;
	char * U3 = (void*)0x2000055a;
	char * U4 = (void*)0x2000055b;
	char * U5 = (void*)0x2000055c;
	char * U6 = (void*)0x2000055d;
	char * U7 = (void*)0x2000055e;
	//char * U8 = (void*)0x2000055f;
	char * U9 = (void*)0x20000560;
//	char * U10 = (void*)0x20000561;

	char *Buffer = (void*)0x20000800;

	//Point memory address
	char *ActualTemperature = (void*)0x20000650;
	//char *ActualInDisplay = (void*)0x20000011;
	char *HotCold = (void*)0x2000066C;
	char *HotCold2 = (void*)0x200004f0;
	char *Brightness = (void*)0x20000670;
	char *OnOff = (void*)0x2000067C;
	char *Celsus_Fahre = (void*)0x20000671;
	char *SettedTemperature = (void*)0x20000680;
	char *SettedTemperature2 = (void*)0x20000504;
	int16_t *SettedTemperature3 = (void*)0x20000504;

	char *OnOff2 = (void*)0x200004f8;

	//pointer to a generic function, no parameter passed
	void ( *ExecuteFunction )( void );

	//pointer to a generic function, 1 char parameter passed
	void ( *ExecuteFunction1 )( char param_1 );

	//pointer to a generic function, 1 char, 2 int, 3 int16 parameter passed
	void ( *ExecuteFunction2 )( char param_1, int param_2, int16_t param_3 );


	void ( *ExecuteFunction3 )(int32_t p1, char *p2, int32_t p3);
	
	//Compare command recived
	if((*U1=='R')&&(*U2=='e')&&(*U3=='a')&&(*U4=='d'))
	{
		if(*U5=='A')
		{
			*Buffer = *ActualTemperature;
			*(Buffer+1)= *(ActualTemperature+1);
			*(Buffer+2)= *HotCold;
			*(Buffer+3)= *OnOff;

			//Celsus??
			*(Buffer+4)= *Celsus_Fahre;

			*(Buffer+5)= *SettedTemperature;
			*(Buffer+6)= *(SettedTemperature+1);

			*(Buffer+7)='\n';

			//write to UART
			ExecuteFunction3 = ( void ( * )( int32_t, char*, int32_t ) )0x08003e7dL;
			ExecuteFunction3(0,Buffer,8);
		}
	}

	if((*U1=='S')&&(*U2=='e')&&(*U3=='t')&&(*U5==' '))
	{
		//Hot/Cold mode
		if(*U4=='2')//OK
		{
			*HotCold = *U6;
			*HotCold2 = *U7;

			//write to i2c eeprom new state
			ExecuteFunction1 = ( void ( * )( char ) )0x080071edL;
			ExecuteFunction1(*HotCold2);
		}
		if(*U4=='3')
		{
			*Brightness = *U6;
		}
		if(*U4=='4')
		{
			*OnOff = *U6;
		}
		if(*U4=='5')
		{
			*Celsus_Fahre = *U6;
		}

		//Update target temperature
		if(*U4=='6')//OK
		{
			*SettedTemperature2 = *U6;
			*(SettedTemperature2+1) = *U7;

			//write to i2c eeprom new state
			ExecuteFunction2 = ( void ( * )( char, int, int16_t ) )0x08007341L;
			ExecuteFunction2(*HotCold, 0x00, *SettedTemperature3);
		}

		//(on||off)&(Heat||cool)
		if(*U4=='7')
		{
			*OnOff = *U6;
			*HotCold = *U7;
		}

		//Set8 B8 1
		if(*U4=='8')
		{
			if(*U6=='B')
			{
				GPIOB->PUPDR = (GPIOB->PUPDR & ~(3 << (2 * *U7))) | (*U9 << (2 * *U7));
			}

			if(*U6=='A')
			{
				GPIOA->PUPDR = (GPIOA->PUPDR & ~(3 << (2 * *U7))) | (*U9 << (2 * *U7));
			}
		}

		//Update On/off state
		if(*U4=='9')//OK
		{
			*OnOff = *U6;
			*OnOff2 = *U7;

			//write to i2c eeprom new state
			ExecuteFunction = ( void ( * )( void ) )0x08005bd9L;
			ExecuteFunction();
		}
	}

	if((*U1=='B') && (*U2=='O') && (*U3=='O') && (*U4=='T') && (*U5=='L'))
	{
		JumpToBootloader();
	}
}

static void JumpToBootloader( void )

{
  uint32_t i = 0;
  void ( *SysMemBootJump )( void );
  /* system BootLoader address*/
  __IO uint32_t BootAddr = 0x1FFF0000;
  /*Turn off global interrupts*/
  __disable_irq();
  /* Turn off the tick timer, reset to default*/
  SysTick->CTRL = 0;
  SysTick->LOAD = 0;
  SysTick->VAL = 0;
  /* Set all clocks to default state, use HSI clock */
  //HAL_RCC_DeInit();
  /* Disable all interrupts, clear all interrupt pending flags*/
  for ( i = 0; i < 1; i++ )
  {
    NVIC->ICER[i] = 0xFFFFFFFF;
    NVIC->ICPR[i] = 0xFFFFFFFF;
  }
  /* Enable global interrupt */
  __enable_irq();
  /* Jump to the system BootLoader BootLoader，*/
  SysMemBootJump = ( void ( * )( void ) ) ( *( ( uint32_t * ) ( BootAddr + 4 ) ) );
  /* set the  stack pointer */
  __set_MSP( *( uint32_t * )BootAddr );
  /* Jump to the system BootLoader */
  SysMemBootJump();
  /* */
  while ( 1 )
  {
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
