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
#include <string.h>
#include <stdio.h>
#include "timer.h"
#include "st7789.h"
#include "LCD_display.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SERVICE_ID 0x22
#define DATA1_ID 0xF001
//#define DATA2_ID 0xF002
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
CAN_RxHeaderTypeDef rx1Header; //CAN1 Bus Transmit Header
CAN_TxHeaderTypeDef tx1Header; //CAN1 Bus Receive Header

CAN_RxHeaderTypeDef rx2Header; //CAN2 Bus Transmit Header
CAN_TxHeaderTypeDef tx2Header; //CAN2 Bus Receive Header

CAN_FilterTypeDef canfil1; //CAN Bus Filter
CAN_FilterTypeDef canfil2; //CAN Bus Filter
uint32_t canMailbox; //CAN Bus Mail box variable

uint8_t Tx1Data[8];
uint8_t Tx2Data[8];

uint8_t Rx1Data[8];
uint8_t Rx2Data[8];

uint32_t ADC_value;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void CAN1_filter_init(){
	canfil1.FilterBank = 10;
	canfil1.FilterMode = CAN_FILTERMODE_IDMASK;
	canfil1.FilterFIFOAssignment = CAN_RX_FIFO0;
	canfil1.FilterIdHigh = 0x7A2 << 5;
	canfil1.FilterIdLow = 0;
	canfil1.FilterMaskIdHigh = 0x7A2 << 5;
	canfil1.FilterMaskIdLow = 0;
	canfil1.FilterScale = CAN_FILTERSCALE_32BIT;
	canfil1.FilterActivation = ENABLE;
	canfil1.SlaveStartFilterBank = 14;
}

void CAN2_filter_init(){
	canfil2.FilterBank = 20;
	canfil2.FilterMode = CAN_FILTERMODE_IDMASK;
	canfil2.FilterFIFOAssignment = CAN_RX_FIFO1;
	canfil2.FilterIdHigh = 0x712 << 5;
	canfil2.FilterIdLow = 0;
	canfil2.FilterMaskIdHigh = 0x712 << 5;
	canfil2.FilterMaskIdLow = 0;
	canfil2.FilterScale = CAN_FILTERSCALE_32BIT;
	canfil2.FilterActivation = ENABLE;
	canfil2.SlaveStartFilterBank = 14;
}

void Tx1_message_define(){
	tx1Header.DLC = 8;
	tx1Header.IDE = CAN_ID_STD;
	tx1Header.RTR = CAN_RTR_DATA;
	tx1Header.StdId = 0x712;
	tx1Header.ExtId = 0x02;
	tx1Header.TransmitGlobalTime = DISABLE;
}

void Tx2_message_define(){
	tx2Header.DLC = 8;
	tx2Header.IDE = CAN_ID_STD;
	tx2Header.RTR = CAN_RTR_DATA;
	tx2Header.StdId = 0x7A2;
	tx2Header.ExtId = 0x02;
	tx2Header.TransmitGlobalTime = DISABLE;
}

//Practice 1 data
#ifdef DATA1_ID

	void Tx1_data(){
		Tx1Data[0] = SERVICE_ID;
		Tx1Data[1] = (DATA1_ID >> 16) & 0xFF;
		Tx1Data[2] = (DATA1_ID >> 8) & 0xFF;
		Tx1Data[3] = DATA1_ID & 0xFF;
	}
#elif DATA2_ID
	void Tx1_data(){
		Tx1Data[0] = SERVICE_ID;
		Tx1Data[1] = (DATA1_ID >> 16) & 0xFF;
		Tx1Data[2] = (DATA1_ID >> 8) & 0xFF;
		Tx1Data[3] = DATA1_ID & 0xFF;
	}
#endif

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  CAN1_filter_init();
  CAN2_filter_init();

  Tx1_message_define();
  Tx2_message_define();

  //Tx2_data();
  Tx1_data();

  HAL_TIM_Base_Start_IT(&htim2);

  HAL_CAN_Start(&hcan1);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ConfigFilter(&hcan1,&canfil1);
  HAL_CAN_ConfigFilter(&hcan2,&canfil2);

  HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO1_MSG_PENDING);

  lcd_init();
  ST7789_Init();

//  ST7789_WriteString(0, 0, "..............................................", Font_7x10, RED, WHITE);
  	HAL_ADC_Start(&hadc1);
  	HAL_ADC_PollForConversion(&hadc1, 100);
  	ADC_value = HAL_ADC_GetValue(&hadc1);
  	setTimer1(10);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(timer1_flag){
		  if(HAL_CAN_AddTxMessage(&hcan2,&tx2Header,Tx2Data,&canMailbox) == HAL_OK){
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1);
		  }
		  setTimer1(100);
	  }
  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 20;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 20;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 57142;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 7;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|LCD_RESET_Pin|LCD_CS_Pin
                          |LCD_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB1 LCD_RESET_Pin LCD_CS_Pin
                           LCD_DC_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|LCD_RESET_Pin|LCD_CS_Pin
                          |LCD_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
uint8_t Buffer[20];
uint8_t Buffer1[20];
uint8_t Buffer2[20];
uint8_t Buffer3[20];
uint8_t Buffer4[20];
uint8_t Buffer5[20];
uint8_t Buffer6[20];
uint8_t Buffer7[20];
uint8_t Buffer8[20];
uint8_t Buffer9[20];
uint8_t Buffer10[20];
uint8_t Buffer11[20];
uint8_t Buffer12[20];
uint8_t Buffer13[20];

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	timerRun();
	//getKeyInput();
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx1Header, Rx1Data) == HAL_OK){
		if(rx1Header.DLC == 8){
			sprintf(&Buffer[0], "IDCAN2:0x%03x", rx1Header.StdId);
			sprintf(&Buffer1[0], "Data0:0x%02x", Rx1Data[0]);
			sprintf(&Buffer2[0], "Data1:0x%02x", Rx1Data[1]);
			sprintf(&Buffer8[0], "Data2:0x%02x", Rx1Data[2]);
			sprintf(&Buffer9[0], "Data3:0x%02x", Rx1Data[3]);
			sprintf(&Buffer10[0], "Data4:0x%02x", Rx1Data[4]);
			sprintf(&Buffer11[0], "Data5:0x%02x", Rx1Data[5]);
			sprintf(&Buffer12[0], "Data6:0x%02x", Rx1Data[6]);
			sprintf(&Buffer13[0], "Data7:0x%02x", Rx1Data[7]);

			ST7789_WriteString(0, 0,&Buffer[0], Font_7x10, RED, WHITE);
			ST7789_WriteString(0, 11, &Buffer1[0], Font_7x10, RED, WHITE);
			ST7789_WriteString(0, 22, &Buffer2[0], Font_7x10, RED, WHITE);
			ST7789_WriteString(0, 33, &Buffer8[0], Font_7x10, RED, WHITE);
			ST7789_WriteString(0, 44, &Buffer9[0], Font_7x10, RED, WHITE);
			ST7789_WriteString(0, 55, &Buffer10[0], Font_7x10, RED, WHITE);
			ST7789_WriteString(0, 66, &Buffer11[0], Font_7x10, RED, WHITE);
			ST7789_WriteString(0, 77, &Buffer12[0], Font_7x10, RED, WHITE);
			ST7789_WriteString(0, 88, &Buffer13[0], Font_7x10, RED, WHITE);
			if(HAL_CAN_AddTxMessage(&hcan1,&tx1Header,Tx1Data,&canMailbox) == HAL_OK){
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);
			}
		}
	}

}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO1, &rx2Header, Rx2Data) == HAL_OK){
		if(rx2Header.DLC == 8){
			sprintf(&Buffer3[0], "IDCAN1:0x%03x", rx2Header.StdId);
			sprintf(&Buffer4[0], "Data0:0x%02x", Rx2Data[0]);
			sprintf(&Buffer5[0], "Data1:0x%02x", Rx2Data[1]);
			sprintf(&Buffer6[0], "Data2:0x%02x", Rx2Data[2]);
			sprintf(&Buffer7[0], "Data3:0x%02x", Rx2Data[3]);

			ST7789_WriteString(0, 99, &Buffer3[0], Font_7x10, RED, WHITE);
			ST7789_WriteString(0, 110, &Buffer4[0], Font_7x10, RED, WHITE);
			ST7789_WriteString(0, 121, &Buffer5[0], Font_7x10, RED, WHITE);
			ST7789_WriteString(0, 132, &Buffer6[0], Font_7x10, RED, WHITE);
			ST7789_WriteString(0, 143, &Buffer7[0], Font_7x10, RED, WHITE);
#ifdef DATA1_ID
			Tx2Data[0] = Rx2Data[0] + 0x40;
			Tx2Data[1] = Rx2Data[1];
			Tx2Data[2] = Rx2Data[2];
			Tx2Data[3] = Rx2Data[3];
			Tx2Data[4] = (ADC_value >> 24) & 0xFF;
			Tx2Data[5] = (ADC_value >> 16) & 0xFF;
			Tx2Data[6] = (ADC_value >> 8) & 0xFF;
			Tx2Data[7] = ADC_value & 0xFF;
#endif
			if(HAL_CAN_AddTxMessage(&hcan2,&tx2Header,Tx2Data,&canMailbox) == HAL_OK){
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);
			}
		}
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
