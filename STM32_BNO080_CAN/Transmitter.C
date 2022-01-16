
/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 * @creator        : Priya & Kanishk : MRM
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BNO080.h"
#include "Quaternion.h"
#include <stdio.h>
#include "stm32f1xx.h"
#include <math.h>
#include <stdlib.h>

#define pi 3.1415
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
CAN_HandleTypeDef hcan;

/* USER CODE BEGIN PV */

CAN_FilterTypeDef sFilterConfig, sFilterConfig1;//struct containing filter settings
CAN_RxHeaderTypeDef RxMessage;	 //struct for recieved data frame
CAN_TxHeaderTypeDef TxMessage;   // struct for transmitted dataframe


uint8_t rxData[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };	//array for recieved data (8 bytes)
uint8_t txData[8] = { 0, 20, 0, 0, 0, 0, 0, 0 };
uint32_t usedmailbox;

float T1;
double T2;
float IMU_1 = 0;
int IMU_2 = 0;
float Prev_E_1 = 0;
float Prev_E_2 = 0;
float Error_1 = 0;
double Error_2 = 0;
float dt = 0.01;
int PWM_1 = 0;
double PWM_2 = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */

void Ports_Init(void);
void Timers_Init(void);
void Inverse_Kinematics(float X_0, float y);
long map(long x, long in_min, long in_max, long out_min, long out_max);//struct containing CAN init settings
unsigned char Conv(float val);

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

	float q[4];
	float quatRadianAccuracy;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

	Ports_Init();
	Timers_Init();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */

	LL_TIM_EnableCounter(TIM3);
	LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);

	TIM3->PSC = 2000;
	// HAL_Delay(100);
	TIM3->PSC = 1500;
	// HAL_Delay(100);
	TIM3->PSC = 1000;
	// HAL_Delay(100);

	TIM3->CCR4 = TIM3->ARR / 2;
	LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
	LL_USART_EnableIT_RXNE(USART1);

	BNO080_Initialization();
	BNO080_enableRotationVector(15);

	TxMessage.IDE = CAN_ID_STD;				//standard identifier format (11bit)
	TxMessage.StdId = 0x446;								//identifier value
	TxMessage.RTR = CAN_RTR_DATA;//indicates frame mode (data frame or remote frame)
	TxMessage.DLC = 8;									//data length (8 bytes)
	TxMessage.TransmitGlobalTime = DISABLE;	//time of transmission is not transmitted along with the data

	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;//filter bank consists of 2 32bit values (mask and ID)
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;//filter set to mask and ID mode
	sFilterConfig.FilterBank = 0;				//filter bank number 0 selected
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;//assign filter bank to FIFO 0
	sFilterConfig.FilterIdHigh = 0x211 << 5;//STD ID value is 7, here shifted by 5 because 11 bits starting from the left are for STD ID (FilterIdHigh is 16bit)
	sFilterConfig.FilterIdLow = 0;										//LSB
	sFilterConfig.FilterMaskIdHigh = 0x211 << 5;//0b111 shifted by 5 for the same reason, first 11 bits are for Identifier
	sFilterConfig.FilterMaskIdLow = 0;								//LSB
	sFilterConfig.FilterActivation = ENABLE;				//activate filter

	HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);
	HAL_CAN_Start(&hcan);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	while (1) {
		// HAL_Delay(10);

		printf("Hello World \n");

		BNO080_dataAvailable();

		if (BNO080_dataAvailable() == 1) {
			q[0] = BNO080_getQuatI();
			q[1] = BNO080_getQuatJ();
			q[2] = BNO080_getQuatK();
			q[3] = BNO080_getQuatReal();

			quatRadianAccuracy = BNO080_getQuatRadianAccuracy();

			Quaternion_Update(&q[0]);

			int pitch_F = BNO080_Pitch;



			txData[0] = pitch_F & 0x000000FF;
			txData[1] = (pitch_F >> 8) & 0x000000FF;
			txData[2] = (pitch_F >> 16) & 0x000000FF;
			txData[3] = (pitch_F >> 24) & 0x000000FF;

			USART1 -> DR = BNO080_Pitch;
			while((USART1 ->SR & USART_SR_TXE) == 0); // TXE -> TRANSMIT EMPTY

			//printf("Hi from tx \n");



			// printf("%d, %d, %d,\n", (int) (BNO080_Roll), (int)BNO080_Pitch,(int) (BNO080_Yaw));


			// HAL_CAN_AddTxMessage(&hcan, &TxMessage, txData, &usedmailbox);

			// printf("%d, %d, %d,\n", (int) (BNO080_Roll), (int) (BNO080_Pitch),(int) (BNO080_Yaw));

			/*
			 * *******************************************************************


			Inverse_Kinematics(2, 3);

			// IMU VALUES
			float Curr_pos_1 = BNO080_Pitch;
			double Curr_Pos_2 = abs((BNO080_Pitch - IMU_2));

			// ERROR
			Error_1 = T1 - Curr_pos_1;
			Error_2 = T2 - Curr_Pos_2;

			if (Error_1 < 0) {

				GPIOB->BSRR = (1 << 20);
			}

			else if (Error_1 > 0) {
				GPIOB->BSRR = (1 << 4);
			}

			Prev_E_1 += Error_1;
			Prev_E_2 += Error_2;

			// dE/dT
			double E_dot_1 = (Error_1 - Prev_E_1) / dt;
			double E_dot_2 = (Error_2 - Prev_E_2) / dt;

			// E.dT
			double E_int_1 = (Error_1) * dt + Prev_E_1;
			double E_int_2 = (Error_2) * dt + Prev_E_2;

			// PID CONSTANTS | SUBJECT TO CHANGE
			double Kp = 4.5;
			double Kd = 0.00;
			double Kc = 0.005;

			// OUPUT PWM SIGNALS
			PWM_1 = (((Kp * Error_1) + (Kc * E_int_1) + (Kd * E_dot_1)));
			PWM_2 = (Kp * Error_2) + (Kc * E_int_2) + (Kd * E_dot_2);

			// CAP PWM SIGNAL
			if (abs(PWM_1) > 80) {
				PWM_1 = 80;
			}

//		  if(abs(PWM_1) < 60)
//		  {
//			  PWM_1 = 60;
//		  }

			if (abs(Error_1) < 3) {
				PWM_1 = 0;
				TIM4->CCR1 = PWM_1;
			}

		//	printf("PWM 1: %d  ", abs(PWM_1));

			TIM4->CCR1 = abs(PWM_1);

			*/


            HAL_Delay(100);
		}

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 8;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  /**SPI2 GPIO Configuration
  PB13   ------> SPI2_SCK
  PB14   ------> SPI2_MISO
  PB15   ------> SPI2_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_13|LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_14;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_HIGH;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV16;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 10;
  LL_SPI_Init(SPI2, &SPI_InitStruct);
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  TIM_InitStruct.Prescaler = 999;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 20;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM3, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM3);
  LL_TIM_SetClockSource(TIM3, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH4);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 10;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct);
  LL_TIM_OC_EnableFast(TIM3, LL_TIM_CHANNEL_CH4);
  LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM3);
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  /**TIM3 GPIO Configuration
  PB1   ------> TIM3_CH4
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  /**USART1 GPIO Configuration
  PA9   ------> USART1_TX
  PA10   ------> USART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
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
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_13);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_6|LL_GPIO_PIN_7);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_12);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_13;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_12;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void Ports_Init(void) {
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;   // ENABLE CLOCK | PORT A
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;   // ENABLE CLOCK | PORT B
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;   // ENABLE ALTERNATE FUNCTION

	// PORT A | PIN 1 | INPUT MODE | ANALOG INPUT
	GPIOA->CRL &= ~(GPIO_CRL_MODE1_0 | GPIO_CRL_MODE1_1);
	GPIOA->CRL &= ~(GPIO_CRL_CNF1_0 | GPIO_CRL_CNF1_1);

	// PORT A | PIN 2 | INPUT MODE | ANALOG INPUT
	GPIOA->CRL &= ~(GPIO_CRL_MODE2_0 | GPIO_CRL_MODE2_1);
	GPIOA->CRL &= ~(GPIO_CRL_CNF2_0 | GPIO_CRL_CNF2_1);

	// PORT A | PIN 4 | OUTPUT MODE | MAX SPEED = 50MHz | PUSH-PULL
	GPIOA->CRL |= GPIO_CRL_MODE4;
	GPIOA->CRL &= ~(GPIO_CRL_CNF4);

	// PORT A | PIN 5 | OUTPUT MODE | MAX SPEED = 50MHz | PUSH-PULL
	GPIOA->CRL |= GPIO_CRL_MODE5;
	GPIOA->CRL &= ~(GPIO_CRL_CNF5);

	// PORT B | PIN 6 | OUTPUT MODE | MAX SPEED = 50MHz | ALTERNATE FUNCTION
	GPIOB->CRL |= GPIO_CRL_MODE6;
	GPIOB->CRL |= GPIO_CRL_CNF6_1;
	GPIOB->CRL &= ~(GPIO_CRL_CNF6_0);

	// PORT B | PIN 7 | OUTPUT MODE | MAX SPEED = 50MHz | ALTERNATE FUNCTION
	GPIOB->CRL |= GPIO_CRL_MODE7;
	GPIOB->CRL |= GPIO_CRL_CNF7_1;
	GPIOB->CRL &= ~(GPIO_CRL_CNF7_0);
}

void Timers_Init(void) {
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;          // TIMER 4 ENABLE
	TIM4->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E; // CHANNEL 1 & CHANNEL 2 -> OUTPUT
	TIM4->CR1 |= TIM_CR1_ARPE;     // ENABLE ARPE | ARPE -> AUTO PRE-LOAD ENABLE

	TIM4->CCMR1 |= TIM_CCMR1_OC1PE;   // CHANNEL 1 PRELOAD ENABLE
	TIM4->CCMR1 |= TIM_CCMR1_OC2PE;   // CHANNEL 2 PRELOAD ENABLE

	// OUPUT COMPARE 1 MODE SET AS PWM MODE 1
	TIM4->CCMR1 |= (TIM_CCMR1_OC1M_2) | (TIM_CCMR1_OC1M_1);
	TIM4->CCMR1 &= ~(TIM_CCMR1_OC1M_0);

	// OUPUT COMPARE 2 MODE SET AS PWM MODE 1
	TIM4->CCMR1 |= (TIM_CCMR1_OC2M_2) | (TIM_CCMR1_OC2M_1);
	TIM4->CCMR1 &= ~(TIM_CCMR1_OC2M_0);

	TIM4->PSC = 1;     // PRESCALAR
	TIM4->ARR = 255;  // AUTO-RELOAD VALUE -> (2^16 - 1)
	TIM4->CCR1 = 0;    // CAPTURE/COMPARE REGISTERS
	TIM4->CCR2 = 0;

	TIM4->EGR |= TIM_EGR_UG; // BEFORE STARTING TIMER -> INITIALIZE ALL REGISTERS
	TIM4->CR1 |= TIM_CR1_CEN;  // COUNTER ENABLE
}


// INVERSE KINEMATICS | PARAMETERS -> DESIRED

void Inverse_Kinematics(float X_0, float y)
{

  // LINK LENGTHS

  float a1 = 5; // LINK LENTGH 1
  float a2 = 2; // LINK LENGTH 2

  // WORKSPACE PROBLEM

  if(((y*y)+(X_0*X_0)) > ((a1+a2)*(a1+a2)))
  {

        X_0 = (a1+a2)*(X_0)/(sqrt((X_0*X_0)+(y*y)));
        y = (a1+a2)*(y)/(sqrt((X_0*X_0)+(y*y)));
  }

  else
  {
         X_0 += 0;
         y += 0;
  }


  // INVERSE KINEMATICS

  double r = sqrt(pow(X_0, 2) + pow(y, 2));
 // Serial.println("R= ");
 // Serial.println(r);
  double phi1;
  double phi2;
  double phi3;

  // ELBOW UP FOR ACTUAL ARM

  if(X_0 < 0 )
  {
     phi1 = -acos((pow(a1, 2) + pow(r, 2) - pow(a2, 2)) / (2*a1*r));

                 phi2 = atan(y / (X_0 + 0.0000001)) + pi;
                 //phi2 = pi - atan(y / (X_0 + 0.0000001)) ;
                 phi3 = -acos((pow(a1, 2) + pow(a2, 2) - pow(r, 2)) / (2*a1*a2));
  }

  else
  {
    phi1 = acos((pow(a1, 2) + pow(r, 2) - pow(a2, 2)) / (2*a1*r));

                phi2 = atan(y / (X_0 + 0.0000001));
                phi3 = acos((pow(a1, 2) + pow(a2, 2) - pow(r, 2)) / (2*a1*a2));
  }
 // Serial.println("phi1 :");
 // Serial.print(phi1);
  // NOTE: LINE 252 & 259 -> X_0 + 0.0000001 -> 0.0000001 IS ADDED TO PREVENT ERROR IN CODE WHEN X_0 = 0

  // JOINT ANGLES

   T1 = (phi1 + phi2)*(180/pi);
   T2 = (phi3 - pi)*(180/pi);

  // Serial.println("T1 = ");
  // Serial.println(T1);

  // Serial.println("T2 = ");
  // Serial.println(T2);



}


unsigned char Conv(float val)
{
	unsigned char ret;
	ret = (unsigned char)(val*255);
	return ret;
}


long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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
	while (1) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


