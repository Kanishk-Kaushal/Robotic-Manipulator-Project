/* USER CODE BEGIN Header */
//Link 1 note
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

#include "lsm9ds1_reg.h"
#include <stdio.h>

#define PI 3.14

//#include "lsm9ds1_read_data_polling.c"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

UART_HandleTypeDef huart1;

// Function to Print via UART
int _write(int fd, char* ptr, int len)
{
    HAL_UART_Transmit(&huart1, (uint8_t *) ptr, len, HAL_MAX_DELAY);
    return len;
}

float lsm9ds1_read_data_polling(void);
//void Blink (void);

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

CAN_FilterTypeDef sFilterConfig, sFilterConfig1;	// struct containing filter settings
CAN_RxHeaderTypeDef RxMessage;	 					// struct for recieved data frame
CAN_TxHeaderTypeDef TxMessage;   					// struct for transmitted dataframe
int rxData[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };		// array for recieved data (8 bytes)
int txData[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };		// array for transmitting data (8 bytes)
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
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_CAN_Init(void);

/* USER CODE BEGIN PFP */

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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */

  float Rolls;
  int int_roll;

  TxMessage.IDE = CAN_ID_STD;				// standard identifier format (11bit)
  TxMessage.StdId = 0x446;					// identifier value
  TxMessage.RTR = CAN_RTR_DATA;				// indicates frame mode (data frame or remote frame)
  TxMessage.DLC = 8;						// data length (8 bytes)
  TxMessage.TransmitGlobalTime = DISABLE;	// time of transmission is not transmitted along with the data

  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;	// filter bank consists of 2 32bit values (mask and ID)
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;		// filter set to mask and ID mode
  sFilterConfig.FilterBank = 0;							// filter bank number 0 selected
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;	// assign filter bank to FIFO 0
  sFilterConfig.FilterIdHigh = 0x211 << 5;				// STD ID value is 7, here shifted by 5 because 11 bits starting from the left are for STD ID (FilterIdHigh is 16bit)
  sFilterConfig.FilterIdLow = 0;						// LSB
  sFilterConfig.FilterMaskIdHigh = 0x211 << 5;			// 0b111 shifted by 5 for the same reason, first 11 bits are for Identifier
  sFilterConfig.FilterMaskIdLow = 0;					// LSB
  sFilterConfig.FilterActivation = ENABLE;				// activate filter

  HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);
  HAL_CAN_Start(&hcan);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

      Rolls = lsm9ds1_read_data_polling();

	  int_roll = Rolls;

	  txData[0] = int_roll; 	// Data to be Transmitted over CAN
	  //txData[0] = 'b';
	  HAL_CAN_AddTxMessage(&hcan, &TxMessage, txData, &usedmailbox);  // Transmitting

	  //HAL_Delay(100);

	  T1 = 60;

	  // IMU VALUES
	  float Curr_pos_1 = lsm9ds1_read_data_polling();
	  double Curr_Pos_2 = abs((IMU_1 - IMU_2));

	  // ERROR
	  Error_1 = T1 - Curr_pos_1;
	  Error_2 = T2 - Curr_Pos_2;

	  if(Error_1 < 0)
	  {

		 GPIOB -> BSRR = (1<<26);
	  }

	  else if(Error_1 > 0)
	  {
		  GPIOB -> BSRR = (1<<11);
	  }

	  Prev_E_1 += Error_1;
	  Prev_E_2 += Error_2;

	  // dE/dT
	  double E_dot_1 = (Error_1 - Prev_E_1)/dt;
	  double E_dot_2 = (Error_2 - Prev_E_2)/dt;

	  // E.dT
	  double E_int_1 = (Error_1)*dt + Prev_E_1;
	  double E_int_2 = (Error_2)*dt + Prev_E_2;

	  // PID CONSTANTS | SUBJECT TO CHANGE
	  double Kp = 4.5;
	  double Kd = 0.00;
	  double Kc = 0.005;

	  // OUPUT PWM SIGNALS
	  PWM_1 =  (((Kp*Error_1) + (Kc*E_int_1) + (Kd*E_dot_1)));
	  PWM_2 =  (Kp*Error_2) + (Kc*E_int_2) + (Kd*E_dot_2);

	  // CAP PWM SIGNAL
	  if(abs(PWM_1) > 80)
	  {
		PWM_1 = 80;
	  }


	  if(abs(Error_1) < 3)
	  {
		 PWM_1 = 0;
		 TIM2 -> CCR3 = PWM_1;
	  }

	  printf("PWM 1: %d  ", abs(PWM_1));

	  TIM2 -> CCR3 = abs(PWM_1);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  __HAL_RCC_CAN1_CLK_ENABLE();
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

//void Blink(void)
//{
//	// BLINKING THE PC13 LED
//
//	GPIOC -> ODR |= 0x2000;
//	HAL_Delay(50);
//	GPIOC -> ODR &= ~0x2000; // ODR13 -> CLEAR | LED -> ON
//	HAL_Delay(50);
//
//}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
