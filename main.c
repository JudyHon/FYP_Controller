/* USER CODE BEGIN Header */
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "string.h"
#include <math.h>
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t RX1_Char = 0x00;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_USART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Receive_IT(&huart1, &RX1_Char, 1);
}

float valueConvert(uint32_t value) {
		float newValue = (float)value * 2 /4095 - 1;
		if (value <= 2000 || value >= 3000) return newValue;
		return 0;
}
uint32_t VR[4];

#define MPU6050_ADDR 0xD0

#define SMPLRT_DIV_REG 0x19
#define CONFIG_REG 0x1A
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

#define PI acos(-1)

void MPU6050_Init (void)
{
	
	uint8_t check, data;
	HAL_I2C_Mem_Read (&hi2c2, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);
	
	if (check == 104) // if the device is present
	{
			data = 0x00;
			HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 1000);
		
			data = 0x07;
			HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, 1000);
	
			data = 0x06;
			HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, CONFIG_REG, 1, &data, 1, 1000);
		
			data = 0x01;
			HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, 1000);
			
			data = 0x18;
			HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, 1000);
		
		
	}
	
}

int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW, Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW;
float Ax, Ay, Az, Gx, Gy, Gz;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float elapsedTime, currentTime, previousTime;
float accVector;

void MPU6050_Read_Accel (void)
{
	uint8_t Rec_Data[6];
	
	HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);
	
	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
	
	Ax = Accel_X_RAW/16384.0 - 0.1;
	Ay = Accel_Y_RAW/16384.0 - 0.02;
	Az = Accel_Z_RAW/16384.0;
	
	accAngleX = (atan(Ay / sqrt(pow(Ax, 2) + pow(Az, 2))) * 180 / PI) - 0.58;
	accAngleY = (atan(-1 * Ax / sqrt(pow(Ay, 2) + pow(Az, 2))) * 180 / PI) + 1.58;
}

void MPU6050_Read_Gyro (void)
{
	previousTime = currentTime;
	currentTime = HAL_GetTick();
	elapsedTime = (currentTime - previousTime) / 1000;
	uint8_t Rec_Data[6];
	
	HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);
	
	Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
	Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
	
	Gx = Gyro_X_RAW/131.0 - 0.02;
	Gy = Gyro_Y_RAW/131.0 + 0.12;
	Gz = Gyro_Z_RAW/131.0 - 0.15;
	
//	Gx = Gyro_X_RAW/131.0;
//	Gy = Gyro_Y_RAW/131.0;
//	Gz = Gyro_Z_RAW/131.0;
	
	gyroAngleX = gyroAngleX + Gx * elapsedTime;
	gyroAngleY = gyroAngleY + Gy * elapsedTime;
	
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	
	char MSG[100];
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart1, &RX1_Char, 1);
	
	uint8_t MPU6050_Addr = 0x68;
	
	uint8_t HMC5883L_Addr = 0x1E;
	uint8_t CRA = 0x70; uint8_t CRB = 0xA0;
	
	uint8_t MR = 0x01; uint8_t DXRA, DXRB, DYRA, DYRB, DZRA, DZRB;
	
	HAL_I2C_Mem_Write(&hi2c2,HMC5883L_Addr<<1,0x00,1,&CRA,1,100);
	HAL_I2C_Mem_Write(&hi2c2,HMC5883L_Addr<<1,0x01,1,&CRB,1,100);
	
	HAL_ADC_Start_DMA(&hadc1, VR, 4); // start adc in dma mode for multichannel
  
	char str[5];
	
	uint8_t button0 = 0, button1 = 0, button2 = 0, button3 = 0, button4 = 0, button5 = 0;
	uint8_t vibrate = 0;
	
	float result;
	
	MPU6050_Init();
	HAL_Delay(1000);
	
	uint8_t Rx_data[10];
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
			strcat(MSG, "START,");
			
			// L-Joystick X
			result = valueConvert(VR[3]);
			if (result > 0.2 || result < - 0.2) {
				sprintf(str, "%d", VR[3]);
				sprintf(str, "%.3f", -result);
				strcat(MSG, str);
			}
			else {strcat(MSG, "0");}
			strcat(MSG, ",");
			
			// L-Joystick Y
			result = valueConvert(VR[2]);
			if (result > 0.2 || result < - 0.2) {
				sprintf(str, "%d", VR[2]);
				sprintf(str, "%.3f", -result);
				strcat(MSG, str);
			}
			else {strcat(MSG, "0");}
			strcat(MSG, ",");
			
			// R-Joystick X
			result = valueConvert(VR[1]);
			if (result > 0.2 || result < - 0.2) {
				sprintf(str, "%d", VR[1]);
				sprintf(str, "%.3f", -result);
				strcat(MSG, str);
			}
			else {strcat(MSG, "0");}
			strcat(MSG, ",");
			
			// R-Joystick Y
			result = valueConvert(VR[0]);
			if (result > 0.2 || result < - 0.2) {
				sprintf(str, "%d", VR[0]);
				sprintf(str, "%.3f", -result);
				strcat(MSG, str);
			}
			else {strcat(MSG, "0");}
			strcat(MSG, ",");
					
			// Button X
//			if (button0 == 1) {
//				if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6))
//				{
//					button0 = 0;
//				}
//				strcat(MSG, "0");
//			}
//			else {
//				if(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6)) {
//					button0 = 1;
//					strcat(MSG, "1");
//				}
//				else {strcat(MSG, "0");}
//			}
//			strcat(MSG, ",");
//			
//			// Button A
//			if (button1 == 1) {
//				if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9))
//				{
//					button1 = 0;
//				}
//				strcat(MSG, "0");
//			}
//			else {
//				if(!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)) {
//					button1 = 1;
//					strcat(MSG, "1");
//				}
//				else {strcat(MSG, "0");}
//			}
//			strcat(MSG, ",");
//			
			// Button B
//			if (button2 == 1) {
//				if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5))
//				{
//					button2 = 0;
//				}
//				strcat(MSG, "0");
//			}
//			else {
//				if(!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)) {
//					button2 = 1;
//					strcat(MSG, "1");
//				}
//				else {strcat(MSG, "0");}
//			}
//			strcat(MSG, ",");
//			
//			// Button Y
//			if (button3 == 1) {
//				if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6))
//				{
//					button3 = 0;
//				}
//				strcat(MSG, "0");
//			}
//			else {
//				if(!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6)) {
//					button3 = 1;
//					strcat(MSG, "1");
//				}
//				else {strcat(MSG, "0");}
//			}
//			strcat(MSG, ",");
//			
//			// Button
//			if (button4 == 1) {
//				if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8))
//				{
//					button4 = 0;
//				}
//				strcat(MSG, "0");
//			}
//			else {
//				if(!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8)) {
//					button4 = 1;
//					strcat(MSG, "1");
//				}
//				else {strcat(MSG, "0");}
//			}		
//			strcat(MSG, ",");
//			
//			// Button
//			if (button5 == 1) {
//				if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7))
//				{
//					button5 = 0;
//				}
//				strcat(MSG, "0");
//			}
//			else {
//				if(!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7)) {
//					button5 = 1;
//					strcat(MSG, "1");
//				}
//				else {strcat(MSG, "0");}
//			}		
//			strcat(MSG, ",");
			
			if (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6)) {
				strcat(MSG, "1");
			} else {
				strcat(MSG, "0");
			}
			strcat(MSG, ",");
			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)) {
				strcat(MSG, "1");
			} else {
				strcat(MSG, "0");
			}
			strcat(MSG, ",");
			if (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)) {
				strcat(MSG, "1");
			} else {
				strcat(MSG, "0");
			}
			strcat(MSG, ",");
			if (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6)) {
				strcat(MSG, "1");
			} else {
				strcat(MSG, "0");
			}
			strcat(MSG, ",");
			if (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8)) {
				strcat(MSG, "1");
			} else {
				strcat(MSG, "0");
			}
			strcat(MSG, ",");
			if (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7)) {
				strcat(MSG, "1");
			} else {
				strcat(MSG, "0");
			}
			strcat(MSG, ",");
			
			// IMU - MPU6050
			
			MPU6050_Read_Accel();
			MPU6050_Read_Gyro();
						
			yaw = yaw + Gz * elapsedTime;
//			accAngleX = atan2(Ay, sqrt(pow(Ax,2) +pow(Az,2)))*180/PI;
//			accAngleY = atan2(Ax, sqrt(pow(Ay, 2) + pow(Az,2)))*180/PI;
//			gyroAngleX = gyroAngleX + Gx * elapsedTime;
//			gyroAngleY = gyroAngleY + Gy * elapsedTime;
			
			roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
			pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
			
			sprintf(str, "%.2f", roll);
			strcat(MSG, str);
			strcat(MSG, ",");

			sprintf(str, "%.2f", pitch);
			strcat(MSG, str);
			strcat(MSG, ",");
			
			sprintf(str, "%.2f", yaw);
			strcat(MSG, str);
			
//			strcat(MSG, ",");
//			HAL_UART_Receive(&huart1, Rx_data, 10, 5);
//			
//			sprintf(str, "%d", (uint8_t)Rx_data);
//			strcat(MSG, str);			
//			
			strcat(MSG, "\r\n");
			
			// UART Transmit
			HAL_UART_Transmit(&huart1, (uint8_t *) MSG, sizeof(MSG), 10);
			memset(MSG, 0, sizeof(MSG));

//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
			
			
			if(RX1_Char == '1') {
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);				
				HAL_UART_Receive_IT(&huart1, &RX1_Char, 1);
				RX1_Char = 0x00;
				vibrate = 1;
			}
			if (vibrate > 0) {
				vibrate++;
				if (vibrate == 1*5) {
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
					vibrate = 0;
				}
			}
			HAL_Delay(10);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  huart1.Init.BaudRate = 115200;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 PB6 
                           PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6 
                          |GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
