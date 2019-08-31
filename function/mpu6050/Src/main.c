/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "MPU6050.h"
#include "stdio.h"
#include "math.h"
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
int16_t Accel_X, Accel_Y, Accel_Z;
int16_t Gyro_X, Gyro_Y, Gyro_Z;
float Ac_X1, Ac_Y1, Ac_Z1, Gy_X1, Gy_Y1, Gy_Z1;
float Ac_X2, Ac_Y2, Ac_Z2;
float Deg_X, Deg_Y, Deg_Z, Deg_XC, Deg_YC, Deg_ZC;
uint8_t MPU6050 = 0;

#define M_PI  (180.0/3.141592)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define MPU6050  MPU6050_DEFAULT_ADDRESS

void MPU6050_Write(uint8_t Address, uint8_t data){
    HAL_I2C_Mem_Write(&hi2c1, MPU6050, Address, 1, (uint8_t *)&data, 1, 10);
}

void MPU6050_Write_bits(uint8_t Address, uint8_t bitStart, uint8_t length, uint8_t data){
    uint8_t tmp = 0;
    HAL_I2C_Mem_Read(&hi2c1, MPU6050, Address, 1, (uint8_t *)&tmp, 1, 10);
    uint8_t mask = 0;
    switch(length){
        case 1: mask = 0x01; break;
        case 2: mask = 0x03; break;
        case 3: mask = 0x07; break;
        case 4: mask = 0x0F; break;
        case 5: mask = 0x1F; break;
        case 6: mask = 0x3F; break;
        case 7: mask = 0x7F; break;
        case 8: mask = 0xFF; break;
    }
    tmp &= ~(mask << bitStart);
    tmp |= (data << bitStart);
    HAL_I2C_Mem_Write(&hi2c1, MPU6050, Address, 1, (uint8_t *)&tmp, 1, 10);
}
 
uint8_t MPU6050_Read(uint8_t Address){
    uint8_t data = 0;
    HAL_I2C_Mem_Read(&hi2c1, MPU6050, Address, 1, (uint8_t *)&data, 1, 10);
    return data;
}
 
void init_MPU6050(void){
    uint8_t temp = MPU6050_Read(MPU6050_RA_WHO_AM_I);
    printf("Who am I = 0x%02X\r\n", temp);
    printf("MPU6050 Initialize..... \r\n");
    printf("------------------------\r\n");
 
    HAL_Delay(100);
    MPU6050_Write_bits(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, 0x01, DISABLE); // SetSleepModeStatus
    HAL_Delay(10);
    MPU6050_Write_bits(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, MPU6050_CLOCK_PLL_XGYRO);      // SetClockSource
    MPU6050_Write_bits(MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, MPU6050_GYRO_FS_250);   // SetFullScaleGyroRange
    MPU6050_Write_bits(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, MPU6050_ACCEL_FS_2); // SetFullScaleAccelRange
}
 
void read_MPU6050_data(void){
    Accel_X = (MPU6050_Read(MPU6050_RA_ACCEL_XOUT_H)<<8) | MPU6050_Read(MPU6050_RA_ACCEL_XOUT_L);
    Accel_Y = (MPU6050_Read(MPU6050_RA_ACCEL_YOUT_H)<<8) | MPU6050_Read(MPU6050_RA_ACCEL_YOUT_L);
    Accel_Z = (MPU6050_Read(MPU6050_RA_ACCEL_ZOUT_H)<<8) | MPU6050_Read(MPU6050_RA_ACCEL_ZOUT_L);
    Gyro_X  = (MPU6050_Read(MPU6050_RA_GYRO_XOUT_H)<<8) | MPU6050_Read(MPU6050_RA_GYRO_XOUT_L);
    Gyro_Y  = (MPU6050_Read(MPU6050_RA_GYRO_YOUT_H)<<8) | MPU6050_Read(MPU6050_RA_GYRO_YOUT_L);
    Gyro_Z  = (MPU6050_Read(MPU6050_RA_GYRO_ZOUT_H)<<8) | MPU6050_Read(MPU6050_RA_GYRO_ZOUT_L);
}
 
int _write(int32_t file, uint8_t *ptr, int32_t len){
    HAL_UART_Transmit(&huart2, ptr, len, 10);
    return len;
}

int fputc(int ch,FILE *f)
{
	HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1 , 0xFFFF);
	return ch;
}
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	init_MPU6050();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		/*
		read_MPU6050_data();
    printf("Accel_X = %6d / Accel_Y = %6d / Accel_Z = %6d \r\n", Accel_X, Accel_Y, Accel_Z);
    printf("Gyro_X  = %6d / Gyro_Y  = %6d / Gyro_Z  = %6d \r\n", Gyro_X, Gyro_Y, Gyro_Z);
    //printf("%6d,%6d,%6d \r\n", Gyro_X, Gyro_Y, Gyro_Z);//processing usart code
    
		printf("--------------------------------------------------------\r\n");
    HAL_Delay(200);
		*/
		
		static uint16_t cnt=1;
    read_MPU6050_data();
    Ac_X1 = (float)Accel_X / 6384.0;
    Ac_Y1 = (float)Accel_Y / 6384.0;
    Ac_Z1 = (float)Accel_Z / 6384.0;
    Gy_X1 = (float)Gyro_X / 131.0;
    Gy_Y1 = (float)Gyro_Y / 131.0;
    Gy_Z1 = (float)Gyro_Z / 131.0;
 
    Ac_X2 = Ac_X2 * ((float)cnt-1) / (float)cnt + Ac_X1 / (float)cnt;
    Ac_Y2 = Ac_Y2 * ((float)cnt-1) / (float)cnt + Ac_Y1 / (float)cnt;
    Ac_Z2 = Ac_Z2 * ((float)cnt-1) / (float)cnt + Ac_Z1 / (float)cnt;
 
    Deg_X = atan(Ac_Y1 / sqrt(pow(Ac_X1, 2) + pow(Ac_Z1, 2))) * 180.0 / M_PI;
    Deg_Y = atan(Ac_X1 / sqrt(pow(Ac_Y1, 2) + pow(Ac_Z1, 2))) * 180.0 / M_PI;
    Deg_Z = atan(sqrt(pow(Ac_X1, 2) + pow(Ac_Y1, 2)) / Ac_Z1) * 180.0 / M_PI;
 
    /* Complementary filter */
    Deg_XC = 0.98 * (Deg_XC + Gy_X1 * 0.005) + 0.02 * Deg_X;
    Deg_YC = 0.98 * (Deg_YC + Gy_Y1 * 0.005) + 0.02 * Deg_Y;
    Deg_ZC = 0.98 * Deg_ZC + 0.02 * Deg_Z;
 
    printf("%5.2f, %5.2f, %5.2f\r\n", Deg_XC, Deg_YC, Deg_ZC);
		
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    HAL_Delay(5);
    if(++cnt>20) cnt=20;
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  hi2c1.Init.ClockSpeed = 400;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
