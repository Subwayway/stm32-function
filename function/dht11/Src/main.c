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
#include "stdio.h"
#include "dwt_stm32_delay.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

uint32_t millis_cnt=0;

uint32_t millis(){
    return millis_cnt;
}
 
uint32_t micros(){
    uint32_t val = SysTick->VAL;
    uint32_t load = SysTick->LOAD;
    return (millis_cnt&0x3FFFFF)*1000 + (load-val)/((load+1)/1000);
}
 
void delay_us(uint32_t us){
    uint32_t temp = micros() + us;
    while(temp > micros());
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define DHT11_port    GPIOA
#define DHT11_pin     1
#define DHT11_pinmode CRL  //  if(pin < 8) CRL , else CRH
#define DHT11_read    IDR
#pragma anon_unions
union {
  uint32_t data;
  struct {
    uint8_t temp_d;
    uint8_t temp_i;
    uint8_t rh_d;
    uint8_t rh_i;
  };
} DHT;
 
uint8_t DHT_read () {
  uint8_t i, check_sum;
  uint32_t start;
  check_sum = 0;
  DHT.data = 0;
 
  GPIOA->CRL = (DHT11_port->DHT11_pinmode&(~(15<<((DHT11_pin%8)*4))))|(7<<((DHT11_pin%8)*4));  // Output Open-drain (Master send LOW signal)
  HAL_Delay(18);
  GPIOA->CRL = (DHT11_port->DHT11_pinmode&(~(15<<((DHT11_pin%8)*4))))|(4<<((DHT11_pin%8)*4));  // Floating input (Master send HIGH signal & data receive)
 
  start = micros();
  while (DHT11_port->DHT11_read&(1<<DHT11_pin)) {  // HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);
    if (micros()-start > 50) return 1;  // 20~40us
  }
  start = micros();
  while (!(DHT11_port->DHT11_read&(1<<DHT11_pin))) {
    if (micros()-start > 120) return 2;  // 80us
  }
  start = micros();
  while (DHT11_port->DHT11_read&(1<<DHT11_pin)) {
    if (micros()-start > 120) return 3;  // 80us
  }
 
  for (i = 0; i < 32; i++) {
    while (!(DHT11_port->DHT11_read&(1<<DHT11_pin))); // 50us
    start = micros();
    while (DHT11_port->DHT11_read&(1<<DHT11_pin));
    if (micros()-start > 50) DHT.data |= (0x80000000 >> i); // "0"=26~28us, "1"=70us
  }
  for (i = 0; i < 8; i++) {
    while (!(DHT11_port->DHT11_read&(1<<DHT11_pin))); // 50us
    start = micros();
    while (DHT11_port->DHT11_read&(1<<DHT11_pin));
    if (micros()-start > 50) check_sum |= (0x80 >> i);
  }
 
  if ((DHT.rh_i + DHT.rh_d + DHT.temp_i + DHT.temp_d) == check_sum) return 0;
  else return 4;
}

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
UART_HandleTypeDef huart2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2,Temp_value, Rh_value;
uint16_t sum, RH, TEMP;
uint8_t check = 0;
 
GPIO_InitTypeDef GPIO_InitStruct;
 
void set_gpio_output (void)
{
	/*Configure GPIO pin output: PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
 
void set_gpio_input (void)
{
	/*Configure GPIO pin input: PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
 
void DHT11_start (void)
{
	set_gpio_output ();  // set the pin as output
	HAL_GPIO_WritePin (GPIOA, GPIO_PIN_1, 0);   // pull the pin low
	delay_us (18000);
HAL_GPIO_WritePin (GPIOA, GPIO_PIN_1, 1);	// wait for 18ms
	set_gpio_input ();   // set as input
}
 
void check_response (void)
{
	delay_us (40);
	if (!(HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_1)))  // if the pin is low
	{
		delay_us (80);  // wait for 80us
		if ((HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_1))) check = 1;  // now if the pin is high response = ok i.e. check =1
	}
	while ((HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_1)));   // wait for the pin to go low
}
 
uint8_t read_data (void)
{
	uint8_t i,j;
	for (j=0;j<8;j++)
	{
		while (!(HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_1)));   // wait for the pin to go high
		delay_us (40);   // wait for 40 us
		if ((HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_1)) == 0)   // if the pin is low 
		{
			i&= ~(1<<(7-j));   // write 0
		}
		else i|= (1<<(7-j));  // if the pin is high, write 1
		while ((HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_1)));  // wait for the pin to go low
	}
	return i;
}

void DHT11_getdata()
{
	DHT11_start();
		check_response ();
		Rh_byte1 = read_data ();
		Rh_byte2 = read_data ();
		Temp_byte1 = read_data ();
		Temp_byte2 = read_data ();
		sum = read_data();
		if (sum == (Rh_byte1+Rh_byte2+Temp_byte1+Temp_byte2))    // if the data is correct
		{
		Temp_value = ((Temp_byte1/10)+48)+((Temp_byte1%10)+48);

		Rh_value = ((Rh_byte1/10)+48)+((Rh_byte1%10)+48);
		}
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
  MX_USART2_UART_Init();
	DWT_Delay_Init ();
  /* USER CODE BEGIN 2 */
HAL_Delay (1000);
char str[20];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		
		
		
    /* USER CODE BEGIN 3 */
		Rh_byte1=0;
		Rh_byte2=0;
		Temp_byte1=0;
		Temp_byte2=0;
		
		DHT11_start ();
		check_response ();
		Rh_byte1 = read_data ();
		Rh_byte2 = read_data ();
		Temp_byte1 = read_data ();
		Temp_byte2 = read_data ();
		sum = read_data();
		if (sum == (Rh_byte1+Rh_byte2+Temp_byte1+Temp_byte2))    // if the data is correct
		{
			printf("%d %d \n", Rh_byte1, Temp_byte1);
		
		HAL_Delay (5000);
		}
		//set_gpio_output ();
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
		HAL_Delay(300);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
		HAL_Delay(300);
		/*
		 uint8_t error_no = DHT_read();
		 printf("%d %d \n", DHT.temp_i, DHT.rh_i);
		HAL_Delay(1000);
		*/
		
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

  /*Configure GPIO pin : PA1 */
	
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


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
