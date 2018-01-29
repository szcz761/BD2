/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint16_t ADC_val[2];
uint8_t ADC1_0[2];
uint8_t SHARP1_0[4];
const uint16_t Duty = 1000;
const uint16_t prog = 2000;
uint8_t start = 0;
uint8_t startleft = 0;
uint8_t data = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM17_Init(void);
                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void adc_lvl();

void ADC_Line();

void forward();

void back();

void forwardRight();

void forwardLeft();

void hardRight();

void hardLeft();

void left();

void right();

void stop();

void lookForEnemy();

void motorsForward();

void fight();

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM17_Init();

  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim17);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  //HAL_ADC_Start_DMA(&hadc, ADC_val, 2);

  HAL_UART_Receive_DMA(&huart1,&data,1);

  motorsForward();

	if(HAL_GPIO_ReadPin(SHARP_L_GPIO_Port, SHARP_L_Pin) == GPIO_PIN_RESET){
		startleft = 1;
		HAL_Delay(500);
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		HAL_Delay(500);
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	}
	else if(HAL_GPIO_ReadPin(SHARP_P_GPIO_Port, SHARP_P_Pin) == GPIO_PIN_RESET){
		startleft = 0;
		HAL_Delay(500);
		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
		HAL_Delay(500);
		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
	}


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  //BT start
	while(HAL_GPIO_ReadPin(MOD_START_GPIO_Port, MOD_START_Pin) == GPIO_PIN_SET){
	//adc_lvl();
	//ADC_Line();
	lookForEnemy();
	fight();
	}
	stop();
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
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

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_7;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 2;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM17 init function */
static void MX_TIM17_Init(void)
{

  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 47;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 999;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

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
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SL_IN1_Pin|SL_IN2_Pin|SP_IN2_Pin|SP_IN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BUZZ_Pin|LED2_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SL_IN1_Pin SL_IN2_Pin SP_IN2_Pin SP_IN1_Pin */
  GPIO_InitStruct.Pin = SL_IN1_Pin|SL_IN2_Pin|SP_IN2_Pin|SP_IN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MOD_START_Pin */
  GPIO_InitStruct.Pin = MOD_START_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(MOD_START_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BUZZ_Pin LED2_Pin LED1_Pin */
  GPIO_InitStruct.Pin = BUZZ_Pin|LED2_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SHARP_P_Pin SHARP_PP_Pin SHARP_LP_Pin */
  GPIO_InitStruct.Pin = SHARP_P_Pin|SHARP_PP_Pin|SHARP_LP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SHARP_L_Pin */
  GPIO_InitStruct.Pin = SHARP_L_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SHARP_L_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void adc_lvl(){
//	if (ADC_val[0] < 500)  //prawy
//		ADC1_0[0] = 1;
//	else
//		ADC1_0[0] = 0;
	ADC1_0[0] = 0;
	if (ADC_val[1] < 1000) //lewy
		ADC1_0[1] = 1;
	else
		ADC1_0[1] = 0;

}

void forward(){
	  TIM2->CCR3 = Duty;
	  TIM2->CCR4 = Duty;
}

void back(){
	HAL_GPIO_TogglePin(SP_IN1_GPIO_Port, SP_IN1_Pin);			//ustawienie jednego silnika na jazde w przeciwna strone
	HAL_GPIO_TogglePin(SP_IN2_GPIO_Port, SP_IN2_Pin);
	HAL_GPIO_TogglePin(SL_IN1_GPIO_Port, SL_IN1_Pin);			//ustawienie jednego silnika na jazde w przeciwna strone
	HAL_GPIO_TogglePin(SL_IN2_GPIO_Port, SL_IN2_Pin);
	forward();
}

void forwardRight(){
	  TIM2->CCR3 = Duty;
	  TIM2->CCR4 = Duty * 0.6;
}

void forwardLeft(){
	  TIM2->CCR3 = Duty * 0.6;
	  TIM2->CCR4 = Duty;
}

void hardRight(){
	 HAL_GPIO_WritePin(SP_IN1_GPIO_Port, SP_IN1_Pin, GPIO_PIN_SET);
	 HAL_GPIO_WritePin(SP_IN2_GPIO_Port, SP_IN2_Pin, GPIO_PIN_RESET);
	 TIM2->CCR3 = Duty;
	 TIM2->CCR4 = Duty;
}

void hardLeft(){
	HAL_GPIO_WritePin(SL_IN1_GPIO_Port, SL_IN1_Pin, GPIO_PIN_SET);		//ustawienie silnikow na jazde do przodu
	HAL_GPIO_WritePin(SL_IN2_GPIO_Port, SL_IN2_Pin, GPIO_PIN_RESET);
	TIM2->CCR3 = Duty;
	TIM2->CCR4 = Duty;
}

void left(){
	  TIM2->CCR3 = 0;
	  TIM2->CCR4 = Duty;
}

void right(){
	  TIM2->CCR3 = Duty; 	 //lewe kolo
	  TIM2->CCR4 = 0;		 //prawe kolo
}

void stop(){
	  TIM2->CCR3 = 0; 	 //lewe kolo
	  TIM2->CCR4 = 0;		 //prawe kolo
}

void lookForEnemy(){
	if (HAL_GPIO_ReadPin(SHARP_L_GPIO_Port, SHARP_L_Pin) == GPIO_PIN_RESET) {
		SHARP1_0[0] = 1;
		SHARP1_0[3] = 0;
	}
	else{
		//SHARP1_0[0] = 0;
	}
	if (HAL_GPIO_ReadPin(SHARP_LP_GPIO_Port, SHARP_LP_Pin) == GPIO_PIN_RESET) {
		SHARP1_0[1] = 1;
		SHARP1_0[0] = 0;
		SHARP1_0[3] = 0;
	}
	else{
		SHARP1_0[1] = 0;
	}
	if (HAL_GPIO_ReadPin(SHARP_PP_GPIO_Port, SHARP_PP_Pin) == GPIO_PIN_RESET) {
		SHARP1_0[2] = 1;
		SHARP1_0[0] = 0;
		SHARP1_0[3] = 0;
	}
	else{
		SHARP1_0[2] = 0;
	}
	if (HAL_GPIO_ReadPin(SHARP_P_GPIO_Port, SHARP_P_Pin) == GPIO_PIN_RESET) {
		SHARP1_0[3] = 1;
		SHARP1_0[0] = 0;
	}
	else{
		//SHARP1_0[3] = 0;
	}
}

void ADC_Line(){
	if(ADC1_0[0] == 1 || ADC1_0[1] == 1){
		 back();
		 HAL_Delay(100);
	}
//	if(ADC1_0[0] == 1 && ADC1_0[1] == 0){   //prawy
//		hardlLeft();
//		HAL_Delay(20);
//	}
//	if(ADC1_0[0] == 0 && ADC1_0[1] == 1){   //lewy
//		 hardRight();
//		 HAL_Delay(20);
//	}
}

void motorsForward(){
	  HAL_GPIO_WritePin(SL_IN1_GPIO_Port, SL_IN1_Pin, GPIO_PIN_RESET);		//ustawienie silnikow na jazde do przodu
	  HAL_GPIO_WritePin(SL_IN2_GPIO_Port, SL_IN2_Pin, GPIO_PIN_SET);

	  HAL_GPIO_WritePin(SP_IN1_GPIO_Port, SP_IN1_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(SP_IN2_GPIO_Port, SP_IN2_Pin, GPIO_PIN_SET);
}

void fight(){
	 if(SHARP1_0[1] || SHARP1_0[2]){
		 motorsForward();
		 if(SHARP1_0[1] && SHARP1_0[2]){
			 forward();
		 }
		 else if(!SHARP1_0[1] && SHARP1_0[2]){
			 forwardRight();
		 }
		 else if(SHARP1_0[1] && !SHARP1_0[2]){
			 forwardLeft();
		 }
		 HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
	 }
	 else if(SHARP1_0[0]){
		 motorsForward();
		 left();
		 startleft = 1;
		 HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
	 }
	 else if(SHARP1_0[3]){
		 motorsForward();
		 startleft = 0;
		 right();
		 HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
	 }
	 else{
		 if(startleft){
			 hardLeft();
		 }
		 else{
			 hardRight();
		 }
	}
}


//void bluetooth(){
//	 static uint16_t cnt = 0; // Licznik wyslanych wiadomosci
//	 uint8_t data[50];// Tablica przechowujaca wysylana wiadomosc.
//	 uint16_t size = 0; // Rozmiar wysylanej wiadomosci ++cnt; // Zwiekszenie licznika wyslanych wiadomosci.
//
//	 ++cnt; // Zwiekszenie licznika wyslanych wiadomosci.
////	 size = sprintf(data, "Liczba wyslanych wiadomosci: %d.\n\r", cnt); // Stworzenie wiadomosci do wyslania oraz przypisanie ilosci wysylanych znakow do zmiennej size.
////	 HAL_UART_Transmit_IT(&huart1, data, size); // Rozpoczecie nadawania danych z wykorzystaniem przerwan
//}

 void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
 {
	 if (data == 128){
		 start = 1;
	 }
	 else if(data == 248){
		 start = 0;
	 }
 }
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
 if (data == 128)
 {
	 start = 1;

 }
 else if(data == 248)
 {
	 start = 0;
 }
}
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//
// // Odebrany znak zostaje przekonwertowany na liczbe calkowita i sprawdzony
// // instrukcja warunkowa
// switch (atoi(&Received)) {
//
// case 0: // Jezeli odebrany zostanie znak 0
// start = 0;
// break;
//
// case 1: // Jezeli odebrany zostanie znak 1
// start = 1;
// break;
//
// default: // Jezeli odebrano nieobslugiwany znak
// //size = sprintf(Data, "Odebrano nieznany znak: %c\n\r", Received);
// break;
// }
//}

//void HAL_SYSTICK_Callback(void) {
//	adc_lvl();
//	ADC_Line();
//
//
//}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
