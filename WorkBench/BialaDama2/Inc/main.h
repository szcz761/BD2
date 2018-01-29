/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define SL_IN1_Pin GPIO_PIN_0
#define SL_IN1_GPIO_Port GPIOA
#define SL_IN2_Pin GPIO_PIN_1
#define SL_IN2_GPIO_Port GPIOA
#define SL_PWM_Pin GPIO_PIN_2
#define SL_PWM_GPIO_Port GPIOA
#define SP_PWM_Pin GPIO_PIN_3
#define SP_PWM_GPIO_Port GPIOA
#define SP_IN2_Pin GPIO_PIN_4
#define SP_IN2_GPIO_Port GPIOA
#define SP_IN1_Pin GPIO_PIN_5
#define SP_IN1_GPIO_Port GPIOA
#define KTIR_P_Pin GPIO_PIN_6
#define KTIR_P_GPIO_Port GPIOA
#define KTIR_L_Pin GPIO_PIN_7
#define KTIR_L_GPIO_Port GPIOA
#define MOD_START_Pin GPIO_PIN_1
#define MOD_START_GPIO_Port GPIOB
#define BUZZ_Pin GPIO_PIN_2
#define BUZZ_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_10
#define LED2_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_11
#define LED1_GPIO_Port GPIOB
#define SHARP_P_Pin GPIO_PIN_12
#define SHARP_P_GPIO_Port GPIOB
#define SHARP_PP_Pin GPIO_PIN_14
#define SHARP_PP_GPIO_Port GPIOB
#define SHARP_LP_Pin GPIO_PIN_15
#define SHARP_LP_GPIO_Port GPIOB
#define SHARP_L_Pin GPIO_PIN_9
#define SHARP_L_GPIO_Port GPIOA
#define PROG1_Pin GPIO_PIN_13
#define PROG1_GPIO_Port GPIOA
#define PROG2_Pin GPIO_PIN_14
#define PROG2_GPIO_Port GPIOA
#define BT_TX_Pin GPIO_PIN_6
#define BT_TX_GPIO_Port GPIOB
#define BT_RX_Pin GPIO_PIN_7
#define BT_RX_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
