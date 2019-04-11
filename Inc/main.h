/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define BUZZER_OUT_Pin GPIO_PIN_13
#define BUZZER_OUT_GPIO_Port GPIOC
#define RELAY_3_Pin GPIO_PIN_14
#define RELAY_3_GPIO_Port GPIOC
#define LED_OVER_TANK_HIGH_Pin GPIO_PIN_3
#define LED_OVER_TANK_HIGH_GPIO_Port GPIOA
#define LED_OVER_TANK_LOW_Pin GPIO_PIN_4
#define LED_OVER_TANK_LOW_GPIO_Port GPIOA
#define LED_SUMP_INLET_Pin GPIO_PIN_5
#define LED_SUMP_INLET_GPIO_Port GPIOA
#define LED_SUMP_TANK_HIGH_Pin GPIO_PIN_6
#define LED_SUMP_TANK_HIGH_GPIO_Port GPIOA
#define LED_SUMP_TANK_LOW_Pin GPIO_PIN_7
#define LED_SUMP_TANK_LOW_GPIO_Port GPIOA
#define POWER_INTERRUPT_Pin GPIO_PIN_2
#define POWER_INTERRUPT_GPIO_Port GPIOB
#define POWER_INTERRUPT_EXTI_IRQn EXTI2_3_IRQn
#define RELAY_2_Pin GPIO_PIN_10
#define RELAY_2_GPIO_Port GPIOB
#define RELAY_1_Pin GPIO_PIN_11
#define RELAY_1_GPIO_Port GPIOB
#define LED_MOTOR_ON_Pin GPIO_PIN_12
#define LED_MOTOR_ON_GPIO_Port GPIOB
#define LED_DRY_RUN_Pin GPIO_PIN_13
#define LED_DRY_RUN_GPIO_Port GPIOB
#define LED_VOLTAGE_HIGH_Pin GPIO_PIN_14
#define LED_VOLTAGE_HIGH_GPIO_Port GPIOB
#define LED_VOLTAGE_LOW_Pin GPIO_PIN_15
#define LED_VOLTAGE_LOW_GPIO_Port GPIOB
#define SW_RESET_Pin GPIO_PIN_8
#define SW_RESET_GPIO_Port GPIOA
#define SUMP_TRIGGER_OUT_Pin GPIO_PIN_9
#define SUMP_TRIGGER_OUT_GPIO_Port GPIOA
#define SUMP_SENSE_INPUT_Pin GPIO_PIN_10
#define SUMP_SENSE_INPUT_GPIO_Port GPIOA
#define SUMP_HIGH_INPUT_Pin GPIO_PIN_11
#define SUMP_HIGH_INPUT_GPIO_Port GPIOA
#define SUMP_LOW_INPUT_Pin GPIO_PIN_12
#define SUMP_LOW_INPUT_GPIO_Port GPIOA
#define OVER_LOW_INPUT_Pin GPIO_PIN_6
#define OVER_LOW_INPUT_GPIO_Port GPIOF
#define OVER_HIGH_INPUT_Pin GPIO_PIN_7
#define OVER_HIGH_INPUT_GPIO_Port GPIOF
#define OVER_TRIGGER_OUT_Pin GPIO_PIN_15
#define OVER_TRIGGER_OUT_GPIO_Port GPIOA
#define OVER_SENSE_INPUT_Pin GPIO_PIN_3
#define OVER_SENSE_INPUT_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
