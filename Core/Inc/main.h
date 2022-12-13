/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define BUFFER_SIZE 256

/* practice the gpio programming */
// #define GPIO_REGISTER
// #define GPIO_IRQ

/* use TIM6 to control the LD2 lighting (1s) */
// #define TIMER

/* UART (UART1) transfer */
// #define UART

/* UART retarget (printf & scanf) */
// #define UART_RETARGET

/* UART IRQ */
// #define UART_IRQ

/* UART DMA (LPUART1) transfer */
#define UART_DMA1
// #define UART_DMA2

/* use BUTTON (PC13) to trigger a pulse (PA1), and update the TIM4 counter (PA8) */
// #define PULSE_COT

/* use TIM6 to achieve a electronic clock */
// #define E_CLOCK

/* use TIM2 CH1 to generate PWM signal (20ms), and achieve a breathing light using LD2 (PA5) */
// #define B_LIGHTING

/* use TIM2 CH1 () to generate PWM signal (2000ms), and control LD2 lighting frequency*/
// #define PWM

/* use TIM3 CH1 (PA6) to generate PWM signal, and use TIM5 CH1 (PA0) to capture the input signal */
// #define CAPTURE_SIGNAL

/* frame communication */
// #define FRAME_COMMUNICATION
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define PULSE_Pin GPIO_PIN_1
#define PULSE_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
