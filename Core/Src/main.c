/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "stm32g474xx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifdef GPIO_REGISTER

  #define GPIOA_MODER   (*(unsigned int *)(0x48000000UL))  // Mode Register
  #define GPIOA_ODR     (*(unsigned int *)(0x48000014UL)) // Output Data Register
  #define GPIOA_BSRR    (*(unsigned int *)(0x48000018UL))  // Bit Set Register

#endif

#ifdef E_CLOCK
typedef struct {
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
} CLOCK_TypeDef;
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#ifdef UART
extern UART_HandleTypeDef huart1;
uint8_t rec_buffer[BUFFER_SIZE];
#endif

#ifdef UART_RETARGET
extern UART_HandleTypeDef huart1;
uint8_t rec_data;   // uart retarget
#endif

#ifdef UART_IRQ
extern UART_HandleTypeDef huart1;
#define LENGTH 10
uint8_t rx_buffer[LENGTH];    // RX buffer
uint8_t rx_flag = 0;    // RX Complete flag: 0 means RX Not Complete; 1 means RX Complete
#endif

#if (defined UART_DMA1) || (defined UART_DMA2)
uint8_t rx_buffer[BUFFER_SIZE];    // receive buffer
__IO uint8_t rx_cot = 0;    // receive data counter
uint8_t rx_flag = 0; // RX Complete flag: 0 means RX Not Complete; 1 means RX Complete flag
#endif

#ifdef GPIO_IRQ
volatile uint8_t led_speed = 0;
#endif

#ifdef PULSE_COT
uint8_t Result = 0;
#endif

#ifdef E_CLOCK
CLOCK_TypeDef clock = {0};
#endif

#ifdef B_LIGHTING
int16_t duty = 0;
#endif

#ifdef CAPTURE_SIGNAL
uint32_t diff = 0;          // store the Diff value, F_sig = (TIM_CLK) / (diff * (Prescaler + 1))
uint8_t measure_flag = 0;   // flag of measurement: 0-not complete; 1-complete
uint8_t cap_idx = 0;        // index of capture: 0-not start; 1-capture once
uint32_t cap_val1 = 0;      // first value of capture
uint32_t cap_val2 = 0;      // second value of capture
#endif

#ifdef FRAME_COMMUNICATION
extern UART_HandleTypeDef huart1;
uint8_t rx_buffer[4];   // rx buffer
uint8_t rx_flag = 0;    // Receive Complete flag: 0-not complete; 1-complete
uint8_t err_flag = 0;   // Instruction flag: 0-correct; 1-error
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_DMA_Init();
  MX_TIM6_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
#ifdef TIMER
  // TIMER Programming
  // clear the UPDATE interrupt flag during the initialization progress of timer,
  // avoiding entering the interrupt as soon as the timer starts
  __HAL_TIM_CLEAR_IT(&htim6, TIM_IT_UPDATE);
  // enable timer UPDATE interrupt (it will be triggered when the counter overflows or 
  // underflows), and start the timer
  HAL_TIM_Base_Start_IT(&htim6);
#endif

#ifdef UART_RETARGET
  printf("UART Retraget:\r\n");
#endif

#ifdef UART_IRQ
  printf("******  UART communication using IT ******\r\n");
  printf("Please enter 10 characters:\r\n");
  HAL_UART_Receive_IT(&huart1, (uint8_t *) rx_buffer, LENGTH);
#endif

#if (defined UART_DMA1) || (defined UART_DMA2)
  printf("****** UART communication using IDLE IT + DMA ******\r\n");
  printf("Please enter arbitrary length characters:\r\n");
#endif

#ifdef GPIO_REGISTER
  // GPIO Register Programming
  RCC->AHB2ENR |= (uint32_t) 0x00000004;    // open the port GPIOA's clock
  GPIOA->MODER = (uint32_t) 0xFFFFF3FF;     // set bit9:bit10 = 01, PA5 mode is Output Push Pull
  GPIOA->ODR |= 1 << 5;                     // set bit5 being 1, PA5 output hight level, open LD2
  GPIOA->ODR &= ~(1 << 5);                  // set bit5 being 0, PA5 output low level, close LD2
  GPIOA->BRR |= (uint32_t) 1 << 5;          // set bit5 being 1, PA5 output low level, open LD2
  GPIOA->BSRR |= (uint32_t) 1 << 5;         // set bit5 being 0, PA5 output high level, close LD2
#endif

#ifdef GPIO_IRQ
  // GPIO IRQ Programming
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
#endif

#ifdef PULSE_COT
  HAL_TIM_Base_Start(&htim4);
  printf("Timer Counter Function Test:\n");
#endif

#ifdef E_CLOCK
  __HAL_TIM_CLEAR_IT(&htim6, TIM_IT_UPDATE);
  HAL_TIM_Base_Start_IT(&htim6);
#endif

#ifdef B_LIGHTING
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  uint16_t max = (uint16_t) htim2.Init.Period + 1;    // 200
  uint16_t step = max / 5;    // 40
#endif

#ifdef PWM
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
#endif

#ifdef CAPTURE_SIGNAL
  printf("Timer Capture Function Test:\n");
  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);   // start TIM5 CH1 to capture input signal
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);     // start TIM3 CH1 to output PWM signal, 10^5 Hz square waveform
#endif

#ifdef FRAME_COMMUNICATION
  printf("****** Communication Frame ******\r\n");
  printf("Please enter instruction:\r\n");
  printf("Head->0xaa Device->0x01 Operation->0x00/0x01 Tail->0x55.\r\n");
  HAL_UART_Receive_IT(&huart1, rx_buffer, 4);
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // printf("Lighting\r\n");
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#ifdef UART
    if (HAL_UART_Receive(&huart1, rec_buffer, 5, 100) == HAL_OK) {
      HAL_UART_Transmit(&huart1, rec_buffer, 5, 100);
    }
#endif

#ifdef UART_IRQ
    if (rx_flag == 1) {
      rx_flag = 0;    // clear RX flag
      printf("Receive Success!\r\n");
      // transfer the data
      HAL_UART_Transmit_IT(&huart1, rx_buffer, LENGTH);
    }
#endif

#ifdef UART_RETARGET
    // use scanf function to receive UART data, need a space to be the end of input, 'y '
    if (scanf("%c", &rec_data) == 1) {
      if (rec_data == 'y') {
        printf("Receive y!\r\n");
      } else {
        printf("Receive others!\r\n");
      }
    }
#endif

#if (defined UART_DMA1) || (defined UART_DMA2) 
    if (rx_flag == 1) {
      rx_flag = 0; // clear RX flag
      // Test function: print the received data
      HAL_UART_Transmit(&huart1, rx_buffer, rx_cot, HAL_MAX_DELAY);
      // clear buffer
      // memset(rx_buffer, 0, BUFFER_SIZE);
      rx_cot = 0;
  #ifdef UART_DMA1
      // restart the DMA transfer, and 255 words each time
      HAL_UART_Receive_DMA(&huart1, rx_buffer, BUFFER_SIZE);
  #endif
    }
#endif

#ifdef GPIO_IRQ
    if (led_speed == 0) {
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      HAL_Delay(1000);
    } else if (led_speed == 1) {
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      HAL_Delay(2000);
    } else if (led_speed == 2) {
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      HAL_Delay(5000);
    }
#endif

#ifdef E_CLOCK
    printf("Time: %02d:%02d:%02d.\r\n", clock.hour, clock.minute, clock.second);
    HAL_Delay(1000);
#endif

#ifdef B_LIGHTING
    for (duty = 0; duty <= max; duty += step) {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty);
      HAL_Delay(100);
    }
    for (duty = max; duty >= 0; duty -= step) {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty);
      HAL_Delay(100);
    }
#endif

#ifdef CAPTURE_SIGNAL
    if (measure_flag == 1) {                      // judge whether the measurement complete
      if (cap_val2 >= cap_val1) {                 // the two captures are in the same period
        diff = cap_val2 - cap_val1;
      } else {                                    // the two captures are not in the same period
        diff = ((0xFFFFFFFF + 1 - cap_val1) + cap_val2);
      }
    }
    // printf("Period is: %.4f ms\r\n", diff / 20000);    // calculate the period
    // printf("Frequency is: %d Hz\r\n", 20000000 / diff);   // calculate the frequency
    // printf("cap_val1: %d, cal_val2: %d\r\n", cap_val1, cap_val2);
    // printf("/********************************/\r\n");

    measure_flag = 0;                             // clear the flag
    HAL_Delay(1000);
    HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);   // start TIM5 CH1 to capture input signal
#endif

#ifdef FRAME_COMMUNICATION
    if (rx_flag == 1) {   // whether data receive complete
      rx_flag = 0;    // clear rx_flag
      // frame format analysis
      if (rx_buffer[0] == 0xaa && rx_buffer[3] == 0x55) {   // frame head and frame tail
        if (rx_buffer[1] == 0x01) {   // device code
          if (rx_buffer[2] == 0x00) {   // operation code
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
            printf("LD2 is close!\r\n");
          } else if (rx_buffer[2] == 0x01) {
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
            printf("LD2 is open!\r\n");
          } else {
            printf("Operation Error: 0x%02x.\r\n", rx_buffer[2]);
            err_flag = 1;
          }
        } else {
          printf("Device Error: 0x%02x.\r\n", rx_buffer[1]);
          err_flag = 1;
        }
      } else {
        printf("Frame Error: 0x%02x 0x%02x.\r\n", rx_buffer[0], rx_buffer[3]);
        err_flag = 1;
      }
    }
    
    if (err_flag == 1) {
      printf("Communication Error! Please send again!\r\n");
    }
    // clear buffer and error flag
    err_flag = 0;
    memset(rx_buffer, 0, 4);
#endif
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  switch (GPIO_Pin) {
#ifdef GPIO_IRQ
    case B1_Pin:
      // if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == 1)
      //   HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
      // else
      //   HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
      // break;
      // HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      led_speed++;
      if (led_speed == 3) led_speed = 0;
#endif

#ifdef PULSE_COT
    case B1_Pin:
      HAL_GPIO_WritePin(PULSE_GPIO_Port, PULSE_Pin, GPIO_PIN_SET);
      HAL_Delay(1);
      HAL_GPIO_WritePin(PULSE_GPIO_Port, PULSE_Pin, GPIO_PIN_RESET);
      HAL_Delay(1);
      Result = __HAL_TIM_GET_COUNTER(&htim4);
      printf("Count = %d.\r\n", Result);
#endif
    default:
      break;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
#ifdef TIMER
  if (htim->Instance == TIM6) {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  }
#endif

#ifdef E_CLOCK
  if (htim->Instance == TIM6) {
    clock.second++;
    if (clock.second == 60) {
      clock.second = 0;
      clock.minute++;
      if (clock.minute == 60) {
        clock.minute = 0;
        clock.hour++;
        if (clock.hour == 24) {
          clock.hour = 0;
        }
      }
    }
  }
#endif
}


void HAL_TIM_IC_CaptureSignalCallback(TIM_HandleTypeDef *htim) {
#ifdef CAPTURE_SIGNAL
  if (htim->Instance == TIM5) {   // judge the timer capture signal
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {    // judge the channel
      if (cap_idx == 0) {   // store the first CCR
        cap_val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        printf("capture 1.\r\n");
        cap_idx = 1;    // change the flag
      } else if (cap_idx == 1) {    // store the second CCR
        cap_val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        printf("capture 2.\r\n");
        HAL_TIM_IC_Stop_IT(htim, TIM_CHANNEL_1);    // pause capture
        cap_idx = 0;
        measure_flag = 1;
      } else {
        Error_Handler();
      }
    }
  }
#endif
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
#ifdef UART_IRQ
  if (huart->Instance == USART1) {
    rx_flag = 1;    // set RX Complete flag
    HAL_UART_Receive_IT(&huart1, rx_buffer, LENGTH);    // enable RX IRQ
  }
#endif

#ifdef FRAME_COMMUNICATION
  if (huart->Instance == USART1) {
    rx_flag = 1;    // set RX Complete flag
    HAL_UART_Receive_IT(&huart1, rx_buffer, 4);    // enable RX IRQ
  }
#endif

#ifdef UART_DMA2
  if (huart->Instance == USART1) {
    printf("RxCplt IRQ.\r\n");
    __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
    HAL_UART_Receive_DMA(&huart1, &rx_buffer[1], (BUFFER_SIZE - 1));
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
  }
#endif
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
