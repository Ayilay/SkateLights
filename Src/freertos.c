/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "stm32f1xx_hal.h"
#include "usart.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// The GPIO Port where all segment pins are attatched
// All 14 segment pins must belong to the same GPIO Port
#define SEGMENTS_PORT	GPIOB

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

// Segment GPIOs
#define SEG1A	(1 << 3)
#define SEG1B	(1 << 4)
#define SEG1C	(1 << 5)
#define SEG1D	(1 << 6)
#define SEG1E	(1 << 7)
#define SEG1F	(1 << 8)
#define SEG1G	(1 << 9)

// Character Definitions, constructed from Segments
#define DIGIT1_0		( SEG1A | SEG1B | SEG1C | SEG1D | SEG1E | SEG1F )
#define DIGIT1_1		( SEG1B | SEG1C )
#define DIGIT1_2		( SEG1A | SEG1B | SEG1D | SEG1E | SEG1G )
#define DIGIT1_3		( SEG1A | SEG1B | SEG1C | SEG1D | SEG1G )
#define DIGIT1_4		( SEG1B | SEG1C | SEG1F | SEG1G )
#define DIGIT1_5		( SEG1A | SEG1C | SEG1D | SEG1F | SEG1G )
#define DIGIT1_6		( SEG1A | SEG1C | SEG1D | SEG1E | SEG1F | SEG1G )
#define DIGIT1_7		( SEG1A | SEG1B | SEG1C )
#define DIGIT1_8		( SEG1A | SEG1B | SEG1C | SEG1D | SEG1E | SEG1F | SEG1G )
#define DIGIT1_9		( SEG1A | SEG1B | SEG1C | SEG1D | SEG1F | SEG1G )

// All the GPIO Pins of Segment 1
#define SEG1	DIGIT1_8

#define NUM_SEGMENTS	7
uint16_t SEGMENT_CODES[NUM_SEGMENTS] = {
		SEG1A,
		SEG1B,
		SEG1C,
		SEG1D,
		SEG1E,
		SEG1F,
		SEG1G,
};

#define NUM_DIGITS 10
uint16_t DIGIT_CODES[NUM_DIGITS] = {
		DIGIT1_0,
		DIGIT1_1,
		DIGIT1_2,
		DIGIT1_3,
		DIGIT1_4,
		DIGIT1_5,
		DIGIT1_6,
		DIGIT1_7,
		DIGIT1_8,
		DIGIT1_9,
};

/* USER CODE END Variables */
osThreadId segmentCyclerHandle;
osThreadId uartSenderHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

inline uint16_t segment1(uint16_t segment);
inline uint16_t segment2(uint16_t segment);
   
/* USER CODE END FunctionPrototypes */

void TaskSegmentCycler(void const * argument);
void TaskUartSender(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of segmentCycler */
  osThreadDef(segmentCycler, TaskSegmentCycler, osPriorityNormal, 0, 64);
  segmentCyclerHandle = osThreadCreate(osThread(segmentCycler), NULL);

  /* definition and creation of uartSender */
  osThreadDef(uartSender, TaskUartSender, osPriorityIdle, 0, 64);
  uartSenderHandle = osThreadCreate(osThread(uartSender), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_TaskSegmentCycler */
/**
  * @brief  Function implementing the segmentCycler thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_TaskSegmentCycler */
void TaskSegmentCycler(void const * argument)
{

  /* USER CODE BEGIN TaskSegmentCycler */

	uint8_t index = 0;
	uint8_t increasing = 1;
	uint16_t segment = 0;

	char buf[32] = {0};

  /* Infinite loop */
  for(;;)
  {
  	segment = SEGMENT_CODES[index];
  	if (increasing) {
  		index ++;

			if (index >= NUM_SEGMENTS) {
				index = NUM_SEGMENTS-2;
				increasing = 0;
			}
  	}
  	else {
  		index --;
			if (index <= 0) {
				index = 0;
				increasing = 1;
			}
  	}

  	// Turn all the Segment GPIO's off, then turn the relevant ones on
  	HAL_GPIO_WritePin(SEGMENTS_PORT, (0xFFFF & SEG1), GPIO_PIN_RESET);
  	HAL_GPIO_WritePin(SEGMENTS_PORT, segment, GPIO_PIN_SET);

  	// TODO Invoke separate handler task to do this
  	sprintf(buf, "Index: %d\r\n", index);
  	HAL_UART_Transmit(&huart1, buf, strlen(buf), 100);

    osDelay(100 / portTICK_PERIOD_MS);
  }
  /* USER CODE END TaskSegmentCycler */
}

/* USER CODE BEGIN Header_TaskUartSender */
/**
* @brief Function implementing the uartSender thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TaskUartSender */
void TaskUartSender(void const * argument)
{
  /* USER CODE BEGIN TaskUartSender */

	// First thing to do is suspend this task. We don't really need to do anything
	// unless manually woken up.
	vTaskSuspend(uartSenderHandle);

  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END TaskUartSender */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

// TODO FIXME
inline uint16_t segment1(uint16_t segment) {
	return segment;
	//return 0x007F & segment;
}

// TODO FIXME
inline uint16_t segment2(uint16_t segment) {
	return segment;
	//return 0x3F80 & (segment << 7);
}
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
