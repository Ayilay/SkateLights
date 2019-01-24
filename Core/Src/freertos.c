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
#include "applicationTasks.h"

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
/* USER CODE BEGIN Variables */

// Memory pool to store strings to be printed over serial
osPoolId uartStrMemPoolHandle;

// Memory pool to contain buffer for the neopixels
osPoolId neopixelBufferPoolHandle;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId uartSenderHandle;
osThreadId neopixelDriverHandle;
osThreadId speedCalculatorHandle;
osMessageQId UartSendQueueHandle;
osMessageQId wheelSpeedQueueHandle;
osMutexId pixelPreBufferMutexHandle;
osSemaphoreId neopixelDriverEnableHandle;
osSemaphoreId uartRxBufReadyHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void defaultTaskUnused(void const * argument);
extern void TaskUartSender(void const * argument);
extern void TaskNeopixelDriver(void const * argument);
extern void TaskSpeedCalculator(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  osPoolDef(uartStrMemPool, 5, uint8_t[64]);
  uartStrMemPoolHandle = osPoolCreate(osPool(uartStrMemPool));

  // Initialize the memory pool for the neopixel buffer, and immediately allocate memory for it
  osPoolDef(neopixelBufferPool, 1, NeopixelArray_t);
  neopixelBufferPoolHandle = osPoolCreate(osPool(neopixelBufferPool));
  osPoolCAlloc(neopixelBufferPoolHandle);

  /* USER CODE END Init */

  /* Create the mutex(es) */
  /* definition and creation of pixelPreBufferMutex */
  osMutexDef(pixelPreBufferMutex);
  pixelPreBufferMutexHandle = osMutexCreate(osMutex(pixelPreBufferMutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of neopixelDriverEnable */
  osSemaphoreDef(neopixelDriverEnable);
  neopixelDriverEnableHandle = osSemaphoreCreate(osSemaphore(neopixelDriverEnable), 1);

  /* definition and creation of uartRxBufReady */
  osSemaphoreDef(uartRxBufReady);
  uartRxBufReadyHandle = osSemaphoreCreate(osSemaphore(uartRxBufReady), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, defaultTaskUnused, osPriorityNormal, 0, 64);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of uartSender */
  osThreadDef(uartSender, TaskUartSender, osPriorityHigh, 0, 128);
  uartSenderHandle = osThreadCreate(osThread(uartSender), NULL);

  /* definition and creation of neopixelDriver */
  osThreadDef(neopixelDriver, TaskNeopixelDriver, osPriorityNormal, 0, 256);
  neopixelDriverHandle = osThreadCreate(osThread(neopixelDriver), NULL);

  /* definition and creation of speedCalculator */
  osThreadDef(speedCalculator, TaskSpeedCalculator, osPriorityNormal, 0, 128);
  speedCalculatorHandle = osThreadCreate(osThread(speedCalculator), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of UartSendQueue */
  osMessageQDef(UartSendQueue, 4, uint32_t);
  UartSendQueueHandle = osMessageCreate(osMessageQ(UartSendQueue), NULL);

  /* definition and creation of wheelSpeedQueue */
  osMessageQDef(wheelSpeedQueue, 1, uint16_t);
  wheelSpeedQueueHandle = osMessageCreate(osMessageQ(wheelSpeedQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_defaultTaskUnused */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */

// lmao global variable
int count = 0;
/* USER CODE END Header_defaultTaskUnused */
void defaultTaskUnused(void const * argument)
{

  /* USER CODE BEGIN defaultTaskUnused */
  /* Infinite loop */
  for(;;)
  {
    ITM_SendCharOnChannel(count++, 1);
    osDelay(100);
  }
  /* USER CODE END defaultTaskUnused */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
