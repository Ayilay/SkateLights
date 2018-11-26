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
#include "spi.h"
#include "tim.h"
#include "gpio.h"
#include "string.h"

// Custom printf implementations for embedded applications from GitHub:
// https://github.com/mpaland/printf
#include "printf.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
		uint8_t G, R, B;
} NeopixelColor_t;

// TODO if NUM_NEOPIXELS*3 is not even, the NeopixelDriver breaks due to buffering
// Fix it
#define NUM_NEOPIXELS	30
typedef struct {
		NeopixelColor_t array[NUM_NEOPIXELS];
} NeopixelArray_t;

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

// Memory pool to store strings to be printed over serial
osPoolId uartStrMemPoolHandle;

// Memory pool to contain buffer for the neopixels
osPoolId neopixelBufferPoolHandle;

/* USER CODE END Variables */
osThreadId segmentCyclerHandle;
osThreadId uartSenderHandle;
osThreadId neopixelDriverHandle;
osMessageQId UartSendQueueHandle;
osSemaphoreId neopixelRWMutexHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

inline uint16_t segment1(uint16_t segment);
inline uint16_t segment2(uint16_t segment);
inline uint8_t convBitToSerial(uint8_t bit);

char* HALStatusToStr(HAL_StatusTypeDef status);
   
/* USER CODE END FunctionPrototypes */

void TaskSegmentCycler(void const * argument);
void TaskUartSender(void const * argument);
void TaskNeopixelDriver(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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

  /* USER CODE BEGIN RTOS_MUTEX */

  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of neopixelRWMutex */
  osSemaphoreDef(neopixelRWMutex);
  neopixelRWMutexHandle = osSemaphoreCreate(osSemaphore(neopixelRWMutex), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of segmentCycler */
  osThreadDef(segmentCycler, TaskSegmentCycler, osPriorityNormal, 0, 256);
  segmentCyclerHandle = osThreadCreate(osThread(segmentCycler), NULL);

  /* definition and creation of uartSender */
  osThreadDef(uartSender, TaskUartSender, osPriorityHigh, 0, 128);
  uartSenderHandle = osThreadCreate(osThread(uartSender), NULL);

  /* definition and creation of neopixelDriver */
  osThreadDef(neopixelDriver, TaskNeopixelDriver, osPriorityNormal, 0, 256);
  neopixelDriverHandle = osThreadCreate(osThread(neopixelDriver), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of UartSendQueue */
  osMessageQDef(UartSendQueue, 4, uint32_t);
  UartSendQueueHandle = osMessageCreate(osMessageQ(UartSendQueue), NULL);

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
	TickType_t prevWakeTime;

	//osStatus status;
	//char* buf = NULL;

	// TODO This is for testing purposes only, remove asap
	// Write to the neopixel buffer and trigger a write sequence
	((NeopixelColor_t*) neopixelBufferPoolHandle)[0].G = 0xDE;
	((NeopixelColor_t*) neopixelBufferPoolHandle)[0].R = 0xAD;
	((NeopixelColor_t*) neopixelBufferPoolHandle)[0].B = 0xBE;

	((NeopixelColor_t*) neopixelBufferPoolHandle)[NUM_NEOPIXELS-1].G = 0xDE;
	((NeopixelColor_t*) neopixelBufferPoolHandle)[NUM_NEOPIXELS-1].R = 0xAD;
	((NeopixelColor_t*) neopixelBufferPoolHandle)[NUM_NEOPIXELS-1].B = 0xBE;

	prevWakeTime = osKernelSysTick();

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

  	// Allocate a chunk of memory on the uartStrings pool, place our string there,
  	// and add the pointer to our string to the uartSendQueue to be printed by the
  	// uartSender task
  	//buf = osPoolAlloc(uartStrMemPoolHandle);
  	//sprintf(buf, "Index: %d\r\n", index);
  	//status = osMessagePut(UartSendQueueHandle, (uint32_t) buf, 10);

  	//if (status != osOK) {
		//	GPIO_SetStatusLED_ERR();
		//	osThreadSuspendAll();
  	//}

  	// Trigger the neopixel driver to write the pixels
  	osThreadResume(neopixelDriverHandle);

    osDelayUntil(&prevWakeTime, 70);
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

	HAL_StatusTypeDef status;
	char* strToPrint;

  /* Infinite loop */
  for(;;)
  {
  	// Wait indefinitely for someone to put a message on the UartSendQueue
  	// As soon as something is put on the queue, wake up and print the string over UART
  	strToPrint = osMessageGet(UartSendQueueHandle, osWaitForever).value.p;
		//status = HAL_UART_Transmit(&huart1, strToPrint, strlen(strToPrint), 10);
		status = HAL_UART_Transmit_DMA(&huart1, (uint8_t*) strToPrint, strlen(strToPrint));

		// Indicate if something went wrong (primitive error checking)
		if (status != HAL_OK) {
			GPIO_SetStatusLED_ERR();
		}
		else {
			GPIO_SetStatusLED_OK();
		}

		// Wait until we're done sending, and keep yielding to lower-priority tasks while waiting
		while (huart1.gState != HAL_UART_STATE_READY) {
			osThreadYield();
		}

		// The string to print was allocated on a memory pool, so free it
		osPoolFree(uartStrMemPoolHandle, strToPrint);

  }
  /* USER CODE END TaskUartSender */
}

/* USER CODE BEGIN Header_TaskNeopixelDriver */
/**
* Inspiration for using SPI/DMA came from
* https://www.rogerclark.net/arduino-stm32-neopixels-ws2812b-using-spi-dma/
*
* Even though bit-banging the neopixel driver pin might give more accurate output,
* this approach is more interruptible because the buffering is low-priority and the
* DMA-driven SPI output is essentially non-interruptible and independent of
* the CPU, making this approach better for a multithreaded environment.
*
* @brief Function implementing the neopixelDriver thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TaskNeopixelDriver */
void TaskNeopixelDriver(void const * argument)
{
  /* USER CODE BEGIN TaskNeopixelDriver */

	// The buffer to stream to the neopixels over SPI
	// Include space for leading and trailing zeros
	#define numData (sizeof(NeopixelArray_t)*3) + 2
	uint8_t neopixelBuffer[numData];

	uint32_t i = 0, j = 0;
	uint8_t temp = 0;
	uint8_t c = 0;
	uint8_t* p;
	NeopixelColor_t* pixArrPtr = ((NeopixelArray_t *) neopixelBufferPoolHandle)->array;

  /* Infinite loop */
  for(;;)
  {
  	osThreadSuspend(neopixelDriverHandle);

  	// Initialize the pointer to the buffer
		p = neopixelBuffer;

		// The first element is a leading zero, set it and increment the buffer pointer
		*p++ = 0;

  	// Construct the neopixel buffer from the neopixel data
  	for (i = 0; i < NUM_NEOPIXELS; i++) {

  		// Iterate through each color (ugly pointer magic because we can't use
  		// direct array indexing)
  		for (j = 0; j < 3; j++) {
				c = *(((uint8_t *) &pixArrPtr[i])+j);

				temp  = convBitToSerial(c & 0x80) << 5;
				temp |= convBitToSerial(c & 0x40) << 2;
				temp |= convBitToSerial(c & 0x20) >> 1;
				*p++ = temp;

				temp  = convBitToSerial(c & 0x20) << 7;
				temp |= convBitToSerial(c & 0x10) << 4;
				temp |= convBitToSerial(c & 0x08) << 1;
				temp |= convBitToSerial(c & 0x04) >> 2;
				*p++ = temp;

				temp  = convBitToSerial(c & 0x04) << 6;
				temp |= convBitToSerial(c & 0x02) << 3;
				temp |= convBitToSerial(c & 0x01);
				*p++ = temp;
  		}
  	}

  	// Add the trailing zero to the buffer
  	*p = 0;

  	// Prepare to transmit the buffer over SPI (wait until all SPI transactions finish)
  	while (hspi1.State != HAL_SPI_STATE_READY) {
  		osThreadYield();
  	}

  	// The SPI peripheral is ready, use DMA to transmit the buffer
  	HAL_SPI_Transmit_DMA(&hspi1, neopixelBuffer, numData);
  }
  /* USER CODE END TaskNeopixelDriver */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

// Useful for debug purposes
char* HALStatusToStr(HAL_StatusTypeDef status) {
  switch (status) {
    case HAL_OK:
      return "HAL_OK";
      break;
    case HAL_TIMEOUT:
      return "HAL_TIMEOUT";
      break;
    case HAL_ERROR:
      return "HAL_ERROR";
      break;
    case HAL_BUSY:
      return "HAL_BUSY";
      break;
    default:
      return "BAD_STAT";
  }
}

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

inline uint8_t convBitToSerial(uint8_t bit) {
	return (bit ? 0b110 : 0b100);
}
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
