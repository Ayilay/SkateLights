#ifndef _TASKS_H
#define _TASKS_H

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

typedef struct {
		uint8_t G, R, B;
} NeopixelColor_t;

// TODO if NUM_NEOPIXELS*3 is not even, the NeopixelDriver breaks due to buffering
// Fix it
#define NUM_NEOPIXELS	30
typedef struct {
		NeopixelColor_t array[NUM_NEOPIXELS];
} NeopixelArray_t;

// These must be kept up to date with CubeMX generated FreeRTOS Constructs
extern osThreadId segmentCyclerHandle;
extern osThreadId uartSenderHandle;
extern osThreadId neopixelDriverHandle;
extern osMessageQId UartSendQueueHandle;
extern osMessageQId wheelSpeedQueueHandle;
extern osSemaphoreId neopixelDriverEnableHandle;
extern osTimerId dispResetTimerHandle;
extern osMutexId pixelPreBufferMutexHandle;

// Memory pool to store strings to be printed over serial
extern osPoolId uartStrMemPoolHandle;

// Memory pool to contain buffer for the neopixels
extern osPoolId neopixelBufferPoolHandle;

void TaskSegmentCycler(void const * argument);
void TaskUartSender(void const * argument);
void TaskNeopixelDriver(void const * argument);

void dispResetTimerCallback(void const * argument);

#endif
