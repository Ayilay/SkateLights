#include "tasks.h"
#include "stm32f1xx_hal.h"
#include "gpio.h"

// The GPIO Port where all segment pins are attatched
// All 14 segment pins must belong to the same GPIO Port
#define SEGMENTS_PORT	GPIOB

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

// Function Prototypes
void randomizeNeopixels();

#if(configUSE_TRACE_FACILITY == 1)
traceString chn;
#endif


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
	uint16_t digit = 0;
	TickType_t prevWakeTime;

#if(configUSE_TRACE_FACILITY == 1)
	chn = xTraceRegisterString("SegmentCyclerChannel");
#endif

	prevWakeTime = osKernelSysTick();

	// First thing this task does is go into low-power mode
	//dispResetTimerCallback(NULL);
  osThreadSuspend(segmentCyclerHandle);
  osDelay(2000);

  /* Infinite loop */
  for(;;)
  {
    for (int i = 0; i < 2; i++) {
      for (int dig = 1; dig <= 8; dig++) {
        digit = DIGIT_CODES[dig];

        // Turn all the Segment GPIO's off, then turn the relevant ones on
        HAL_GPIO_WritePin(SEGMENTS_PORT, (0xFFFF & SEG1), GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SEGMENTS_PORT, digit, GPIO_PIN_SET);

        osDelay(2100/8);

        // Create a random new string of neopixel colors
        //randomizeNeopixels();

        // Signal to the neopixel driver task that we want it to update the neopixel colors
        //osSemaphoreRelease(neopixelDriverEnableHandle);
        //osStatus status = osSemaphoreRelease(neopixelDriverEnableHandle);
      }

      osDelay(2500);
    }
    osThreadSuspend(segmentCyclerHandle);

#if(configUSE_TRACE_FACILITY == 1)
		if (status != osOK) {
			vTracePrint(chn, "Failed to release neopixelDriverEnable semaphore");
		}
		else {
			vTracePrint(chn, "Semaphore neopixelDriverEnable given up");
		}
#endif

  	// WARNING: When this task is suspended for long amounts of time and then resumed, then
  	// osDelayUntil will exit immediately until the prevWakeTime "catches up" (extreme runtimes of
  	// up to 500 ms have been observed)
    //osDelayUntil(&prevWakeTime, 70);
    osDelay(400);
  }
  /* USER CODE END TaskSegmentCycler */
}


/**
 * When this timer runs out of time, it means the skateboard is probably stopped
 * Turn the screen off and go into low power mode
 */
void dispResetTimerCallback(void const * argument) {
	//osThreadSuspend(segmentCyclerHandle);
	HAL_GPIO_WritePin(GPIOB, 0xFFFF, GPIO_PIN_RESET);

	char* str = osPoolAlloc(uartStrMemPoolHandle);
	sprintf(str, "Entering Sleep Mode\r\n");
	osMessagePut(UartSendQueueHandle, (uint32_t) str, osWaitForever);
}

/**
 * Writes a surprise combination to the neopixel prebuffer
 */
void randomizeNeopixels() {
	static int r = 0;
	int g, b;

	// Obtain the pixel pre-buffer mutex so that we can modify the prebuffer
	osStatus status = osMutexWait(pixelPreBufferMutexHandle, 20);

	if (status != osOK) {
#if(configUSE_TRACE_FACILITY == 1)
		vTracePrint(chn, "Failed to obtain preBufferMutex");
#endif
		return;
	}

	NeopixelColor_t* pixArrPtr = ((NeopixelArray_t *) neopixelBufferPoolHandle)->array;

	r += 20 % 255;
	g = (r + 20) % 255;
	b = (r + 40) % 255;

	for (int i = 0; i < NUM_NEOPIXELS; i++) {
		pixArrPtr[i].R = r;
		pixArrPtr[i].G = g;
		pixArrPtr[i].B = b;
	}

	// Done creating the pre-buffer, release the mutex
	osMutexRelease(pixelPreBufferMutexHandle);
}
