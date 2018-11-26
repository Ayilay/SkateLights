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

  	// Trigger the neopixel driver to write the pixels
  	osThreadResume(neopixelDriverHandle);

    osDelayUntil(&prevWakeTime, 70);
  }
  /* USER CODE END TaskSegmentCycler */
}


/**
 * When this timer runs out of time, it means the skateboard is probably stopped
 * Turn the screen off and go into low power mode
 */
void dispResetTimerCallback(void const * argument) {
	osThreadSuspend(segmentCyclerHandle);
	HAL_GPIO_WritePin(GPIOB, 0xFFFF & ~LED_ERR_Pin, GPIO_PIN_RESET);
}
