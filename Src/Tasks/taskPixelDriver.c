#include "stm32f1xx_hal.h"
#include "tasks.h"
#include "spi.h"

inline uint8_t convBitToSerial(uint8_t bit);

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
  	// Suspend this thread asap. The neopixel driving happens
  	osThreadSuspend(neopixelDriverHandle);

  	// Lock access to the pre-buffer so that we can construct the buffer
  	osMutexWait(pixelPreBufferMutexHandle, osWaitForever);

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

  	// Done constructing the buffer, release the prebuffer mutex
  	osMutexRelease(pixelPreBufferMutexHandle);

  	// Prepare to transmit the buffer over SPI (wait until all SPI transactions finish)
  	while (hspi1.State != HAL_SPI_STATE_READY) {
  		osThreadYield();
  	}

  	// The SPI peripheral is ready, use DMA to transmit the buffer
  	HAL_SPI_Transmit_DMA(&hspi1, neopixelBuffer, numData);
  }
  /* USER CODE END TaskNeopixelDriver */
}

inline uint8_t convBitToSerial(uint8_t bit) {
	return (bit ? 0b110 : 0b100);
}
