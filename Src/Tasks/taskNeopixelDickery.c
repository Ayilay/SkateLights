#include "tasks.h"
#include "stm32f1xx_hal.h"
#include "gpio.h"

void TaskNeopixelDickery(void const * argument) {
	int count = 0;
	int r, g, b;
	int alt1 = 0;
	int flag = 0;
	NeopixelColor_t* pixArrPtr = ((NeopixelArray_t *) neopixelBufferPoolHandle)->array;

	r = 0x08;
	g = 0x00;
	b = 0x00;

	osStatus status;

	while (1) {

		// Populate the prebuffer
		status = osMutexWait(pixelPreBufferMutexHandle, 20);

		// We timed out waiting for the mutex handle, abort this iteration
		// and try again
		if (status != osOK)
			continue;

		for (int i = NUM_NEOPIXELS; i > 0; i--) {

			// Shift the entire pixel array to the right
			pixArrPtr[i].R = pixArrPtr[i-1].R;
			pixArrPtr[i].G = pixArrPtr[i-1].G;
			pixArrPtr[i].B = pixArrPtr[i-1].B;

			// Every now and then add some fancy
			if (count % (NUM_NEOPIXELS / 2) == 0) {
				flag = 1;
				if (alt1) {
					pixArrPtr[1].R = 0x00;
					pixArrPtr[1].G = 0x08;
					pixArrPtr[1].B = 0x08;

					pixArrPtr[2].R = 0x00;
					pixArrPtr[2].G = 0x88;
					pixArrPtr[2].B = 0x88;

				}
				else {
					pixArrPtr[1].R = 0x00;
					pixArrPtr[1].G = 0x08;
					pixArrPtr[1].B = 0x00;

					pixArrPtr[2].R = 0x00;
					pixArrPtr[2].G = 0x48;
					pixArrPtr[2].B = 0x00;
				}

				//pixArrPtr[2].R = 0x00;
				//pixArrPtr[2].G = 0xCC;
				//pixArrPtr[2].B = 0xCC;
			}

		}

		// Add the first pixel
		pixArrPtr[0].R = r;
		pixArrPtr[0].G = g;
		pixArrPtr[0].B = b;

		if (flag) {
			alt1 = 1-alt1;
			flag = 0;
		}

		if (++count >= NUM_NEOPIXELS-2)
			count = 0;

		// Done creating the pre-buffer, release the mutex
		osMutexRelease(pixelPreBufferMutexHandle);

		osSemaphoreRelease(neopixelDriverEnableHandle);

		osDelay(1000/NUM_NEOPIXELS / 1);
	}
}
