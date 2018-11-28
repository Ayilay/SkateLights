#include "cmsis_os.h"
#include "stm32f1xx_hal.h"
#include "tasks.h"
#include "usart.h"
#include "gpio.h"
#include "string.h"

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
  	// TODO Use a Mail queue instead of a message queue

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
