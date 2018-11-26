#include "tasks.h"
#include "cmsis_os.h"
#include "tim.h"
#include "stm32f1xx_hal.h"

#define SPEED_TIMER		htim2
#define WHEEL_RADIUS 	10.8
#define TIME_MULT			1000

void TaskSpeedCalculator(void const * argument) {
	uint16_t deltaT = 0;
	uint8_t* buf;

	// Enable the wheel speed counter
	HAL_TIM_Base_Start(&SPEED_TIMER);

	for (;;) {
		// Wait forever until a new speed is posted on the queue
		deltaT = (uint16_t) osMessageGet(wheelSpeedQueueHandle, osWaitForever).value.v;

		// Resume this thread if it is suspended
		osThreadResume(segmentCyclerHandle);
		osTimerStart(dispResetTimerHandle, 5000);

		// Ensure no divide-by-zero condition occurs
		if (deltaT == 0)
			deltaT = 1;

		//speed = (TIME_MULT / deltaT) * WHEEL_RADIUS;

		// Print the speed to the user for debug purposes
		buf = osPoolAlloc(uartStrMemPoolHandle);
		sprintf((char*) buf, "DeltaT: %d\r\n", deltaT);
		osMessagePut(UartSendQueueHandle, (uint32_t) buf, 50);
	}
}

/**
 * Interrupt Callback for the spedometer. Every time the wheel makes
 * a full revolution, the time between revolutions is recorded
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	// Record the current change in time since the last revolution
	uint16_t deltaT = SPEED_TIMER.Instance->CNT;

	// Reset the counter
	__HAL_TIM_DISABLE(&SPEED_TIMER);
	__HAL_TIM_SET_COUNTER(&SPEED_TIMER, 0);
	__HAL_TIM_ENABLE(&SPEED_TIMER);

	// Send the deltaT to the global speed queue to be parsed
	osMessagePut(wheelSpeedQueueHandle, deltaT, 0);
}
