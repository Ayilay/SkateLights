#include "tasks.h"
#include "cmsis_os.h"
#include "tim.h"
#include "printf.h"
#include "stm32f1xx_hal.h"

#define SPEED_TIMER		htim2
#define WHEEL_DIAM 		10.8
#define TIME_MULT			1000.0F
#define DIST_PER_REV	(3.141592F * WHEEL_DIAM)
#define CM_S_TO_MPH		44.704

#if(configUSE_TRACE_FACILITY == 1)
	traceHandle SpeedInterruptHandle;
#endif

void TaskSpeedCalculator(void const * argument) {
	uint16_t deltaT = 0;
	uint8_t* buf;
	float speed;

#if(configUSE_TRACE_FACILITY == 1)
	SpeedInterruptHandle = xTraceSetISRProperties("SpeedInterruptHandle", 3);
#endif

	// Enable the wheel speed counter
	HAL_TIM_Base_Start(&SPEED_TIMER);

	for (;;) {
		// Wait forever until a new speed is posted on the queue
		deltaT = (uint16_t) osMessageGet(wheelSpeedQueueHandle, osWaitForever).value.v;

		// Resume this thread if it is suspended
		osThreadResume(segmentCyclerHandle);
		osTimerStart(dispResetTimerHandle, 15000);

		// Print the speed to the user for debug purposes
		buf = osPoolAlloc(uartStrMemPoolHandle);
		if (deltaT == 0) {
			sprintf((char*) buf, "TIM CNT == 0 FUCK\r\n");
		}
		else {
			// SUPERMULT = 75.897444
			#define SUPERMULT (TIME_MULT * DIST_PER_REV / CM_S_TO_MPH)
			//speed = (TIME_MULT / deltaT) * DIST_PER_REV / CM_S_TO_MPH;
			speed = SUPERMULT / deltaT;

			if (speed > 20) {
				sprintf((char*) buf, "Way too fast, deltaT of %d\r\n", deltaT);
			}
			else {
				sprintf((char*) buf, "Speed: %0.2f mph/s\tDeltaT: %d\r\n", speed, deltaT);
			}
		}
		osMessagePut(UartSendQueueHandle, (uint32_t) buf, 50);
	}
}

/**
 * Interrupt Callback for the spedometer. Every time the wheel makes
 * a full revolution, the time between revolutions is recorded
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

#if(configUSE_TRACE_FACILITY == 1)
	vTraceStoreISRBegin(SpeedInterruptHandle);
#endif

  portBASE_TYPE taskWoken = pdFALSE;

	// Record the current change in time since the last revolution
	volatile uint16_t deltaT = SPEED_TIMER.Instance->CNT;

	// Reset the counter
	__HAL_TIM_DISABLE(&SPEED_TIMER);
	__HAL_TIM_SET_COUNTER(&SPEED_TIMER, 0);
	__HAL_TIM_ENABLE(&SPEED_TIMER);

	// Send the deltaT to the global speed queue to be parsed
	if (deltaT != 0) {
		//osMessagePut(wheelSpeedQueueHandle, deltaT, 0);
    xQueueSendFromISR(wheelSpeedQueueHandle, deltaT, &taskWoken);
	}

#if(configUSE_TRACE_FACILITY == 1)
	vTraceStoreISREnd(0);
#endif
	portEND_SWITCHING_ISR(taskWoken);
}
