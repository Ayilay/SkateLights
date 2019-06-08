#include "applicationTasks.h"
#include "cmsis_os.h"
#include "tim.h"
#include "printf.h"
#include "stm32f1xx_hal.h"

#define SPEED_TIMER		htim2
#define WHEEL_DIAM 		10.8
#define TIME_MULT			1000.0F
#define DIST_PER_REV	(3.141592F * WHEEL_DIAM)
#define CM_S_TO_MPH		44.704

int globalSpeed = 0;


uint16_t digitsOnes[10] = {
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

uint16_t digitsTens[10] = {
	DIGIT2_0,
	DIGIT2_1,
	DIGIT2_2,
	DIGIT2_3,
	DIGIT2_4,
	DIGIT2_5,
	DIGIT2_6,
	DIGIT2_7,
	DIGIT2_8,
	DIGIT2_9,
};

void TaskSpeedCalculator(void const * argument) {
	uint16_t deltaT = 0;
	uint8_t* buf;
	float speed;

	// Enable the wheel speed counter
	HAL_TIM_Base_Start(&SPEED_TIMER);

	for (;;) {
		// Wait forever until a new speed is posted on the queue
		deltaT = (uint16_t) osMessageGet(wheelSpeedQueueHandle, osWaitForever).value.v;

		// Print the speed to the user for debug purposes
		//buf = osPoolAlloc(uartStrMemPoolHandle);
		if (deltaT == 0) {
			sprintf((char*) buf, "TIM CNT == 0 FUCK\r\n");
		}
		else {
			// SUPERMULT = 75.897444
			#define SUPERMULT (TIME_MULT * DIST_PER_REV / CM_S_TO_MPH)
			speed = SUPERMULT / deltaT * 20;
			globalSpeed = speed;

			if (speed > 20) {
				sprintf((char*) buf, "Way too fast, deltaT of %d\r\n", deltaT);
			}
			else {
				sprintf((char*) buf, "Speed: %0.2f mph/s\tDeltaT: %d\r\n", speed, deltaT);
			}
		}
		//osMessagePut(UartSendQueueHandle, (uint32_t) buf, 50);
	}
}


void TaskDisplaySpeed(void const * argument) {

	while(1) {
		osDelay(200);
		uint16_t dig1 = digitsOnes[(int)  globalSpeed % 10];
		uint16_t dig2 = digitsTens[(int) (globalSpeed / 10)];
		uint16_t speedPins = dig1 | dig2;
		HAL_GPIO_WritePin(GPIOB, 0xFFFF, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, speedPins, GPIO_PIN_SET);
	}
}

/**
 * Interrupt Callback for the spedometer. Every time the wheel makes
 * a full revolution, the time between revolutions is recorded
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

  portBASE_TYPE taskWoken = pdFALSE;

	// Record the current change in time since the last revolution
	volatile uint32_t deltaT = SPEED_TIMER.Instance->CNT;

	// Reset the counter
	__HAL_TIM_DISABLE(&SPEED_TIMER);
	__HAL_TIM_SET_COUNTER(&SPEED_TIMER, 0);
	__HAL_TIM_ENABLE(&SPEED_TIMER);

	// Send the deltaT to the global speed queue to be parsed
	if (deltaT != 0) {
    xQueueSendFromISR(wheelSpeedQueueHandle, (const void* const) deltaT, &taskWoken);
	}

	portEND_SWITCHING_ISR(taskWoken);
}
