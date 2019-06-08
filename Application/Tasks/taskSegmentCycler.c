#include "applicationTasks.h"
#include "stm32f1xx_hal.h"
#include "gpio.h"

// The GPIO Port where all segment pins are attatched
// All 14 segment pins must belong to the same GPIO Port
#define SEGMENTS_PORT	GPIOB

#define NUM_DIGITS 10

// Segment GPIOs
#define SEG1A	GPIO_PIN_0    /*  Digit 1 (Ones) */
#define SEG1B	GPIO_PIN_1
#define SEG1C	GPIO_PIN_9
#define SEG1D	GPIO_PIN_8
#define SEG1E	GPIO_PIN_7
#define SEG1F	GPIO_PIN_6
#define SEG1G	GPIO_PIN_5

#define SEG2A GPIO_PIN_4    /*  Digit 2 (Tens) */
#define SEG2B GPIO_PIN_10
#define SEG2C GPIO_PIN_11
#define SEG2D GPIO_PIN_15
#define SEG2E GPIO_PIN_14
#define SEG2F GPIO_PIN_13
#define SEG2G GPIO_PIN_12

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

#define DIGIT2_0		( SEG2A | SEG2B | SEG2C | SEG2D | SEG2E | SEG2F )
#define DIGIT2_1		( SEG2B | SEG2C )
#define DIGIT2_2		( SEG2A | SEG2B | SEG2D | SEG2E | SEG2G )
#define DIGIT2_3		( SEG2A | SEG2B | SEG2C | SEG2D | SEG2G )
#define DIGIT2_4		( SEG2B | SEG2C | SEG2F | SEG2G )
#define DIGIT2_5		( SEG2A | SEG2C | SEG2D | SEG2F | SEG2G )
#define DIGIT2_6		( SEG2A | SEG2C | SEG2D | SEG2E | SEG2F | SEG2G )
#define DIGIT2_7		( SEG2A | SEG2B | SEG2C )
#define DIGIT2_8		( SEG2A | SEG2B | SEG2C | SEG2D | SEG2E | SEG2F | SEG2G )
#define DIGIT2_9		( SEG2A | SEG2B | SEG2C | SEG2D | SEG2F | SEG2G )

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

// Currently this is dead code
void TaskSegmentCycler(void const * argument)
{
  for(;;)
  {
    osDelay(500);
  }
}
