#include "stm32f1xx_hal.h"
#include <stdint.h>

/**
 *  Custom function to send a character to any ITM channel (not just channel 0)
 *  Code copied from CMSIS
 */
void ITM_SendCharOnChannel (uint32_t ch, uint32_t port) {

  if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
      ((ITM->TER & (1UL << port)     ) != 0UL)   )     /* ITM Port #0 enabled */
  {
    while (ITM->PORT[port].u32 == 0UL)
    {
      __NOP();
    }
    ITM->PORT[port].u8 = (uint8_t) ch;
  }
}

/**
 *  Custom function to send an entire string over an ITM channel
 *  Code modified from CMSIS
 */
void ITM_SendStrOnChannel (uint8_t* str, uint32_t port) {
  if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
      ((ITM->TER & (1UL << port)     ) != 0UL)   )     /* ITM Port #0 enabled */
  {
    // Iterate over the entire string
    while (*str) {

      // Ensure the previous character finishes transmitting
      while (ITM->PORT[port].u32 == 0UL)
        __NOP();

      ITM->PORT[port].u8 = (uint8_t) *str++;
    }
  }
}
