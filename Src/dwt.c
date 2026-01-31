
#define STM32H753xx

#include "dwt.h"
#include <stm32h7xx_hal.h>

int DWT_delay_init() {
    /* Disable and re-enable TRC() */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* Disable and re-enable the clock cycle counter */
    DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
    DWT->CTRL |=  DWT_CTRL_CYCCNTENA_Msk;

    DWT->CYCCNT = 0;

    __ASM volatile ("nop");
    __ASM volatile ("nop");
    __ASM volatile ("nop");
    __ASM volatile ("nop");
    __ASM volatile ("nop");

    return (DWT->CYCCNT ? 1 : 0);
}

void DWT_delay_us(uint32_t micros) {
    uint32_t cycle_count = (micros*DWT->CYCCNT)/1000000UL;
    uint32_t initial_tick = DWT->CYCCNT;
    while((DWT->CYCCNT-initial_tick) < cycle_count) { __ASM volatile ("nop"); }
}