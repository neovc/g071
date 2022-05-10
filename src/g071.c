#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <errno.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include <libopencm3/cm3/vector.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/stm32/syscfg.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>

extern const struct rcc_clock_scale rcc_clock_config[];

int
main(void)
{
	rcc_clock_setup(&(rcc_clock_config[RCC_CLOCK_CONFIG_HSI_PLL_64MHZ]));
	flash_prefetch_enable();
	flash_set_ws((RUNNING_CLOCK > 48?2:1));
	/* eanble flash's DCache & ICache */
	flash_icache_enable();
}

