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
int printf_(const char *format, ...);

#define std_printf printf_
#define IRQ2NVIC_PRIOR(x)        ((x)<<4)

void
setup_usart_speed(uint32_t usart, uint32_t baudrate)
{
	if ((usart != USART1) && (usart != USART2) && (usart != USART3))
		return;

	if (baudrate == 0)
		baudrate = 115200;

	usart_set_baudrate(usart, baudrate);
	usart_set_databits(usart, 8);
	usart_set_stopbits(usart, USART_STOPBITS_1);
	/* don't enable usart tx/rx here to drop bogus data in tx/rx line */
	/* usart_set_mode(usart, USART_MODE_TX_RX); */
	usart_set_parity(usart, USART_PARITY_NONE);

	/* Enable USART Receive interrupt. */
	usart_enable_rx_interrupt(usart);
}

static void
setup_usart2(void)
{
	/* USART2, PA2 PA3, AF7 */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_USART2);

	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);
	gpio_set_af(GPIOA, GPIO_AF1, GPIO2 | GPIO3);

	/* can't has high priority than MAX_SYSCALL_INTERRUPT_PRIORITY */
	nvic_set_priority(NVIC_USART2_IRQ, IRQ2NVIC_PRIOR(configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1));
	nvic_enable_irq(NVIC_USART2_IRQ);
	usart_enable(USART2);
	usart_set_mode(USART2, USART_MODE_TX_RX);
}

void
usart_timeout_putc(uint32_t usart, char c)
{
	int i = 0;

	if (usart == 0)
		return;

#define USART_LOOP 10000
	/* check usart's tx buffer is empty */
	while ((i < USART_LOOP) && ((USART_ISR(usart) & USART_ISR_TXE) == 0))
		i ++;
	if (i < USART_LOOP)
		usart_send(usart, c);
}

void
_putchar(char c)
{
	usart_timeout_putc(USART2, c);
	if (c == '\n')
		usart_timeout_putc(USART2, '\r');
}

int
main(void)
{
	rcc_clock_setup(&(rcc_clock_config[RCC_CLOCK_CONFIG_HSI_PLL_64MHZ]));
	flash_prefetch_enable();
	flash_set_ws((RUNNING_CLOCK > 48?2:1));
	/* eanble flash's DCache & ICache */
	flash_icache_enable();
	setup_usart2();
    setup_usart_speed(USART2, 115200);
	
	std_printf("HELLO WORLD\n");
}

