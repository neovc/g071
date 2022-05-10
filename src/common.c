#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/syscfg.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/rng.h>

#include <csp/csp.h>
#include <csp/csp_crc32.h>
#include <csp/drivers/can.h>
#include <csp/interfaces/csp_if_kiss.h>
#include <csp/csp_endian.h>
#include <csp/arch/csp_clock.h>

#include <param/param_serializer.h>

#include "tianyi.h"

#define HIST_COUNT 5
static char cmd_hist[HIST_COUNT][MAX_USART_RX_LEN + 1];
gosh_cmd_handler gosh_cb = NULL;
BaseType_t woken = pdTRUE;
QueueHandle_t g_usart_rx_queue = NULL, g_usart_cmd_queue = NULL, g_usart_tx_queue = NULL;
QueueSetHandle_t g_usart_set = NULL;

char *usart_rx_buf = NULL, gosh_prompt[40] = "# ";
struct cmd_args *repeat_cmds = NULL;
struct table_offset *g_table_offset = NULL;
const uint32_t g_usart_matrix[7] = {0, USART1, USART2, USART3, LPUART1, 0, 0};
uint32_t usart_console = 0, usart_kiss = 0, uptime = 0, g_utc_time = 0, device_flash_size, flash_backup_cfg_addr = FLASH_BACKUP_CFG_ADDR;
uint16_t ftp_timeout = 5000;
uint8_t freertos_started = 0, fs_mounted = 0, spi_flash_status = 0, usart_started = 0, in_ftp_operation = 0, this_can_id = 5, param_iteration = 0;
uint8_t escape_mode = 0, to_feed_iwdg = 1, can_bus_init_count = 0;
int usart_rx_len = 0, kiss_tx_state = 0, max_hist_count = -1, cur_hist_count = -1;

const char param_type_name[17][4] = {
	{"U8"}, {"U16"}, {"U32"}, {"U64"},
	{"I8"}, {"I16"}, {"I32"}, {"I64"},
	{"X8"}, {"X16"}, {"X32"}, {"X64"},
	{"DBL"}, {"FLT"}, {"STR"}, {"DAT"},
	{"BL"}
};

#ifndef TIANYI_RELEASE
uint8_t log_mask = (1 << 3) | (1 << 0); /* ERROR & INFO */
#else
uint8_t log_mask = 0;
#endif

extern csp_iface_t csp_if_kiss;
extern struct g_params_config *g_conf_ptr;
extern char *flash_memory_ptr;

#define USART_RX_MAX		16

struct usart_rx_data {
	uint32_t usart;
	uint8_t c[USART_RX_MAX];
	int len;
};

#define USART_CR1_FIFOEN		BIT29

#define USART_CR3_RXFTIE_EN		BIT28
#define USART_CR3_RXFIFO_SHIFT		25
#define USART_FIFO_THRESH_RX_FULL	5 /* 8/8 */
#define USART_FIFO_THRESH_SEVENEIGTH	4 /* 7/8 */
#define USART_FIFO_THRESH_THREEQTR	3 /* 3/4 */
#define USART_FIFO_THRESH_HALF		2 /* 1/2 */
#define USART_FIFO_THRESH_QUARTER	1 /* 1/4 */
#define USART_FIFO_THRESH_EIGTH		0 /* 1/8 */

void
setup_usart_speed(uint32_t usart, uint32_t baudrate, int enable_rxfifo)
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

	if (enable_rxfifo) {
		USART_CR1(usart) |= USART_CR1_FIFOEN; /* enable FIFO */
		USART_CR3(usart) |= (USART_FIFO_THRESH_SEVENEIGTH << USART_CR3_RXFIFO_SHIFT); /* RX FIFO 7/8 */
		USART_CR3(usart) |= USART_CR3_RXFTIE_EN; /* enable rx fifo interrupt */
		USART_CR1(usart) |= USART_CR1_IDLEIE; /* enable idle interrupt */
	}

	/* Enable USART Receive interrupt. */
	usart_enable_rx_interrupt(usart);
}

void
handle_console_input(char data, void *unused)
{
	int finish = 0, i, r = 1;

	if (usart_rx_buf == NULL) /* buffer is not ready */
		return;

	if (data == 0x0) {
		/* input NULL character */
		usart_rx_len = 0;
		return;
	}

	if (data == '\r' || data == '\n') {
		/* end of line */
		usart_rx_buf[usart_rx_len] = '\0';
		finish = 1;
	} else if ((data == 0x08) || (data == 0x7f)) {
		/* backspace */
		if (usart_rx_len > 0)
			usart_rx_len --;
	} else if (data == 0x17) { /* ^W Erase Word */
		if (usart_rx_len > 0) {
			for (i = usart_rx_len - 1; i >= 0; i --) {
				if (usart_rx_buf[i] == ' ')
					break;
			}
			if (i > 0)
				usart_rx_len = i + 1;
			else
				usart_rx_len = 0;
		}
	} else if (data == 0x15) { /* ^U Erase Line */
		usart_rx_len = 0;
#ifndef BUILD_SMALL_BINARY
	} else if (data == 0x1b) { /* Escape Word */
		escape_mode = 1;
		r = 0; /* keep value of escape_mode */
	} else if (data == 0x5b) {
		if (escape_mode == 1) {
			escape_mode = 2;
			r = 0; /* keep value of escape_mode */
		} else if (escape_mode == 0) {
			usart_rx_buf[usart_rx_len ++] = data;
		}
	} else if (data >= 'A' && data <= 'D') {
		if (escape_mode == 2) {
			/* UP KEY -> 'A', DOWN KEY -> 'B', LEFT KEY -> 'D', RIGHT KEY -> 'C' */
			if (max_hist_count >= 0) {
				if (data == 'A') {
					/* UP */
					if (cur_hist_count >= 0) {
						memcpy(usart_rx_buf, &(cmd_hist[cur_hist_count % HIST_COUNT][0]), MAX_USART_RX_LEN);
						usart_rx_len = strlen(usart_rx_buf);
						cur_hist_count --;
					}
				} else if (data == 'B') {
					/* DOWN */
					if (cur_hist_count < max_hist_count) {
						memcpy(usart_rx_buf, &(cmd_hist[cur_hist_count % HIST_COUNT][0]), MAX_USART_RX_LEN);
						usart_rx_len = strlen(usart_rx_buf);
						cur_hist_count ++;
					}
				}
			}
		} else if (escape_mode == 0) {
			usart_rx_buf[usart_rx_len ++] = data;
		}
#endif
	} else {
		usart_rx_buf[usart_rx_len ++] = data;
	}

	if (r == 1)
		escape_mode = 0;

	usart_rx_buf[usart_rx_len] = '\0'; /* terminate cmd line */
	if (finish == 1 || (usart_rx_len >= (MAX_USART_RX_LEN - 1))) {
		if (g_usart_cmd_queue != NULL)
			xQueueSendToBack(g_usart_cmd_queue, usart_rx_buf, 10);
		if (usart_rx_len > 0) {
			if (max_hist_count == -1)
				max_hist_count = 0;
			else
				max_hist_count ++;
			cur_hist_count = max_hist_count;
			memcpy(&(cmd_hist[max_hist_count % HIST_COUNT][0]), usart_rx_buf, MAX_USART_RX_LEN);
		}
		usart_rx_len = 0;
		console_puts("\r\n"); /* new line */
	} else {
		console_puts("\r"); /* clear current input buffer */
		console_puts(gosh_prompt);
		console_puts(usart_rx_buf);
	}
}

void
setup_gosh_callback(gosh_cmd_handler f)
{
	gosh_cb = f;
}

void
generic_usart_handler(void *args)
{
	struct usart_rx_data d;
	char c;
	QueueSetMemberHandle_t s;
	int r = 0, len, i, argc, pos;
	char cmd[MAX_USART_RX_LEN], *argv[MAX_ARGS], *p;

	while (1) {
		s = xQueueSelectFromSet(g_usart_set, portMAX_DELAY);
		if (s == g_usart_rx_queue) {
			r = xQueueReceive(g_usart_rx_queue, &d, 0);
			if (r != pdPASS)
				continue;
			if (d.usart == usart_kiss) {
				csp_kiss_rx(&csp_if_kiss, d.c, d.len, NULL);
			} else if (d.usart == usart_console) {
				for (i = 0; i < d.len; i ++)
					handle_console_input(d.c[i], NULL);
			}
		} else if (s == g_usart_cmd_queue) {
			r = xQueueReceive(g_usart_cmd_queue, cmd, 0);

			if (r != pdPASS)
				continue;
			if (repeat_cmds != NULL) /* repeat cmd disable after next execution */
				repeat_cmds->interval = 0;
			len = strlen(cmd);
			argc = pos = 0;
			/* to find count of arguments */
			while (pos < len && argc < MAX_ARGS) {
				/* strip prefix ' ' & '\t' */
				while (pos < len && ((cmd[pos] == ' ') || (cmd[pos] == '\t')))
					pos ++;

				if (pos == len || cmd[pos] == '\0')
					break;
				p = cmd + pos;
				argv[argc ++] = p;

				while (pos < len && ((cmd[pos] != ' ') && (cmd[pos] != '\t')))
					pos ++;

				if (pos == len) break;
				else cmd[pos ++] = '\0';
			}

			if (argc > 0 && gosh_cb != NULL)
				gosh_cb(argv, argc);
			console_puts(gosh_prompt); /* cmd line prefix */
		} else if (s == g_usart_tx_queue) {
			r = xQueueReceive(g_usart_tx_queue, &c, 0);
			if (r != pdPASS)
				continue;
			len = uxQueueMessagesWaiting(g_usart_tx_queue);
			do {
				usart_timeout_putc(usart_kiss, c);
				if (len > 0) {
					r = xQueueReceive(g_usart_tx_queue, &c, 0);
					len --;
				} else {
					r = pdFAIL;
				}
			} while (r == pdPASS);
		}
	}
}

#define USART_RX_QUEUE_LENGTH	512
#define USART_TX_QUEUE_LENGTH	1024
#define USART_CMD_QUEUE_LENGTH	2

void
start_usart_task(void)
{
	if ((usart_console != 0) || (usart_kiss != 0)) {
		g_usart_rx_queue = xQueueCreate(USART_RX_QUEUE_LENGTH, sizeof(struct usart_rx_data));

		usart_rx_buf = pvPortMalloc(MAX_USART_RX_LEN + 4);
		memset(usart_rx_buf, 0, MAX_USART_RX_LEN + 4);
		g_usart_cmd_queue = xQueueCreate(USART_CMD_QUEUE_LENGTH, MAX_USART_RX_LEN + 4);
		memset(&(cmd_hist[0][0]), 0, HIST_COUNT * MAX_USART_RX_LEN);

		g_usart_set = xQueueCreateSet(USART_RX_QUEUE_LENGTH + USART_CMD_QUEUE_LENGTH + USART_TX_QUEUE_LENGTH);

		g_usart_tx_queue = xQueueCreate(USART_TX_QUEUE_LENGTH, sizeof(char));
		xQueueAddToSet(g_usart_rx_queue, g_usart_set);
		xQueueAddToSet(g_usart_tx_queue, g_usart_set);
		xQueueAddToSet(g_usart_cmd_queue, g_usart_set);

		if (pdPASS != xTaskCreate(generic_usart_handler, "UART", 800, NULL, 2 | portPRIVILEGE_BIT, NULL)) {
			ERROR_LOG("NEW GOSH TASK ERROR\n");
		}
	}

	if (usart_console != 0) {
		/* enable usart_console & usart_kiss here
		 * don't init early, otherwise will cause freertos hangs.
		 */
		usart_enable(usart_console);
		/* according RM0432, Section 44.5.5
		 * 1) Enable the USART by writing the UE bit in USART_CR1 register to 1.
		 * 2) Set the TE bit in USART_CR1 to send an idle frame as first transmission.
		 */
		usart_set_mode(usart_console, USART_MODE_TX); /* enable TX only here */
		usart_started = 1;
	}
}

int
watcher_handler(void *p)
{
	if (repeat_cmds == NULL)
		return 1;

	if (gosh_cb != NULL)
		gosh_cb(repeat_cmds->argv, repeat_cmds->argc);
	repeat_cmds->timer_id ++;
	if (repeat_cmds->timer_id > WATCH_TIMER_ID_MAX || repeat_cmds->timer_id < WATCH_TIMER_ID_MIN)
		repeat_cmds->timer_id = WATCH_TIMER_ID_MIN;

	if ((repeat_cmds->interval > 0) && (repeat_cmds->enabled == 1)) {
		register_timer_ts_action_handler(repeat_cmds->timer_id, repeat_cmds->interval, 0, watcher_handler, NULL, 0, 0);
	} else {
		repeat_cmds->enabled = 0;
	}
	return 0;
}

void
generic_usart_isr_handler(uint32_t usart)
{
	struct usart_rx_data d;

	if (usart == 0)
		return;

	d.usart = usart;
	d.len = 0;

	if ((USART_ISR(usart) & USART_ISR_IDLE) != 0) {
		/* CLEAR IDLEIE FLAG */
		USART_ICR(usart) |= USART_ICR_IDLECF;
	}

	/* Check if we were called because of RXNE */
	while ((d.len < USART_RX_MAX) &&
		((USART_CR1(usart) & USART_CR1_RXNEIE) != 0) &&
		((USART_ISR(usart) & USART_ISR_RXNE) != 0)) {
		d.c[d.len ++] = usart_recv(usart);
	}

	if ((d.len > 0) && g_usart_rx_queue && (uxQueueSpacesAvailable(g_usart_rx_queue) > 0)) {
		xQueueSendToBackFromISR(g_usart_rx_queue, &d, &woken);
	}

	/* Check if we were called because of TXE. */
	if (((USART_CR1(usart) & USART_CR1_TXEIE) != 0) &&
		((USART_ISR(usart) & USART_ISR_TXE) != 0)) {
		/* Indicate that we are sending out data. */
		/* Disable the TXE interrupt as we don't need it anymore. */
		USART_CR1(usart) &= ~USART_CR1_TXEIE;
	}

	if ((USART_ISR(usart) & USART_ISR_ORE) != 0) {
		/* CLEAR ORE FLAG */
		USART_ICR(usart) |= USART_ICR_ORECF;
	}
}

void
usart2_isr(void)
{
	generic_usart_isr_handler(USART2);
}

void
usart3_isr(void)
{
	generic_usart_isr_handler(USART3);
}

void
lpuart1_isr(void)
{
	generic_usart_isr_handler(LPUART1);
}

void
usart1_isr(void)
{
	generic_usart_isr_handler(USART1);
}

uint32_t
swapu32(uint32_t n)
{
	return (((n & 0xff000000) >> 24) | ((n & 0xff) << 24) |
		((n & 0xff0000) >> 8) | ((n & 0xff00) << 8));
}

void
usart_timeout_putc(uint32_t usart, char c)
{
	int i = 0;

	if (usart == 0 || usart_started == 0)
		return;

	/* check usart's tx buffer is empty */
	while ((i < USART_LOOP) && ((USART_ISR(usart) & USART_ISR_TXE) == 0))
		i ++;
	if (i < USART_LOOP)
		usart_send(usart, c);
}

void
usart_send_hex(uint32_t usart, uint8_t *ptr, int len, int mode, int skip_leading_zero)
{
	uint8_t c, d;
	int i;
	int first = 1;

	if (ptr == NULL || len <= 0 || usart == 0)
		return;

	for (i = 0; i < len; i ++) {
		c = ptr[i];
		if (skip_leading_zero == 1 && first == 1 && c == 0)
			continue;
		first = 0;
		if (mode == 1) { /* hex mode */
			d = (c >> 4) & 0x0f;
			if (d >= 0xa)
				d += 'A' - 0xa;
			else
				d += '0';
			usart_timeout_putc(usart, d);

			d = c & 0x0f;
			if (d >= 0xa)
				d += 'A' - 0xa;
			else
				d += '0';
			usart_timeout_putc(usart, d);
		} else { /* text mode */
			usart_timeout_putc(usart, c);
		}
	}
}

void
console_puts(char *ptr)
{
	if (ptr == NULL || usart_console == 0)
		return;

	while (ptr[0]) {
		usart_timeout_putc(usart_console, ptr[0]);
		ptr ++;
	}
}

void
usart_send_string(uint32_t usart, uint8_t *ptr)
{
	if (ptr == NULL || usart == 0)
		return;

	while (ptr[0]) {
		usart_timeout_putc(usart, ptr[0]);
		ptr ++;
	}
}

void
can1_rx0_isr(void)
{
	uint32_t id;
	bool ext, rtr;
	uint8_t fmi, length, data[8];
	can_frame_t frame;

	while (CAN_RF0R(CAN1) & CAN_RF0R_FMP0_MASK) {
		/* LOOP CAN RX TILL FIFO0 IS EMPTY */
		can_receive(CAN1, 0, true, &id, &ext, &rtr, &fmi, &length, data, NULL);
		frame.id = id;
		frame.dlc = length;
		memcpy(frame.data, data, frame.dlc);
		csp_can_rx_frame(&frame, &woken);
	}
}

#define CAN_SEND_LOOP 10000

static uint32_t can_tsr_terr[3] = {CAN_TSR_TERR0, CAN_TSR_TERR1, CAN_TSR_TERR2};
static uint32_t can_tsr_rqcp[3] = {CAN_TSR_RQCP0, CAN_TSR_RQCP1, CAN_TSR_RQCP2};
static uint32_t can_tsr_alst[3] = {CAN_TSR_ALST0, CAN_TSR_ALST1, CAN_TSR_ALST2};
static uint32_t can_tsr_txok[3] = {CAN_TSR_TXOK0, CAN_TSR_TXOK1, CAN_TSR_TXOK2};

int
real_can_send(can_id_t id, uint8_t * data, uint8_t dlc, int retry)
{
	int i = 0, r, j = -1;
	uint32_t can_tsr;
	if (data == NULL || dlc <= 0)
		return -1;

	r = can_transmit(CAN1, id, true, false, dlc, data);
	if (r == -1)
		return -1;

	for (i = 0; i < CAN_SEND_LOOP; i ++) {
		can_tsr = CAN_TSR(CAN1);
		if (can_tsr & can_tsr_rqcp[r]) {
			j = -1;
			/* request completed, check transmit status */
			if (can_tsr & can_tsr_terr[r]) {
				log_info("CAN1 MAILBOX%d Transmission Error, retry = %d, loop = %d, LEC= 0x%x\n", r, retry, i,
						(CAN_ESR(CAN1) >> 4) & 0x7);
			} else if (can_tsr & can_tsr_alst[r]) {
				log_info("CAN1 MAILBOX%d Arbitration Lost, retry = %d, loop = %d, LEC = 0x%x\n", r, retry, i,
						(CAN_ESR(CAN1) >> 4) & 0x7);
			} else if (can_tsr & can_tsr_txok[r]) {
				j = 0;
			} else {
				log_info("CAN1 MAILBOX%d, STRANGE CAN_TSR REG -> 0x%x, retry = %d, loop = %d\n", r, can_tsr, retry, i);
			}
			return j;
		}
	}

	ERROR_LOG("CAN1 MAILBOX%d TIMEOUT Error, retry = %d, loop = %d, LEC = 0x%x\n", r, retry, i, (CAN_ESR(CAN1) >> 4) & 0x7);
	return -1;
}

#define CAN_PARAM_SJW	CAN_BTR_SJW_1TQ
#define CAN_PARAM_TS1	CAN_BTR_TS1_6TQ
#define CAN_PARAM_TS2	CAN_BTR_TS2_3TQ
#define CAN_INIT_LOOP	5

int
init_can1(void)
{
	uint32_t id, mask;
	int loop;

	for (loop = 0; loop < CAN_INIT_LOOP; loop ++) {
		/* reset CAN1 register first */
		can_reset(CAN1);

		/* CAN cell init.
		 * Setting the bitrate to 512KBit. APB1 = 120MHz,
		 * prescaler = 24 -> 5MHz time quanta frequency.
		 * 1tq sync + 6tq bit segment1 (TS1) + 3tq bit segment2 (TS2) =
		 * 10 time quanto per bit period, therefore 5MHz/10 = 500KHz
		 */

		/* 500 Kbps */
		if (0 == can_init(CAN1, false /* ttcm */, true /* abom */,
				false /* awum */,
#if (AUTO_CAN_RETRANSMIT == 0)
				true /* nart */,
#else
				false /* nart */,
#endif
				false /* rflm */, true /* txfp */,
				CAN_PARAM_SJW, CAN_PARAM_TS1, CAN_PARAM_TS2, RUNNING_CLOCK/5, false, false)
			) {
			/* init OK */
			break;
		}

		if (freertos_started == MAGIC_55)
			vTaskDelay(pdMS_TO_TICKS(500)); /* retry again after 0.5 seconds */
	}

	if (loop == CAN_INIT_LOOP) {
		return 1;
	}

	id = make_can_dest(this_can_id) << 3; /* RM0432 v8, p2049 */
	mask = make_can_dest(0xff) << 3; /* RM0432 v8, p2049 */
	can_filter_id_mask_32bit_init(0, id, mask /* 0x0 */, 0 /* FIFO */, true);

	can_enable_irq(CAN1, CAN_IER_FMPIE0);
	return 0;
}

#define MAX_CAN_ERRCNT 20

void
check_can_esr_reg(void)
{
	int rx_err, tx_err;
	uint32_t can_esr;

	can_esr = CAN_ESR(CAN1);
	rx_err = (can_esr >> 24) & 0xff;
	tx_err = (can_esr >> 16) & 0xff;

	if (rx_err > 0 || tx_err > 0) {
		ERROR_LOG("CAN ESR: RX ERR %d, TX ERR %d, BUS OFF %d, ERR PASSIVE %d, ERR WARNING %d\n", rx_err, tx_err, (can_esr >> 2) & 0x1, (can_esr >> 1) & 0x1, can_esr & 0x1);
	}

	if (rx_err > MAX_CAN_ERRCNT || tx_err > MAX_CAN_ERRCNT) {
		/* to restart CAN1 */
		can_bus_init_count ++;
		if (init_can1()) {
			log_info("#%d REINIT CAN1 -> FAILED.\n", can_bus_init_count);
		} else {
			ERROR_LOG("#%d REINIT CAN1 -> OK\n", can_bus_init_count);
		}
	}
}

int
can_send(can_id_t id, uint8_t * data, uint8_t dlc)
{
	int r = -1, i;
#ifndef TIANYI_RELEASE
	TickType_t start, end;
#endif

	if (data == NULL || dlc <= 0)
		return -1;

#ifndef TIANYI_RELEASE
	start = xTaskGetTickCount();
#endif

#if (AUTO_CAN_RETRANSMIT == 0)
	for (i = 1; (i >= 0) && (r == -1); i --) { /* software can retransmit */
#else
	for (i = 0; (i >= 0) && (r == -1); i --) {
#endif
		r = real_can_send(id, data, dlc, i);
	}

	if (r == -1) {
#ifndef TIANYI_RELEASE
		end = xTaskGetTickCount();
		ERROR_LOG("can_send() failed, which takes %d ms\n", end - start);
#endif
		check_can_esr_reg();
	}

	return r;
}

int
can_send_isr(can_id_t id, uint8_t * data, uint8_t dlc)
{
	int i = 0, r, j = -1;
	uint32_t can_tsr;
	if (data == NULL || dlc <= 0)
		return -1;

	r = can_transmit(CAN1, id, true, false, dlc, data);
	if (r == -1)
		return -1;

	for (i = 0; i < CAN_SEND_LOOP; i ++) {
		can_tsr = CAN_TSR(CAN1);
		if (can_tsr & can_tsr_rqcp[r]) {
			/* request completed, check transmit status */
			if (can_tsr & can_tsr_txok[r])
				j = 0;
			return j;
		}
	}

	return -1;
}

void
kiss_usart_putc(char c)
{
	if (g_usart_tx_queue)
		xQueueSendToBack(g_usart_tx_queue, &c, 0);
}

void
console_putc(char c)
{
	usart_timeout_putc(usart_console, c);
}

void
_putchar(char c)
{
	usart_timeout_putc(usart_console, c);
	if (c == '\n')
		usart_timeout_putc(usart_console, '\r');
}

/**
 * Dumps a chunk of memory to the screen
 */
void
hex_dump(char *src, int len)
{
	int i, j, k;
	char text[17];

	text[16] = '\0';
	std_printf("%p : ", src);
	for (i = 0, j = 0; i < len; i ++) {
		j ++;
		std_printf("%02X ", ((volatile unsigned char *)src)[i]);
		if (j == 8)
			std_printf(" ");
		if (j == 16 || (i == (len - 1))) {
			if (j < 8)
				std_printf(" ");
			for (k = j; k < 16; k ++) {
				std_printf("   ");
			}

			memcpy(text, &((char *)src)[i + 1 - j], j);
			for (k = 0; k < j; k ++) {
				if ((text[k] < 32) || (text[k] > 126)) {
					text[k] = '.';
				}
			}
			text[j] = '\0';
			if (j == 16)
				std_printf(" |%s|\n", text);
			else
				std_printf(" |%s\n", text);
			if (i < (len - 1)) {
				std_printf("%p : ", src + i + 1);
			}
			j = 0;
		}
	}
	if (i % 16)
		std_printf("\n");
}

#define MIDDLE_PAGE_NUM (device_flash_size / FLASH_PAGESIZE / 2)

/* return 0 if OK
 * return 1 if timeout
 */
int
flash_timeout_wait_for_last_operation(void)
{
	int i = 0;
	while ((i < FLASH_LOOP) && ((FLASH_SR & FLASH_SR_BSY) == FLASH_SR_BSY))
		i ++;

	return i < FLASH_LOOP?0:1;
}

/* return actual byte written to flash */
int
flash_write_double_word(uint32_t address, uint32_t u1, uint32_t u2)
{
	/* Ensure that all flash operations are complete. */
	if (flash_timeout_wait_for_last_operation()) {
		log_info("timeout while waiting previous flash operation complete\n");
		return 0;
	}

	/* clear error status first */
	flash_clear_status_flags();

	/* Program the double word. */
	/* according RM0432, P109, 3.3.7
	 * 1) Write a first word in an address aligned with double word
	 * 2) Write the second word
	 *
	 * DON'T USE MMIO64(address) here
	 */
	MMIO32(address) = u1;
	MMIO32(address + 4) = u2;

	/* Wait for the write to complete. */
	flash_wait_for_last_operation();

	if (FLASH_SR & 0xff) {
		log_info("write double word at address 0x%x error, FLASH SR = 0x%x\n", address, FLASH_SR);
		return 0;
	}
	return 8;
}

/* return 0 if OK
 * return 1 if failed
 */
int
flash_reset_page(uint32_t page)
{
	/* Ensure that all flash operations are complete. */
	if (flash_timeout_wait_for_last_operation()) {
		log_info("timeout while waiting previous flash operation complete\n");
		return 1;
	}

	flash_clear_status_flags();

	/* clear BKER flag first
	 * we need to clear FLASH_CR_PG flag for page erase
	 */
	FLASH_CR &= ~((FLASH_CR_PNB_MASK << FLASH_CR_PNB_SHIFT) | FLASH_CR_BKER | FLASH_CR_PG);
	if (page >= MIDDLE_PAGE_NUM) {
		FLASH_CR |= (page - MIDDLE_PAGE_NUM) << FLASH_CR_PNB_SHIFT;
		FLASH_CR |= FLASH_CR_PER | FLASH_CR_BKER;
	} else {
		FLASH_CR |= page << FLASH_CR_PNB_SHIFT;
		FLASH_CR |= FLASH_CR_PER;
	}
	FLASH_CR |= FLASH_CR_START;

	flash_wait_for_last_operation();

	FLASH_CR &= ~(FLASH_CR_PER | FLASH_CR_BKER);
	if (flash_timeout_wait_for_last_operation()) {
		log_info("flash reset page wait timeout\n");
		return 1;
	}
	return 0;
}

/* return 0 if OK
 * return 1 if failed
 */
int
flash_reset_all_pages(void)
{
	if (flash_timeout_wait_for_last_operation()) {
		log_info("timeout while waiting previous flash operation complete\n");
		return 1;
	}
	FLASH_CR &= ~FLASH_CR_PG;
	FLASH_CR |= FLASH_CR_MER1 | FLASH_CR_MER2;
	FLASH_CR |= FLASH_CR_START;

	flash_wait_for_last_operation();
	FLASH_CR &= ~(FLASH_CR_MER1 | FLASH_CR_MER2);
	return 0;
}

/* return actual bytes write to flash */
int
flash_write_data(uint32_t start, char *data, int len)
{
	int i, j, k = 0, size, retry;
	uint32_t page_start;
	uint32_t u32_1, u32_2;

	/* check if start_address is in proper range */
	if ((start < (FLASH_BASEADDR)) || (start >= (FLASH_BASEADDR + device_flash_size))) {
		ERROR_LOG("start address 0x%x isn't in flash bank 1\n", start);
		return 0;
	}

	/* Ensure that all flash operations are complete. */
	if (flash_timeout_wait_for_last_operation()) {
		log_info("timeout while waiting previous flash operation complete\n");
		return 0;
	}

	flash_unlock();

	flash_clear_status_flags();

	if (start % FLASH_PAGESIZE == 0) {
		/* start of flash page size */
		page_start = start;
		size = FLASH_PAGESIZE;
	} else {
		page_start = (start / FLASH_PAGESIZE) * FLASH_PAGESIZE;
		size = page_start + FLASH_PAGESIZE - start;
	}

	for (i = 0; i < len; ) {
		if (FLASH_PAGESIZE != (j = flash_read_data(page_start, FLASH_PAGESIZE, flash_memory_ptr))) {
			ERROR_LOG("flash read whole page data failed\n");
			break;
		}

		if ((len - i) < size)
			size = len - i;

		memcpy(flash_memory_ptr + start - page_start, data, size);

		if (flash_reset_page((page_start - FLASH_BASEADDR) / FLASH_PAGESIZE))
			break;

		/* according to STM32L4+ errata pdf, ES0393, Section 2.2.2
		 * we need to disable dcache before programing flash, re-enable dcache after programming flash.
		 */

		/* Disable DCache */
		flash_dcache_disable();

		/* Enable writes to flash. */
		FLASH_CR |= FLASH_CR_PG;

#define FLASH_WRITE_RETRY_TIMES	3

		for (j = 0; j < FLASH_PAGESIZE; j += sizeof(uint64_t), page_start += sizeof(uint64_t)) {
			u32_1 = *(uint32_t *)(flash_memory_ptr + j);
			u32_2 = *(uint32_t *)(flash_memory_ptr + j + sizeof(uint32_t));
			for (retry = 0; retry < FLASH_WRITE_RETRY_TIMES; retry ++) {
				if (sizeof(uint64_t) == flash_write_double_word(page_start, u32_1, u32_2))
					break;
			}

			if (retry == FLASH_WRITE_RETRY_TIMES) {
				ERROR_LOG("flash write error at 0x%x after retry %d times.\n", page_start, FLASH_WRITE_RETRY_TIMES);
				break;
			}
		}
		/* Disable writes to flash. */
		FLASH_CR &= ~FLASH_CR_PG;

		/* Reset DCache */
		flash_dcache_reset();

		/* Enable DCache */
		flash_dcache_enable();

		if (j != FLASH_PAGESIZE) /* sth is wrong */
			break;

		i += size;
		data += size;
		k += size;
		size = FLASH_PAGESIZE;
	}

	flash_lock();
	return k;
}

/* return actual read byte from flash */
int
flash_read_data(uint32_t start, int length, char *data)
{
	int i;
	uint32_t p;
	uint32_t *m= (uint32_t *) start;

	if (length <= 0 || data == NULL || (start % 4)) /* start address should be 4 byte align */
		return 0;

	p = (int) data;
	if (p % 4) {
		log_info("buffer address %p isn't 4 byte align\n", data);
		return 0;
	}

	if ((length % 4) == 0) {
		length = length / 4;
	} else {
		length = (length / 4 + 1);
	}

	for (i = 0; i < length; i ++) {
		*((uint32_t *) data) = *(m + i);
		data += 4;
	}

	return i * 4;
}

/* hard fault handler in C,
 * with stack frame location as input parameter
 * called from HardFault_Handler in file xxx.s
 */
void
dump_call_stack(unsigned int * hardfault_args)
{
#ifndef BUILD_SMALL_BINARY
	unsigned int stacked_r0;
	unsigned int stacked_r1;
	unsigned int stacked_r2;
	unsigned int stacked_r3;
	unsigned int stacked_r12;
	unsigned int stacked_lr;
	unsigned int stacked_pc;
	unsigned int stacked_psr;

	stacked_r0 = ((unsigned long) hardfault_args[0]);
	stacked_r1 = ((unsigned long) hardfault_args[1]);
	stacked_r2 = ((unsigned long) hardfault_args[2]);
	stacked_r3 = ((unsigned long) hardfault_args[3]);

	stacked_r12 = ((unsigned long) hardfault_args[4]);
	stacked_lr = ((unsigned long) hardfault_args[5]);
	stacked_pc = ((unsigned long) hardfault_args[6]);
	stacked_psr = ((unsigned long) hardfault_args[7]);

	std_printf("R0 = %x\n", stacked_r0);
	std_printf("R1 = %x\n", stacked_r1);
	std_printf("R2 = %x\n", stacked_r2);
	std_printf("R3 = %x\n", stacked_r3);
	std_printf("R12 = %x\n", stacked_r12);
	std_printf("LR [R14] = %x  subroutine call return address\n", stacked_lr);
	std_printf("PC [R15] = %x  program counter\n", stacked_pc);
	std_printf("PSR = %x\n", stacked_psr);

	std_printf("BFAR = %x\n", (*((volatile unsigned long *)(0xE000ED38))));
	std_printf("CFSR = %x\n", (*((volatile unsigned long *)(0xE000ED28))));
	std_printf("HFSR = %x\n", (*((volatile unsigned long *)(0xE000ED2C))));
	std_printf("DFSR = %x\n", (*((volatile unsigned long *)(0xE000ED30))));
	std_printf("AFSR = %x\n", (*((volatile unsigned long *)(0xE000ED3C))));
	std_printf("SCB_SHCSR = %x\n", SCB_SHCSR);
#endif

	while (1);
}

static void
generic_fault_handler(void)
{
	__asm__ volatile ("TST LR, #4");
	__asm__ volatile ("ITE EQ");
	__asm__ volatile ("MRSEQ R0, MSP");
	__asm__ volatile ("MRSNE R0, PSP");
	__asm__ volatile ("B dump_call_stack");
}

#define FLASH_OBL	MMIO32(0x1FF00000)

static void
simple_fault_handler(const char s)
{
	to_feed_iwdg = 0;

	std_printf("\n[%c - %dS]:\nHFSR = 0x%x, CFSR = 0x%x, MMFAR = 0x%x, BFAR = 0x%x, AFSR = 0x%x, DFSR = %x, SCB_SHCSR = 0x%x, FLASH_OPTR = 0x%x, OBL = 0x%x\n",
			s, uptime,
			SCB_HFSR, SCB_CFSR, SCB_MMFAR, SCB_BFAR, SCB_AFSR,
			(*((volatile unsigned long *)(0xE000ED30))), SCB_SHCSR, FLASH_OPTR, FLASH_OBL);
	while (1);
}

void
usage_fault_handler(void)
{
	simple_fault_handler('U');
}

void
hard_fault_handler(void)
{
	simple_fault_handler('H');
}

void
mem_manage_handler(void)
{
	simple_fault_handler('M');
}

/*
 * some parameter serialize/deserialze functions based on code from csp-client
 */
void
param_set_data(char *src, int len, char *dst, int max_len)
{
	int i, j, x, val;
	char c;

	if (src == NULL || len <= 0 || dst == NULL || max_len <= 0)
		return;

	for (i = 0, j = 0, x = 0; i < len && j < max_len; i ++) {
		c = src[i];

		if ('0' <= c && c <= '9') {
			val = c - '0';
		} else if ('a' <= c && c <= 'f') {
			val = 10 + c - 'a';
		} else if ('A' <= c && c <= 'F') {
			val = 10 + c - 'A';
		} else {
			ERROR_LOG("bad char 0x%x('%c') at pos #%d, just skip it ...\n", c, c, i);
			continue;
		}

		if (x == 0) {
			dst[j] = val;
			x = 1;
		} else {
			dst[j] = dst[j] << 4 | val;
			x = 0;
			j ++;
		}
	}
}

int
param_betoh(param_type_t type, void * item)
{
	switch (type) {
	case PARAM_UINT16:
	case PARAM_INT16:
	case PARAM_X16:
	{
		*(uint16_t *) item = csp_betoh16(*(uint16_t *) item);
		return 1;
	}
	case PARAM_UINT32:
	case PARAM_INT32:
	case PARAM_X32:
	{
		*(uint32_t *) item = csp_betoh32(*(uint32_t *) item);
		return 1;
	}
	case PARAM_UINT64:
	case PARAM_INT64:
	case PARAM_X64:
	{
		*(uint64_t *) item = csp_betoh64(*(uint64_t *) item);
		return 1;
	}
	case PARAM_FLOAT:
	{
		*(float *) item = csp_ntohflt(*(float *) item);
		return 1;
	}
	case PARAM_DOUBLE:
	{
		*(double *) item = csp_ntohdbl(*(double *) item);
		return 1;
	}
	default:
		return 0;
	}
}

int
param_htobe(param_type_t type, void * item)
{
	switch (type) {
	case PARAM_UINT16:
	case PARAM_INT16:
	case PARAM_X16:
	{
		*(uint16_t *) item = csp_htobe16(*(uint16_t *) item);
		return 1;
	}
	case PARAM_UINT32:
	case PARAM_INT32:
	case PARAM_X32:
	{
		*(uint32_t *) item = csp_htobe32(*(uint32_t *) item);
		return 1;
	}
	case PARAM_UINT64:
	case PARAM_INT64:
	case PARAM_X64:
	{
		*(uint64_t *) item = csp_htobe64(*(uint64_t *) item);
		return 1;
	}
	case PARAM_FLOAT:
	{
		*(float *) item = csp_htonflt(*(float *) item);
		return 1;
	}
	case PARAM_DOUBLE:
	{
		*(double *) item = csp_htondbl(*(double *) item);
		return 1;
	}
	default:
		return 0;
	}
}

int
param_serialize_item_direct(uint16_t addr, param_type_t type, int size, uint8_t * buf, uint16_t * pos, unsigned int maxlen, void * item, param_serializer_flags flags)
{
	void *tmp = NULL, *p;

	/* Check length */
	if (((flags & F_PACKED) ? 0 : sizeof(uint16_t)) + size + *pos > maxlen)
		return -1;

	/* Address */
	if (flags & F_TO_BIG_ENDIAN)
		addr = csp_htobe16(addr);

	/* Include address if not packed */
	if ((flags & F_PACKED) == 0) {
		if ((flags & F_DRY_RUN) == 0)
			memcpy(&buf[*pos], &addr, sizeof(uint16_t));
		*pos += sizeof(uint16_t);
	}

	p = item;
	if (flags & F_TO_BIG_ENDIAN) {
		/* Do not try conversion on strings */
		if (type != PARAM_DATA && type != PARAM_STRING) {
			/* Generate temporary stack space for the converted value:
			 * The memory we are serializing may not be touched, so we need a temporary variable of variable length */
			tmp = csp_buffer_get_data(size);
			memcpy(tmp, p, size);

			/* Convert endians - inplace - on the tmp variable */
			param_htobe(type, tmp);

			/* Overwrite the pointer to the data, with the tmp pointer */
			p = tmp;
		}

	}

	if ((flags & F_DRY_RUN) == 0)
		memcpy(&buf[*pos], p, size);
	*pos += size;
	if (tmp)
		csp_buffer_free_data(tmp);
	return sizeof(uint16_t) + size;
}

/* remote parameter service */
void
rparam_service_handler(csp_conn_t *c, csp_packet_t *p)
{
	rparam_query *q;

	if (c == NULL || p == NULL || p->data == NULL)
		return;

	q = (rparam_query *) p->data;
	switch (q->action) {
	case RPARAM_TABLE: /* 0x44 */
		handle_rparam_table_cmd(c, p, q);
		break;
	case RPARAM_GET: /* 0x00 */
		handle_rparam_get_cmd(c, p, q);
		break;
	case RPARAM_SET: /* 0xFF */
		handle_rparam_set_cmd(c, p, q);
		break;
	case RPARAM_COPY: /* 0x77 */
	case RPARAM_LOAD: /* 0x88 */
	case RPARAM_SAVE: /* 0x99 */
		handle_rparam_copy_cmd(c, p, q);
		break;
#if 0
	case RPARAM_SET_TO_FILE: /* 0xEE */
		break;
	case RPARAM_CLEAR: /* 0xAA */
		break;
	case RPARAM_REPLY: /* 0x55 */
		break;
#endif
	default:
		break;
	}

	csp_buffer_free(p);
}

/* based on code from book 'Beginning STM32' */
extern void vPortSVCHandler( void ) __attribute__ (( naked ));
extern void xPortPendSVHandler( void ) __attribute__ (( naked ));
extern void xPortSysTickHandler( void );

void sv_call_handler(void) {
	vPortSVCHandler();
}

void pend_sv_handler(void) {
	xPortPendSVHandler();
}

void sys_tick_handler(void) {
	xPortSysTickHandler();
}

/* check integrity of ram-based program */
#define MAGIC "TYUD"

#if SUPPORT_LFS
typedef void (*pFunction)(void);
pFunction Jump_To_Application;
extern lfs_t g_lfs;
extern struct lfs_config g_lfs_cfg;

/* check 'TYUD' + CRC32(4B) at the end of binary file */

/* return 0 if check failed
 * return 1 if OK
 */
static int
check_ram_file(char *p, int size)
{
	uint32_t crc, crc2;

	if (p == NULL || size <= 8)
		return 0;

	/* make sure that it's ram image */
	if ((*(volatile uint32_t *) (p + 4) & APP_ADDRESS) != APP_ADDRESS)
		return 0;

	if (memcmp(p + size - 8, MAGIC, 4) == 0) {
		/* it's new ram image with crc */
		memcpy(&crc2, p + size - 4, 4);
		crc = csp_crc32_memory((uint8_t *) p, size - 8);
		if (crc != crc2) /* crc mismatched */
			return 0;

		/* zero last 8 bytes */
		p[size - 8] = p[size - 7] = p[size - 6] = p[size - 5] = 0;
		p[size - 4] = p[size - 3] = p[size - 2] = p[size - 1] = 0;
	}

	return 1;
}

/* load ram-based program from file system */
void
boot_loader(char *f)
{
	int r, offset = 0, block;
	struct lfs_info lfs_info;
	lfs_file_t lfs_file;
	char *p = (char *)(APP_ADDRESS);

	if (f == NULL || f[0] == '\0')
		return;

	r = lfs_stat(&g_lfs, f, &lfs_info);
	if (r != 0) {
		ERROR_LOG("bl: stat(%s) return errcode %d\n", f, r);
		return;
	}

	if (lfs_info.size >= 140 * 1024) {
		ERROR_LOG("bl: size(%s) = #%d bytes > 140K\n", f, lfs_info.size);
		return;
	}

	r = lfs_file_open(&g_lfs, &lfs_file, f, LFS_O_RDONLY);
	if (r != 0) {
		ERROR_LOG("bl: open(%s) return errcode %d\n", f, r);
		return;
	}

#define LOAD_BLOCKSIZE 32768

	while (offset < lfs_info.size) {
		if ((lfs_info.size - offset) >= LOAD_BLOCKSIZE)
			block = LOAD_BLOCKSIZE;
		else
			block = lfs_info.size - offset;

		uv_hw_wdg_reset(); /* feed hardware dog every loop because it's short */
		r = lfs_file_read(&g_lfs, &lfs_file, p + offset, block);
		if (r != block) {
			ERROR_LOG("bl: read(%s, off #%d, #%d) return errcode %d\n", f, offset, block, r);
			break;
		} else {
			offset += block;
		}
	}

	uv_hw_wdg_reset(); /* feed hardware dog */
	lfs_file_close(&g_lfs, &lfs_file);
	if (offset >= lfs_info.size) { /* file read completed */
		/* Boot the application if it's valid. */
		if (check_ram_file(p, lfs_info.size)) {
			/* disable interrupt here */
			taskENTER_CRITICAL();
			/* Set vector table base address. */
			SCB_VTOR = APP_ADDRESS;
			/* Initialise master stack pointer. */
			__asm__ volatile("msr msp, %0"::"g"
					(*(volatile uint32_t *)APP_ADDRESS));
			/* Jump to application. */
			(*(void (**)())(APP_ADDRESS + 4))();
		} else {
			ERROR_LOG("bl: wrong %s binary magic word: 0x%x\n", f, ((*(volatile uint32_t *) (APP_ADDRESS + 4)) & APP_ADDRESS));
		}
	}

	if (g_conf_ptr) {
		g_conf_ptr->poweron_cnt ++;
		g_conf_ptr->swload_count = 0; /* reset ram load count to zero */
	}

	/* because we already fill 0x20000000 with file data which is not OK for current flash binary
	 * we need to reboot board to reload correct data from flash
	 */
	cpu_reset(); /* boot load failed, just wait for watchdog to reboot board */
}
#endif

#define SPI_CONFIG_COUNT	3

/* return 0 if OK
 * return 1 if failed
 */
int
read_params_config(void *p)
{
	int i;
	uint32_t crc32, crc;
	struct g_params_config *c;

	if (p == NULL)
		return 1;

	c = (struct g_params_config *)p;

	/* read from spi flash */
	for (i = 0; i < SPI_CONFIG_COUNT; i ++) { /* try max 3 sectors */
		if (spi_flash_read_buffer(c->buffer, c->base + i * FLASH_PAGESIZE, c->len)) {
			ERROR_LOG("read #%d data from spi flash 0x%x failed\n", c->len, c->base + i * FLASH_PAGESIZE);
			continue;
		}

		/* check crc here */
		memcpy(&crc32, c->buffer + c->len - 4, sizeof(crc32));
		crc = csp_crc32_memory(c->buffer, c->len - 4);

		if (crc32 != 0xffffffff && crc == crc32)
			break;
	}
	if (i == SPI_CONFIG_COUNT) {
		c->cfg_read_err_cnt ++;
		return 1;
	}

	memcpy(&crc32, c->buffer + c->len - 4, sizeof(crc32));
	crc = csp_crc32_memory(c->buffer, c->len - 4);

	if (crc != crc32) {
		ERROR_LOG("CRC 0x#%d != 0x%x\n", crc, crc32);
		c->cfg_read_err_cnt ++;
		return 1;
	}

	memcpy(c->orig, c->buffer, c->len); /* backup original data */

	for (i = 0; i < c->count; i ++) {
		if ((c->table[i].p != NULL) && (c->table[i].size > 0) &&
			(c->table[i].flags & PARAM_F_PERSIST)) {
			memcpy(c->table[i].p, c->buffer + c->table[i].addr, c->table[i].size);
		}
	}

	return 0;
}

#define PARAM_RETRY_CNT 3

/* return 0 if OK
 * return 1 if failed
 */
static int
flash_config_write_and_check(struct g_params_config *c, uint32_t addr)
{
	int i, j;

	if (c == NULL || (addr & FLASH_BASEADDR) == 0)
		return 1;

	for (j = 0; j < PARAM_RETRY_CNT; j ++) {
		i = flash_write_data(addr, c->buffer, c->len);
		if (i >= c->len) {
			/* read and check data */
			i = flash_read_data(addr, c->len, c->tmp);
			if ((i >= c->len) && (memcmp(c->tmp, c->buffer, c->len) == 0))
				break;
		}
		ERROR_LOG("#%d try to write and check #%d/#%d data to mcu flash 0x%x failed\n", j + 1, c->len, i, addr);
	}

	if (j == PARAM_RETRY_CNT) {
		c->cfg_write_err_cnt ++;
		return 1;
	} else {
		return 0;
	}
}

/* return 0 if OK
 * return 1 if failed
 */
static int
spi_config_write_and_check(struct g_params_config *c, uint32_t addr)
{
	int i;

	if (c == NULL)
		return 1;

	for (i = 0; i < PARAM_RETRY_CNT; i ++) {
		/* write & check data */
		if ((0 == spi_flash_erase_sector(addr)) &&
		    (0 == spi_flash_write_buffer(c->buffer, addr, c->len)) &&
		    (0 == spi_flash_read_buffer(c->tmp, addr, c->len)) &&
		    (0 == memcmp(c->buffer, c->tmp, c->len)))
				break;
		ERROR_LOG("write and check #%d data to spi flash 0x%x failed\n", c->len, addr);
	}

	if (i == PARAM_RETRY_CNT) {
		c->cfg_write_err_cnt ++;
		return 1;
	} else {
		return 0;
	}
}

/* return count of bad main param configs, return 0 if ALL OK */
int
save_params_config(void *p)
{
	int i, r = 0;
	uint32_t crc;
	struct g_params_config *c;

	if (p == NULL)
		return 1;

	param_iteration ++; /* increase 1 when saving parameters */
	c = (struct g_params_config *) p;
	memcpy(c->buffer, c->orig, c->len); /* read original data to buffer */
	for (i = 0; i < c->count; i ++) {
		if ((c->table[i].p != NULL) && (c->table[i].size > 0) &&
			(c->table[i].flags & PARAM_F_PERSIST)) {
			memcpy(c->buffer + c->table[i].addr, c->table[i].p, c->table[i].size);
		}
	}
	crc = csp_crc32_memory(c->buffer, c->len - 4);
	memcpy(c->buffer + c->len - 4, &crc, sizeof(crc));

	/* SPI FLASH */
	for (i = 0; i < SPI_CONFIG_COUNT; i ++) {
		if (spi_config_write_and_check(c, c->base + i * FLASH_PAGESIZE)) {
			/* something is wrong, don't proceed to write to next spi sector */
			std_printf("write #%d main config to spi flash 0x%x failed\n", i + 1, c->base + i * FLASH_PAGESIZE);
			r ++;
			continue;
		}
	}
	return r;
}

/* check CRC of main/backup config */
/* return 0 if CRC check failed
 * return 1 if OK
 */
int
check_params_config(void *p, int backup)
{
	int r, i, matched;
	struct g_params_config *c;
	uint32_t from;
	uint32_t crc, crc2;

	if (p == NULL)
		return 0;

	c = (struct g_params_config *) p;
	from = backup?c->backup_addr:c->base;

	for (i = 0, r = 1; i < SPI_CONFIG_COUNT; i ++) {
		if (spi_flash_read_buffer(c->buffer, from + i * FLASH_PAGESIZE, c->len)) {
			std_printf("read #%d data from spi flash 0x%x failed\n", c->len, from + i * FLASH_PAGESIZE);
			r = 0;
			continue;
		}

		crc = csp_crc32_memory(c->buffer, c->len - 4);
		memcpy(&crc2, c->buffer + c->len - 4, sizeof(crc2));

		if (crc2 != 0xffffffff && crc == crc2) {
			matched = 1;
		} else {
			matched = 0;
			r = 0;
		}
		std_printf("SPI %s CONFIG #%d -> %s, CALC CRC 0x%x %c= READ CRC2 0x%x\n", backup?"BACKUP":"MAIN", i, matched?"OK":"ERROR",
				crc, matched?'=':'!', crc2);
	}

	/* check flash backup config */
	if (backup) {
		for (i = 0; i < 2; i ++) {
			if (c->len == flash_read_data(flash_backup_cfg_addr + i * FLASH_PAGESIZE, c->len, c->buffer)) {
				crc = csp_crc32_memory(c->buffer, c->len - 4);
				memcpy(&crc2, c->buffer + c->len - 4, sizeof(crc2));

				if (crc2 != 0xffffffff && crc == crc2) {
					matched = 1;
				} else {
					matched = 0;
					r = 0;
				}
				std_printf("FLASH BACKUP CONFIG #%d -> %s, CALC CRC 0x%x %c= READ CRC2 0x%x\n", i, matched?"OK":"ERROR",
						crc, matched?'=':'!', crc2);
			} else {
				std_printf("FLASH BACKUP CONFIG #%d -> READ ERROR\n", i);
			}
		}
	}

	return r;
}

/* return 0 if OK
 * return 1 if FAILED
 */
int
restore_params_config(void *p, int is_full)
{
	int i;
	struct g_params_config *c;
	uint32_t crc, crc2;

	if (p == NULL)
		return 1;

	c = (struct g_params_config *) p;

	for (i = 0; i < SPI_CONFIG_COUNT; i ++) {
		if (spi_flash_read_buffer(c->buffer, c->backup_addr + i * FLASH_PAGESIZE, c->len)) {
			ERROR_LOG("read #%d data from spi flash 0x%x failed\n", c->len, c->backup_addr + i * FLASH_PAGESIZE);
			continue;
		}
		/* check crc here */
		memcpy(&crc2, c->buffer + c->len - 4, sizeof(crc2));
		crc = csp_crc32_memory(c->buffer, c->len - 4);

		if (crc2 != 0xffffffff && crc == crc2)
			break;
	}

	if (i == SPI_CONFIG_COUNT) {
		/* all three config copy are bad
		 * try to read from MCU flash at last
		 */
		if ((c->len != flash_read_data(flash_backup_cfg_addr, c->len, c->buffer)) &&
			(c->len != flash_read_data(flash_backup_cfg_addr + FLASH_PAGESIZE, c->len, c->buffer))) {
			/* MCU Flash are bad too */
			ERROR_LOG("read #%d data from mcu flash 0x%x/0x%x failed\n", c->len, flash_backup_cfg_addr, flash_backup_cfg_addr + FLASH_PAGESIZE);
			return 1;
		}
	}

	crc = csp_crc32_memory(c->buffer, c->len - 4);
	memcpy(&crc2, c->buffer + c->len - 4, sizeof(crc2));

	if (crc2 == 0xffffffff || crc != crc2) {
		/* backup config's crc error */
		return 1;
	}

	if (is_full == 1)
		memcpy(c->orig, c->buffer, c->len); /* backup original data */

	/* update BACKUP parameters only */
	for (i = 0; i < c->count; i ++) {
		if (c->table[i].p == NULL || c->table[i].size == 0)
		       continue;
		if (((is_full == 1) && (c->table[i].flags & PARAM_F_PERSIST)) ||
		    (c->table[i].flags & PARAM_F_BACKUP)) {
			memcpy(c->table[i].p, c->buffer + c->table[i].addr, c->table[i].size);
		}
	}

	return 0;
}

/* return count of bad backup param configs, return 0 if ALL OK */
int
backup_params_config(void *p)
{
	int i, r = 0;
	struct g_params_config *c;
	uint32_t crc;

	if (p == NULL)
		return 1;

	c = (struct g_params_config *) p;

	memset(c->buffer, 0, c->len);

	for (i = 0; i < c->count; i ++) {
		if (c->table[i].p == NULL || c->table[i].size == 0)
		       continue;
		if (c->table[i].flags & (PARAM_F_BACKUP | PARAM_F_PERSIST)) {
			/* copy all persistent/backup parameters to backup address */
			memcpy(c->buffer + c->table[i].addr, c->table[i].p, c->table[i].size);
		}
	}
	crc = csp_crc32_memory(c->buffer, c->len - 4);
	memcpy(c->buffer + c->len - 4, &crc, sizeof(crc));

	for (i = 0; i < SPI_CONFIG_COUNT; i ++) {
		if (spi_config_write_and_check(c, c->backup_addr + i * FLASH_PAGESIZE)) {
			std_printf("write #%d backup config to spi flash 0x%x failed\n", i + 1, c->backup_addr + i * FLASH_PAGESIZE);
			r ++;
		}
	}

	/* write to flash backup config now */
	for (i = 0; i < 2; i ++) {
		if (flash_config_write_and_check(c, flash_backup_cfg_addr + i * FLASH_PAGESIZE)) {
			std_printf("write #%d backup config to mcu flash 0x%x failed\n", SPI_CONFIG_COUNT + 1 + i, flash_backup_cfg_addr + i * FLASH_PAGESIZE);
			r ++;
		}
	}

	return r;
}

void
print_hex(uint8_t *src, int len)
{
	int i;

	if (len <= 0 || src == NULL)
		return;
	std_printf("0x");
	for (i = 0; i < len; i ++)
		std_printf("%02X", src[i]);
	std_printf("\n");
}

void
print_ios(uint8_t *src, int len)
{
	int i, first = 1;

	if (len <= 0 || src == NULL)
		return;

	for (i = 0; i < len; i ++) {
		if ((src[i] & 0x80) == 0x80) {
			if (first) {
				std_printf("ON: %d", (src[i] & 0x7F));
				first = 0;
			} else {
				std_printf("/%d", (src[i] & 0x7F));
			}
		}
	}

	first = 1;
	for (i = 0; i < len; i ++) {
		if ((src[i] != 0) && (src[i] & 0x80) == 0) {
			if (first) {
				std_printf("; OFF: %d", (src[i] & 0x7F));
				first = 0;
			} else {
				std_printf("/%d", (src[i] & 0x7F));
			}
		}
	}
	std_printf("\n");
}

#if SUPPORT_LFS
static int
flash_memory_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size)
{
	int r;

	if (g_conf_ptr == NULL)
		return - EACCES;

	if ((g_conf_ptr->flash_fs_pos + (block * c->block_size + off) + size) >= g_conf_ptr->base)
		return - EACCES;

	r = flash_read_data(g_conf_ptr->flash_fs_pos + (block * c->block_size + off), (int) size, (char *) buffer);

	return (r == ((int) size) ? 0: -EIO);
}

static int
flash_memory_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size)
{
	int r;

	if (g_conf_ptr == NULL)
		return - EACCES;

	if ((g_conf_ptr->flash_fs_pos + (block * c->block_size + off) + size) >= g_conf_ptr->base)
		return - EACCES;

	/* don't disable interrupt, otherwise will cause freeze */
	r = flash_write_data(g_conf_ptr->flash_fs_pos + (block * c->block_size + off), (char *)buffer, (int) size);

	return (r == ((int) size) ? 0: -EIO);
}

static int
flash_memory_erase(const struct lfs_config *c, lfs_block_t block)
{
	int r;

	if (g_conf_ptr == NULL)
		return - EACCES;

	if ((g_conf_ptr->flash_fs_pos + block * c->block_size) >= g_conf_ptr->base)
		return - EACCES;

	r = flash_reset_page(g_conf_ptr->flash_fs_pos + block * c->block_size);

	return (r == 0? 0: -EIO);
}

static int
flash_memory_sync(const struct lfs_config *c)
{
	return 0;
}

void
flash_lfs_init(struct lfs_config *p, int flash_block_count)
{
	if (p == NULL)
		return;

	p->read  = flash_memory_read;
	p->prog  = flash_memory_prog;
	p->erase = flash_memory_erase;
	p->sync  = flash_memory_sync;

	p->read_size = FLASH_PAGESIZE;
	p->prog_size = FLASH_PAGESIZE;
	p->block_size = FLASH_PAGESIZE;
	p->block_count = flash_block_count > 0? flash_block_count: 120;
	p->lookahead = FLASH_PAGESIZE;

	p->read_buffer = NULL;
	p->prog_buffer = NULL;
	p->lookahead_buffer = NULL;
	p->file_buffer = NULL;
}

static int
spi_flash_memory_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size)
{
	int r;

	if (g_conf_ptr == NULL)
		return - EACCES;

	r = spi_flash_read_buffer((uint8_t *)buffer, (uint32_t) (g_conf_ptr->flash_fs_pos + (block * c->block_size + off)), (int) size);

	return (r == 0 ? 0: -EIO);
}

static int
spi_flash_memory_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size)
{
	int r;

	if (g_conf_ptr == NULL)
		return - EACCES;

	r = spi_flash_write_buffer((uint8_t *)buffer, (uint32_t) (g_conf_ptr->flash_fs_pos + (block * c->block_size + off)), (int) size);

	return (r == 0 ? 0: -EIO);
}

static int
spi_flash_memory_erase(const struct lfs_config *c, lfs_block_t block)
{
	int r;

	if (g_conf_ptr == NULL)
		return - EACCES;

	r = spi_flash_erase_sector(g_conf_ptr->flash_fs_pos + block * c->block_size);

	return (r == 0? 0: -EIO);
}

static int
spi_flash_memory_sync(const struct lfs_config *c)
{
	return 0;
}

void
spi_flash_lfs_init(struct lfs_config *p, int flash_block_count)
{
	if (p == NULL)
		return;

	p->read  = spi_flash_memory_read;
	p->prog  = spi_flash_memory_prog;
	p->erase = spi_flash_memory_erase;
	p->sync  = spi_flash_memory_sync;

	p->read_size = FLASH_PAGESIZE;
	p->prog_size = FLASH_PAGESIZE;
	p->block_size = FLASH_PAGESIZE;
	p->block_count = flash_block_count > 0? flash_block_count: 8128; /* max 8192, but we reserve some sectores */
	p->lookahead = FLASH_PAGESIZE;

	p->read_buffer = NULL;
	p->prog_buffer = NULL;
	p->lookahead_buffer = NULL;
	p->file_buffer = NULL;
}
#endif

void
handle_rparam_table_cmd(csp_conn_t *c, csp_packet_t *p, rparam_query *q)
{
	param_table_t *t = NULL;
	int i, j;
	uint16_t offset = 0, end = 0xff;

	if (g_table_offset == NULL || g_conf_ptr == NULL || c == NULL || p == NULL || p->data == NULL || q == NULL || q->action != RPARAM_TABLE || q->mem >= MAX_RPARAM_TABLE)
		return;

	offset = g_table_offset[q->mem].start;
	end = g_table_offset[q->mem].end;

	if (offset < 0xffff) {
		/* main table 0, uv rx table 1, telemetry table 4, uv tx table 5 */
		t = (param_table_t *) csp_buffer_get_data(g_conf_ptr->count * sizeof(param_table_t));
		j = 0;

		if (t == NULL)
			return;

		for (i = 0; i < g_conf_ptr->count; i ++) {
			if (g_conf_ptr->table[i].addr >= offset && g_conf_ptr->table[i].addr < end) {
				t[j].addr = g_conf_ptr->table[i].addr - offset;
				t[j].type = g_conf_ptr->table[i].type;
				t[j].size = g_conf_ptr->table[i].size;
				t[j].flags = PARAM_F_PERSIST;
				t[j].count = 1;
				strncpy(t[j].name, g_conf_ptr->table[i].name, MAX_PARAM_NAME_LEN);
				j ++;
			}
		}

		csp_sfp_send(c, t, j * sizeof(param_table_t), 180, 1000); /* takes count of sizeof(sfp_header_t) and kiss header */
	}

	if (t) csp_buffer_free_data(t);
}

void
handle_rparam_get_cmd(csp_conn_t *c, csp_packet_t *p, rparam_query *q)
{
	int i, j;
	uint16_t pos = 0, start = 0;
	void *m;
	csp_packet_t *t;
	rparam_query *r;
	uint16_t offset = 0, end = 0;

	if (g_table_offset == NULL || g_conf_ptr == NULL || c == NULL || p == NULL || p->data == NULL || q == NULL || q->action != RPARAM_GET || q->mem >= MAX_RPARAM_TABLE)
		return;

	offset = g_table_offset[q->mem].start;
	end = g_table_offset[q->mem].end;

	if (offset < 0xffff) {
		/* main table 0, uv rx table 1, telemetry table 4, uv tx table 5 */
		q->length = csp_ntoh16(q->length);
		if (q->length == 0) {
			/* get full table */
			j = g_conf_ptr->len + g_conf_ptr->count * sizeof(uint16_t) + 16; /* 16, takes accout of rparam_query */
			m = csp_buffer_get_data(j);
			if (m == NULL)
				return;
			pos = start = offsetof(rparam_query, packed);
			for (i = 0; i < g_conf_ptr->count; i ++) {
				if (g_conf_ptr->table[i].addr >= offset && g_conf_ptr->table[i].addr < end) {
					param_serialize_item_direct(g_conf_ptr->table[i].addr - offset, g_conf_ptr->table[i].type,
						g_conf_ptr->table[i].size, m, &pos, j, g_conf_ptr->table[i].p, F_TO_BIG_ENDIAN);
				}
			}

		} else {
			/* get single name */
			q->addr[0] = csp_ntoh16(q->addr[0]);
			for (i = 0; i < g_conf_ptr->count; i ++) {
				if (g_conf_ptr->table[i].addr == (q->addr[0] + offset) && g_conf_ptr->table[i].addr < end)
					break;
			}

			if (i == g_conf_ptr->count)
				return;
			/* we found match */
			j = g_conf_ptr->table[i].size + sizeof(uint16_t) + 16; /* 16, takes accout of rparam_query */
			m = csp_buffer_get_data(j);
			if (m == NULL)
				return;
			pos = start = offsetof(rparam_query, packed);
			param_serialize_item_direct(g_conf_ptr->table[i].addr - offset, g_conf_ptr->table[i].type,
				g_conf_ptr->table[i].size, m, &pos, j, g_conf_ptr->table[i].p, F_TO_BIG_ENDIAN);
		}

		if (pos > start) {
			t = csp_buffer_get(CSP_MTU);
			if (t) {
				r = (rparam_query *)m;
				r->mem = q->mem;
				r->total = r->seq = 0;
				r->action = RPARAM_REPLY;
				r->checksum = csp_hton16(0xB00B);
				r->length = csp_hton16(pos - start);

				t->length = pos;
				memcpy(t->data, m, pos);
				if (!csp_send(c, t, 1000)) { /* SEND FAILED */
					csp_buffer_free(t);
				}
			}
		}
		csp_buffer_free_data(m);
	}
}

void
handle_rparam_set_cmd(csp_conn_t *c, csp_packet_t *p, rparam_query *q)
{
	int i;
	uint16_t pos = 0, addr = 0;
	csp_packet_t *t;
	uint16_t offset = 0, end = 0;

	if (g_table_offset == NULL || g_conf_ptr == NULL || c == NULL || p == NULL || p->data == NULL || q == NULL || q->action != RPARAM_SET || q->mem >= MAX_RPARAM_TABLE)
		return;

	i = g_conf_ptr->count;
	offset = g_table_offset[q->mem].start;
	end = g_table_offset[q->mem].end;

	if (offset < 0xffff) {
		/* main table 0, uv rx table 1, telemetry table 4, uv tx table 5 */
		q->length = csp_ntoh16(q->length);
		while (pos < q->length) {
			/* Read address from data */
			memcpy(&addr, q->packed + pos, sizeof(uint16_t));
			/* assuming big endian */
			addr = csp_betoh16(addr);
			addr += offset;
			pos += sizeof(uint16_t);
			for (i = 0; i < g_conf_ptr->count; i ++) {
				if ((g_conf_ptr->table[i].flags != 0) && ((g_conf_ptr->table[i].flags & PARAM_F_READONLY) == 0) && (g_conf_ptr->table[i].addr == addr) && (g_conf_ptr->table[i].addr < end))
					break;
			}

			if (i == g_conf_ptr->count)
				break;
			/* we found match */
			memcpy(g_conf_ptr->table[i].p, q->packed + pos, g_conf_ptr->table[i].size);
			if (g_conf_ptr->table[i].type != PARAM_DATA && g_conf_ptr->table[i].type != PARAM_STRING) {
				param_betoh(g_conf_ptr->table[i].type, g_conf_ptr->table[i].p);
			}
			pos += g_conf_ptr->table[i].size;
		}

		t = csp_buffer_get(CSP_MTU);
		if (t) {
			t->length = 1;
			t->data[0] = (i == g_conf_ptr->count)? RPARAM_ERROR : RPARAM_SET_OK;
			if (!csp_send(c, t, 1000)) { /* SEND FAILED */
				csp_buffer_free(t);
			}
		}
	}
}

void
handle_rparam_copy_cmd(csp_conn_t *c, csp_packet_t *p, rparam_query *q)
{
	csp_packet_t *t;

	if (c == NULL || p == NULL || g_conf_ptr == NULL || p->data == NULL || q == NULL)
		return;

	if (q->action != RPARAM_COPY && q->action != RPARAM_SAVE && q->action != RPARAM_LOAD)
		return;

	if (q->mem < MAX_RPARAM_TABLE) {
		t = csp_buffer_get(CSP_MTU);
		if (t == NULL)
			return;
		t->length = 1;
		if (q->action == RPARAM_SAVE) {
			t->data[0] = (0 == save_params_config(g_conf_ptr)) ? RPARAM_SAVE_OK : RPARAM_ERROR;
		} else if (q->action == RPARAM_COPY) {
			t->data[0] = RPARAM_COPY_OK;
		} else if (q->action == RPARAM_LOAD) {
			t->data[0] = (0 == read_params_config(g_conf_ptr)) ? RPARAM_LOAD_OK : RPARAM_ERROR;
		}

		if (!csp_send(c, t, 1000)) { /* SEND FAILED */
			csp_buffer_free(t);
		}
	}
}

static float auto_us_cnt = 2.396; /* previous auto calibrate on uv board */

/* delay 0.1 microsecond (1e-7 seconds, 0.1us) */
void
my_delay_us(uint32_t us)
{
	uint32_t i;

	us = (uint32_t) (auto_us_cnt * us);
	for (i = 0; i < us; i ++)
		__asm__("nop");
}

#define TEST_DELAY_US_CNT 10000000

void
auto_calibrate_us_cnt(void)
{
	TickType_t start, end;

	float auto_cnt;

	start = xTaskGetTickCount();
	my_delay_us(TEST_DELAY_US_CNT);
	end = xTaskGetTickCount();

	auto_cnt = (end - start) * 10000.0 / (TEST_DELAY_US_CNT * 1.0 * auto_us_cnt);

	auto_us_cnt = 1.0 / auto_cnt;

#if 0
	start = xTaskGetTickCount();
	delay_auto_us(TEST_DELAY_US_CNT);
	end = xTaskGetTickCount();

	end = (end - start) * 10000;
	ERROR_LOG("diff of delay_auto_us(%d) is %d us, %d, auto_us_cnt * 100 = %d\n", TEST_DELAY_US_CNT, (int) (end - TEST_DELAY_US_CNT), end, (int) (auto_us_cnt * 10000.0));
#endif
}

uint8_t
get_boot_cause(void)
{
	uint8_t r;

	r = (RCC_CSR >> 24) & 0xff;

	RCC_CSR |= RCC_CSR_RMVF; /* clear reset flag */

	if (r & 0x20) {
		/* iwdg reset */
		return 1;
	} else if (r & 0x10) {
		/* software reset */
		return 2;
	} else if (r & 0x40) {
		/* window watchdog reset */
		return 3;
	} else if (r & 0x80) {
		/* lower power reset */
		return 4;
	} else if (r & 0x01) {
		/* firewall reset */
		return 5;
	} else if (r & 0x02) {
		/* option byte loader reset */
		return 6;
	} else if (r & 0x0C) {
		/* power off/on, hardware watchdog */
		return 7;
	} else {
		/* other reset */
		return 0xff;
	}
}

const char *
get_param_type(param_type_t type)
{
	if (type > PARAM_BOOL)
		return "ERR";
	return &(param_type_name[type][0]);
}

void
list_params_config(struct g_params_config *p, int start, int end, uint8_t list_flag, uint8_t asterisk_flag)
{
	int r, i, j;
	char tmp[64], c, *name;
	float f = 0.0;
	void *src = NULL;

	if (p == NULL)
		return;

	end = (end <= p->count)?end:p->count;
	start = (start >= 0)?start:0;

	for (i = start; i < end; i ++) {
		if ((list_flag != 0) && ((p->table[i].flags & list_flag) == 0))
			continue;

		r = 0;
		j = 0;
		c = (p->table[i].flags & asterisk_flag)?'*':' ';

		src = (list_flag == 0)?p->table[i].p:(p->buffer + p->table[i].addr);

		if (src == NULL)
			continue;

		name = (char *) get_param_type(p->table[i].type);
		std_printf("  0x%03X %c %-14s %-4s ", p->table[i].addr, c, p->table[i].name, name);
		switch(p->table[i].type) {
		case PARAM_X8:
			r = *((uint8_t *) src);
			std_printf("0x%02x\n", r);
			break;
		case PARAM_X16:
			r = *((uint16_t *) src);
			std_printf("0x%04x\n", r);
			break;
		case PARAM_X32:
			r = *((uint32_t *) src);
			std_printf("0x%08x\n", r);
			break;
		case PARAM_UINT8:
			r = *((uint8_t *) src);
			j = 1;
			break;
		case PARAM_UINT16:
			r = *((uint16_t *) src);
			j = 1;
			break;
		case PARAM_UINT32:
			r = *((uint32_t *) src);
			j = 1;
			break;
		case PARAM_INT8:
			r = *((int8_t *) src);
			j = 2;
			break;
		case PARAM_INT16:
			r = *((int16_t *) src);
			j = 2;
			break;
		case PARAM_INT32:
			r = *((int32_t *) src);
			j = 2;
			break;
		case PARAM_DOUBLE:
			j = 3;
			f = *((double *) src);
			break;
		case PARAM_FLOAT:
			j = 3;
			f = *((float *) src);
			break;
		case PARAM_STRING:
			memcpy(tmp, src, p->table[i].size);
			tmp[p->table[i].size] = '\0';
			std_printf("\"%s\"\n", tmp);
			break;
		case PARAM_DATA:
			memcpy(tmp, src, p->table[i].size);
			tmp[p->table[i].size] = '\0';
			if (p->table[i].flags & PARAM_F_EPS_IOS)
				print_ios((uint8_t *)tmp, p->table[i].size);
			else
				print_hex((uint8_t *)tmp, p->table[i].size);
			break;
		case PARAM_BOOL:
			r = *((uint8_t *) src);
			std_printf("%s\n", r?"true":"false");
			break;
		default:
			break;
		}

		if (j == 1) {
			/* unsigned int */
			std_printf("%u\n", r);
		} else if (j == 2) {
			/* signed int */
			std_printf("%d\n", r);
		} else if (j == 3) {
			/* float */
			std_printf("%.2f\n", f);
		}
	}
}

/* return 0 if OK
 * return 1 if failed
 */
int
set_params_value(struct g_params_config *p, char *name, char *value)
{
	int i, j;
	double d;
	float f;
	uint8_t t8;
	uint16_t t16;

	if (p == NULL || name == NULL || value == NULL || name[0] == '\0' || value[0] == '\0')
		return 1;

	for (i = 0; i < p->count; i ++) {
		if (strcasecmp(p->table[i].name, name) == 0)
			break;
	}

	if (i == p->count)
		return 1;

	switch (p->table[i].type) {
	case PARAM_UINT8:
	case PARAM_INT8:
	case PARAM_X8:
		t8 = atoi(value) & 0xff;
		memcpy(p->table[i].p, &t8, p->table[i].size);
		break;
	case PARAM_UINT16:
	case PARAM_INT16:
	case PARAM_X16:
		t16 = atoi(value) & 0xffff;
		memcpy(p->table[i].p, &t16, p->table[i].size);
		break;
	case PARAM_UINT32:
	case PARAM_INT32:
	case PARAM_X32:
		j = atoi(value) & 0xffffffff;
		memcpy(p->table[i].p, &j, p->table[i].size);
		break;
	case PARAM_FLOAT:
		f = strtof(value, NULL);
		memcpy(p->table[i].p, &f, p->table[i].size);
		break;
	case PARAM_DOUBLE:
		d = strtod(value, NULL);
		memcpy(p->table[i].p, &d, p->table[i].size);
		break;
	case PARAM_STRING:
		j = strlen(value);
		memset(p->table[i].p, 0, p->table[i].size);
		memcpy(p->table[i].p, value, j);
		break;
	case PARAM_DATA:
		/* convert hex string to hex data */
		memset(p->table[i].p, 0, p->table[i].size);
		param_set_data(value, strlen(value), p->table[i].p, p->table[i].size);
		break;
	case PARAM_BOOL:
		if (strcasecmp(value, "true") == 0)
			t8 = 1;
		else
			t8 = 0;
		memcpy(p->table[i].p, &t8, p->table[i].size);
		break;
	default:
		break;
	}

	return 0;
}

#if SUPPORT_LFS
char *
lfs_err_str(int r)
{
	switch (r) {
	case LFS_ERR_OK:
		return "OK";
	case LFS_ERR_IO:
		return "ERR_IO";
	case LFS_ERR_CORRUPT:
		return "CORRUPT";
	case LFS_ERR_NOENT:
		return "NOENT";
	case LFS_ERR_EXIST:
		return "EXIST";
	case LFS_ERR_NOTDIR:
		return "NOTDIR";
	case LFS_ERR_ISDIR:
		return "ISDIR";
	case LFS_ERR_NOTEMPTY:
		return "NOTEMPTY";
	case LFS_ERR_BADF:
		return "BADF";
	case LFS_ERR_FBIG:
		return "TOOBIG";
	case LFS_ERR_INVAL:
		return "INVAL";
	case LFS_ERR_NOSPC:
		return "NOSPACE";
	case LFS_ERR_NOMEM:
		return "NOMEM";
	default:
		return "UNKNOWN";
	}
}

void
lfs_operation(int argc, char **argv)
{
	int r, j, i;
	lfs_dir_t lfs_dir;
	struct lfs_info lfs_info;
	lfs_file_t lfs_file;
	char tmp[64];

	if (argc < 2 || argv == NULL)
		return;

	if (in_ftp_operation == 1) {
		std_printf("IN FTP OPERATION\n");
		return;
	}

	/* lfs mount/read/write/rm command */
	if (strcasecmp(argv[1], "mount") == 0) {
		if (fs_mounted == 1) {
			std_printf("LFS ALREADY MOUNTED\n");
		} else {
			r = lfs_mount(&g_lfs, &g_lfs_cfg);
			if (r) { /* init fs */
				std_printf("MOUNT LFS RETURN %d, FORMAT...\n", r);
				r = lfs_format(&g_lfs, &g_lfs_cfg);
				std_printf("FORMAT LFS RETURN %s\n", lfs_err_str(r));
				r = lfs_mount(&g_lfs, &g_lfs_cfg);
			}

			if (r == 0) {
				std_printf("LFS MOUNTED ON AT 0x%x\n", g_conf_ptr->flash_fs_pos);
				fs_mounted = 1;
			} else {
				std_printf("LFS MOUNTE RETURN ERROR %s AT 0x%x\n", lfs_err_str(r), g_conf_ptr->flash_fs_pos);
				fs_mounted = 0;
			}
		}
	} else if (strcasecmp(argv[1], "format") == 0) {
		if (fs_mounted == 0) {
			r = lfs_format(&g_lfs, &g_lfs_cfg);
			std_printf("FORMAT LFS RETURN %s\n", lfs_err_str(r));
		} else {
			std_printf("LFS IS MOUNTED, PLEASE UMOUNT IT FIRST\n");
		}
	} else if (strcasecmp(argv[1], "rm") == 0 && argc >= 3) {
		if (fs_mounted == 0) {
			std_printf("LFS NOT MOUNTED\n");
			return;
		}
		r = lfs_remove(&g_lfs, argv[2]);
		std_printf("LFS REMOVE(%s) -> %s\n", argv[2], lfs_err_str(r));
	} else if (strcasecmp(argv[1], "read") == 0 && argc >= 3) {
		if (fs_mounted == 0) {
			std_printf("LFS NOT MOUNTED\n");
			return;
		}
		r = lfs_file_open(&g_lfs, &lfs_file, argv[2], LFS_O_RDONLY);
		if (r == 0) {
			r = lfs_file_read(&g_lfs, &lfs_file, tmp, 63);
			if (r <= 0) {
				std_printf("LFS READ(%s, 63) -> FAILED, %s\n", argv[2], lfs_err_str(r));
			} else {
				tmp[r] = '\0';
				std_printf("LFS READ(%s, 63) -> OK,  %d, %s\n", argv[2], r, tmp);
			}

			r = lfs_file_close(&g_lfs, &lfs_file);
		} else {
			std_printf("LFS OPEN(%s) RETURN %s\n", argv[2], lfs_err_str(r));
		}
	} else if (strcasecmp(argv[1], "write") == 0 && argc >= 4) {
		if (fs_mounted == 0) {
			std_printf("LFS NOT MOUNTED\n");
			return;
		}
		r = lfs_file_open(&g_lfs, &lfs_file, argv[2], LFS_O_RDWR | LFS_O_CREAT | LFS_O_TRUNC);
		if (r == 0) {
			j = strlen(argv[3]);
			r = lfs_file_write(&g_lfs, &lfs_file, argv[3], j);
			if (r <= 0) {
				std_printf("LFS WRITE(%s, %s, %d) -> FAILED %s\n", argv[2], argv[3], j, lfs_err_str(r));
			} else {
				std_printf("LFS WRITE(%s, %s, %d) -> OK %d\n", argv[2], argv[3], j, r);
			}

			r = lfs_file_close(&g_lfs, &lfs_file);
		} else {
			std_printf("LFS OPEN(%s) RETURN %s\n", argv[2], lfs_err_str(r));
		}
	} else if ((strcasecmp(argv[1], "mkdir") == 0) && (argc >= 3)) {
		if (fs_mounted == 0) {
			std_printf("LFS NOT MOUNTED\n");
			return;
		}
		r = lfs_mkdir(&g_lfs, argv[2]);
		std_printf("MKDIR(%s) -> %s\n", argv[2], lfs_err_str(r));
	} else if (strcasecmp(argv[1], "ls") == 0) {
		if (fs_mounted == 0) {
			std_printf("LFS NOT MOUNTED\n");
			return;
		}
		r = lfs_dir_open(&g_lfs, &lfs_dir, argc >= 3?argv[2]:"/");
		if (r == 0) {
			i = 1;
			while (lfs_dir_read(&g_lfs, &lfs_dir, &lfs_info) == 1) {
				if (strcmp(lfs_info.name, ".") == 0 || strcmp(lfs_info.name, "..") == 0)
					continue;
				std_printf("#%d: %s, SIZE #%06d, %s\n", i ++, (lfs_info.type == LFS_TYPE_REG ?"REG":"DIR"), lfs_info.size, lfs_info.name);
			}
			lfs_dir_close(&g_lfs, &lfs_dir);
		} else {
			std_printf("LFS LS %s RETURN %s\n", argc >= 3?argv[2]:"/", lfs_err_str(r));
		}
	} else if (strcasecmp(argv[1], "umount") == 0) {
		if (fs_mounted == 0) {
			std_printf("LFS NOT MOUNTED\n");
		} else {
			r = lfs_unmount(&g_lfs);
			std_printf("LFS UNMOUNTED -> %s\n", lfs_err_str(r));
			fs_mounted = 0;
		}
	}
}
#endif

void
check_bin_file(int is_ram, int size, uint32_t *crc, uint32_t *crc2)
{
	char *p;

	if (size <= 8 || crc == NULL || crc2 == NULL)
		return;

	if (is_ram) {
		p = (char *)(APP_ADDRESS);
		/* don't check magic string because it was zero by boot_loader(). */
		*crc = *crc2 = csp_crc32_memory((uint8_t *) p, size);
	} else {
		p = (char *)(FLASH_BASEADDR);
		memcpy(crc2, p + size + 4, 4);
		if (memcmp(p + size, MAGIC, 4) == 0) {
			*crc = csp_crc32_memory((uint8_t *) p, size);
		} else {
			*crc = -100;
		}
	}
}

/* return 0 if OK
 * return 1 if failed
 */
char *
read_file(char *path, int *len)
{
#if SUPPORT_LFS
	lfs_file_t f;
	int r;
	char *data = NULL;

	if (path == NULL)
		return NULL;

	r = lfs_file_open(&g_lfs, &f, path, LFS_O_RDWR);
	if (r != 0)
		return NULL;
	/* get file size */
	r = lfs_file_seek(&g_lfs, &f, 0, LFS_SEEK_END);

	lfs_file_seek(&g_lfs, &f, 0, LFS_SEEK_SET);
	data = csp_buffer_get_data(r + 1);
	if (data == NULL) {
		lfs_file_close(&g_lfs, &f);
		return NULL;
	}

	if (r != lfs_file_read(&g_lfs, &f, data, r)) {
		lfs_file_close(&g_lfs, &f);
		csp_buffer_free_data(data);
		return NULL;
	}

	lfs_file_close(&g_lfs, &f);
	if (len) *len = r;
	return data;
#else
	return NULL;
#endif
}

void
spi_test_all(int argc, char **argv)
{
	int i, j, k, n, r;
	/* check lfs mount status first */
	if (spi_flash_status != MAGIC_55) {
		ERROR_LOG("SPI FLASH STATUS IS NOT OK -> 0x%x\n", spi_flash_status);
		return;
	}

#if SUPPORT_LFS
	if (fs_mounted == 1) {
		r = lfs_unmount(&g_lfs);
		if (r != LFS_ERR_OK) {
			ERROR_LOG("LFS IS MOUNTED, UNMOUNT IT RETURN -> %s\n", lfs_err_str(r));
			return;
		} else {
			ERROR_LOG("LFS IS MOUNTED, UNMOUNT IT RETURN -> %s\n", lfs_err_str(r));
			fs_mounted = 0;
		}
	}
#endif

	n = (argc >= 2)?atoi(argv[1]):20;
	for (i = SPI_RESERVED_SECTOR, k = 0; i < n; i ++) {
		memset(flash_memory_ptr, 0x55, FLASH_PAGESIZE);
		r = spi_flash_erase_sector(i * FLASH_PAGESIZE);
		if (r == 0) {
			r = spi_flash_write_buffer((uint8_t *) flash_memory_ptr, (uint32_t) (i * FLASH_PAGESIZE), FLASH_PAGESIZE);
			if (r == 0) {
				memset(flash_memory_ptr, 0x0, FLASH_PAGESIZE);
				r = spi_flash_read_buffer((uint8_t *) flash_memory_ptr, (uint32_t) (i * FLASH_PAGESIZE), FLASH_PAGESIZE);
				if (r == 0) {
					for (j = 0; j < FLASH_PAGESIZE; j ++) {
						if (flash_memory_ptr[j] != 0x55) {
							ERROR_LOG("data[%d] is 0x%x, should be 0x55!\n", j, flash_memory_ptr[j]);
							k ++;
							break;
						}
					}
					if (j == FLASH_PAGESIZE)
						ERROR_LOG("TESTING %d SECTOR -> OK\n", i);
				} else {
					ERROR_LOG("READ FROM SPI SECTOR %d RETURN CODE %d\n", i, r);
					k ++;
				}
			} else {
				k ++;
				ERROR_LOG("WRITE 0x55 TO SPI SECTOR %d RETURN CODE %d\n", i, r);
			}
		} else {
			k ++;
			ERROR_LOG("ERASE SPI SECTOR %d RETURN CODE %s\n", i, r);
		}
	}
	std_printf("SPI FULL TEST %d SECTOR, ERROR COUNT -> %d\n", n - SPI_RESERVED_SECTOR, k);
}

/* return 0 if OK
 * return 1 if FAILED
 */
int
write_check_spiflash(int i)
{
	int j;

	/* check lfs mount status first */
	if (spi_flash_status != MAGIC_55)
		return 1;

	if (i < (SPI_RESERVED_SECTOR - 1))
		return 0;

#if SUPPORT_LFS
	if (fs_mounted == 1)
		return 0;
#endif

	memset(flash_memory_ptr, 0x55, FLASH_PAGESIZE);

	if (1 == spi_flash_erase_sector(i * FLASH_PAGESIZE))
		return 1;

	if (1 == spi_flash_write_buffer((uint8_t *) flash_memory_ptr, (uint32_t) (i * FLASH_PAGESIZE), FLASH_PAGESIZE))
		return 1;

	memset(flash_memory_ptr, 0x0, FLASH_PAGESIZE);
	if (1 == spi_flash_read_buffer((uint8_t *) flash_memory_ptr, (uint32_t) (i * FLASH_PAGESIZE), FLASH_PAGESIZE))
		return 1;

	for (j = 0; j < FLASH_PAGESIZE; j ++) {
		if (flash_memory_ptr[j] != 0x55)
			break;
	}

	return (j != FLASH_PAGESIZE);
}

void
print_mpu_vars(void)
{
	extern uint32_t __privileged_functions_start__[];
	extern uint32_t __privileged_functions_end__[];
	extern uint32_t __FLASH_segment_start__[];
	extern uint32_t __FLASH_segment_end__[];
	extern uint32_t __privileged_data_start__[];
	extern uint32_t __privileged_data_end__[];
	extern uint32_t __SRAM_segment_start__[];
	extern uint32_t __SRAM_segment_end__[];
	extern uint32_t __syscalls_flash_start__[];
	extern uint32_t __syscalls_flash_end__[];

	std_printf("FLASH: 0x%x -> 0x%x, SRAM: 0x%x -> 0x%x\n",
			(uint32_t) __FLASH_segment_start__, (uint32_t) __FLASH_segment_end__,
			(uint32_t) __SRAM_segment_start__, (uint32_t) __SRAM_segment_end__);

	std_printf("PRIVILEGED FUNC: 0x%x -> 0x%x, SIZE 0x%x\n",
			(uint32_t) __privileged_functions_start__, (uint32_t) __privileged_functions_end__,
			(uint32_t) __privileged_functions_end__ - (uint32_t) __privileged_functions_start__);

	std_printf("PRIVILEGED DATA: 0x%x -> 0x%x, SIZE 0x%x\n",
			(uint32_t) __privileged_data_start__, (uint32_t) __privileged_data_end__,
			(uint32_t) __privileged_data_end__ - (uint32_t) __privileged_data_start__);

	std_printf("SYSCALL FLASH: 0x%x -> 0x%x, SIZE 0x%x\n",
			(uint32_t) __syscalls_flash_start__, (uint32_t) __syscalls_flash_end__,
			(uint32_t) __syscalls_flash_end__ - (uint32_t) __syscalls_flash_start__);
}

void
update_option_bytes(uint32_t data)
{
	flash_wait_for_last_operation();

	if (FLASH_CR & FLASH_CR_OPTLOCK) {
		flash_unlock();
		flash_unlock_option_bytes();
	}

	FLASH_OPTR = data;
	FLASH_CR |= FLASH_CR_OPTSTRT;
	flash_wait_for_last_operation();
	flash_lock();
}

#define SRAM_PE		(1 << 24)
#define NSWBOOT0	(1 << 26)
#define NBOOT0		(1 << 27)

void
enable_sram2_parity_check(void)
{
	uint32_t t;

	t = FLASH_OPTR;

	if (t & SRAM_PE) {
		/* SRAM PARITY CHECK DISBALED, ENABLE IT NOW */
		t &= ~(SRAM_PE);
		update_option_bytes(t);
	}
}

uint32_t
random_(void)
{
	uint32_t r;
	if (false == rng_get_random(&r))
		return xTaskGetTickCount();
	else
		return r;
}

void
setup_rng(void)
{
	rcc_set_clock48_source(RCC_CCIPR_CLK48SEL_MSI);
	rcc_periph_clock_enable(RCC_RNG);
	rng_interrupt_disable();
	rng_enable(); /* ENABLE RNG NOW */
}

void
clock_set_time(csp_timestamp_t *utc)
{
	if (utc)
		g_utc_time = utc->tv_sec;
}

void
clock_get_time(csp_timestamp_t *utc)
{
	if (utc) {
		utc->tv_sec = g_utc_time;
		utc->tv_nsec = (xTaskGetTickCount() % pdMS_TO_TICKS(1000)) * 1000000;
	}
}
