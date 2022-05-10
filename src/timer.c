#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <time.h>
#include <string.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include "tianyi.h"

#define TIMER_CYCLE 50 /* 50 ms */
#define MAX_TIMER_ACTION_NUM 32

TickType_t last_timer_tick = 0, current_timer_tick = 0;

struct timer_action {
	uint32_t tick, interval, id;
	timer_action_handler f;
	void *p;
	uint32_t ts;
	uint32_t crc;
	uint8_t is_repeat, is_rewind, is_delay, is_fast;
};

int delay_action_size = 0, max_timer_idx = 0;

static struct timer_action action_list[MAX_TIMER_ACTION_NUM];

/* return 0 if failed
 * return 1 if OK
 */
int
register_timer_ts_action_handler(uint32_t timer_id, uint32_t interval, int is_repeat, timer_action_handler f, void *p, uint32_t ts, uint32_t crc)
{
	int i, j = -1;
	struct timer_action *t = NULL;

	if (timer_id == 0) return 0;

	if (interval == 0 && ts == 0)
		return 0;

	/* try to check duplicated delay cmd */
	if (ts > 0 && crc > 0 && timer_id >= DELAY_TIMER_ID_MIN && timer_id <= DELAY_TIMER_ID_MAX) {
		for (i = 0; i < MAX_TIMER_ACTION_NUM; i ++) {
			t = action_list + i;
			if (t->ts == ts && t->crc == crc) {
				/* duplicated command found */
				return 0;
			}
		}
	}

	/* new time ts action */
	for (i = 0; i < MAX_TIMER_ACTION_NUM; i ++) {
		t = action_list + i;
		if (t->id == timer_id) {
			/* we may encounter dupliate timer ID */
			/* but we just update the list */
			j = i;
			break;
		} else if (t->id == 0) {
			if (j == -1) j = i;
		}
	}

	if (j != -1) {
		t = action_list + j;
		t->id = timer_id;
		t->f = f; t->p = p;
		t->interval = interval; t->is_repeat = is_repeat;
		if (interval > 0)
			t->tick = last_timer_tick + interval;
		else
			t->tick = 0;
		t->ts = ts;
		t->crc = crc;

		if (t->tick < last_timer_tick)
			t->is_rewind = 1;
		else
			t->is_rewind = 0;

		if (t->id >= DELAY_TIMER_ID_MIN && t->id <= DELAY_TIMER_ID_MAX) {
			delay_action_size ++;
			t->is_delay = MAGIC_55;
		}

		j ++;
		if (j > max_timer_idx)
			max_timer_idx = j;
		return 1;
	}
	return 0;
}

/* return 0 if failed
 * return 1 if OK
 */
int
unregister_timer_action_handler(uint32_t timer_id)
{
	int i;
	struct timer_action *t = NULL;

	if (timer_id == 0) return 0;

	for (i = 0; i < MAX_TIMER_ACTION_NUM; i ++) {
		t = action_list + i;
		if (t->id == timer_id) {
			if (t->is_delay == MAGIC_55 && delay_action_size > 0)
				delay_action_size --;
			i ++;
			if (i == max_timer_idx)
				max_timer_idx --;
			memset(t, 0, sizeof(struct timer_action));
			return 1;
		}
	}
	return 0;
}

/* return 0 if failed
 * return 1 if OK
 */
int
unregister_timer_section_action_handler(uint32_t from_id, uint32_t to_id)
{
	int i, j;
	struct timer_action *t = NULL;

	if ((from_id == 0) || (to_id == 0)) return 0;

	for (i = MAX_TIMER_ACTION_NUM - 1; i >= 0; i --) {
		t = action_list + i;
		if ((t->id >= from_id) && (t->id <= to_id)) {
			if (t->is_delay == MAGIC_55 && delay_action_size > 0)
				delay_action_size --;
			j = i + 1;
			if (j == max_timer_idx)
				max_timer_idx --;
			memset(t, 0, sizeof(struct timer_action));
		}
	}
	return 1;
}

void
recalibrate_timer_action_vars(void)
{
	int i, j = 0, k = 0;
	struct timer_action *t = NULL;

	for (i = 0; i < MAX_TIMER_ACTION_NUM; i ++ ){
		t = action_list + i;
		if (t->id > 0) {
			k = i;
			if ((t->id >= DELAY_TIMER_ID_MIN) && (t->id <= DELAY_TIMER_ID_MAX)) {
				j ++;
			}
		}
	}
	delay_action_size = j;
	max_timer_idx = k + 1;
}

extern void cleanup_csp_uses_list(void);

void
uv_timer_service(void *unused)
{
	uint32_t tick_count = 0;
	int i, r;
	struct timer_action *t;
	uint32_t cur_ts;

	memset(&action_list, 0, sizeof(struct timer_action) * MAX_TIMER_ACTION_NUM);

	delay_action_size = 0, max_timer_idx = 0;
	cur_ts = 0;

	vTaskDelay(pdMS_TO_TICKS(1000)); /* delay 1 second */
	last_timer_tick = 0;
	while (1) {
		current_timer_tick = xTaskGetTickCount();
		/* run every 0.1 seconds */
		tick_count += TIMER_CYCLE;

		if (current_timer_tick < last_timer_tick) {
			/* uint32_t overflow */
			for (i = 0; i < MAX_TIMER_ACTION_NUM; i ++) {
				/* reset all actions' is_rewind flag */
				action_list[i].is_rewind = 0;
			}
		}

		last_timer_tick = current_timer_tick;

		/* begin to loop timer action list */
		for (i = 0; i < max_timer_idx /* MAX_TIMER_ACTION_NUM */; i ++) {
			t = action_list + i;
			if (t->id > 0 && (t->is_rewind == 0) &&
				(((t->tick > 0) && (t->tick <= last_timer_tick)) || ((t->ts > 0) && (cur_ts >= t->ts)))) {
				/* we need to run this action now */
				if (t->f != NULL)
					r = t->f(t->p);
				else
					r = 0;

				if (t->is_repeat && r != 0) {
					/* callback return 1, to stop repeat now */
					t->is_repeat = 0;
				}

				if (t->is_repeat && t->ts == 0 && t->interval > 0) {
					t->tick = last_timer_tick + t->interval; /* we need to check uint32_t rewind */
					if (t->tick < last_timer_tick)
						t->is_rewind = 1;
				} else {
					if (t->is_delay == MAGIC_55 && delay_action_size > 0)
						delay_action_size --;
					memset(t, 0, sizeof(struct timer_action));
				}
			}
		}

		/* doublecheck max_timer_idx's value here */
		if ((max_timer_idx < MAX_TIMER_ACTION_NUM) && (action_list[max_timer_idx].id != 0)) {
			max_timer_idx ++;
		} else if ((max_timer_idx > 0) && (action_list[max_timer_idx - 1].id == 0)) {
			max_timer_idx --;
		}

		if (tick_count >= 2000) { /* every 2 seconds */
			cleanup_csp_uses_list();
			tick_count = 0;
		}

		vTaskDelayUntil(&current_timer_tick, pdMS_TO_TICKS(TIMER_CYCLE));
	}
}
