#ifndef _SLIDER_H_
#define _SLIDER_H_

#include <math.h>
#include "stdint.h"
#include "stdio.h"
#include "string.h"
#include "driver/gptimer.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/gpio_ll.h"

#define SLDR_STEP_PIN 14
#define SLDR_DIR_PIN 12
#define SLDR_EN_PIN 13
#define TIMER_FREQ 1000000.0

//static uint32_t SLDR_PRECS = 64;
//static uint32_t SLDR_GEAR_L = 60;
static uint32_t STEPS_PER_MM = 160;//round(SLDR_PRECS*200 / SLDR_GEAR_L);

static gptimer_handle_t s_timer_handle;
static gptimer_alarm_config_t s_alarm_cfg = {.reload_count = 0};

typedef enum 
{
	SLDR_CMD_NONE,
	SLDR_CMD_START,
	SLDR_CMD_STOP
} SliderCommands;

typedef struct _slider_state
{
	bool is_connected;
	bool write_done;
	bool data_ready;
	char request[128];
	char * answer;
	uint32_t req_len;
	int cnt;
	uint32_t handle;

	float task_x;
	float task_v;
	float task_a;
	uint32_t task_step;
	uint32_t task_acc_steps;

	float cur_x;
	float cur_v;
	float cur_a;
	uint32_t cur_step;
	uint32_t cur_period;

	bool dir;

	uint8_t cmd;
} SliderState;

volatile SliderState d_state;

static inline void slider_stop()
{
	gptimer_stop(s_timer_handle);
	gptimer_disable(s_timer_handle);
	gpio_set_level((gpio_num_t)SLDR_STEP_PIN, 0);
}

static inline void slider_enable()
{
	gpio_set_level((gpio_num_t)SLDR_EN_PIN, 0);
}
static inline void slider_disable()
{
	gpio_set_level((gpio_num_t)SLDR_EN_PIN, 1);
}

static inline int slider_start()
{
	d_state.cur_step = 0;
	if (d_state.task_x == d_state.cur_x)
		return 1;
	if (d_state.task_x < d_state.cur_x) {
		d_state.dir = false;
	} else {
		d_state.dir = true;
	}

	d_state.task_a = fabs(d_state.task_a);
	d_state.task_step = round(fabs(d_state.task_x - d_state.cur_x) * (float)(STEPS_PER_MM));
	d_state.task_acc_steps = round( (d_state.task_v*d_state.task_v / (2 * d_state.task_a))* (float)(STEPS_PER_MM) );
	if ( d_state.task_acc_steps*2 > d_state.task_step )
		d_state.task_acc_steps = d_state.task_step/2;
	d_state.task_v = sqrt(fabs((float)(d_state.task_acc_steps) * 2 * d_state.task_a / (float)(STEPS_PER_MM) ));

	d_state.cur_period = round(TIMER_FREQ / (d_state.task_v * (float)(STEPS_PER_MM)/d_state.task_acc_steps));

	//slider_enable();
	//vTaskDelay(pdMS_TO_TICKS(10));
	gpio_set_level((gpio_num_t)SLDR_DIR_PIN, d_state.dir);

	s_alarm_cfg.alarm_count = d_state.cur_period;
    gptimer_set_alarm_action(s_timer_handle, &s_alarm_cfg);
	gptimer_enable(s_timer_handle);
    gptimer_start(s_timer_handle);
    return 0;
}

static bool xISR(struct gptimer_t * timer, const gptimer_alarm_event_data_t * data, void * obj);

static inline void slider_init()
{
	//uint64_t mask = (1ULL << SLDR_STEP_PIN) | (1ULL << SLDR_DIR_PIN) | (1ULL << SLDR_EN_PIN);
    gpio_config_t gpio_conf = {
        // config gpios
        .pin_bit_mask = (1ULL << SLDR_STEP_PIN) | (1ULL << SLDR_DIR_PIN) | (1ULL << SLDR_EN_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    // set the gpios as per gpio_conf
    ESP_ERROR_CHECK(gpio_config(&gpio_conf));

    gptimer_config_t timer_conf = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = TIMER_FREQ,
    };

    timer_conf.flags.intr_shared = false;

    ESP_ERROR_CHECK(gptimer_new_timer(&timer_conf, &s_timer_handle));

    gptimer_event_callbacks_t cb_group;
    cb_group.on_alarm = xISR;
    s_alarm_cfg.flags.auto_reload_on_alarm = 1;
    gptimer_register_event_callbacks(s_timer_handle, &cb_group, &d_state);

    d_state.cur_x = 0;
    d_state.cur_v = 0;
    d_state.cur_a = 0;
    d_state.cur_step = 0;
    d_state.cur_period = 0;

    d_state.task_x = 50.0; // kludge
    d_state.task_v = 50.0;
    d_state.task_a = 50.0;

    d_state.cmd = SLDR_CMD_NONE;
}

static inline int slider_task(char * msg, int len)
{
	//int i_ch = 0;
	if (len <= 0) return len;
	switch (msg[0])
	{
		case 'X':
			d_state.cmd = SLDR_CMD_START;
			return sscanf(msg, "X%f S%f A%f\n", &(d_state.task_x), &(d_state.task_v), &(d_state.task_a));
		case 'C':
			d_state.cmd = SLDR_CMD_STOP;
			break;
		default:
			break;
	}
	return 0;
}

#endif // _SLIDER_H_