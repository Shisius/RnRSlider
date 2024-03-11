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
#define TIMER_MAX_PERIOD 1000000000.0
#define START_SPEED 

//static uint32_t SLDR_PRECS = 64;
//static uint32_t SLDR_GEAR_L = 60;
static float STEPS_PER_MM = 213.33333;//round(SLDR_PRECS*200 / SLDR_GEAR_L);

static gptimer_handle_t s_timer_handle;
static gptimer_alarm_config_t s_alarm_cfg = {.reload_count = 0};

typedef enum 
{
	SLDR_CMD_NONE,
	SLDR_CMD_START,
	SLDR_CMD_STOP,
	SLDR_CMD_HOLD,
	SLDR_CMD_ORIG
} SliderCommands;

typedef struct _slider_state
{
	bool is_connected;
	bool write_done;
	bool data_ready;
	bool is_running;

	char request[128];
	// char * answer;
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

	int cur_x_st;
	int cur_v_st;
	int task_a_st;

	uint32_t cur_step;
	uint32_t cur_period;
	int64_t accel_time;

	bool dir;

	uint8_t cmd;
} SliderState;

//volatile SliderState d_state;

// static inline uint32_t steps2ticks(uint32_t steps)
// {
// 	return 
// }

static inline void slider_stop(SliderState * state)
{
	if (state->is_running) {
		gptimer_stop(s_timer_handle);
		gptimer_disable(s_timer_handle);
	}
	gpio_set_level((gpio_num_t)SLDR_STEP_PIN, 0);
	state->is_running = false;
}

static inline void slider_enable()
{
	gpio_set_level((gpio_num_t)SLDR_EN_PIN, 0);
}
static inline void slider_disable()
{
	gpio_set_level((gpio_num_t)SLDR_EN_PIN, 1);
}

static inline int slider_start(SliderState * d_state)
{
	d_state->cur_step = 0;
	d_state->accel_time = 0;
	if (d_state->task_x == d_state->cur_x)
		return 1;
	if (d_state->task_x < d_state->cur_x) {
		d_state->dir = false;
	} else {
		d_state->dir = true;
	}

	d_state->task_a = fabs(d_state->task_a);
	d_state->task_a_st = d_state->task_a * STEPS_PER_MM;
	if (d_state->task_a_st <= 0) d_state->task_a_st = 1;
	d_state->task_step = round(fabs(d_state->task_x - d_state->cur_x) * (float)(STEPS_PER_MM));
	d_state->task_acc_steps = round( (d_state->task_v*d_state->task_v / (2 * d_state->task_a))* (float)(STEPS_PER_MM) );
	if ( d_state->task_acc_steps*2 > d_state->task_step )
		d_state->task_acc_steps = d_state->task_step/2;
	d_state->task_v = sqrt(fabs((float)(d_state->task_acc_steps) * 2 * d_state->task_a / (float)(STEPS_PER_MM) ));

	d_state->cur_period = round(TIMER_FREQ / (10 * d_state->task_v * (float)(STEPS_PER_MM)/d_state->task_acc_steps));

	//slider_enable();
	//vTaskDelay(pdMS_TO_TICKS(10));
	gpio_set_level((gpio_num_t)SLDR_DIR_PIN, d_state->dir);

	s_alarm_cfg.alarm_count = d_state->cur_period;
    ESP_ERROR_CHECK(gptimer_set_alarm_action(s_timer_handle, &s_alarm_cfg));
	ESP_ERROR_CHECK(gptimer_enable(s_timer_handle));
    ESP_ERROR_CHECK(gptimer_start(s_timer_handle));
    d_state->is_running = true;
    return 0;
}

static bool xISR(gptimer_handle_t timer, const gptimer_alarm_event_data_t * data, void * obj);

static inline void slider_init(SliderState * d_state)
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
    // s_alarm_cfg.flags.auto_reload_on_alarm = 1;
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(s_timer_handle, &cb_group, d_state));

    d_state->cur_x = 0;
    d_state->cur_v = 0;
    d_state->cur_a = 0;
    d_state->cur_step = 0;
    d_state->cur_period = 0;

    d_state->task_x = 50.0; // kludge
    d_state->task_v = 50.0;
    d_state->task_a = 10.0;

    d_state->cur_x_st = 0;
    d_state->cur_v_st = 0;
    d_state->task_a_st = d_state->task_a * STEPS_PER_MM;
    d_state->accel_time = 0;

    d_state->cmd = SLDR_CMD_NONE;
    d_state->is_running = false;

    d_state->is_connected = false;
    d_state->write_done = true;
    d_state->data_ready = false; 
    d_state->cnt = 0;

    slider_enable();
}

static inline int slider_task(char * msg, int len, SliderState * d_state)
{
	//int i_ch = 0;
	if (len <= 0) return len;
	switch (msg[0])
	{
		case 'X':
			d_state->cmd = SLDR_CMD_START;
			return sscanf(msg, "X%f S%f A%f\n", &(d_state->task_x), &(d_state->task_v), &(d_state->task_a));
		case 'C':
			d_state->cmd = SLDR_CMD_STOP;
			break;
		case 'H':
			d_state->cmd = SLDR_CMD_HOLD;
			break;
		case 'O':
			d_state->cmd = SLDR_CMD_ORIG;
			break;
		default:
			break;
	}
	return 0;
}

#endif // _SLIDER_H_

