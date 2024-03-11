/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <inttypes.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

#include "time.h"
#include "sys/time.h"

#include "slider.h"

#define SPP_TAG "SPP_ACCEPTOR_DEMO"
#define SLIDER_TAG "RNR_SLIDER"
#define SPP_SERVER_NAME "SPP_SERVER"
#define EXAMPLE_DEVICE_NAME "ESP_SPP_ACCEPTOR"
#define SPP_SHOW_DATA 1
#define SPP_SHOW_SPEED 0
#define SPP_SHOW_MODE SPP_SHOW_DATA // SPP_SHOW_SPEED    /*Choose show mode: show data or speed*/

static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;
static const bool esp_spp_enable_l2cap_ertm = true;

static struct timeval time_new, time_old;
static long data_num = 0;

static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

static SliderState d_state;

static char *bda2str(uint8_t * bda, char *str, size_t size)
{
    if (bda == NULL || str == NULL || size < 18) {
        return NULL;
    }

    uint8_t *p = bda;
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
            p[0], p[1], p[2], p[3], p[4], p[5]);
    return str;
}

static void print_speed(void)
{
    float time_old_s = time_old.tv_sec + time_old.tv_usec / 1000000.0;
    float time_new_s = time_new.tv_sec + time_new.tv_usec / 1000000.0;
    float time_interval = time_new_s - time_old_s;
    float speed = data_num * 8 / time_interval / 1000.0;
    ESP_LOGI(SPP_TAG, "speed(%fs ~ %fs): %f kbit/s" , time_old_s, time_new_s, speed);
    data_num = 0;
    time_old.tv_sec = time_new.tv_sec;
    time_old.tv_usec = time_new.tv_usec;
}

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    char bda_str[18] = {0};

    switch (event) {
    case ESP_SPP_INIT_EVT:
        if (param->init.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
            esp_spp_start_srv(sec_mask, role_slave, 0, SPP_SERVER_NAME);
        } else {
            ESP_LOGE(SPP_TAG, "ESP_SPP_INIT_EVT status:%d", param->init.status);
        }
        break;
    case ESP_SPP_DISCOVERY_COMP_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
        break;
    case ESP_SPP_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_OPEN_EVT");
        break;
    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CLOSE_EVT status:%d handle:%"PRIu32" close_by_remote:%d", param->close.status,
                 param->close.handle, param->close.async);
        break;
    case ESP_SPP_START_EVT:
        if (param->start.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT handle:%"PRIu32" sec_id:%d scn:%d", param->start.handle, param->start.sec_id,
                     param->start.scn);
            esp_bt_dev_set_device_name(EXAMPLE_DEVICE_NAME);
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        } else {
            ESP_LOGE(SPP_TAG, "ESP_SPP_START_EVT status:%d", param->start.status);
        }
        break;
    case ESP_SPP_CL_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT");
        break;
    case ESP_SPP_DATA_IND_EVT:
#if (SPP_SHOW_MODE == SPP_SHOW_DATA)
        /*
         * We only show the data in which the data length is less than 128 here. If you want to print the data and
         * the data rate is high, it is strongly recommended to process them in other lower priority application task
         * rather than in this callback directly. Since the printing takes too much time, it may stuck the Bluetooth
         * stack and also have a effect on the throughput!
         */
        ESP_LOGI(SPP_TAG, "ESP_SPP_DATA_IND_EVT len:%d handle:%"PRIu32,
                 param->data_ind.len, param->data_ind.handle);
        // d_state.handle = param->data_ind.handle;
        ESP_LOGI(SLIDER_TAG, "PARSE RESULT = %d", slider_task((char*)(param->data_ind.data), param->data_ind.len, &d_state));
        if (param->data_ind.len < 128 && !(d_state.data_ready)) {
            d_state.req_len = param->data_ind.len;
            memcpy(d_state.request, param->data_ind.data, param->data_ind.len); 
            d_state.data_ready = true;          
        }   
#else
        gettimeofday(&time_new, NULL);
        data_num += param->data_ind.len;
        if (time_new.tv_sec - time_old.tv_sec >= 3) {
            print_speed();
        }
#endif
        break;
    case ESP_SPP_CONG_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CONG_EVT");
        break;
    case ESP_SPP_WRITE_EVT:
        d_state.write_done = true;
        ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT");
        break;
    case ESP_SPP_SRV_OPEN_EVT:
        d_state.handle = param->srv_open.handle;
        d_state.is_connected = true;
        ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT status:%d handle:%"PRIu32", rem_bda:[%s]", param->srv_open.status,
                 param->srv_open.handle, bda2str(param->srv_open.rem_bda, bda_str, sizeof(bda_str)));
        gettimeofday(&time_old, NULL);
        break;
    case ESP_SPP_SRV_STOP_EVT:
        d_state.is_connected = false;
        ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_STOP_EVT");
        break;
    case ESP_SPP_UNINIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_UNINIT_EVT");
        break;
    default:
        break;
    }
}

void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    char bda_str[18] = {0};

    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT:{
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(SPP_TAG, "authentication success: %s bda:[%s]", param->auth_cmpl.device_name,
                     bda2str(param->auth_cmpl.bda, bda_str, sizeof(bda_str)));
        } else {
            ESP_LOGE(SPP_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT:{
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit) {
            ESP_LOGI(SPP_TAG, "Input pin code: 0000 0000 0000 0000");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        } else {
            ESP_LOGI(SPP_TAG, "Input pin code: 1234");
            esp_bt_pin_code_t pin_code;
            pin_code[0] = '2';
            pin_code[1] = '3';
            pin_code[2] = '9';
            pin_code[3] = '6';
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        }
        break;
    }

#if (CONFIG_EXAMPLE_SSP_ENABLED == true)
    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %"PRIu32, param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%"PRIu32, param->key_notif.passkey);
        break;
    case ESP_BT_GAP_KEY_REQ_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
        break;
#endif

    case ESP_BT_GAP_MODE_CHG_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_MODE_CHG_EVT mode:%d bda:[%s]", param->mode_chg.mode,
                 bda2str(param->mode_chg.bda, bda_str, sizeof(bda_str)));
        break;

    default: {
        ESP_LOGI(SPP_TAG, "event: %d", event);
        break;
    }
    }
    return;
}

static void slider_comm_task(void * data)
{
    SliderState * state = (SliderState *)(data);
    uint8_t cnt = 0;
    while (1) {
        state->cur_x = (float)(state->cur_x_st) / (float)(STEPS_PER_MM);
        state->cur_v = (float)(state->cur_v_st) / (float)(STEPS_PER_MM);
        if (state->data_ready) {
            printf("incoming %ld, %s\n", state->req_len, state->request); // Kludge
            memset(state->request, 0, 128); // Kludge
            state->data_ready = false;
            char answer[128];
            sprintf(answer, "hep %3.1f, %3.1f, %3.1f\n", state->task_x, state->task_v, state->task_a); // Kludge
            esp_spp_write(state->handle, strlen(answer), (uint8_t*)(answer));
            state->write_done = false;

            // state->cmd = SLDR_CMD_NONE;
            switch (state->cmd)
            {
                case SLDR_CMD_START:
                    if (state->is_running) break;
                    slider_enable();
                    vTaskDelay( pdMS_TO_TICKS(10));
                    if (slider_start(state) != 0) {
                        slider_stop(state);
                        // slider_disable();
                    }
                    //state->cmd = SLDR_CMD_NONE;
                    break;
                case SLDR_CMD_STOP:
                    slider_stop(state);
                    slider_disable();
                    //state->cmd = SLDR_CMD_NONE;
                    break;
                case SLDR_CMD_HOLD:
                    slider_stop(state);
                    slider_enable();
                    break;
                case SLDR_CMD_ORIG:
                    slider_stop(state);
                    state->cur_x_st = 0;
                    state->cur_v_st = 0;
                    break;
                default:
                    break;
            }
            state->cmd = SLDR_CMD_NONE;
        } else {
            cnt++;
            if (cnt % 10 == 0 && state->write_done && state->is_connected) {
                char msg[128];
                sprintf(msg, "X%3.1f S%3.1f\n", state->cur_x, state->cur_v);
                esp_spp_write(state->handle, strlen(msg), (uint8_t*)(msg));
                state->write_done = false;
            }
            vTaskDelay( pdMS_TO_TICKS(10));
        }
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    char bda_str[18] = {0};
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    slider_init(&d_state);
    // xTaskCreate(slider_comm_task, "slidercomm", 2048, &d_state, configMAX_PRIORITIES - 1, NULL); // kludge

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
#if (CONFIG_EXAMPLE_SSP_ENABLED == false)
    bluedroid_cfg.ssp_en = false;
#endif
    if ((ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s initialize bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s enable bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s gap register failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp register failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    esp_spp_cfg_t bt_spp_cfg = {
        .mode = esp_spp_mode,
        .enable_l2cap_ertm = esp_spp_enable_l2cap_ertm,
        .tx_buffer_size = 0, /* Only used for ESP_SPP_MODE_VFS mode */
    };
    if ((ret = esp_spp_enhanced_init(&bt_spp_cfg)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp init failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

#if (CONFIG_EXAMPLE_SSP_ENABLED == true)
    /* Set default parameters for Secure Simple Pairing */
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
#endif

    /*
     * Set default parameters for Legacy Pairing
     * Use variable pin, input pin code when pairing
     */
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);

    ESP_LOGI(SPP_TAG, "Own address:[%s]", bda2str((uint8_t *)esp_bt_dev_get_address(), bda_str, sizeof(bda_str)));

    xTaskCreate(slider_comm_task, "slidercomm", 2048, &d_state, configMAX_PRIORITIES - 1, NULL);
}

static bool xISR(gptimer_handle_t timer, const gptimer_alarm_event_data_t * event_data, void * obj)
{
    SliderState * state = (SliderState *)(obj);

    GPIO.out_w1ts = (1ULL << SLDR_STEP_PIN);

    state->cur_step++;

    if (state->dir)
        state->cur_x_st++;
    else
        state->cur_x_st--;

    if (state->cur_step >= state->task_step)
    {
        slider_stop(state);
        return 0;
    }

    if (state->cur_step <= state->task_acc_steps) {
        state->accel_time += state->cur_period;
        state->cur_v_st = state->task_a_st * state->accel_time / TIMER_FREQ; 
        uint64_t fa = TIMER_FREQ / state->task_a_st;
        uint64_t ft = state->accel_time <= (fa / (TIMER_MAX_PERIOD/TIMER_FREQ)) ? TIMER_MAX_PERIOD / fa : TIMER_FREQ / state->accel_time;
        state->cur_period = fa * ft;
        //state->cur_v_st = TIMER_FREQ / state->cur_period;
        //sqrt(fabs( 2.0 * (state->task_a) * (float)(state->cur_step) / (float)(STEPS_PER_MM) ));
    } else if (state->cur_step >= (state->task_step - state->task_acc_steps)) {
        state->accel_time -= state->cur_period;
        state->cur_v_st = state->task_a_st * state->accel_time / TIMER_FREQ; 
        uint64_t fa = TIMER_FREQ / state->task_a_st;
        uint64_t ft = state->accel_time <= (fa / (TIMER_MAX_PERIOD/TIMER_FREQ)) ? TIMER_MAX_PERIOD / fa : TIMER_FREQ / state->accel_time;
        state->cur_period = fa * ft;
        //state->cur_v_st = TIMER_FREQ / state->cur_period;
    	//float d_step = (float)((state->task_step)-(state->cur_step));
    	//float v_sqr = fabs( 2.0*(state->task_a)*d_step / ((float)(STEPS_PER_MM)) );
    	//float new_v = sqrt(v_sqr);
    } else {
        state->cur_v_st = TIMER_FREQ / state->cur_period;
        // state->cur_period = state->cur_v_st == 0 ? TIMER_FREQ / 1000 : TIMER_FREQ / state->cur_v_st;
    }

    GPIO.out_w1tc = (1ULL << SLDR_STEP_PIN);
    //s_alarm_cfg.alarm_count = event_data->alarm_value + state->cur_period;
    gptimer_alarm_config_t alarm_cfg = {.alarm_count = event_data->alarm_value + state->cur_period,};
    gptimer_set_alarm_action(timer, &alarm_cfg);
    return 1;
}
