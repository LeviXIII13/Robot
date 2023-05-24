#include "HC_SR04.h"

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_check.h"
#include "soc/rtc.h"
#include "driver/mcpwm.h"
#include "driver/gpio.h"

const char *HCSR04_TAG = "HC_SR04";

static xQueueHandle Forward_cap_queue;
static xQueueHandle Left_cap_queue;
static xQueueHandle Right_cap_queue;

static uint32_t forward_cap_val_begin_of_sample = 0;
static uint32_t forward_cap_val_end_of_sample = 0;

static uint32_t left_cap_val_begin_of_sample = 0;
static uint32_t left_cap_val_end_of_sample = 0;

static uint32_t right_cap_val_begin_of_sample = 0;
static uint32_t right_cap_val_end_of_sample = 0;


static bool Forward_echo_isr_handler(mcpwm_unit_t mcpwm, mcpwm_capture_channel_id_t cap_sig, const cap_event_data_t *edata,
                                     void *arg)
{
    // calculate the interval in the ISR,
    // so that the interval will be always correct even when cap_queue is not handled in time and overflow.
    BaseType_t high_task_wakeup = pdFALSE;
    if (edata->cap_edge == MCPWM_POS_EDGE)
    {
        // store the timestamp when pos edge is detected
        forward_cap_val_begin_of_sample = edata->cap_value;
        forward_cap_val_end_of_sample = forward_cap_val_begin_of_sample;
    }
    else
    {
        forward_cap_val_end_of_sample = edata->cap_value;
        uint32_t pulse_count = forward_cap_val_end_of_sample - forward_cap_val_begin_of_sample;
        // send measurement back though queue
        xQueueSendFromISR(Forward_cap_queue, &pulse_count, &high_task_wakeup);
    }
    return high_task_wakeup == pdTRUE;
}

static bool Left_echo_isr_handler(mcpwm_unit_t mcpwm, mcpwm_capture_channel_id_t cap_sig, const cap_event_data_t *edata,
                                     void *arg)
{
    // calculate the interval in the ISR,
    // so that the interval will be always correct even when cap_queue is not handled in time and overflow.
    BaseType_t high_task_wakeup = pdFALSE;
    if (edata->cap_edge == MCPWM_POS_EDGE)
    {
        // store the timestamp when pos edge is detected
        left_cap_val_begin_of_sample = edata->cap_value;
        left_cap_val_end_of_sample = left_cap_val_begin_of_sample;
    }
    else
    {
        left_cap_val_end_of_sample = edata->cap_value;
        uint32_t pulse_count = left_cap_val_end_of_sample - left_cap_val_begin_of_sample;
        // send measurement back though queue
        xQueueSendFromISR(Left_cap_queue, &pulse_count, &high_task_wakeup);
    }
    return high_task_wakeup == pdTRUE;
}

static bool Right_echo_isr_handler(mcpwm_unit_t mcpwm, mcpwm_capture_channel_id_t cap_sig, const cap_event_data_t *edata,
                                     void *arg)
{
    // calculate the interval in the ISR,
    // so that the interval will be always correct even when cap_queue is not handled in time and overflow.
    BaseType_t high_task_wakeup = pdFALSE;
    if (edata->cap_edge == MCPWM_POS_EDGE)
    {
        // store the timestamp when pos edge is detected
        right_cap_val_begin_of_sample = edata->cap_value;
        right_cap_val_end_of_sample = right_cap_val_begin_of_sample;
    }
    else
    {
        right_cap_val_end_of_sample = edata->cap_value;
        uint32_t pulse_count = right_cap_val_end_of_sample - right_cap_val_begin_of_sample;
        // send measurement back though queue
        xQueueSendFromISR(Right_cap_queue, &pulse_count, &high_task_wakeup);
    }
    return high_task_wakeup == pdTRUE;
}


void Generate_Trig(void *arg)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (true)
    {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500));
        ESP_ERROR_CHECK(gpio_set_level(HCSR04_FORWARD_TRIG, 1)); // set high
        esp_rom_delay_us(10);
        ESP_ERROR_CHECK(gpio_set_level(HCSR04_FORWARD_TRIG, 0)); // set low

        ESP_ERROR_CHECK(gpio_set_level(HCSR04_LEFT_TRIG, 1)); // set high
        esp_rom_delay_us(10);
        ESP_ERROR_CHECK(gpio_set_level(HCSR04_LEFT_TRIG, 0)); // set low

        ESP_ERROR_CHECK(gpio_set_level(HCSR04_RIGHT_TRIG, 1)); // set high
        esp_rom_delay_us(10);
        ESP_ERROR_CHECK(gpio_set_level(HCSR04_RIGHT_TRIG, 0)); // set low
    }
}



void HCSR04_Init(void)
{
    Forward_cap_queue = xQueueCreate(1, sizeof(uint32_t));
    Left_cap_queue = xQueueCreate(1, sizeof(uint32_t));
    Right_cap_queue = xQueueCreate(1, sizeof(uint32_t));

    ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_0, HCSR04_FORWARD_ECHO));
    ESP_ERROR_CHECK(gpio_pulldown_en(HCSR04_FORWARD_ECHO));
    mcpwm_capture_config_t forward_conf = {
        .cap_edge = MCPWM_BOTH_EDGE,
        .cap_prescale = 1,
        .capture_cb = Forward_echo_isr_handler,
        .user_data = NULL};
    ESP_ERROR_CHECK(mcpwm_capture_enable_channel(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, &forward_conf));
    ESP_LOGI(HCSR04_TAG, "Forward Echo pin configured");

    ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_1, HCSR04_LEFT_ECHO));
    ESP_ERROR_CHECK(gpio_pulldown_en(HCSR04_LEFT_ECHO));
    mcpwm_capture_config_t left_conf = {
        .cap_edge = MCPWM_BOTH_EDGE,
        .cap_prescale = 1,
        .capture_cb = Left_echo_isr_handler,
        .user_data = NULL};
    ESP_ERROR_CHECK(mcpwm_capture_enable_channel(MCPWM_UNIT_0, MCPWM_SELECT_CAP1, &left_conf));
    ESP_LOGI(HCSR04_TAG, "Left Echo pin configured");

    ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_2, HCSR04_RIGHT_ECHO));
    ESP_ERROR_CHECK(gpio_pulldown_en(HCSR04_RIGHT_ECHO));
    mcpwm_capture_config_t right_conf = {
        .cap_edge = MCPWM_BOTH_EDGE,
        .cap_prescale = 1,
        .capture_cb = Right_echo_isr_handler,
        .user_data = NULL};
    ESP_ERROR_CHECK(mcpwm_capture_enable_channel(MCPWM_UNIT_0, MCPWM_SELECT_CAP2, &right_conf));
    ESP_LOGI(HCSR04_TAG, "Right Echo pin configured");

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pin_bit_mask = HCSR04_FORWARD_TRIG_SEL | HCSR04_LEFT_TRIG_SEL | HCSR04_RIGHT_TRIG_SEL,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_set_level(HCSR04_FORWARD_TRIG | HCSR04_LEFT_TRIG | HCSR04_RIGHT_TRIG, 0));
    ESP_LOGI(HCSR04_TAG, "Trig pin configured");
}

void Fowrward_Distance_Get(void *param)
{
    float* forward_distance = (float*)param;
    while (true)
    {
        uint32_t pulse_count;
        // block and wait for new measurement
        xQueueReceive(Forward_cap_queue, &pulse_count, portMAX_DELAY);
        uint32_t pulse_width_us = pulse_count * (1000000.0 / rtc_clk_apb_freq_get());
        // following formula is based on: https://www.elecrow.com/download/HC_SR04%20Datasheet.pdf
        if (pulse_width_us > 35000)
        {
            // out of range
            continue;
        }
        *forward_distance = (float)pulse_width_us / 58;
        ESP_LOGI(HCSR04_TAG, "Pulse width: %uus, Forward Measured distance: %.2fcm", (unsigned int)pulse_width_us, *forward_distance);
    }
}

void Left_Distance_Get(void *param)
{
    float* left_distance = (float*)param;
    while (true)
    {
        uint32_t pulse_count;
        // block and wait for new measurement
        xQueueReceive(Left_cap_queue, &pulse_count, portMAX_DELAY);
        uint32_t pulse_width_us = pulse_count * (1000000.0 / rtc_clk_apb_freq_get());
        // following formula is based on: https://www.elecrow.com/download/HC_SR04%20Datasheet.pdf
        if (pulse_width_us > 35000)
        {
            // out of range
            continue;
        }
        *left_distance = (float)pulse_width_us / 58;
        ESP_LOGI(HCSR04_TAG, "Pulse width: %uus, Left Measured distance: %.2fcm", (unsigned int)pulse_width_us, *left_distance);
    }
}

void Right_Distance_Get(void *param)
{
    float* right_distance = (float*)param;
    while (true)
    {
        uint32_t pulse_count;
        // block and wait for new measurement
        xQueueReceive(Right_cap_queue, &pulse_count, portMAX_DELAY);
        uint32_t pulse_width_us = pulse_count * (1000000.0 / rtc_clk_apb_freq_get());
        // following formula is based on: https://www.elecrow.com/download/HC_SR04%20Datasheet.pdf
        if (pulse_width_us > 35000)
        {
            // out of range
            continue;
        }
        *right_distance = (float)pulse_width_us / 58;
        ESP_LOGI(HCSR04_TAG, "Pulse width: %uus, Right Measured distance: %.2fcm", (unsigned int)pulse_width_us, *right_distance);
    }
}

