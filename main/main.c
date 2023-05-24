#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "HC_SR04.h"
#include "rotary_encoder.h"
#include "Motor.h"

TaskHandle_t Trig_Generate_Handle;
TaskHandle_t Forward_Distance_Get_Handle;
TaskHandle_t Left_Distance_Get_Handle;
TaskHandle_t Right_Distance_Get_Handle;
TaskHandle_t Velocity_Get_Handle;

float forward_distance = 0;
float left_distance = 0;
float right_distance = 0;

rotary_encoder_t *encoder_L = NULL;
rotary_encoder_t *encoder_R = NULL;
float velocity_left = 0;
float velocity_right = 0;

void Encoder_Init(void);
void Speed_Refresh(void *param);


void app_main(void)
{
    HCSR04_Init();
    Encoder_Init();
    Motor_gpio_init();
    Motor_SetDuty_L_R(40, 100);
    xTaskCreate(Generate_Trig, "Trig Generator", 1024, NULL, 5, &Trig_Generate_Handle);
    xTaskCreate(Fowrward_Distance_Get, "Get Forward Distance", 1024 * 3, (void*)&forward_distance, 5, &Forward_Distance_Get_Handle);
    xTaskCreate(Left_Distance_Get, "Get Left Distance", 1024 * 3, (void*)&left_distance, 5, &Left_Distance_Get_Handle);
    xTaskCreate(Right_Distance_Get, "Get Right Distance", 1024 * 3, (void*)&right_distance, 5, &Right_Distance_Get_Handle);
    xTaskCreate(Speed_Refresh, "Velocity Get", 1024 * 4, NULL, 5, Velocity_Get_Handle);

    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


void Encoder_Init(void)
{
    // Create rotary encoder instance
    rotary_encoder_config_t config_L = ROTARY_ENCODER_DEFAULT_CONFIG((rotary_encoder_dev_t)pcnt_unit_L, C1_L, C2_L);

    ESP_ERROR_CHECK(rotary_encoder_new_ec11(&config_L, &encoder_L));

    // Filter out glitch (1us)
    ESP_ERROR_CHECK(encoder_L->set_glitch_filter(encoder_L, 1));
    // Start encoder
    ESP_ERROR_CHECK(encoder_L->start(encoder_L));

    // Create rotary encoder instance
    rotary_encoder_config_t config_R = ROTARY_ENCODER_DEFAULT_CONFIG((rotary_encoder_dev_t)pcnt_unit_R, C1_R, C2_R);
    ESP_ERROR_CHECK(rotary_encoder_new_ec11(&config_R, &encoder_R));

    // Filter out glitch (1us)
    ESP_ERROR_CHECK(encoder_R->set_glitch_filter(encoder_R, 1));
    // Start encoder
    ESP_ERROR_CHECK(encoder_R->start(encoder_R));
}

void Speed_Refresh(void *param)
{
    while (true)
    {
        velocity_left = 0;
        velocity_right = 0;
        pcnt_counter_clear(pcnt_unit_L);
        pcnt_counter_clear(pcnt_unit_R);
        vTaskDelay(pdMS_TO_TICKS(10));
        velocity_left = (encoder_L->get_counter_value(encoder_L)) % 100;
        velocity_right = (encoder_R->get_counter_value(encoder_R)) % 100;
        ESP_LOGI("Velocity", "velocity: L:%f R:%f\n", velocity_left, velocity_right);
    }
}
