#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#include "HC_SR04.h"
#include "rotary_encoder.h"
#include "Motor.h"

TaskHandle_t Trig_Generate_Handle;
TaskHandle_t Forward_Distance_Get_Handle;
TaskHandle_t Left_Distance_Get_Handle;
TaskHandle_t Right_Distance_Get_Handle;
TaskHandle_t Speed_Refresh_Handle;

TaskHandle_t State_Scan_Handle;
TaskHandle_t State_Switch_Handle;

TaskHandle_t State2_GoStraight_Handle;

QueueHandle_t StartStop_Queue;

float forward_distance = 0;
float left_distance = 0;
float right_distance = 0;

SemaphoreHandle_t xSemaphore_velocity;
rotary_encoder_t *encoder_L = NULL;
rotary_encoder_t *encoder_R = NULL;
float velocity_left = 0;
float velocity_right = 0;

static float target_velocity_L = 10;
static float target_velocity_R = 10;
static float Step = 0.5;
static float duty_L = 0;
static float duty_R = 0;

void Encoder_Init(void);
void Speed_Refresh(void *param);
void hysteresis_comparison(void);

void Start_Stop(void *param);
uint8_t i = 0;

void app_main(void)
{
    StartStop_Queue = xQueueCreate(1, sizeof(uint8_t));
    xSemaphore_velocity = xSemaphoreCreateMutex();
    HCSR04_Init();
    Encoder_Init();
    Motor_gpio_init();
    // xTaskCreate(Generate_Trig, "Trig Generator", 1024, NULL, 5, &Trig_Generate_Handle);
    // xTaskCreate(Fowrward_Distance_Get, "Get Forward Distance", 1024 * 3, (void *)&forward_distance, 5, &Forward_Distance_Get_Handle);
    // xTaskCreate(Left_Distance_Get, "Get Left Distance", 1024 * 3, (void *)&left_distance, 5, &Left_Distance_Get_Handle);
    // xTaskCreate(Right_Distance_Get, "Get Right Distance", 1024 * 3, (void *)&right_distance, 5, &Right_Distance_Get_Handle);
    // xTaskCreate(Speed_Refresh, "Speed Refresh", 1024 * 4, NULL, 5, &Speed_Refresh_Handle);

    xTaskCreate(Start_Stop, "Start Stop", 1024, NULL, 7, NULL);

    i = 1;
    xQueueSend(StartStop_Queue, (void*)&i, 0);
    vTaskDelay(pdMS_TO_TICKS(5000));
    i = 0;
    xQueueSend(StartStop_Queue, (void*)&i, 0);
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// void State_Scan(void *param)
// {
//     while (true)
//     {
//         if (forward_distance && left_distance && right_distance)
//         {
//         }
//         else if ()
//         {
//         }
//         else if ()
//         {
//         }
//         else if ()
//         {
//         }
//         else if ()
//         {
//         }
//         else if ()
//         {
//         }
//         vTaskDelay(pdMS_TO_TICKS(10));
//     }
// }

// void State_Switch(void *param)
// {
//     ;
// }

void State2_GoStraight(void *param)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (true)
    {
        hysteresis_comparison();
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));
    }
}

void Start_Stop(void *param)
{
    static uint8_t ON_OFF_state = 0;
    while (true)
    {
        xQueueReceive(StartStop_Queue, &ON_OFF_state, portMAX_DELAY);
        if (ON_OFF_state)
        {
            xTaskCreate(Generate_Trig, "Trig Generator", 1024, NULL, 4, &Trig_Generate_Handle);
            xTaskCreate(Fowrward_Distance_Get, "Get Forward Distance", 1024 * 3, (void *)&forward_distance, 4, &Forward_Distance_Get_Handle);
            xTaskCreate(Left_Distance_Get, "Get Left Distance", 1024 * 3, (void *)&left_distance, 4, &Left_Distance_Get_Handle);
            xTaskCreate(Right_Distance_Get, "Get Right Distance", 1024 * 3, (void *)&right_distance, 4, &Right_Distance_Get_Handle);
            xTaskCreate(Speed_Refresh, "Speed Refresh", 1024 * 4, NULL, 5, &Speed_Refresh_Handle);

            // xTaskCreate(State_Scan, "State Scan", 1024, NULL, 6, &State_Scan_Handle);
            // xTaskCreate(State_Switch, "State Switch", 1024, NULL, 6, &State_Switch_Handle);
            xTaskCreate(State2_GoStraight, "State2: just go straightly", 1024*2, NULL, 6, &State2_GoStraight_Handle);
        }
        else
        {
            vTaskDelete(State2_GoStraight_Handle);
            Motor_OFF();
            vTaskDelete(Speed_Refresh_Handle);
            vTaskDelete(Trig_Generate_Handle);
            vTaskDelete(Forward_Distance_Get_Handle);
            vTaskDelete(Left_Distance_Get_Handle);
            vTaskDelete(Right_Distance_Get_Handle);
        }
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

void hysteresis_comparison(void)
{
    xSemaphoreTake(xSemaphore_velocity, portMAX_DELAY);
    if ((target_velocity_L - velocity_left) > 0)
    {
        duty_L = duty_L + Step;
    }
    else if ((target_velocity_L - velocity_left) < 0)
    {
        duty_L = duty_L - Step;
    }

    if ((duty_L > 100))
    {
        duty_L = 100;
    }
    if (duty_L < -100)
    {
        duty_L = -100;
    }

    if ((target_velocity_R - velocity_right) > 0)
    {
        duty_R = duty_R + Step;
    }
    else if ((target_velocity_R - velocity_right) < 0)
    {
        duty_R = duty_R - Step;
    }
    if ((duty_R > 100))
    {
        duty_R = 100;
    }
    if (duty_R < -100)
    {
        duty_R = -100;
    }
    xSemaphoreGive(xSemaphore_velocity);
    Motor_SetDuty_L_R(duty_L, duty_R);
}

void Speed_Refresh(void *param)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (true)
    {
        xSemaphoreTake(xSemaphore_velocity, portMAX_DELAY);
        velocity_left = 0;
        velocity_right = 0;
        pcnt_counter_clear(pcnt_unit_L);
        pcnt_counter_clear(pcnt_unit_R);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
        velocity_left = (encoder_L->get_counter_value(encoder_L)) % 100;
        velocity_right = (encoder_R->get_counter_value(encoder_R)) % 100;
        xSemaphoreGive(xSemaphore_velocity);
        ESP_LOGI("Velocity", "velocity: L:%f R:%f\n", velocity_left, velocity_right);
    }
}
