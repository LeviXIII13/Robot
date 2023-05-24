#ifndef MOTOR_H
#define MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/mcpwm.h"
#include "driver/gpio.h"

#define Motor_L_A GPIO_NUM_17
#define Motor_L_B GPIO_NUM_18

#define Motor_R_A GPIO_NUM_15
#define Motor_R_B GPIO_NUM_7

void Motor_gpio_init(void);
void Motor_L_SetDuty(float duty_cycle);
void Motor_R_SetDuty(float duty_cycle);
void Motor_SetDuty_All(float duty_cycle);
void Motor_SetDuty_L_R(float duty_cycle_L, float duty_cycle_R);
void Motor_OFF(void);


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* MOTOR_H */