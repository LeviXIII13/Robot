#include "Motor.h"

void Motor_gpio_init(void)
{
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, Motor_L_A);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, Motor_L_B);
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000; // frequency = 1kHz,
    pwm_config.cmpr_a = 0;                          // initial duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;                          // initial duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;     // up counting mode
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config); // Configure PWM0A & PWM0B with above settings

    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, Motor_R_A);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0B, Motor_R_B);
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &pwm_config); // Configure PWM0A & PWM0B with above settings
}

void Motor_L_SetDuty(float duty_cycle)
{
    /* motor moves in forward direction, with duty cycle = duty % */
    if (duty_cycle > 0)
    {
        mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, duty_cycle);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
    }
    /* motor moves in backward direction, with duty cycle = -duty % */
    else
    {
        mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, -duty_cycle);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    }
}

void Motor_R_SetDuty(float duty_cycle)
{
    /* motor moves in forward direction, with duty cycle = duty % */
    if (duty_cycle > 0)
    {
        mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A);
        mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B, duty_cycle);
        mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
    }
    /* motor moves in backward direction, with duty cycle = -duty % */
    else
    {
        mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B);
        mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, -duty_cycle);
        mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    }
}

void Motor_SetDuty_All(float duty_cycle)
{
    Motor_L_SetDuty(duty_cycle);
    Motor_R_SetDuty(duty_cycle);
}

void Motor_SetDuty_L_R(float duty_cycle_L, float duty_cycle_R)
{
    Motor_L_SetDuty(duty_cycle_L);
    Motor_R_SetDuty(duty_cycle_R);
}

