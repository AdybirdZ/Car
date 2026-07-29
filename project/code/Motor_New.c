#include "Motor_New.h"

volatile int32 motor_new_duty[2] = {0, 0};
volatile uint8 motor_new_direction[2] = {MOTOR_NEW_STOP, MOTOR_NEW_STOP};

static int32 Motor_New_Limit_Duty (int32 duty)
{
    if(duty < 0)
    {
        duty = -duty;
    }

    if(duty > MOTOR_NEW_DUTY_MAX)
    {
        duty = MOTOR_NEW_DUTY_MAX;
    }

    return duty;
}

static uint32 Motor_New_Convert_Duty (int32 duty)
{
    return (uint32)duty * PWM_DUTY_MAX / MOTOR_NEW_DUTY_MAX;
}

void Motor_New_Init (void)
{
    // TB6612 的 STBY 已外接 3.3V，软件只需要初始化方向和 PWM 引脚。
    gpio_init(MOTOR_NEW_LEFT_IN1_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(MOTOR_NEW_LEFT_IN2_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(MOTOR_NEW_RIGHT_IN1_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(MOTOR_NEW_RIGHT_IN2_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);

    pwm_init(MOTOR_NEW_LEFT_PWM_PIN, MOTOR_NEW_PWM_FREQUENCY_HZ, 0);
    pwm_init(MOTOR_NEW_RIGHT_PWM_PIN, MOTOR_NEW_PWM_FREQUENCY_HZ, 0);

    motor_new_duty[MOTOR_NEW_LEFT] = 0;
    motor_new_duty[MOTOR_NEW_RIGHT] = 0;
    motor_new_direction[MOTOR_NEW_LEFT] = MOTOR_NEW_STOP;
    motor_new_direction[MOTOR_NEW_RIGHT] = MOTOR_NEW_STOP;
}

void Motor_New_Set_Direction (uint8 motor, uint8 direction)
{
    if(motor > MOTOR_NEW_RIGHT)
    {
        return;
    }

    if(MOTOR_NEW_LEFT == motor)
    {
        if(MOTOR_NEW_FORWARD == direction)
        {
            gpio_high(MOTOR_NEW_LEFT_IN1_PIN);
            gpio_low(MOTOR_NEW_LEFT_IN2_PIN);
        }
        else if(MOTOR_NEW_REVERSE == direction)
        {
            gpio_low(MOTOR_NEW_LEFT_IN1_PIN);
            gpio_high(MOTOR_NEW_LEFT_IN2_PIN);
        }
        else
        {
            direction = MOTOR_NEW_STOP;
            gpio_high(MOTOR_NEW_LEFT_IN1_PIN);
            gpio_high(MOTOR_NEW_LEFT_IN2_PIN);
        }
    }
    else
    {
        // 两轮镜像安装，右轮前进时的输入电平与左轮相反。
        if(MOTOR_NEW_FORWARD == direction)
        {
            gpio_low(MOTOR_NEW_RIGHT_IN1_PIN);
            gpio_high(MOTOR_NEW_RIGHT_IN2_PIN);
        }
        else if(MOTOR_NEW_REVERSE == direction)
        {
            gpio_high(MOTOR_NEW_RIGHT_IN1_PIN);
            gpio_low(MOTOR_NEW_RIGHT_IN2_PIN);
        }
        else
        {
            direction = MOTOR_NEW_STOP;
            gpio_high(MOTOR_NEW_RIGHT_IN1_PIN);
            gpio_high(MOTOR_NEW_RIGHT_IN2_PIN);
        }
    }

    motor_new_direction[motor] = direction;
}

void Motor_New_Set_Duty (uint8 motor, int32 duty)
{
    uint32 pwm_duty;

    if(motor > MOTOR_NEW_RIGHT)
    {
        return;
    }

    duty = Motor_New_Limit_Duty(duty);
    pwm_duty = Motor_New_Convert_Duty(duty);

    if(MOTOR_NEW_LEFT == motor)
    {
        pwm_set_duty(MOTOR_NEW_LEFT_PWM_PIN, pwm_duty);
    }
    else
    {
        pwm_set_duty(MOTOR_NEW_RIGHT_PWM_PIN, pwm_duty);
    }

    motor_new_duty[motor] = duty;
}

void Motor_New_Set_Output (uint8 motor, int32 output)
{
    if(output > 0)
    {
        Motor_New_Set_Direction(motor, MOTOR_NEW_FORWARD);
        Motor_New_Set_Duty(motor, output);
    }
    else if(output < 0)
    {
        Motor_New_Set_Direction(motor, MOTOR_NEW_REVERSE);
        Motor_New_Set_Duty(motor, -output);
    }
    else
    {
        Motor_New_Brake(motor);
    }
}

void Motor_New_Brake (uint8 motor)
{
    Motor_New_Set_Duty(motor, 0);
    Motor_New_Set_Direction(motor, MOTOR_NEW_STOP);
}

void Motor_New_Stop_All (void)
{
    Motor_New_Brake(MOTOR_NEW_LEFT);
    Motor_New_Brake(MOTOR_NEW_RIGHT);
}

uint8 Motor_New_Get_Direction (uint8 motor)
{
    if(motor > MOTOR_NEW_RIGHT)
    {
        return MOTOR_NEW_STOP;
    }

    return motor_new_direction[motor];
}
