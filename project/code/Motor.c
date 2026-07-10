#include "Motor.h"

void Motor_Init ()
{
    gpio_init(MOTOR1_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                          	// GPIO 初始化为输出 默认上拉输出高
    pwm_init(MOTOR1_PWM, 17000, 0);                                                	// PWM 通道初始化频率 17KHz 占空比初始为 0

    gpio_init(MOTOR2_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                          	// GPIO 初始化为输出 默认上拉输出高
    pwm_init(MOTOR2_PWM, 17000, 0);                                                	// PWM 通道初始化频率 17KHz 占空比初始为 0
}

void Set_PWM (int8 duty, int8 motor)
{
    if(duty > PWM_MAX)                                                              // 防止给的占空比绝对值过大
    {
        duty = PWM_MAX;
    }
    if(duty < -PWM_MAX)
    {
        duty = -PWM_MAX;
    }

    if(duty >= 0)                                                           	    // 正转
    {
        if(motor == MOTOR1)
        {
            gpio_set_level(MOTOR1_DIR, GPIO_HIGH);                                 	// DIR输出高电平
            pwm_set_duty(MOTOR1_PWM, duty * (PWM_DUTY_MAX / 100));                  // 计算占空比
        }
        else if(motor == MOTOR2)
        {
            gpio_set_level(MOTOR2_DIR, GPIO_HIGH);                                 	// DIR输出高电平
            pwm_set_duty(MOTOR2_PWM, duty * (PWM_DUTY_MAX / 100));                  // 计算占空比
        }
    }
    else
    {
        if(motor == MOTOR1)
        {
            gpio_set_level(MOTOR1_DIR, GPIO_LOW);                                   // DIR输出低电平
            pwm_set_duty(MOTOR1_PWM, (-duty) * (PWM_DUTY_MAX / 100));               // 计算占空比
        }
        else if(motor == MOTOR2)
        {
            gpio_set_level(MOTOR2_DIR, GPIO_LOW);                                   // DIR输出低电平
            pwm_set_duty(MOTOR2_PWM, (-duty) * (PWM_DUTY_MAX / 100));               // 计算占空比
        }
    }
}