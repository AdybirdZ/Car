#include "Motor.h"
#include "Motor_PID.h"
#include "Angle_PID.h"
#include "Gray_Line.h"

bool enable_motor_output = true;
volatile float motor_pwm_duty[2] = {0, 0};

void Motor_Init ()
{
    // MOTOR1对应右轮，方向引脚默认输出高电平，PWM初始占空比为0
    gpio_init(MOTOR1_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(MOTOR1_PWM, 17000, 0);

    // MOTOR2对应左轮，方向引脚默认输出高电平，PWM初始占空比为0
    gpio_init(MOTOR2_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(MOTOR2_PWM, 17000, 0);
}

void Set_PWM (float duty, int8 motor)
{
    if(!enable_motor_output)
    {
        if(motor == LEFT_MOTOR || motor == RIGHT_MOTOR)
        {
            motor_pwm_duty[motor] = 0;
        }
        return;
    }

    // 限制输出，防止占空比超过电机允许范围
    if(duty > PWM_MAX)
    {
        duty = PWM_MAX;
    }
    if(duty < -PWM_MAX)
    {
        duty = -PWM_MAX;
    }

    if(motor == LEFT_MOTOR || motor == RIGHT_MOTOR)
    {
        motor_pwm_duty[motor] = duty;
    }

    if(duty >= 0)
    {
        if(motor == LEFT_MOTOR)
        {
            // 左轮由MOTOR2驱动，正占空比对应方向引脚高电平
            gpio_set_level(MOTOR2_DIR, GPIO_HIGH);
            pwm_set_duty(MOTOR2_PWM, duty * (PWM_DUTY_MAX / 100));
        }
        else if(motor == RIGHT_MOTOR)
        {
            // 右轮由MOTOR1驱动，正占空比对应方向引脚高电平
            gpio_set_level(MOTOR1_DIR, GPIO_HIGH);
            pwm_set_duty(MOTOR1_PWM, duty * (PWM_DUTY_MAX / 100));
        }
    }
    else
    {
        if(motor == LEFT_MOTOR)
        {
            // 负占空比通过方向引脚低电平实现反转，PWM使用绝对值
            gpio_set_level(MOTOR2_DIR, GPIO_LOW);
            pwm_set_duty(MOTOR2_PWM, (-duty) * (PWM_DUTY_MAX / 100));
        }
        else if(motor == RIGHT_MOTOR)
        {
            gpio_set_level(MOTOR1_DIR, GPIO_LOW);
            pwm_set_duty(MOTOR1_PWM, (-duty) * (PWM_DUTY_MAX / 100));
        }
    }
}

void Motor_Stop ()
{
    // 停止闭环控制并清除目标，随后关闭两个PWM输出
    enable_motor_pid = false;
    enable_angle_pid = false;
    enable_gray_line = false;

    motor_target_offset[LEFT_MOTOR] = 0;
    motor_target_offset[RIGHT_MOTOR] = 0;

    Motor_PID_Clear(&Motor_Left_PID);
    Motor_PID_Clear(&Motor_Right_PID);
    Angle_PID_Clear(&Angle_PID);

    pwm_set_duty(MOTOR2_PWM, 0);
    pwm_set_duty(MOTOR1_PWM, 0);
    motor_pwm_duty[LEFT_MOTOR] = 0;
    motor_pwm_duty[RIGHT_MOTOR] = 0;
}
