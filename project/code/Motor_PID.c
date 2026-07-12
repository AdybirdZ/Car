#include "Motor_PID.h"

Motor_PID_Struct Motor_Left_PID;
Motor_PID_Struct Motor_Right_PID;

volatile float motor_target_offset[2]       = {0, 0};
volatile int16 motor_encoder_location[2]    = {0, 0};
volatile int16 motor_encoder_offset[2]          = {0, 0};

static float Motor_PID_Limit (float value, float limit)         // 积分及输出限幅函数
{
    if(limit < 0)                                               // 防止输成负数
    {
        limit = -limit;
    }

    if(value > limit)
    {
        value = limit;
    }
    else if(value < -limit)
    {
        value = -limit;
    }

    return value;
}

void Motor_PID_Structure_Init (Motor_PID_Struct *pid, float kp, float ki, float kd, float output_max, float integral_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->target = 0;
    pid->actual = 0;
    pid->err = 0;
    pid->err_last = 0;
    pid->integral = 0;
    pid->output = 0;

    pid->output_max = output_max;
    pid->integral_max = integral_max;
}

void Motor_PID_Target_Init (float target)
{
    motor_target_offset[LEFT_MOTOR] = target;
    motor_target_offset[RIGHT_MOTOR] = target;
}

void Motor_PID_Set (Motor_PID_Struct *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void Motor_PID_Clear (Motor_PID_Struct *pid)
{
    pid->target = 0;
    pid->actual = 0;
    pid->err = 0;
    pid->err_last = 0;
    pid->integral = 0;
    pid->output = 0;
}

float Motor_PID_Calc (Motor_PID_Struct *pid, float target, float actual)
{
    pid->target = target;
    pid->actual = actual;
    pid->err = pid->target - pid->actual;

    pid->integral += pid->err;
    pid->integral = Motor_PID_Limit(pid->integral, pid->integral_max);

    pid->output = pid->kp * pid->err
                + pid->ki * pid->integral
                + pid->kd * (pid->err - pid->err_last);
    pid->output = Motor_PID_Limit(pid->output, pid->output_max);

    pid->err_last = pid->err;

    return pid->output;
}

float Motor_PID_Control (Motor_PID_Struct *pid, float target, float actual, int8 motor)
{
    float duty = Motor_PID_Calc(pid, target, actual);

    if(target > 0 && duty < 0)          // 防止电机突然反转，造成电流过大烧毁电机
    {
        duty = 0;
    }
    else if(target < 0 && duty > 0)
    {
        duty = 0;
    }

    Set_PWM(duty, motor);

    return motor_pwm_duty[motor];
}
