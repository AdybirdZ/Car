#include "Angle_PID.h"

Angle_PID_Struct Angle_PID;

bool enable_angle_pid = false;
volatile float angle_target = ANGLE_PID_TARGET_DEFAULT;
volatile float angle_actual = ANGLE_PID_TARGET_DEFAULT;

static float Angle_PID_Limit (float value, float limit)           // 积分及输出限幅函数
{
    if(limit < 0)                                                 // 防止输成负数
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

float Angle_Normalize (float angle)
{
    while(angle >= 360.0f)
    {
        angle -= 360.0f;
    }

    while(angle < 0.0f)
    {
        angle += 360.0f;
    }

    return angle;
}

float Angle_Error (float target, float actual)
{
    float err = Angle_Normalize(target) - Angle_Normalize(actual);

    if(err > 180.0f)
    {
        err -= 360.0f;
    }
    else if(err < -180.0f)
    {
        err += 360.0f;
    }

    return err;
}

void Angle_PID_Structure_Init (Angle_PID_Struct *pid, float kp, float ki, float kd, float output_max, float integral_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->target = ANGLE_PID_TARGET_DEFAULT;
    pid->actual = ANGLE_PID_TARGET_DEFAULT;
    pid->err = 0;
    pid->err_last = 0;
    pid->integral = 0;
    pid->output = 0;

    pid->output_max = output_max;
    pid->integral_max = integral_max;
}

void Angle_PID_Target_Init (float target)
{
    angle_target = Angle_Normalize(target);
}

void Angle_PID_Set (Angle_PID_Struct *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void Angle_PID_Clear (Angle_PID_Struct *pid)
{
    pid->target = ANGLE_PID_TARGET_DEFAULT;
    pid->actual = ANGLE_PID_TARGET_DEFAULT;
    pid->err = 0;
    pid->err_last = 0;
    pid->integral = 0;
    pid->output = 0;
}

float Angle_PID_Calc (Angle_PID_Struct *pid, float target, float actual)
{
    pid->target = target;
    pid->actual = actual;
    pid->err = Angle_Error(pid->target, pid->actual);

    pid->integral += pid->err;
    pid->integral = Angle_PID_Limit(pid->integral, pid->integral_max);

    pid->output = pid->kp * pid->err
                + pid->ki * pid->integral
                + pid->kd * (pid->err - pid->err_last);
    pid->output = Angle_PID_Limit(pid->output, pid->output_max);

    pid->err_last = pid->err;

    return pid->output;
}

void Angle_PID_Control (Angle_PID_Struct *pid, float target, float actual)
{
    float duty = Angle_PID_Calc(pid, target, actual);

    Set_PWM(duty, LEFT_MOTOR);
    Set_PWM(-duty, RIGHT_MOTOR);
}
