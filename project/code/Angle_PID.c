#include "Angle_PID.h"
#include "Motor_PID.h"

Angle_PID_Struct Angle_PID;

bool enable_angle_pid = false;
volatile float angle_target = ANGLE_PID_TARGET_DEFAULT;
volatile float angle_actual = ANGLE_PID_TARGET_DEFAULT;

/*
函数功能：把value限制在[-limit,limit]范围内
参数：
value：要限制的数值（积分值或输出值）
limit：上限（正数），自动取绝对值
*/
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

/*
函数功能：角度归一化，将角度收拢到[0°，360°]内
参数：
angle：任意角度值
*/
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

/*
函数功能：角度误差计算（目标角度-当前角度）
参数：
target：目标角度（度）
actual：当前实际角度（度）
返回值：误差角度，范围[-180°，180°]，正数=需要左转，负数＝需要右转
*/
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

/*
函数功能：角度PID结构体初始化，给PID控制器填入参数并清零历史状态
参数：
pid：要初始化的PID结构体指针
kp，ki，kd：PID三个系数
output_max：输出限幅上限
integral_max：积分限幅上限
*/
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

/*
函数功能：设置角度PID的目标值
参数：
target：目标朝向角度，会先收拢到[0°，360°]再写入
*/
void Angle_PID_Target_Init (float target)
{
    angle_target = Angle_Normalize(target);
}

/*
函数功能：在线修改PID参数（运行时调参用）
参数：
pid：要修改的PID结构体
kp，ki，kd：新的PID参数
*/
void Angle_PID_Set (Angle_PID_Struct *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

/*
函数功能：角度PID完全复位，清空所有运行状态，回到初始化时的状态
参数：
pid：要复位的PID结构体
*/
void Angle_PID_Clear (Angle_PID_Struct *pid)
{
    pid->target = ANGLE_PID_TARGET_DEFAULT;
    pid->actual = ANGLE_PID_TARGET_DEFAULT;
    pid->err = 0;
    pid->err_last = 0;
    pid->integral = 0;
    pid->output = 0;
}

/*
函数功能：角度PID核心计算，返回PWM占空比
参数：
pid：PID结构体（携带系数和历史状态）
target：目标朝向角度（度）
actual：当前实际朝向角度（度）
*/
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

/*
函数功能：角度PID控制。角度环输出作为左右轮的差速速度目标，
再由两个速度PID分别输出PWM，实现角度外环、速度内环的串级控制。
参数：
pid：PID结构体
target：目标朝向角度（度）
actual：当前实际朝向角度（度）
*/
void Angle_PID_Control (Angle_PID_Struct *pid, float target, float actual)
{
    float speed_target = Angle_PID_Calc(pid, target, actual);

    Motor_PID_Control(&Motor_Left_PID, speed_target, motor_encoder_offset[LEFT_MOTOR], LEFT_MOTOR);
    Motor_PID_Control(&Motor_Right_PID, -speed_target, motor_encoder_offset[RIGHT_MOTOR], RIGHT_MOTOR);
}
