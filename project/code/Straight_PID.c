#include "Straight_PID.h"

Straight_PID_Struct Straight_PID;

volatile bool enable_straight_pid = false;
volatile float straight_angle_target = 0.0f;
volatile float straight_angle_actual = 0.0f;
volatile float straight_base_speed = 0.0f;
volatile float straight_speed_correction = 0.0f;
volatile float straight_motor_target[2] = {0.0f, 0.0f};

/*
函数功能：把value限制在[-limit, limit]范围内
参数：
value：需要限制的数值
limit：限制值，函数内部自动取绝对值
*/
static float Straight_PID_Limit (float value, float limit)
{
    if(limit < 0.0f)
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
函数功能：直行角度PID结构体初始化，写入PID参数并清零运行状态
参数：
pid：直行角度PID结构体指针
kp、ki、kd：PID参数
output_max：外环输出的最大速度修正量
integral_max：积分限幅
*/
void Straight_PID_Structure_Init (Straight_PID_Struct *pid, float kp, float ki, float kd, float output_max, float integral_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->target = 0.0f;
    pid->actual = 0.0f;
    pid->err = 0.0f;
    pid->err_last = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;

    pid->output_max = output_max;
    pid->integral_max = integral_max;
}

/*
函数功能：启动直行双环PID控制，以当前传入角度为目标航向
参数：
angle_target：直行目标Yaw角度
base_speed：左右轮的基础速度目标
*/
void Straight_PID_Start (float angle_target, float base_speed)
{
    enable_angle_pid = false;
    enable_motor_pid = false;
    enable_straight_pid = false;

    Straight_PID_Clear(&Straight_PID);
    Motor_PID_Clear(&Motor_Left_PID);
    Motor_PID_Clear(&Motor_Right_PID);

    straight_angle_target = Angle_Normalize(angle_target);
    straight_angle_actual = straight_angle_target;
    straight_base_speed = base_speed;
    straight_speed_correction = 0.0f;
    straight_motor_target[LEFT_MOTOR] = base_speed;
    straight_motor_target[RIGHT_MOTOR] = base_speed;

    enable_straight_pid = true;
}

/*
函数功能：停止直行串级控制并清零外环状态
参数：无
*/
void Straight_PID_Stop ()
{
    enable_straight_pid = false;
    straight_base_speed = 0.0f;
    straight_speed_correction = 0.0f;
    straight_motor_target[LEFT_MOTOR] = 0.0f;
    straight_motor_target[RIGHT_MOTOR] = 0.0f;
    Straight_PID_Clear(&Straight_PID);
}

/*
函数功能：在线修改直行角度PID参数
参数：
pid：直行角度PID结构体指针
kp、ki、kd：新的PID参数
*/
void Straight_PID_Set (Straight_PID_Struct *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

/*
函数功能：清零直行角度PID运行状态
参数：
pid：直行角度PID结构体指针
*/
void Straight_PID_Clear (Straight_PID_Struct *pid)
{
    pid->target = 0.0f;
    pid->actual = 0.0f;
    pid->err = 0.0f;
    pid->err_last = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

/*
函数功能：计算直行角度外环输出，返回左右轮速度目标的修正量
参数：
pid：直行角度PID结构体指针
target：目标Yaw角度
actual：当前Yaw角度
*/
float Straight_PID_Calc (Straight_PID_Struct *pid, float target, float actual)
{
    pid->target = target;
    pid->actual = actual;
    pid->err = Angle_Error(pid->target, pid->actual);

    pid->integral += pid->err;
    pid->integral = Straight_PID_Limit(pid->integral, pid->integral_max);

    pid->output = pid->kp * pid->err
                + pid->ki * pid->integral
                + pid->kd * (pid->err - pid->err_last);
    pid->output = Straight_PID_Limit(pid->output, pid->output_max);

    pid->err_last = pid->err;

    return pid->output;
}

/*
函数功能：执行直行双环PID控制，角度外环生成差速修正量，两个速度内环分别输出PWM
参数：
pid：直行角度PID结构体指针
target：目标Yaw角度
actual：当前Yaw角度
base_speed：左右轮的基础速度目标
*/
void Straight_PID_Control (Straight_PID_Struct *pid, float target, float actual, float base_speed)
{
    straight_angle_actual = actual;
    straight_speed_correction = Straight_PID_Calc(pid, target, actual);
    straight_speed_correction = Straight_PID_Limit(straight_speed_correction, base_speed);

    straight_motor_target[LEFT_MOTOR] = base_speed + straight_speed_correction;
    straight_motor_target[RIGHT_MOTOR] = base_speed - straight_speed_correction;

    Motor_PID_Control(&Motor_Left_PID, straight_motor_target[LEFT_MOTOR], motor_encoder_offset[LEFT_MOTOR], LEFT_MOTOR);
    Motor_PID_Control(&Motor_Right_PID, straight_motor_target[RIGHT_MOTOR], motor_encoder_offset[RIGHT_MOTOR], RIGHT_MOTOR);
}
