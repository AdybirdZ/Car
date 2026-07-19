#include "Motor_PID.h"

Motor_PID_Struct Motor_Left_PID;
Motor_PID_Struct Motor_Right_PID;

bool enable_motor_pid = true;
volatile float motor_target_offset[2]       = {0, 0};
volatile int16 motor_encoder_location[2]    = {0, 0};
volatile int16 motor_encoder_offset[2]      = {0, 0};

/*
函数功能：限幅函数，将数值限制在[-limit,limit]之间，防止越界
参数：
value：要限制的数值（速度PID的积分或输出）
limit：限幅，自动取正
*/
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

/*
函数功能：速度PID结构体初始化：填入系数和限幅值，清零历史状态
参数：
pid：要初始化的PID结构体
kp，ki，kd：PID的三个系数
output_max：输出限幅
integral_max:积分限幅
*/
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

/*
函数功能：电机目标速度初始化，给左右两轮设定相同的初始目标速度
参数：
target：左右两轮的初始目标速度
*/
void Motor_PID_Target_Init (float target)
{
    motor_target_offset[LEFT_MOTOR] = target;
    motor_target_offset[RIGHT_MOTOR] = target;
}

/*
函数功能：在线修改速度PID参数（不动运行状态，只改系数）
参数：
pid：&Motor_Left_PID或&Motor_Right_PID
kp，ki，kd：新的PID参数
*/
void Motor_PID_Set (Motor_PID_Struct *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

/*
函数功能：速度PID复位，清空运行状态
参数：
pid：&Motor_Left_PID或&Motor_Right_PID
*/
void Motor_PID_Clear (Motor_PID_Struct *pid)
{
    pid->target = 0;
    pid->actual = 0;
    pid->err = 0;
    pid->err_last = 0;
    pid->integral = 0;
    pid->output = 0;
}

/*
函数功能：速度PID核心计算，返回PWM占空比
参数：
pid：速度PID结构体
target：目标速度
actual：实际速度
*/
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

/*
函数功能：速度PID控制：算占空比→防止反转→写PWM，返回实际写入的占空比
参数：
pid：速度PID结构体
target：目标速度
actual：实际速度
motor：指定电机（LEFT_MOTOR=0，RIGHT_MOTOR=1）
*/
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
