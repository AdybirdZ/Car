#include "Ball_PID.h"

Ball_PID_Struct Ball_PID;

volatile int16 ball_k230_position = 0;
volatile uint8 ball_k230_position_ready = 0;
float ball_step_target_angle = 0.0f;

/*
函数功能：将数值限制在[-limit, limit]范围内
参数说明：value为待限制数值，limit为限幅绝对值
*/
static float Ball_PID_Limit (float value, float limit)
{
    if(limit < 0.0f)
    {
        limit = -limit;
    }

    if(value > limit)
    {
        return limit;
    }
    if(value < -limit)
    {
        return -limit;
    }
    return value;
}

/*
函数功能：初始化钢球位置PID
参数说明：无
说明：默认目标为K230返回0，PID参数默认均为0，防止首次测试时步进电机突然大幅动作。
*/
void Ball_PID_Init (void)
{
    Ball_PID.kp = BALL_PID_KP_DEFAULT;
    Ball_PID.ki = BALL_PID_KI_DEFAULT;
    Ball_PID.kd = BALL_PID_KD_DEFAULT;
    Ball_PID.target = BALL_PID_TARGET_DEFAULT;
    Ball_PID.actual = 0.0f;
    Ball_PID.err = 0.0f;
    Ball_PID.err_last = 0.0f;
    Ball_PID.integral = 0.0f;
    Ball_PID.output = 0.0f;
    Ball_PID.output_max = BALL_PID_OUTPUT_MAX_DEFAULT;
    Ball_PID.integral_max = BALL_PID_INTEGRAL_MAX_DEFAULT;
    Ball_PID.step_frequency_hz = BALL_PID_STEP_FREQUENCY_DEFAULT;

    ball_k230_position = 0;
    ball_k230_position_ready = 0;
    ball_step_target_angle = 0.0f;

    // 以前代码的K230双线识别，电赛不用
    enable_k230_line = false;
}

/*
函数功能：修改钢球PID参数
参数说明：kp、ki、kd为比例、积分、微分系数
*/
void Ball_PID_Set (float kp, float ki, float kd)
{
    Ball_PID.kp = kp;
    Ball_PID.ki = ki;
    Ball_PID.kd = kd;
}

/*
函数功能：设置钢球在K230画面中的目标位置
参数说明：target为K230返回值，0表示画面正中间
*/
void Ball_PID_Set_Target (float target)
{
    Ball_PID.target = target;
}

/*
函数功能：设置步进电机执行PID角度指令时的脉冲频率
参数说明：frequency_hz为STEP频率，必须大于0
*/
void Ball_PID_Set_Step_Frequency (uint32 frequency_hz)
{
    if(frequency_hz > 0)
    {
        Ball_PID.step_frequency_hz = frequency_hz;
    }
}

/*
函数功能：清除PID历史状态，但保留PID参数、目标值和步进频率
参数说明：无
*/
void Ball_PID_Clear (void)
{
    Ball_PID.actual = 0.0f;
    Ball_PID.err = 0.0f;
    Ball_PID.err_last = 0.0f;
    Ball_PID.integral = 0.0f;
    Ball_PID.output = 0.0f;
    ball_step_target_angle = 0.0f;
}

/*
函数功能：根据K230返回的钢球位置计算步进电机绝对目标角度
参数说明：actual为K230返回值，正数表示钢球偏右，负数表示钢球偏左
返回值：限幅后的步进电机目标角度，单位为度
*/
float Ball_PID_Calc (float actual)
{
    Ball_PID.actual = actual;
    Ball_PID.err = Ball_PID.target - Ball_PID.actual;

    Ball_PID.integral += Ball_PID.err;
    Ball_PID.integral = Ball_PID_Limit(Ball_PID.integral, Ball_PID.integral_max);

    Ball_PID.output = Ball_PID.kp * Ball_PID.err
                    + Ball_PID.ki * Ball_PID.integral
                    + Ball_PID.kd * (Ball_PID.err - Ball_PID.err_last);
    Ball_PID.output *= BALL_PID_OUTPUT_DIRECTION;
    Ball_PID.output = Ball_PID_Limit(Ball_PID.output, Ball_PID.output_max);

    Ball_PID.err_last = Ball_PID.err;
    return Ball_PID.output;
}

/*
函数功能：读取并处理一帧K230钢球位置数据
参数说明：无
返回值：1表示成功处理#XXX$数字帧，0表示当前无完整数字帧或格式错误
说明：Serial_Get_Message会自动取得已经去除#和$的XXX字符串，再由数字解析函数转为int16。
*/
uint8 Ball_PID_Process (void)
{
    char message[SERIAL_BUFFER_SIZE] = {0};
    int16 position = 0;

    if(!Serial_Get_Message(message, SERIAL_BUFFER_SIZE))
    {
        return 0;
    }

    if(!Serial_Parse_Signed_Int(message, &position))
    {
        return 0;
    }

    ball_k230_position = position;
    ball_k230_position_ready = 1;
    ball_step_target_angle = Ball_PID_Calc((float)position);

    Step_To_Angle(ball_step_target_angle, Ball_PID.step_frequency_hz);
    return 1;
}
