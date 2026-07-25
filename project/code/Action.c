#include "Action.h"
#include "Straight_PID.h"

uint16 straight_count = 0;

/*
函数功能：切换到角度环转弯模式
参数：
target：绝对目标角度
*/
static void Action_Angle_PID_Start (float target)
{
    enable_angle_pid = false;
    enable_motor_pid = false;
    Straight_PID_Stop();
    enable_gray_line = false;

    Motor_PID_Clear(&Motor_Left_PID);
    Motor_PID_Clear(&Motor_Right_PID);
    Angle_PID_Clear(&Angle_PID);
    Angle_PID_Target_Init(target);

    enable_angle_pid = true;
}

/*
函数功能：原地旋转到指定朝向，返回值为0或1（1表示转到位了，0表示超时或没转到位）
参数：
target：绝对目标角度（范围在0°到360°之间）
*/
uint8 Action_Turn_To (float target)
{
    uint16 timeout_count = 0;
    uint8 stable_count = 0;
    uint8 last_enable_motor_pid = enable_motor_pid;
    uint8 last_enable_gray_line = enable_gray_line;
    float error = 0.0f;

    if(!enable_motor_output || !enable_position)
    {
        return 0;
    }

    Position_Update();
    angle_actual = euler_angle[YAW];
    target = Angle_Normalize(target);

    Action_Angle_PID_Start(target);

    while(timeout_count < (ACTION_TURN_TIMEOUT_MS / ANGLE_PID_PERIOD_MS))                       // 转弯3秒即超时，强行退出
    {
        error = Angle_Error(angle_target, angle_actual);

        if(error < ACTION_TURN_ANGLE_TOLERANCE && error > -ACTION_TURN_ANGLE_TOLERANCE)         // 误差在5°以内，持续0.1秒视为稳定
        {
            stable_count++;
            if(stable_count >= ACTION_TURN_STABLE_COUNT)
            {
                break;
            }
        }
        else
        {
            stable_count = 0;
        }

        system_delay_ms(ANGLE_PID_PERIOD_MS);
        timeout_count ++;
    }

    Set_PWM(0, LEFT_MOTOR);
    Set_PWM(0, RIGHT_MOTOR);
    Angle_PID_Clear(&Angle_PID);

    enable_angle_pid = false;
    enable_motor_pid = last_enable_motor_pid;
    enable_gray_line = last_enable_gray_line;

    return (stable_count >= ACTION_TURN_STABLE_COUNT);
}

/*
函数功能：让小车原地转动指定角度
参数：
angle：要转多少度（负数：向左转；正数：向右转）
*/
uint8 Action_Turn (float angle)
{
    Position_Update();
    angle_actual = euler_angle[YAW];

    return Action_Turn_To(angle_actual + angle);
}

/*
函数功能：按指定方向原地转动指定角度
参数：
angle：转动角度的绝对值，单位为度
direction：ACTION_TURN_CLOCKWISE为顺时针，ACTION_TURN_COUNTERCLOCKWISE为逆时针
返回值：1表示全部转动完成，0表示参数错误或任一分段未稳定到位
*/
uint8 Action_Turn_Direction (float angle, int8 direction)
{
    float step_angle = 0.0f;
    float step_target = 0.0f;

    if(direction != ACTION_TURN_CLOCKWISE && direction != ACTION_TURN_COUNTERCLOCKWISE)
    {
        return 0;
    }

    if(angle < 0.0f)
    {
        angle = -angle;
    }

    Position_Update();
    step_target = euler_angle[YAW];

    while(angle > 0.0f)
    {
        step_angle = (angle > ACTION_TURN_DIRECTION_STEP) ? ACTION_TURN_DIRECTION_STEP : angle;
        step_target = Angle_Normalize(step_target + step_angle * direction);

        while(!Action_Turn_To(step_target))
        {
            system_delay_ms(ANGLE_PID_PERIOD_MS);
        }

        angle -= step_angle;
    }

    return 1;
}

/*
函数功能：快捷右转90°
参数：无
*/
uint8 Action_Turn_Right ()
{
    return Action_Turn(ACTION_TURN_RIGHT_ANGLE);
}

/*
函数功能：快捷左转90°
参数：无
*/
uint8 Action_Turn_Left ()
{
    return Action_Turn(ACTION_TURN_LEFT_ANGLE);
}

/*
函数功能：左右轮使用相同速度目标行驶指定时间，随后停车
参数：
target：左右轮速度环目标
duration_ms：行驶时间，单位为毫秒
*/
void Action_Drive_Equal_Target (float target, uint32 duration_ms)
{
    enable_angle_pid = false;
    Straight_PID_Stop();
    Motor_PID_Target_Init(target);
    Motor_PID_Clear(&Motor_Left_PID);
    Motor_PID_Clear(&Motor_Right_PID);
    enable_motor_pid = true;

    system_delay_ms(duration_ms);
    Motor_Stop();
}

/*
同Action_Turn，但不关心结果，无返回值
*/
void Turn (float angle)
{
    (void)Action_Turn(angle);
}

// 1cm对应time为46
void Straight_Forward (float time)
{
    Straight_PID_Start(angle_target, MOTOR_PID_TARGET_OFFSET);

    while(straight_count < ((time + MOTOR_PID_PERIOD_MS - 1) / MOTOR_PID_PERIOD_MS))
    {
        straight_count++;
        system_delay_ms(MOTOR_PID_PERIOD_MS);
    }

    straight_count = 0;
    Motor_Stop();
}

void Straight_Backward (float time)
{
    Straight_PID_Start(angle_target, -MOTOR_PID_TARGET_OFFSET);

    while(straight_count < ((time + MOTOR_PID_PERIOD_MS - 1) / MOTOR_PID_PERIOD_MS))
    {
        straight_count++;
        system_delay_ms(MOTOR_PID_PERIOD_MS);
    }

    straight_count = 0;
    Motor_Stop();
}
