#include "Action.h"

/*
函数功能：原地旋转到指定朝向，返回值为0或1（1表示转到位了，0表示超时或没转到位）
参数：
target：绝对目标角度（范围在0°-360°之间）
*/
uint8 Action_Turn_To (float target)
{
    uint16 timeout_count = 0;
    uint8 stable_count = 0;
    uint8 last_enable_motor_pid = enable_motor_pid;
    uint8 last_enable_angle_pid = enable_angle_pid;
    uint8 last_enable_gray_line = enable_gray_line;
    float error = 0.0f;

    if(!enable_motor_output || !enable_position)
    {
        return 0;
    }

    Position_Update();
    angle_actual = euler_angle[YAW];
    target = Angle_Normalize(target);

    Angle_PID_Clear(&Angle_PID);
    Angle_PID_Target_Init(target);

    enable_gray_line = false;
    enable_motor_pid = false;
    enable_angle_pid = true;

    while(timeout_count < (ACTION_TURN_TIMEOUT_MS / ANGLE_PID_PERIOD_MS))                       // 转弯5秒即超时，强行退出
    {
        error = Angle_Error(angle_target, angle_actual);

        if(error < ACTION_TURN_ANGLE_TOLERANCE && error > -ACTION_TURN_ANGLE_TOLERANCE)         // 误差在2°以内，持续0.1秒视为稳定
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

    enable_angle_pid = last_enable_angle_pid;
    enable_motor_pid = last_enable_motor_pid;
    enable_gray_line = last_enable_gray_line;

    return (stable_count >= ACTION_TURN_STABLE_COUNT);
}

/*
函数功能：让小车原地转动指定角度（正数：向左转；负数：向右转）
参数：
angle：要转多少度
*/
uint8 Action_Turn (float angle)
{
    Position_Update();
    angle_actual = euler_angle[YAW];

    return Action_Turn_To(angle_actual + angle);
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
同Action_Turn，但不关心结果，无返回值
*/
void Turn (float angle)
{
    (void)Action_Turn(angle);
}
