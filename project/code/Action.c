#include "Action.h"

uint8 Action_Turn (float angle)
{
    uint16 timeout_count = 0;
    uint8 stable_count = 0;
    uint8 last_enable_motor_pid = enable_motor_pid;
    uint8 last_enable_angle_pid = enable_angle_pid;
    uint8 last_enable_gray_line = enable_gray_line;
    float target = 0.0f;
    float error = 0.0f;

    if(!enable_motor_output || !enable_position)
    {
        return 0;
    }

    Position_Update();
    angle_actual = euler_angle[YAW];
    target = Angle_Normalize(angle_actual + angle);

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

uint8 Action_Turn_Right ()
{
    return Action_Turn(ACTION_TURN_RIGHT_ANGLE);
}

uint8 Action_Turn_Left ()
{
    return Action_Turn(ACTION_TURN_LEFT_ANGLE);
}

void Turn (float angle)
{
    (void)Action_Turn(angle);
}
