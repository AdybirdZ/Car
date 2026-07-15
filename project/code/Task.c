#include "Task.h"
// 目前只有一个task：直角转弯
bool enable_task = true;
uint16 task_all_white_count = 0;
uint8 task_turn_lock = 0;
float task_turn_target_angle = 0.0f;

static uint8 Task_Is_All_White ()
{
    if(gray_line_black_level)
    {
        return (0x00 == gray_value);
    }

    return (0xFF == gray_value);
}

void Task_Init ()
{
    task_all_white_count = 0;
    task_turn_lock = 0;
    task_turn_target_angle = 0.0f;

    if(enable_task)
    {
        enable_motor_pid = true;
        enable_angle_pid = false;
        enable_gray_line = true;
    }
}

void Task_Update ()
{
    if(!enable_task || !enable_gray)
    {
        return;
    }

    Gray_Update();

    if(!Task_Is_All_White())
    {
        task_all_white_count = 0;
        task_turn_lock = 0;
        enable_motor_pid = true;
        enable_angle_pid = false;
        enable_gray_line = true;
        return;
    }

    if(task_turn_lock)
    {
        return;
    }

    task_all_white_count ++;

    if(task_all_white_count >= TASK_ALL_WHITE_COUNT)
    {
        task_all_white_count = 0;
        task_turn_lock = 1;
        task_turn_target_angle = Angle_Normalize(task_turn_target_angle - TASK_LEFT_TURN_ANGLE);
        Action_Turn_To(task_turn_target_angle);
        enable_motor_pid = true;
        enable_angle_pid = false;
        enable_gray_line = true;
    }
}
