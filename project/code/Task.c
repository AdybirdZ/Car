#include "Task.h"

// 当前任务：车辆经过中间四路全黑的停车线时停车。
bool enable_task = true;
uint8 task_stop_flag = 0;

/*
函数功能：检测A点启停线
参数：无
返回值：1表示左2、左1、右1、右2四路均检测到黑色，0表示不是启停线
*/
static uint8 Task_Is_Stop_Line (void)
{
    return (gray_line_black_level == gray_data[TASK_STOP_LEFT_2_INDEX]
         && gray_line_black_level == gray_data[TASK_STOP_LEFT_1_INDEX]
         && gray_line_black_level == gray_data[TASK_STOP_RIGHT_1_INDEX]
         && gray_line_black_level == gray_data[TASK_STOP_RIGHT_2_INDEX]);
}

/*
函数功能：任务模块初始化，清除停车锁存标志并恢复红外巡线输出
参数：无
*/
void Task_Init (void)
{
    task_stop_flag = 0;

}

/*
函数功能：刷新红外数据并检测A点启停线，检测到后立即停车且锁存结果
参数：无
说明：本函数应在车辆行驶期间被周期调用，例如每次主循环调用一次。
*/
void Task_Update (void)
{
    if(!enable_task || task_stop_flag)
    {
        return;
    }

    if(Task_Is_Stop_Line())
    {
        task_stop_flag = 1;
        enable_gray_line = false;
        Motor_PID_New_Stop();
    }
}
