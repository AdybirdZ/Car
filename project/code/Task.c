#include "Task.h"
#include "OLED.h"
#include "Light_and_Buzzer.h"

// 当前任务：车辆经过中间四路全黑的停车线时停车。
bool enable_task = true;
uint8 task_stop_flag = 0;
static uint8 task_stop_pending = 0;
static uint8 task_stop_delay_ticks = 0;

/*
函数功能：检测A点启停线
参数：无
返回值：1表示左2、左1、右1、右2四路均检测到黑色，0表示不是启停线
*/
static uint8 Task_Is_Stop_Line (void)
{
    uint8 index = 0;
    uint8 black_count = 0;

    for(index = 0; index < GRAY_CHANNEL_NUM; index ++)
    {
        if(gray_line_black_level == gray_data[index])
        {
            black_count ++;
        }
    }

    return (black_count >= 3U);
}

/*
函数功能：任务模块初始化，清除停车锁存标志并恢复红外巡线输出
参数：无
*/
void Task_Init (void)
{
    task_stop_flag = 0;
    task_stop_pending = 0;
    task_stop_delay_ticks = 0;
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

    if(!task_stop_pending)
    {
        if(Task_Is_Stop_Line())
        {
            task_stop_pending = 1;
            task_stop_delay_ticks = TASK_STOP_DELAY_TICKS;
        }
        return;
    }

    if(task_stop_delay_ticks > 0)
    {
        task_stop_delay_ticks --;
    }

    if(task_stop_delay_ticks == 0)
    {
        task_stop_flag = 1;
        enable_gray_line = false;
        Motor_PID_New_Stop();
        OLED_Stop_Time();
        // 停车动作完成后开启蜂鸣器；task_stop_flag锁存，避免重复触发。
        Buzz(1);
    }
}
