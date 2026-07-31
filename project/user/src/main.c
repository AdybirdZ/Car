/*********************************************************************************************************************
* MSPM0G3507 Opensource Library 即（MSPM0G3507 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
* 
* 本文件是 MSPM0G3507 开源库的一部分
* 
* MSPM0G3507 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
* 
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
* 
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
* 
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
* 
* 文件名称          mian
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK 5.37
* 适用平台          MSPM0G3507
* 店铺链接          https://seekfree.taobao.com/
********************************************************************************************************************/

#include "headfile.h"
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

static void Main_Check_Step_Encoder_Before_Task (void)
{
    // 上电定位已将当前 A/B 计数清零；相对角度大于+20度即为异常上抬。
    // 不在主循环读取 PWM 绝对角度，避免其关闭全局中断而丢失 K230 串口数据。
    if(Step_Encoder_Get_Relative_Angle()
       > STEP_ENCODER_PRESTART_MAX_POSITIVE_OFFSET_DEG)
    {
        Step_Stop();
        Init_Software_Reset();
    }
}

static void Main_Buzz_Start_Wait (void)
{
    uint8 count;

    Buzz(1);
    for(count = 0; count < 100U; count++)
    {
        Main_Check_Step_Encoder_Before_Task();
        system_delay_ms(10);
    }
    Buzz(0);
    Main_Check_Step_Encoder_Before_Task();
}

int main (void)
{
    uint8 selected_mode = 0;

    Init();
    Task1_Init();
    Task3_Init();
    Task4_Init();
    Task5_Init();
    Task6_Init();
    Task7_Init();

    while(true)
    {
        selected_mode = 0;

    // mode=1静止平衡；mode=2巡线；mode=3钢球往返；mode=4钢球平衡巡线。
    while(true)
    {
        Main_Check_Step_Encoder_Before_Task();
        Button_Scan_10ms();
        OLED_Process();
        if(TASK6_MODE_NUMBER != mode)
        {
            Task6_Cancel();
        }
        if(TASK7_MODE_NUMBER != mode)
        {
            Task7_Cancel();
        }

        if(TASK1_MODE_NUMBER == mode)
        {
            Task3_Cancel_Prepare();
            Task4_Cancel_Prepare();
            Task5_Cancel_Prepare();
            Task1_Prepare();
            Task1();
        }
        else if(TASK3_MODE_NUMBER == mode)
        {
            Task1_Cancel_Prepare();
            Task4_Cancel_Prepare();
            Task5_Cancel_Prepare();
            Task3_Prepare();
            Task3();
        }
        else if(TASK4_MODE_NUMBER == mode)
        {
            Task1_Cancel_Prepare();
            Task3_Cancel_Prepare();
            Task5_Cancel_Prepare();
            Task4_Prepare();
            Task4();
        }
        else if(TASK5_MODE_NUMBER == mode)
        {
            Task1_Cancel_Prepare();
            Task3_Cancel_Prepare();
            Task4_Cancel_Prepare();
            Task5_Prepare();
            Task5();
        }
        else if(TASK6_MODE_NUMBER == mode)
        {
            Task1_Cancel_Prepare();
            Task3_Cancel_Prepare();
            Task4_Cancel_Prepare();
            Task6_Prepare();
            Task6();
        }
        else if(TASK7_MODE_NUMBER == mode)
        {
            Task1_Cancel_Prepare();
            Task3_Cancel_Prepare();
            Task4_Cancel_Prepare();
            Task5_Cancel_Prepare();
            Task7();
        }
        else
        {
            Task1_Cancel_Prepare();
            Task3_Cancel_Prepare();
            Task4_Cancel_Prepare();
            Task5_Cancel_Prepare();
        }

        if(TASK7_MODE_NUMBER != mode && Button_Get_Start_Event())
        {
            if(TASK1_MODE_NUMBER == mode && Task1_Is_Ready())
            {
                selected_mode = TASK1_MODE_NUMBER;
                break;
            }
            if(2U == mode)
            {
                selected_mode = 2U;
                break;
            }
            if(TASK3_MODE_NUMBER == mode && Task3_Is_Ready())
            {
                selected_mode = TASK3_MODE_NUMBER;
                break;
            }
            if(TASK4_MODE_NUMBER == mode && Task4_Is_Ready())
            {
                selected_mode = TASK4_MODE_NUMBER;
                break;
            }
            if(TASK5_MODE_NUMBER == mode && Task5_Is_Ready())
            {
                selected_mode = TASK5_MODE_NUMBER;
                break;
            }
            if(TASK6_MODE_NUMBER == mode && Task6_Is_Ready())
            {
                selected_mode = TASK6_MODE_NUMBER;
                break;
            }
        }

        system_delay_ms(BUTTON_SCAN_PERIOD_MS);
    }

    if(TASK1_MODE_NUMBER == selected_mode)
    {
        Main_Buzz_Start_Wait();
        Task1_Start();
    }
    else if(2U == selected_mode)
    {
        // 原mode=2：A30松开后蜂鸣1秒，再开始计时和巡线。
        Main_Buzz_Start_Wait();
        OLED_Start_Time();
        enable_gray_line = true;
        Motor_PID_New_Start(gray_line_base_offset, gray_line_base_offset);
    }
    else if(TASK3_MODE_NUMBER == selected_mode)
    {
        Main_Buzz_Start_Wait();
        Task3_Start();
    }
    else if(TASK4_MODE_NUMBER == selected_mode)
    {
        Main_Buzz_Start_Wait();
        Task4_Start();
    }
    else if(TASK5_MODE_NUMBER == selected_mode)
    {
        Main_Buzz_Start_Wait();
        Task5_Start();
    }
    else
    {
        Main_Buzz_Start_Wait();
        Task6_Start();
    }

    while(true)
    {
        if(TASK1_MODE_NUMBER == selected_mode)
        {
            Task1();
            if(Task1_Is_Stop_Requested())
            {
                Task1_Init();
                OLED_Show_Status("SELECT MODE");
                break;
            }
        }
        else if(2U == selected_mode)
        {
            Gray_Line_Update_Target();
            Task_Update();
            if(task_stop_flag)
            {
                Task_Init();
                OLED_Show_Status("SELECT MODE");
                break;
            }
        }
        else if(TASK3_MODE_NUMBER == selected_mode)
        {
            Task3();
            if(Task3_Is_Stop_Requested())
            {
                Task3_Init();
                OLED_Show_Status("SELECT MODE");
                break;
            }
        }
        else if(TASK4_MODE_NUMBER == selected_mode)
        {
            Task4();
            if(Task4_Is_Stop_Requested())
            {
                Task4_Init();
                OLED_Show_Status("SELECT MODE");
                break;
            }
        }
        else if(TASK5_MODE_NUMBER == selected_mode)
        {
            Task5();
            if(Task5_Is_Stop_Requested())
            {
                Task5_Init();
                OLED_Show_Status("SELECT MODE");
                break;
            }
        }
        else
        {
            Task6();
            if(Task6_Is_Stop_Requested())
            {
                Task6_Init();
                Task5_Init();
                OLED_Show_Status("SELECT MODE");
                break;
            }
        }
        OLED_Process();
        system_delay_ms(10);
    }
    }
}
