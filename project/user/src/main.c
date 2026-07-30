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

int main (void)
{
    uint8 selected_mode = 0;

    Init();
    Task1_Init();
    Task3_Init();
    Task4_Init();

    // mode=1静止平衡；mode=2巡线；mode=3钢球往返；mode=4钢球平衡巡线。
    while(true)
    {
        Button_Scan_10ms();
        OLED_Process();

        if(TASK1_MODE_NUMBER == mode)
        {
            Task3_Cancel_Prepare();
            Task4_Cancel_Prepare();
            Task1_Prepare();
            Task1();
        }
        else if(TASK3_MODE_NUMBER == mode)
        {
            Task1_Cancel_Prepare();
            Task4_Cancel_Prepare();
            Task3_Prepare();
            Task3();
        }
        else if(TASK4_MODE_NUMBER == mode)
        {
            Task1_Cancel_Prepare();
            Task3_Cancel_Prepare();
            Task4_Prepare();
            Task4();
        }
        else
        {
            Task1_Cancel_Prepare();
            Task3_Cancel_Prepare();
            Task4_Cancel_Prepare();
        }

        if(Button_Get_Start_Event())
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
        }

        system_delay_ms(BUTTON_SCAN_PERIOD_MS);
    }

    if(TASK1_MODE_NUMBER == selected_mode)
    {
        Buzz(1);
        system_delay_ms(1000);
        Buzz(0);
        Task1_Start();
    }
    else if(2U == selected_mode)
    {
        // 原mode=2：A30松开后蜂鸣1秒，再开始计时和巡线。
        Buzz(1);
        system_delay_ms(1000);
        Buzz(0);
        OLED_Start_Time();
        enable_gray_line = true;
        Motor_PID_New_Start(gray_line_base_offset, gray_line_base_offset);
    }
    else if(TASK3_MODE_NUMBER == selected_mode)
    {
        Buzz(1);
        system_delay_ms(1000);
        Buzz(0);
        Task3_Start();
    }
    else
    {
        Buzz(1);
        system_delay_ms(1000);
        Buzz(0);
        Task4_Start();
    }

    while(true)
    {
        if(TASK1_MODE_NUMBER == selected_mode)
        {
            Task1();
        }
        else if(2U == selected_mode)
        {
            Gray_Line_Update_Target();
            Task_Update();
        }
        else if(TASK3_MODE_NUMBER == selected_mode)
        {
            Task3();
        }
        else
        {
            Task4();
        }
        OLED_Process();
        system_delay_ms(10);
    }
}
