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

// **************************** 代码区域 ****************************

volatile uint8 pit_flag = 0;

uint8 oscilloscope_count = 0;

int main (void)
{
    clock_init(SYSTEM_CLOCK_80M);   // 时钟配置及系统初始化<务必保留>
    debug_init();					// 调试串口信息初始化

	// 此处编写用户代码 例如外设初始化代码等

    Light_and_Buzz_Init();
    Motor_Init();
    Encoder_Init();
    // WIFI_Init();

    // Motor_PID_Init(&Motor1_PID, 0.2f, 0.0f, 0.0f, PWM_MAX, 50);
    // Motor_PID_Init(&Motor2_PID, 0.2f, 0.0f, 0.0f, PWM_MAX, 50);

    // pit_ms_init(PIT_TIM_G12, MOTOR_PID_PERIOD_MS, motor_pid_pit_handler, (void *)&pit_flag);

    interrupt_global_enable(0);                 // 中断使能

    // 此处编写用户代码 例如外设初始化代码等

    motor_target_speed[LEFT_MOTOR] = 15;
    motor_target_speed[RIGHT_MOTOR] = 15;

    Set_PWM(20, LEFT_MOTOR);

    while(true)
    {
        /* if(pit_flag)
        {
            pit_flag = 0;
            oscilloscope_count ++;

            if(4 <= oscilloscope_count)
            {
                oscilloscope_count = 0;
                WIFI_Oscilloscope_Process();
            }
        }*/
        // 此处编写需要循环执行的代码

        motor_encoder_speed[LEFT_MOTOR] = absolute_encoder_get_offset(LEFT_ENCODER_INDEX);
        system_delay_ms(200);

        // 此处编写需要循环执行的代码
    }
}
