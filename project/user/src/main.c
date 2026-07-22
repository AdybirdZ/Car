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
int main (void)
{
    Init();

    // 此处编写用户代码 例如外设初始化代码等

    enable_angle_pid = false;
    enable_gray_line = false;
    system_delay_ms(3000);
    Angle_PID_Target_Init(euler_angle[YAW]);    // 第一次上电时，默认方向角为180°，要初始化一下，把初始方向角也设为180°而非0°
    Buzz(1);
    system_delay_ms(1000);
    Buzz(0);

    Straight_Forward(6050.0f);

    system_delay_ms(3000);

    Straight_Backward(4880.0f);

    system_delay_ms(3000);

    Action_Turn_Left();

    system_delay_ms(1000);

    Straight_Forward(4800.0f);

    system_delay_ms(3000);

    Action_Turn_Right();

    system_delay_ms(1000);

    Straight_Forward(3700.0f);

    system_delay_ms(3000);

    Straight_Backward(3800.0f);     //

    system_delay_ms(3000);

    Action_Turn_Right();

    system_delay_ms(1000);

    Straight_Forward(2270.0f);

    system_delay_ms(3000);

    Action_Turn_Left();

    system_delay_ms(1000);

    Straight_Forward(1250.0f);

    system_delay_ms(3000);

    Action_Turn_Left();

    system_delay_ms(1500);

    Straight_Forward(1200.0f);

    system_delay_ms(3000);

    Straight_Backward(2500.0f);

    system_delay_ms(3000);

    Action_Turn_Right();

    system_delay_ms(1000);

    Straight_Forward(3420.0f);      //

    system_delay_ms(3000);

    Action_Turn_Left();

    system_delay_ms(1000);

    Straight_Forward(1220.0f);

    system_delay_ms(3000);

    Action_Turn_Left();

    system_delay_ms(1000);

    Straight_Forward(2220.0f);

    system_delay_ms(3000);

    Action_Turn_Right();

    system_delay_ms(1000);

    Straight_Forward(1280.0f);

    system_delay_ms(3000);

    Action_Turn_Right();

    system_delay_ms(1000);

    Straight_Forward(2220.0f);

    system_delay_ms(3000);

    Action_Turn_Left();

    system_delay_ms(1000);

    Straight_Forward(1600.0f);

    while(true)
    {
        Buzz(1);
        system_delay_ms(1000);
        Buzz(0);
        system_delay_ms(1000);
    }
}
