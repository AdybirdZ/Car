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

#define MOTOR_PID_PERIOD_MS        (5)
#define LEFT_MOTOR                 (MOTOR1)
#define RIGHT_MOTOR                (MOTOR2)
#define LEFT_ENCODER_INDEX         (1)
#define RIGHT_ENCODER_INDEX        (0)
#define WIFI_SSID                  "呆呆鸟的大型鸟窝"
#define WIFI_PASSWORD              "wangluomima"
#define TCP_TARGET_IP              WIFI_SPI_TARGET_IP
#define TCP_TARGET_PORT            WIFI_SPI_TARGET_PORT
#define WIFI_LOCAL_PORT            "6666"

volatile uint8 pit_flag = 0;
uint8 oscilloscope_count = 0;
volatile int16 motor_target_speed[2]        = {0, 0};
volatile int16 motor_encoder_location[2]    = {0, 0};
volatile int16 motor_encoder_speed[2]       = {0, 0};
volatile int8 motor_pwm_duty[2]             = {0, 0};

void motor_pid_pit_handler (uint32 event, void *ptr)
{
    (void)event;            // 暂时无需判断中断触发源，直接丢弃

    *((volatile uint8 *)ptr) = 1;

    motor_encoder_location[LEFT_MOTOR] = absolute_encoder_get_location(LEFT_ENCODER_INDEX);
    motor_encoder_location[RIGHT_MOTOR] = absolute_encoder_get_location(RIGHT_ENCODER_INDEX);

    motor_encoder_speed[LEFT_MOTOR] = absolute_encoder_get_offset(LEFT_ENCODER_INDEX);
    motor_encoder_speed[RIGHT_MOTOR] = absolute_encoder_get_offset(RIGHT_ENCODER_INDEX);

    motor_pwm_duty[LEFT_MOTOR] = Motor_PID_Control(&Motor1_PID, motor_target_speed[LEFT_MOTOR], motor_encoder_speed[LEFT_MOTOR], LEFT_MOTOR);
    motor_pwm_duty[RIGHT_MOTOR] = Motor_PID_Control(&Motor2_PID, motor_target_speed[RIGHT_MOTOR], motor_encoder_speed[RIGHT_MOTOR], RIGHT_MOTOR);
}

int main (void)
{
    clock_init(SYSTEM_CLOCK_80M);   // 时钟配置及系统初始化<务必保留>
    debug_init();					// 调试串口信息初始化
	// 此处编写用户代码 例如外设初始化代码等

    while(wifi_spi_init(WIFI_SSID, WIFI_PASSWORD))
    {
        printf("\r\n connect wifi failed. \r\n");
        system_delay_ms(100);
    }

    if(1 != WIFI_SPI_AUTO_CONNECT)
    {
        while(wifi_spi_socket_connect("TCP", TCP_TARGET_IP, TCP_TARGET_PORT, WIFI_LOCAL_PORT))
        {
            printf("\r\n Connect TCP Servers error, try again.");
            system_delay_ms(100);
        }
    }

    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);

    Light_and_Buzz_Init();
    Motor_Init();
    absolute_encoder_init(LEFT_ENCODER_INDEX);
    absolute_encoder_init(RIGHT_ENCODER_INDEX);

    Motor_PID_Init(&Motor1_PID, 1.0, 0.0, 0.0, PWM_MAX, 50);
    Motor_PID_Init(&Motor2_PID, 1.0, 0.0, 0.0, PWM_MAX, 50);

    pit_ms_init(PIT_TIM_G12, MOTOR_PID_PERIOD_MS, motor_pid_pit_handler, (void *)&pit_flag);

    interrupt_global_enable(0);                 // 中断使能

    // 此处编写用户代码 例如外设初始化代码等

    motor_target_speed[LEFT_MOTOR] = 15;
    motor_target_speed[RIGHT_MOTOR] = 15;

    while(true)
    {
        if(pit_flag)
        {
            pit_flag = 0;
            oscilloscope_count ++;

            if(4 <= oscilloscope_count)
            {
                oscilloscope_count = 0;

                seekfree_assistant_oscilloscope_data.data[0] = motor_encoder_speed[LEFT_MOTOR];
                seekfree_assistant_oscilloscope_data.data[1] = motor_encoder_speed[RIGHT_MOTOR];
                seekfree_assistant_oscilloscope_data.channel_num = 2;
                seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);

                seekfree_assistant_data_analysis();
            }
        }
        // 此处编写需要循环执行的代码

        // system_delay_ms(1000);
        // motor_target_speed[LEFT_MOTOR] = -20;
        // motor_target_speed[RIGHT_MOTOR] = -20;
        // system_delay_ms(1000);
        // motor_target_speed[LEFT_MOTOR] = 0;
        // motor_target_speed[RIGHT_MOTOR] = 0;
        // system_delay_ms(1000);

        // 此处编写需要循环执行的代码
    }
}
