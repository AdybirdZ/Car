#include "Motor.h"
#include "Motor_PID.h"
#include "Position.h"
#include "Angle_PID.h"
#include "Gray.h"
#include "WIFI.h"
#include "isr.h"

#define LEFT_KP_INDEX      (2)
#define RIGHT_KP_INDEX     (3)
#define LEFT_KI_INDEX      (4)
#define RIGHT_KI_INDEX     (5)
#define LEFT_KD_INDEX      (6)
#define RIGHT_KD_INDEX     (7)

#define ANGLE_KP_INDEX     (2)
#define ANGLE_KI_INDEX     (3)
#define ANGLE_KD_INDEX     (4)

uint8 oscilloscope_count = 0;
bool enable_WIFI = true;
bool enable_parameter_process = false;      // 是否启用PID调参模式

/*
函数功能：WiFi在线调参，检测上位机传来的参数更新，实时写入PID参数
参数：无
*/
static void WIFI_Parameter_Process ()
{
    /*if(seekfree_assistant_parameter_update_flag[LEFT_KP_INDEX])
    {
        seekfree_assistant_parameter_update_flag[LEFT_KP_INDEX] = 0;
        Motor_PID_Set(&Motor_Left_PID, seekfree_assistant_parameter[LEFT_KP_INDEX], Motor_Left_PID.ki, Motor_Left_PID.kd);
    }

    if(seekfree_assistant_parameter_update_flag[RIGHT_KP_INDEX])
    {
        seekfree_assistant_parameter_update_flag[RIGHT_KP_INDEX] = 0;
        Motor_PID_Set(&Motor_Right_PID, seekfree_assistant_parameter[RIGHT_KP_INDEX], Motor_Right_PID.ki, Motor_Right_PID.kd);
    }

    if(seekfree_assistant_parameter_update_flag[LEFT_KI_INDEX])
    {
        seekfree_assistant_parameter_update_flag[LEFT_KI_INDEX] = 0;
        Motor_PID_Set(&Motor_Left_PID, Motor_Left_PID.kp, seekfree_assistant_parameter[LEFT_KI_INDEX], Motor_Left_PID.kd);
    }

    if(seekfree_assistant_parameter_update_flag[RIGHT_KI_INDEX])
    {
        seekfree_assistant_parameter_update_flag[RIGHT_KI_INDEX] = 0;
        Motor_PID_Set(&Motor_Right_PID, Motor_Right_PID.kp, seekfree_assistant_parameter[RIGHT_KI_INDEX], Motor_Right_PID.kd);
    }

    if(seekfree_assistant_parameter_update_flag[LEFT_KD_INDEX])
    {
        seekfree_assistant_parameter_update_flag[LEFT_KD_INDEX] = 0;
        Motor_PID_Set(&Motor_Left_PID, Motor_Left_PID.kp, Motor_Left_PID.ki, seekfree_assistant_parameter[LEFT_KD_INDEX]);
    }

    if(seekfree_assistant_parameter_update_flag[RIGHT_KD_INDEX])
    {
        seekfree_assistant_parameter_update_flag[RIGHT_KD_INDEX] = 0;
        Motor_PID_Set(&Motor_Right_PID, Motor_Right_PID.kp, Motor_Right_PID.ki, seekfree_assistant_parameter[RIGHT_KD_INDEX]);
    }*/

    if(seekfree_assistant_parameter_update_flag[ANGLE_KP_INDEX])
    {
        seekfree_assistant_parameter_update_flag[ANGLE_KP_INDEX] = 0;
        Angle_PID_Set(&Angle_PID, seekfree_assistant_parameter[ANGLE_KP_INDEX], Angle_PID.ki, Angle_PID.kd);
    }

    if(seekfree_assistant_parameter_update_flag[ANGLE_KI_INDEX])
    {
        seekfree_assistant_parameter_update_flag[ANGLE_KI_INDEX] = 0;
        Angle_PID_Set(&Angle_PID, Angle_PID.kp, seekfree_assistant_parameter[ANGLE_KI_INDEX], Angle_PID.kd);
    }

    if(seekfree_assistant_parameter_update_flag[ANGLE_KD_INDEX])
    {
        seekfree_assistant_parameter_update_flag[ANGLE_KD_INDEX] = 0;
        Angle_PID_Set(&Angle_PID, Angle_PID.kp, Angle_PID.ki, seekfree_assistant_parameter[ANGLE_KD_INDEX]);
    }
}

/*
函数功能：WiFi模块初始化
参数：无
*/
void WIFI_Init ()
{
    while(wifi_spi_init(WIFI_SSID, WIFI_PASSWORD))
    {
        printf("\r\n 正在尝试连接到WIFI…… \r\n");
        system_delay_ms(100);
    }

    while(wifi_spi_socket_connect("TCP", TCP_TARGET_IP, TCP_TARGET_PORT, WIFI_LOCAL_PORT))
    {
        printf("\r\n 正在尝试连接到TCP服务器…… \r\n");
        system_delay_ms(100);
    }

    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);         // 逐飞助手初始化
}

/*
函数功能：WiFi示波器发送，把运行数据打包上传到上位机实时显示
参数：无
*/
void WIFI_Oscilloscope_Process ()
{
    if(!enable_WIFI)
    {
        return;
    }

    // 通道0:yaw，1/2:左右编码器速度，3/4:左右PWM，5:PIT心跳，6/7:电机PID和电机输出使能状态。
    seekfree_assistant_oscilloscope_data.data[0] = euler_angle[YAW];
    seekfree_assistant_oscilloscope_data.data[1] = motor_encoder_offset[LEFT_MOTOR];
    seekfree_assistant_oscilloscope_data.data[2] = motor_encoder_offset[RIGHT_MOTOR];
    seekfree_assistant_oscilloscope_data.data[3] = motor_pwm_duty[LEFT_MOTOR];
    seekfree_assistant_oscilloscope_data.data[4] = motor_pwm_duty[RIGHT_MOTOR];
    seekfree_assistant_oscilloscope_data.data[5] = (float)pit_tick_count;
    seekfree_assistant_oscilloscope_data.data[6] = enable_motor_pid ? 1.0f : 0.0f;
    seekfree_assistant_oscilloscope_data.data[7] = enable_motor_output ? 1.0f : 0.0f;
    seekfree_assistant_oscilloscope_data.channel_num = SEEKFREE_ASSISTANT_SET_OSCILLOSCOPE_COUNT;
    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);

    seekfree_assistant_data_analysis();

    if(enable_motor_output && enable_parameter_process)
    {
        WIFI_Parameter_Process();
    }
}
