#include "Motor.h"
#include "Motor_PID.h"
#include "WIFI.h"

#define LEFT_KP_INDEX      (2)
#define RIGHT_KP_INDEX     (3)
#define LEFT_KI_INDEX      (4)
#define RIGHT_KI_INDEX     (5)
#define LEFT_KD_INDEX      (6)
#define RIGHT_KD_INDEX     (7)

static void WIFI_Parameter_Process ()
{
    if(seekfree_assistant_parameter_update_flag[LEFT_KP_INDEX])
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
    }
}

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

void WIFI_Oscilloscope_Process ()
{
    seekfree_assistant_oscilloscope_data.data[0] = motor_encoder_offset[LEFT_MOTOR];
    seekfree_assistant_oscilloscope_data.data[1] = motor_encoder_offset[RIGHT_MOTOR];
    seekfree_assistant_oscilloscope_data.channel_num = 2;
    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);

    seekfree_assistant_data_analysis();
    WIFI_Parameter_Process();
}
