#include "Motor.h"
#include "Motor_PID.h"
#include "Position.h"
#include "Angle_PID.h"
#include "Gray.h"
#include "WIFI.h"

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
bool enable_WIFI = false;
bool enable_parameter_process = true;      // 是否启用PID调参模式

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
    uint8 channel_num = 0;

    if(!enable_WIFI)
    {
        return;
    }

    /*if(enable_motor_output)
    {
        seekfree_assistant_oscilloscope_data.data[channel_num++] = motor_encoder_offset[LEFT_MOTOR];
        seekfree_assistant_oscilloscope_data.data[channel_num++] = motor_encoder_offset[RIGHT_MOTOR];
    }*/

    if(enable_position)
    {
        //seekfree_assistant_oscilloscope_data.data[channel_num++] = euler_angle[ROLL];
        //seekfree_assistant_oscilloscope_data.data[channel_num++] = euler_angle[PITCH];
        seekfree_assistant_oscilloscope_data.data[channel_num++] = euler_angle[YAW];
    }

    if(enable_gray && channel_num < SEEKFREE_ASSISTANT_SET_OSCILLOSCOPE_COUNT)
    {
        Gray_Update();
        seekfree_assistant_oscilloscope_data.data[channel_num++] = gray_value;
    }

    if(channel_num)
    {
        seekfree_assistant_oscilloscope_data.channel_num = channel_num;
        seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
    }

    seekfree_assistant_data_analysis();

    if(enable_motor_output && enable_parameter_process)
    {
        WIFI_Parameter_Process();
    }
}
