#include "Motor.h"
#include "WIFI.h"

extern volatile float motor_encoder_speed[2];

#define WIFI_LEFT_DUTY_PARAMETER_INDEX      (2)
#define WIFI_RIGHT_DUTY_PARAMETER_INDEX     (3)

static void WIFI_Parameter_Process ()
{
    if(seekfree_assistant_parameter_update_flag[WIFI_LEFT_DUTY_PARAMETER_INDEX])
    {
        seekfree_assistant_parameter_update_flag[WIFI_LEFT_DUTY_PARAMETER_INDEX] = 0;
        Set_PWM(seekfree_assistant_parameter[WIFI_LEFT_DUTY_PARAMETER_INDEX], LEFT_MOTOR);
    }

    if(seekfree_assistant_parameter_update_flag[WIFI_RIGHT_DUTY_PARAMETER_INDEX])
    {
        seekfree_assistant_parameter_update_flag[WIFI_RIGHT_DUTY_PARAMETER_INDEX] = 0;
        Set_PWM(seekfree_assistant_parameter[WIFI_RIGHT_DUTY_PARAMETER_INDEX], RIGHT_MOTOR);
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
    seekfree_assistant_oscilloscope_data.data[0] = motor_encoder_speed[LEFT_MOTOR];
    seekfree_assistant_oscilloscope_data.data[1] = motor_encoder_speed[RIGHT_MOTOR];
    seekfree_assistant_oscilloscope_data.data[2] = motor_pwm_duty[LEFT_MOTOR];
    seekfree_assistant_oscilloscope_data.data[3] = motor_pwm_duty[RIGHT_MOTOR];
    seekfree_assistant_oscilloscope_data.channel_num = 4;
    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);

    seekfree_assistant_data_analysis();
    WIFI_Parameter_Process();
}
