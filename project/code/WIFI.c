#include "Motor.h"
#include "WIFI.h"

extern volatile int16 motor_encoder_speed[2];

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
    seekfree_assistant_oscilloscope_data.channel_num = 2;
    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);

    seekfree_assistant_data_analysis();
}