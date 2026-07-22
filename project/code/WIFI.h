#ifndef _WIFI_H_
#define _WIFI_H_

#include "zf_common_headfile.h"

#define WIFI_SSID                  "呆呆鸟的大型鸟窝"
#define WIFI_PASSWORD              "wangluomima"
#define TCP_TARGET_IP              WIFI_SPI_TARGET_IP
#define TCP_TARGET_PORT            WIFI_SPI_TARGET_PORT
#define WIFI_LOCAL_PORT            "6666"
#define OSCILLOSCOPE_FREQ          (1)                      // 多少PIT周期发送一次示波器数据到电脑

extern uint8 oscilloscope_count;
extern bool enable_WIFI;
extern bool enable_parameter_process;

void WIFI_Init ();
void WIFI_Process ();
void WIFI_Delay_ms (uint32 delay_ms);
void WIFI_Oscilloscope_Process ();

#endif
