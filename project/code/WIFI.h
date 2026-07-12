#ifndef _WIFI_H_
#define _WIFI_H_

#include "zf_common_headfile.h"

#define WIFI_SSID                  "呆呆鸟的大型鸟窝"
#define WIFI_PASSWORD              "wangluomima"
#define TCP_TARGET_IP              WIFI_SPI_TARGET_IP
#define TCP_TARGET_PORT            WIFI_SPI_TARGET_PORT
#define WIFI_LOCAL_PORT            "6666"

void WIFI_Init ();
void WIFI_Oscilloscope_Process ();

#endif
