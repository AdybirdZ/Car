#ifndef _GRAY_H_
#define _GRAY_H_

#include "zf_common_headfile.h"

#define GRAY_DAT_PIN        (A24)
#define GRAY_CLK_PIN        (A25)
#define GRAY_INVALID_VALUE  (0x00)
#define GRAY_CHANNEL_NUM    (8)             // 灰度传感器通道数量

extern bool enable_gray;
extern bool gray_data_ready;
extern uint8 gray_value;
extern uint8 gray_data[GRAY_CHANNEL_NUM];

void Gray_Init ();
uint8 Gray_Read ();
void Gray_Update ();
void Gray_Wait_First_Data ();

#endif
