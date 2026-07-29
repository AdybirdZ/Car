#ifndef _IR_H_
#define _IR_H_

#include "zf_common_headfile.h"

#define IR_CHANNEL_NUM      (8)             // 红外巡线传感器通道数量

// IR1在车辆最左侧，IR8在车辆最右侧；如果实际接线不同，只修改下面8个宏即可。
#define IR_CHANNEL_1_PIN    (A16)
#define IR_CHANNEL_2_PIN    (A17)
#define IR_CHANNEL_3_PIN    (A18)
#define IR_CHANNEL_4_PIN    (A19)
#define IR_CHANNEL_5_PIN    (A20)
#define IR_CHANNEL_6_PIN    (A21)
#define IR_CHANNEL_7_PIN    (A22)
#define IR_CHANNEL_8_PIN    (A23)

// 多数数字红外模块检测到黑线时输出低电平；若实测相反，将其改为GPIO_HIGH。
#define IR_ACTIVE_LEVEL     (GPIO_LOW)

extern bool enable_ir;
extern uint8 ir_value;
extern uint8 ir_data[IR_CHANNEL_NUM];

void IR_Init (void);
uint8 IR_Read (void);
void IR_Update (void);

#endif
