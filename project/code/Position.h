#ifndef _POSITION_H_
#define _POSITION_H_

#include "zf_common_headfile.h"

#define ROLL                        (0)
#define PITCH                       (1)
#define YAW                         (2)
#define POSITION_POLLING_ENABLE     (1)     // 1=由PIT主动读取IMU，避免依赖INT2外部中断

extern float euler_angle[3];
extern bool enable_position;

void Position_Init ();
void Position_Update ();

#endif
