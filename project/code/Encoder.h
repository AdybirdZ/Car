#ifndef _ENCODER_H_
#define _ENCODER_H_

#include "zf_common_headfile.h"
#include "Motor.h"

#define LEFT_ENCODER_INDEX         (1)
#define RIGHT_ENCODER_INDEX        (0)
#define ENCODER_RESOLUTION         (4096)
#define ENCODER_SPEED_OFFSET_MAX   (3800.0f)

void Encoder_Init ();

#endif
