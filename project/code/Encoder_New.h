#ifndef _ENCODER_NEW_H_
#define _ENCODER_NEW_H_

#include "zf_common_headfile.h"

#define ENCODER_NEW_LEFT                (0)
#define ENCODER_NEW_RIGHT               (1)

#define ENCODER_NEW_LEFT_A_PIN          (B6)
#define ENCODER_NEW_LEFT_B_PIN          (B7)
#define ENCODER_NEW_RIGHT_A_PIN         (A16)
#define ENCODER_NEW_RIGHT_B_PIN         (A17)

#define ENCODER_NEW_PULSES_PER_REV      (455.0f)
#define ENCODER_NEW_WHEEL_DIAMETER_MM   (50.0f)
#define ENCODER_NEW_SAMPLE_PERIOD_MS    (10U)
#define ENCODER_NEW_PI                  (3.1415926f)

extern volatile uint32 encoder_new_count[2];
extern volatile float encoder_new_speed[2];

void Encoder_New_Init (void);
void Encoder_New_Update_Speed (void);
void Encoder_New_Clear (void);
float Encoder_New_Get_Speed (uint8 motor);

#endif
