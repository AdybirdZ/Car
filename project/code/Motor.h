#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "zf_common_headfile.h"

#define MOTOR1_DIR               (B13 )
#define MOTOR1_PWM               (PWM_TIM_A0_CH1_B12)

#define MOTOR2_DIR               (B9 )
#define MOTOR2_PWM               (PWM_TIM_A0_CH0_B8 )

#define MOTOR1                   (1)
#define MOTOR2                   (2)

void Motor_Init ();
void Set_PWM (int8 duty, int8 motor);

#endif