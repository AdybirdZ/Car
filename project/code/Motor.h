#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "zf_common_headfile.h"

#define MOTOR1_DIR               (B13 )                     // 注：MOTOR1相关代码为右轮
#define MOTOR1_PWM               (PWM_TIM_A0_CH1_B12)

#define MOTOR2_DIR               (B9 )                      // 注：MOTOR2相关代码为左轮
#define MOTOR2_PWM               (PWM_TIM_A0_CH0_B8 )

#define LEFT_MOTOR               (0)
#define RIGHT_MOTOR              (1)

#define PWM_MAX                  (25)

extern bool enable_motor_output;
extern volatile float motor_pwm_duty[2];

void Motor_Init ();
void Set_PWM (float duty, int8 motor);

#endif
