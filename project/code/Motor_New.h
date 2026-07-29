#ifndef _MOTOR_NEW_H_
#define _MOTOR_NEW_H_

#include "zf_common_headfile.h"

#define MOTOR_NEW_LEFT                     (0U)
#define MOTOR_NEW_RIGHT                    (1U)

#define MOTOR_NEW_STOP                     (0U)
#define MOTOR_NEW_FORWARD                  (1U)
#define MOTOR_NEW_REVERSE                  (2U)

#define MOTOR_NEW_PWM_FREQUENCY_HZ         (10000U)
#define MOTOR_NEW_DUTY_MAX                 (4000)

// 左轮：TB6612 A 通道
#define MOTOR_NEW_LEFT_IN1_PIN             (A27)
#define MOTOR_NEW_LEFT_IN2_PIN             (A26)
#define MOTOR_NEW_LEFT_PWM_PIN             (PWM_TIM_A0_CH0_B8)

// 右轮：TB6612 B 通道
#define MOTOR_NEW_RIGHT_IN1_PIN            (B13)
#define MOTOR_NEW_RIGHT_IN2_PIN            (B12)
#define MOTOR_NEW_RIGHT_PWM_PIN            (PWM_TIM_A0_CH1_B9)

extern volatile int32 motor_new_duty[2];
extern volatile uint8 motor_new_direction[2];

void Motor_New_Init (void);
void Motor_New_Set_Direction (uint8 motor, uint8 direction);
void Motor_New_Set_Duty (uint8 motor, int32 duty);
void Motor_New_Set_Output (uint8 motor, int32 output);
void Motor_New_Brake (uint8 motor);
void Motor_New_Stop_All (void);
uint8 Motor_New_Get_Direction (uint8 motor);

#endif
