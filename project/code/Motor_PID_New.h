#ifndef _MOTOR_PID_NEW_H_
#define _MOTOR_PID_NEW_H_

#include "zf_common_headfile.h"
#include "Motor_New.h"
#include "Encoder_New.h"

#define MOTOR_PID_NEW_PERIOD_MS          (10U)
#define MOTOR_PID_NEW_TIMER              (PIT_TIM_G0)
#define MOTOR_PID_NEW_TARGET_DEFAULT     (200.0f)
#define MOTOR_PID_NEW_KP_DEFAULT         (0.025f)
#define MOTOR_PID_NEW_KI_DEFAULT         (0.5f)
#define MOTOR_PID_NEW_KD_DEFAULT         (0.0f)

typedef struct
{
    float kp;
    float ki;
    float kd;
    float target;
    float actual;
    float error;
    float last_error;
    float previous_error;
    float output;
} Motor_PID_New_Struct;

extern Motor_PID_New_Struct motor_pid_new[2];
extern volatile bool enable_motor_pid_new;

void Motor_PID_New_Init (void);
void Motor_PID_New_Set (uint8 motor, float kp, float ki, float kd);
void Motor_PID_New_Set_Target (uint8 motor, float target);
void Motor_PID_New_Set_Targets (float left_target, float right_target);
void Motor_PID_New_Start (float left_target, float right_target);
void Motor_PID_New_Stop (void);
void Motor_PID_New_Clear (uint8 motor);
float Motor_PID_New_Control (uint8 motor);
void Motor_PID_New_Process (void);

#endif
