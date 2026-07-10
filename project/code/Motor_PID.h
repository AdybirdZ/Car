#ifndef _MOTOR_PID_H_
#define _MOTOR_PID_H_

#include "Encoder.h"
#include "Motor.h"

#define MOTOR_PID_PERIOD_MS        (5)

typedef struct
{
    float kp;
    float ki;
    float kd;

    float target;
    float actual;
    float err;
    float err_last;
    float integral;
    float output;

    float integral_max;
    float output_max;
} Motor_PID_Struct;

extern Motor_PID_Struct Motor1_PID;
extern Motor_PID_Struct Motor2_PID;

void Motor_PID_Init (Motor_PID_Struct *pid, float kp, float ki, float kd, float output_max, float integral_max);
void Motor_PID_Set (Motor_PID_Struct *pid, float kp, float ki, float kd);
void Motor_PID_Clear (Motor_PID_Struct *pid);
float Motor_PID_Calc (Motor_PID_Struct *pid, float target, float actual);
int8 Motor_PID_Control (Motor_PID_Struct *pid, float target, float actual, int8 motor);

#endif