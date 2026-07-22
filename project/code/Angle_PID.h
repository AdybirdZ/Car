#ifndef _ANGLE_PID_H_
#define _ANGLE_PID_H_

#include "Motor.h"

#define ANGLE_PID_PERIOD_MS         (20)
#define ANGLE_PID_INTEGRAL_MAX      (1000.0f)
#define ANGLE_PID_TARGET_DEFAULT    (0.0f)

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
} Angle_PID_Struct;

extern Angle_PID_Struct Angle_PID;
extern volatile bool enable_angle_pid;
extern volatile float angle_target;
extern volatile float angle_actual;

void Angle_PID_Structure_Init (Angle_PID_Struct *pid, float kp, float ki, float kd, float output_max, float integral_max);
void Angle_PID_Target_Init (float target);
void Angle_PID_Set (Angle_PID_Struct *pid, float kp, float ki, float kd);
void Angle_PID_Clear (Angle_PID_Struct *pid);
float Angle_PID_Calc (Angle_PID_Struct *pid, float target, float actual);
void Angle_PID_Control (Angle_PID_Struct *pid, float target, float actual);
float Angle_Normalize (float angle);
float Angle_Error (float target, float actual);

#endif
