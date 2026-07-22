#ifndef _STRAIGHT_PID_H_
#define _STRAIGHT_PID_H_

#include "Angle_PID.h"
#include "Motor_PID.h"

#define STRAIGHT_PID_PERIOD_MS         (20)
#define STRAIGHT_PID_INTEGRAL_MAX      (1000.0f)
#define STRAIGHT_PID_OUTPUT_MAX        (MOTOR_PID_TARGET_OFFSET)        // 最大输出为一个轮子目标速度两倍，一个轮子速度为0

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
} Straight_PID_Struct;

extern Straight_PID_Struct Straight_PID;
extern volatile bool enable_straight_pid;
extern volatile float straight_angle_target;
extern volatile float straight_angle_actual;
extern volatile float straight_base_speed;
extern volatile float straight_speed_correction;
extern volatile float straight_motor_target[2];

void Straight_PID_Structure_Init (Straight_PID_Struct *pid, float kp, float ki, float kd, float output_max, float integral_max);
void Straight_PID_Start (float angle_target, float base_speed);
void Straight_PID_Stop ();
void Straight_PID_Set (Straight_PID_Struct *pid, float kp, float ki, float kd);
void Straight_PID_Clear (Straight_PID_Struct *pid);
float Straight_PID_Calc (Straight_PID_Struct *pid, float target, float actual);
void Straight_PID_Control (Straight_PID_Struct *pid, float target, float actual, float base_speed);

#endif
