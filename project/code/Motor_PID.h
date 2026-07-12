#ifndef _MOTOR_PID_H_
#define _MOTOR_PID_H_

#include "Encoder.h"
#include "Motor.h"

#define MOTOR_PID_PERIOD_MS        (20)
#define MOTOR_PID_TARGET_OFFSET    (500.0f)
#define MOTOR_PID_OUTPUT_MAX       (3.0f)
#define MOTOR_PID_INTEGRAL_MAX     (700.0f)

#define MOTOR_LEFT_OFFSET_PER_DUTY     (51.0f)
#define MOTOR_RIGHT_OFFSET_PER_DUTY    (31.8f)

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

extern Motor_PID_Struct Motor_Left_PID;
extern Motor_PID_Struct Motor_Right_PID;
extern volatile float motor_target_offset[2];
extern volatile int16 motor_encoder_location[2];
extern volatile int16 motor_encoder_offset[2];

void Motor_PID_Init (Motor_PID_Struct *pid, float kp, float ki, float kd, float output_max, float integral_max);
void Motor_PID_Set (Motor_PID_Struct *pid, float kp, float ki, float kd);
void Motor_PID_Clear (Motor_PID_Struct *pid);
float Motor_PID_Calc (Motor_PID_Struct *pid, float target, float actual);
float Motor_PID_Control (Motor_PID_Struct *pid, float target, float actual, int8 motor);

#endif
