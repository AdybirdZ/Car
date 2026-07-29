#ifndef _BALL_PID_H_
#define _BALL_PID_H_

#include "zf_common_headfile.h"
#include "Serial.h"
#include "Step.h"

// K230返回0表示钢球位于画面中心，PID输出单位为步进电机目标角度（度）。
#define BALL_PID_TARGET_DEFAULT          (0.0f)
#define BALL_PID_KP_DEFAULT              (0.0f)
#define BALL_PID_KI_DEFAULT              (0.0f)
#define BALL_PID_KD_DEFAULT              (0.0f)
#define BALL_PID_OUTPUT_MAX_DEFAULT      (30.0f)
#define BALL_PID_INTEGRAL_MAX_DEFAULT    (1000.0f)
#define BALL_PID_STEP_FREQUENCY_DEFAULT  (500)


#define BALL_PID_OUTPUT_DIRECTION        (1.0f)     // 若实车调试发现电机修正方向相反，只需把1.0f改为-1.0f

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
    uint32 step_frequency_hz;
} Ball_PID_Struct;

extern Ball_PID_Struct Ball_PID;
extern volatile int16 ball_k230_position;
extern volatile uint8 ball_k230_position_ready;
extern float ball_step_target_angle;

void Ball_PID_Init (void);
void Ball_PID_Set (float kp, float ki, float kd);
void Ball_PID_Set_Target (float target);
void Ball_PID_Set_Step_Frequency (uint32 frequency_hz);
void Ball_PID_Clear (void);
float Ball_PID_Calc (float actual);
uint8 Ball_PID_Process (void);

#endif
