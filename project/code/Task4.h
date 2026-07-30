#ifndef _TASK4_H_
#define _TASK4_H_

#include "zf_common_headfile.h"

#define TASK4_MODE_NUMBER                 (4U)
#define TASK4_GRAY_TARGET                 (300.0f)
#define TASK4_RUN_TIME_TENTHS             (80U)
#define TASK4_BALL_MAX_CM                 (12.0f)
#define TASK4_BALL_ANGLE_PER_CM           (0.3f)
#define TASK4_STEP_FREQUENCY_HZ           (500U)

typedef enum
{
    TASK4_IDLE = 0,
    TASK4_WAIT_K230,
    TASK4_READY,
    TASK4_RUNNING,
    TASK4_DONE
} Task4_State;

extern volatile Task4_State task4_state;
extern volatile float task4_ball_position_cm;
extern volatile float task4_step_target_angle;

void Task4_Init (void);
void Task4_Prepare (void);
void Task4_Cancel_Prepare (void);
uint8 Task4_Is_Ready (void);
void Task4_Start (void);
void Task4_Tick_10ms (void);
void Task4_Process (void);

#endif
