#ifndef _TASK_H_
#define _TASK_H_

#include "Action.h"
#include "Gray.h"
#include "Gray_Line.h"
#include "Motor_PID.h"

#define TASK_ALL_WHITE_MS              (230)
#define TASK_ALL_WHITE_COUNT           ((TASK_ALL_WHITE_MS + MOTOR_PID_PERIOD_MS - 1) / MOTOR_PID_PERIOD_MS)
#define TASK_LEFT_TURN_ANGLE           (90.0f)

extern bool enable_task;
extern uint16 task_all_white_count;
extern uint8 task_turn_lock;
extern float task_turn_target_angle;

void Task_Init ();
void Task_Update ();

#endif
