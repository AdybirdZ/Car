#ifndef _TASK_H_
#define _TASK_H_

#include "Action.h"
#include "Gray.h"
#include "Gray_Line.h"
#include "Motor_PID.h"

#define TASK_ALL_WHITE_MS              (300)
#define TASK_ALL_WHITE_COUNT           (TASK_ALL_WHITE_MS / MOTOR_PID_PERIOD_MS)

extern bool enable_task;
extern uint16 task_all_white_count;
extern uint8 task_turn_lock;

void Task_Init ();
void Task_Update ();

#endif
