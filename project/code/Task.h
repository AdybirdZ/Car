#ifndef _TASK_H_
#define _TASK_H_

#include "IR.h"
#include "IR_Line.h"
#include "Motor.h"

// IR1到IR8按车体从左至右排列；左2、左1、右1、右2为中间四路。
#define TASK_STOP_LEFT_2_INDEX         (2)
#define TASK_STOP_LEFT_1_INDEX         (3)
#define TASK_STOP_RIGHT_1_INDEX        (4)
#define TASK_STOP_RIGHT_2_INDEX        (5)

extern bool enable_task;
extern uint8 task_stop_flag;

void Task_Init (void);
void Task_Update (void);

#endif
