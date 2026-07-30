#ifndef _TASK_H_
#define _TASK_H_

#include "Gray_Line.h"
#include "Motor_PID_New.h"

// 灰度通道0到7按车体从左至右排列；2、3、4、5为中间四路。
#define TASK_STOP_LEFT_2_INDEX         (2)
#define TASK_STOP_LEFT_1_INDEX         (3)
#define TASK_STOP_RIGHT_1_INDEX        (4)
#define TASK_STOP_RIGHT_2_INDEX        (5)
#define TASK_STOP_DELAY_MS             (520)
#define TASK_UPDATE_PERIOD_MS          (10)
#define TASK_STOP_DELAY_TICKS          (TASK_STOP_DELAY_MS / TASK_UPDATE_PERIOD_MS)

extern bool enable_task;
extern uint8 task_stop_flag;

void Task_Init (void);
void Task_Update (void);

#endif
