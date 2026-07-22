#ifndef _ACC_PID_H_
#define _ACC_PID_H_

#include "Position.h"

#define PI                              (3.14159f)
#define ACC_PID_PERIOD_MS               (20)
#define ACC_PID_PERIOD_S                (ACC_PID_PERIOD_MS / 1000.0f)
#define ACC_PID_GRAVITY                 (9.80665f)
#define ACC_PID_CALIBRATION_COUNT       (50)

extern bool enable_acc_position;
extern volatile float acc_horizontal_x;
extern volatile float acc_horizontal_y;
extern volatile float acc_velocity_x;
extern volatile float acc_velocity_y;
extern volatile float acc_distance_x;
extern volatile float acc_distance_y;
extern volatile float acc_bias_x;
extern volatile float acc_bias_y;

void Acc_PID_Init ();
void Acc_PID_Update ();
void Acc_PID_Reset ();

#endif
