#ifndef _STEP_H_
#define _STEP_H_

#include "zf_common_headfile.h"

// D36A第一路控制接口：ST1->B10，DIR1->B11，EN1->B2，且控制板与驱动板必须共地
#define STEP_PWM_PIN                  (PWM_TIM_G6_CH0_B10)
#define STEP_DIR_PIN                  (B11)
#define STEP_EN_PIN                   (B2)

#define STEP_DIRECTION_FORWARD        (0)                   // 正转
#define STEP_DIRECTION_REVERSE        (1)                   // 反转

#define STEP_DEFAULT_FREQUENCY_HZ     (2)                   // 2Hz表示每0.5秒产生一个脉冲
#define STEP_PWM_DUTY                 (1)                   // 官方默认占空比为10000，所以1/10000=0.01%，能产生短脉冲

// 步进电机和驱动板参数：常见两相步进电机为200整步/圈，D36A默认设置为16细分
#define STEP_MOTOR_FULL_STEPS         (200)
#define STEP_MOTOR_MICROSTEP         (16)

extern uint32 step_frequency_hz;
extern uint8 step_direction;
extern bool step_enabled;
extern bool step_running;

void Step_Init (void);
void Step_Set_Frequency (uint32 frequency_hz);
void Step_Set_Direction (uint8 direction);
void Step_Enable (void);
void Step_Disable (void);
void Step_Start (void);
void Step_Stop (void);

// 按带符号角度转动，函数返回后停止脉冲但继续保持电机使能
void Step_Move_Angle (float angle_deg, uint32 frequency_hz);

#endif
