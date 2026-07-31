#ifndef _STEP_H_
#define _STEP_H_

#include "zf_common_headfile.h"

// D36A第一路控制接口：ST1->B10，DIR1->B11，EN1->B2，且控制板与驱动板必须共地
#define STEP_PWM_PIN                  (PWM_TIM_G6_CH0_B10)
#define STEP_PULSE_GPIO_PIN           (B10)
#define STEP_DIR_PIN                  (B11)
#define STEP_EN_PIN                   (B2)

#define STEP_DIRECTION_FORWARD        (0)                   // 正转
#define STEP_DIRECTION_REVERSE        (1)                   // 反转

#define STEP_DEFAULT_FREQUENCY_HZ     (2)                   // 2Hz表示每0.5秒产生一个脉冲
#define STEP_PULSE_WIDTH_US           (5)                   // D36A官方例程中提到，使用约5us的高电平脉冲
#define STEP_PWM_DUTY_MIN             (1)                   // 低频时至少保留一个PWM计数单位

// 步进电机和驱动板参数：200整步一圈，D36A默认设置为16细分
#define STEP_MOTOR_FULL_STEPS         (200)
#define STEP_MOTOR_MICROSTEP          (16)
#define STEP_MOTOR_MICROSTEP_ANGLE_DEG \
    (360.0f / (STEP_MOTOR_FULL_STEPS * STEP_MOTOR_MICROSTEP))

extern uint32 step_frequency_hz;
extern uint8 step_direction;
extern bool step_enabled;
extern bool step_running;
extern float step_current_angle;

void Step_Init (void);
void Step_Set_Frequency (uint32 frequency_hz);
void Step_Set_Direction (uint8 direction);
void Step_Enable (void);
void Step_Disable (void);
void Step_Start (void);
void Step_Stop (void);
uint32 Step_Get_PWM_Duty (uint32 frequency_hz);
void Step_Output_Pulses (uint32 pulse_count, uint32 frequency_hz);

// 按带符号角度转动，函数返回后停止脉冲但继续保持电机使能
void Step_Move_Angle (float angle, uint32 frequency_hz);

// 从当前角度转到指定的绝对角度；Step_Init后当前角度定义为0度
void Step_To_Angle (float angle, uint32 frequency_hz);

#endif
