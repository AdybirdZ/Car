#ifndef _STEP_H_
#define _STEP_H_

#include "zf_common_headfile.h"

// D36A第一路控制接口：ST1->B10，DIR1->B11，EN1->B2，且控制板与驱动板必须共地。
#define STEP_PWM_PIN                  (PWM_TIM_G6_CH0_B10)
#define STEP_DIR_PIN                  (B11)
#define STEP_EN_PIN                   (B2)

#define STEP_DIRECTION_FORWARD        (0)
#define STEP_DIRECTION_REVERSE        (1)

#define STEP_DEFAULT_FREQUENCY_HZ     (2)                   // 2Hz表示每0.5秒产生一个脉冲
#define STEP_PWM_DUTY                 (1U)                  // 2Hz时高电平约46us，接近官方例程的短脉冲

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

#endif
