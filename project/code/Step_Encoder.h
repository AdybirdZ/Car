#ifndef _STEP_ENCODER_H_
#define _STEP_ENCODER_H_

#include "zf_common_headfile.h"

// MS42CG默认接口：A/B为1024线正交信号，四倍频，每圈4096个计数
#define STEP_ENCODER_A_PIN                 (A0)
#define STEP_ENCODER_B_PIN                 (A1)
#define STEP_ENCODER_PWM_PIN               (B20)
#define STEP_ENCODER_Z_PIN                 (B21)

#define STEP_ENCODER_COUNTS_PER_REV        (4096.0f)
// 16细分时每个脉冲约对应1.28个编码器计数；1计数死区可避免跨微步来回修正。
#define STEP_ENCODER_TOLERANCE_COUNT       (1)
#define STEP_ENCODER_MAX_BURST_PULSES      (8)
#define STEP_ENCODER_NO_MOVE_PULSE_LIMIT   (64)
#define STEP_ENCODER_MAX_CONTROL_CYCLES    (2500)
#define STEP_ENCODER_PWM_SAMPLE_COUNT      (5)
#define STEP_ENCODER_PWM_MAX_SPREAD_DEG    (2.0f)
// mode=6按键步长为0.1125度，因此绝对定位死区必须小于一个微步。
#define STEP_ENCODER_STARTUP_TOLERANCE_DEG (0.06f)
#define STEP_ENCODER_STARTUP_MAX_BURST     (32)
#define STEP_ENCODER_STARTUP_MAX_CYCLES    (300)
// mode=7 手动标定每次只允许输出一个微步；较宽死区防止在机械回差附近反复往返。
#define STEP_ENCODER_MODE7_TOLERANCE_DEG   (0.08f)
#define STEP_ENCODER_MODE7_MAX_BURST       (1)
#define STEP_ENCODER_MODE7_MAX_CYCLES      (12)

// 正转时若实际角度/计数减小，把该值改为-1；不用改A、B接线。
#define STEP_ENCODER_COUNT_SIGN            (1)

// ===== 上电初始化目标角度：在此行填写 0.0~360.0 的机械绝对角度 =====
#define STEP_ENCODER_DEFAULT_STARTUP_TARGET_ANGLE  (79.13f)
#define STEP_ENCODER_STARTUP_FREQUENCY_HZ  (500)
#define STEP_ENCODER_PRESTART_MAX_POSITIVE_OFFSET_DEG (20.0f)

typedef enum
{
    STEP_ENCODER_OK = 0,
    STEP_ENCODER_PWM_ERROR,
    STEP_ENCODER_NO_FEEDBACK,
    STEP_ENCODER_DIRECTION_ERROR,
    STEP_ENCODER_TIMEOUT
} Step_Encoder_Status;

extern volatile int32 step_encoder_count;
extern volatile int32 step_encoder_zero_count;
extern volatile uint32 step_encoder_z_count;
extern volatile float step_encoder_initial_absolute_angle;
extern volatile uint8 step_encoder_initial_absolute_valid;
extern float step_encoder_startup_target_angle;

uint8 Step_Encoder_Init (void);
uint8 Step_Encoder_Read_Absolute_Angle (float *angle);
uint8 Step_Encoder_Is_Above_Prestart_Limit (void);
float Step_Encoder_Get_Startup_Target_Angle (void);
uint8 Step_Encoder_Set_Startup_Target_Angle (float angle);
uint8 Step_Encoder_Save_Startup_Target_Angle (void);
float Step_Encoder_Get_Relative_Angle (void);
float Step_Encoder_Relative_To_Absolute_Angle (float relative_angle);
void Step_Encoder_Set_Zero (void);
Step_Encoder_Status Step_Encoder_Move_To_Relative_Angle (float angle, uint32 frequency_hz);
Step_Encoder_Status Step_Encoder_Goto_Startup_Angle (void);
Step_Encoder_Status Step_Encoder_Goto_Startup_Angle_Mode7 (void);
Step_Encoder_Status Step_Encoder_Get_Status (void);

#endif
