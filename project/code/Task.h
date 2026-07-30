#ifndef _TASK_H_
#define _TASK_H_

#include "Gray_Line.h"
#include "Motor_PID_New.h"
#include "Serial.h"
#include "Step.h"

// 灰度通道0到7按车体从左至右排列；2、3、4、5为中间四路。
#define TASK_STOP_LEFT_2_INDEX          (2)
#define TASK_STOP_LEFT_1_INDEX          (3)
#define TASK_STOP_RIGHT_1_INDEX         (4)
#define TASK_STOP_RIGHT_2_INDEX         (5)
#define TASK_STOP_DELAY_MS              (520)
#define TASK_UPDATE_PERIOD_MS           (10)
#define TASK_STOP_DELAY_TICKS           (TASK_STOP_DELAY_MS / TASK_UPDATE_PERIOD_MS)

// mode=1静止钢球平衡参数。
#define TASK1_MODE_NUMBER               (1U)
#define TASK1_BALL_ANGLE_PER_CM         (2.0f)
#define TASK1_ZERO_TARGET_OFFSET_ANGLE  (0.0f)    // 钢球位于0cm时的相对目标角度
#define TASK1_BRAKE_TRIGGER_CM          (1.0f)    // 从外侧返回中心、穿过正负1cm时触发一次急刹
#define TASK1_BRAKE_ANGLE_PER_CM_PER_S  (5.0f)    // 速度每增加1cm/s，急刹幅值增加5度
#define TASK1_BRAKE_MAX_ANGLE           (30.0f)
#define TASK1_BRAKE_FREQUENCY_HZ        (3000U)
#define TASK1_SAMPLE_PERIOD_S           (0.1f)    // K230位置和速度每0.1秒更新一次

// mode=3：钢球从0cm运行到+5cm，再返回并稳定在-5cm。
#define TASK3_MODE_NUMBER               (3U)
#define TASK3_POSITIVE_POSITION_CM      (5.0f)
#define TASK3_NEGATIVE_POSITION_CM      (-5.0f)
#define TASK3_POSITION_TOLERANCE_CM     (1.0f)
#define TASK3_TO_POSITIVE_ANGLE         (-5.0f)   // 编码器角度减小，使钢球向正方向加速
#define TASK3_TO_NEGATIVE_ANGLE         (5.0f)    // 编码器角度增大，使钢球向负方向加速
#define TASK3_MOVE_FREQUENCY_HZ         (500U)
#define TASK3_BALL_ANGLE_PER_CM         (1.0f)

// mode=4钢球平衡巡线测试参数。
#define TASK4_MODE_NUMBER               (4U)
#define TASK4_GRAY_TARGET               (200.0f)
#define TASK4_RUN_TIME_TICKS            (800U)     // 10ms * 800 = 8.00s
#define TASK4_BALL_MAX_CM               (12.0f)
#define TASK4_BALL_ANGLE_PER_CM         (0.3f)     // 位置权重：每偏离1cm，目标倾斜0.3度
#define TASK4_STEP_FREQUENCY_HZ         (500U)     // K230钢球平衡时的步进频率
#define TASK4_LAUNCH_STEP_FREQUENCY_HZ  (3000U)    // 含8脉冲分段反馈，20度估算约87ms
#define TASK4_LAUNCH_TILT_ANGLE         (-20.0f)   // Step.c约定负角度为顺时针

typedef enum
{
    TASK4_IDLE = 0,
    TASK4_WAIT_K230,
    TASK4_READY,
    TASK4_RUNNING,
    TASK4_DONE
} Task4_State;

typedef enum
{
    TASK3_IDLE = 0,
    TASK3_WAIT_K230,
    TASK3_READY,
    TASK3_MOVE_TO_POSITIVE,
    TASK3_MOVE_TO_NEGATIVE,
    TASK3_HOLD_NEGATIVE
} Task3_State;

extern bool enable_task;
extern uint8 task_stop_flag;
extern volatile Task4_State task4_state;
extern volatile float task4_ball_position_cm;
extern volatile float task4_step_target_angle;
extern volatile float task3_absolute_target_angle;

void Task_Init (void);
void Task_Update (void);
void Task1_Init (void);
void Task1_Prepare (void);
void Task1_Cancel_Prepare (void);
uint8 Task1_Is_Ready (void);
void Task1_Start (void);
void Task1 (void);
float Task_Calculate_Absolute_Target_Angle (float target_position_cm);
void Task3_Init (void);
void Task3_Prepare (void);
void Task3_Cancel_Prepare (void);
uint8 Task3_Is_Ready (void);
void Task3_Start (void);
void Task3 (void);
void Task4_Init (void);
void Task4_Prepare (void);
void Task4_Cancel_Prepare (void);
uint8 Task4_Is_Ready (void);
void Task4_Start (void);
void Task4_Tick_10ms (void);
void Task4 (void);

#endif
