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
#define TASK1_TARGET_POSITION_CM        (0.0f)
#define TASK1_BALL_ANGLE_PER_CM         (2.0f)
#define TASK1_ZERO_TARGET_OFFSET_ANGLE  (0.0f)    // 钢球位于0cm时的相对目标角度
#define TASK1_STEP_FREQUENCY_HZ         (2000U)
#define TASK1_SAMPLE_PERIOD_S           (0.1f)    // K230位置和速度每0.1秒更新一次

// 钢球位置、速度串级控制参数：位置误差平方后换算为目标速度，速度误差再换算为摆杆倾角。
#define BALL_VELOCITY_TARGET_PER_CM2    (2.0f)    // 偏离目标1cm时目标速度2cm/s；偏离2cm时目标速度8cm/s
#define BALL_VELOCITY_MAX_TARGET_SPEED  (8.0f)    // 限制远离目标时的最大目标速度，单位cm/s
#define BALL_VELOCITY_ANGLE_PER_CM_PER_S (1.0f)  // 速度误差1cm/s，对应1度的相对目标倾角
#define BALL_VELOCITY_MAX_TARGET_ANGLE  (30.0f)   // 保护步进电机和机构的最大相对倾角

// mode=3：钢球从0cm运行到+5cm，再返回并稳定在-5cm。
#define TASK3_MODE_NUMBER               (3U)
#define TASK6_MODE_NUMBER               (6U)
#define TASK6_ANGLE_STEP_DEG            (0.05f)
#define TASK3_POSITIVE_POSITION_CM      (5.0f)
#define TASK3_NEGATIVE_POSITION_CM      (-5.0f)
#define TASK3_POSITION_TOLERANCE_CM     (1.0f)
#define TASK3_POSITIVE_REVERSE_POSITION_CM (3.0f)  // 到达此位置后由正向运动切换为负向运动
#define TASK3_TO_POSITIVE_ANGLE         (10.0f)    // 实测正角度使钢球向正半轴运动
#define TASK3_TO_NEGATIVE_ANGLE         (-6.0f)   // 实测负角度使钢球向负半轴运动
#define TASK3_MOVE_FREQUENCY_HZ         (500U)
#define TASK3_BASE_ANGLE_PER_CM         (1.0f)     // 水管弯曲对应的静态位置基准
#define TASK3_NEGATIVE_HOLD_ANGLE_OFFSET (-0.5f)  // -5cm位置的额外静态倾角偏置，当前使基准由-5度变为-5.5度
#define TASK3_NEGATIVE_HOLD_BASE_ANGLE  (TASK1_ZERO_TARGET_OFFSET_ANGLE + TASK3_NEGATIVE_POSITION_CM * TASK3_BASE_ANGLE_PER_CM + TASK3_NEGATIVE_HOLD_ANGLE_OFFSET)
#define TASK3_STABLE_MIN_POSITION_CM    (-6.0f)
#define TASK3_STABLE_MAX_POSITION_CM    (-4.0f)
#define TASK3_STABLE_TIME_TENTHS        (10U)      // 10个0.1秒计时单位，即连续稳定1秒
#define TASK3_NEGATIVE_BRAKE_POSITION_CM (-2.1f)   // 前往-5cm途中提前急刹的位置
#define TASK3_NEGATIVE_BRAKE_ANGLE       (30.0f)   // 到达急刹点时的固定相对目标角度
#define TASK3_BRAKE_FREQUENCY_HZ        (3000U)

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
    TASK3_HOLD_NEGATIVE,
    TASK3_DONE
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
float Task_Calculate_Velocity_Control_Angle (float target_position_cm,
                                             float ball_position_cm,
                                             float ball_speed_cm_per_s);
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
void Task6_Init (void);
void Task6_Cancel (void);
void Task6 (void);

#endif
