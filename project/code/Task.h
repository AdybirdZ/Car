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
#define TASK_STOP_CONFIRM_TICKS         (5)      // 3路黑色连续50ms后确认停车线

// mode=1静止钢球平衡参数。
#define TASK1_MODE_NUMBER               (1)
#define TASK1_TARGET_POSITION_CM        (0.0f)
#define TASK1_BALL_ANGLE_PER_CM         (2.0f)
#define TASK1_ZERO_TARGET_OFFSET_ANGLE  (0.0f)    // 钢球位于0cm时的相对目标角度
#define TASK1_STEP_FREQUENCY_HZ         (2000)
#define TASK1_SAMPLE_PERIOD_S           (0.1f)    // K230位置和速度每0.1秒更新一次
#define TASK1_INTEGRAL_KI_DEG_PER_CM_S  (0.3f)   // 位置积分每1cm*s补偿0.3度
#define TASK1_INTEGRAL_ERROR_LIMIT_CM   (1.0f)    // 只积累中心附近的小误差
#define TASK1_INTEGRAL_MAX_CM_S         (10.0f)   // 积分限幅，最大补偿约1.5度
#define TASK1_INTEGRAL_MAX_DT_S         (0.2f)    // 丢帧时限制单次积分时间

// 钢球位置、速度串级控制参数：位置误差平方后换算为目标速度，速度误差再换算为摆杆倾角。
#define BALL_VELOCITY_TARGET_PER_CM2    (2.0f)    // 偏离目标1cm时目标速度2cm/s；偏离2cm时目标速度8cm/s
#define BALL_VELOCITY_MAX_TARGET_SPEED  (8.0f)    // 限制远离目标时的最大目标速度，单位cm/s
#define BALL_VELOCITY_ANGLE_PER_CM_PER_S (1.9f)   // 速度误差1cm/s，对应1.9度的相对目标倾角
#define BALL_VELOCITY_MAX_TARGET_ANGLE  (30.0f)   // 保护步进电机和机构的最大相对倾角
#define BALL_POSITION_FILTER_ALPHA      (0.5f)    // K230位置一阶低通系数
#define TASK3_NEGATIVE_MAX_ANGLE_STEP_DEG (1.5f)  // 负向终点每次最多改变的摆杆角度
#define TASK4_BALL_MAX_ANGLE_STEP_DEG   (1.2f)    // 行驶中每次最多改变的摆杆角度
#define TASK5_BALL_MAX_ANGLE_STEP_DEG   (1.2f)

#undef BALL_POSITION_FILTER_ALPHA
#define BALL_POSITION_FILTER_ALPHA        (0.60f)

// mode=3：钢球从0cm运行到+5cm，再返回并稳定在-5cm。
#define TASK3_MODE_NUMBER               (3)
// D36A当前为16细分，每次按键严格移动一个STEP脉冲对应的角度。
#define TASK6_ANGLE_STEP_DEG            (STEP_MOTOR_MICROSTEP_ANGLE_DEG)
#define TASK7_ANGLE_STEP_DEG            (0.10f)
#define TASK3_POSITIVE_POSITION_CM      (5.0f)
#define TASK3_NEGATIVE_POSITION_CM      (-5.0f)
#define TASK3_POSITION_TOLERANCE_CM     (1.0f)
#define TASK3_TO_POSITIVE_ANGLE         (13.0f)    // 正向起步倾角，增大以保证钢球能接近+5cm
#define TASK3_TO_NEGATIVE_ANGLE         (-10.0f)   // 实测负角度使钢球向负半轴运动
#define TASK3_MOVE_FREQUENCY_HZ         (500)
#define TASK3_POSITIVE_BRAKE_POSITION_CM (3.0f)    // 正向固定倾角推进至此处后急刹
#define TASK3_POSITIVE_BRAKE_ANGLE      (-40.0f)   // 正向运动的反向急刹角度
#define TASK3_POSITIVE_REVERSE_POSITION_CM (3.8f)  // 二次函数以+5cm为目标时，通过此处立即折返
#define TASK3_POSITIVE_CONTROL_FREQUENCY_HZ (TASK1_STEP_FREQUENCY_HZ)
#define TASK3_BASE_ANGLE_PER_CM         (1.0f)     // 水管弯曲对应的静态位置基准
#define TASK3_NEGATIVE_HOLD_ANGLE_OFFSET (0.8f)    // 上调约1度，使负向平衡点由约-6cm回到-5cm
#define TASK3_NEGATIVE_HOLD_BASE_ANGLE  (TASK1_ZERO_TARGET_OFFSET_ANGLE + TASK3_NEGATIVE_POSITION_CM * TASK3_BASE_ANGLE_PER_CM + TASK3_NEGATIVE_HOLD_ANGLE_OFFSET)
#define TASK3_NEGATIVE_BRAKE_POSITION_CM (-2.5f)   // 此处开始连续减速，避免钢球冲出-6cm
#define TASK3_NEGATIVE_TARGET_SPEED_PER_CM2 (0.7f) // 更早降低负向目标速度
#define TASK3_NEGATIVE_MAX_TARGET_SPEED  (3.0f)    // 负向阶段限速，单位cm/s
#define TASK3_NEGATIVE_ANGLE_PER_CM_PER_S (3.0f)   // 负向速度误差制动增益，单位度/(cm/s)
#define TASK3_BRAKE_FREQUENCY_HZ        (3000)    // +3cm处正向急刹的转动频率

// mode=4钢球平衡巡线测试参数。
#undef TASK3_TO_POSITIVE_ANGLE
#undef TASK3_POSITIVE_REVERSE_POSITION_CM
#define TASK3_TO_POSITIVE_ANGLE           (10.0f)
#define TASK3_POSITIVE_REVERSE_POSITION_CM (4.0f)

#undef TASK3_NEGATIVE_BRAKE_POSITION_CM
#define TASK3_NEGATIVE_BRAKE_POSITION_CM  (-2.5f)
#define TASK3_NEGATIVE_BRAKE_ANGLE         (40.0f)

/* Restore the last verified Task3 tuning without changing other tasks. */
#undef TASK3_TO_POSITIVE_ANGLE
#undef TASK3_POSITIVE_REVERSE_POSITION_CM
#undef TASK3_NEGATIVE_HOLD_ANGLE_OFFSET
#define TASK3_TO_POSITIVE_ANGLE            (10.0f)
#define TASK3_POSITIVE_REVERSE_POSITION_CM (4.0f)
#define TASK3_NEGATIVE_HOLD_ANGLE_OFFSET  (-0.5f)
#define TASK3_NEGATIVE_BRAKE_REFERENCE_SPEED_CM_PER_S (2.0f)
#define TASK3_NEGATIVE_BRAKE_POSITION_GAIN_CM_PER_CM_PER_S (0.30f)
#define TASK3_NEGATIVE_BRAKE_POSITION_ADJUST_LIMIT_CM (0.70f)
#define TASK3_NEGATIVE_SLOW_BRAKE_START_POSITION_CM (0.5f)
#define TASK3_NEGATIVE_HOLD_START_POSITION_CM (-4.0f)
#define TASK3_NEGATIVE_SLOW_BRAKE_END_ANGLE (15.0f)
#define TASK3_NEGATIVE_SPEED_REFERENCE_CM_PER_S (1.5f)
#define TASK3_NEGATIVE_SPEED_BRAKE_GAIN_DEG_PER_CM_PER_S (4.0f)
#define TASK3_NEGATIVE_SLOW_BRAKE_MAX_ANGLE (25.0f)

#define TASK4_MODE_NUMBER               (4)
#define TASK4_GRAY_TARGET               (200.0f)
#define TASK4_RUN_TIME_TICKS            (800)     // 10ms * 800 = 8.00s
#define TASK4_ACCEL_TIME_TICKS          (150)     // 1.5秒S曲线缓慢起步
#define TASK4_DECEL_TIME_TICKS          (145)     // 最后1.5秒S曲线缓慢停车
#define TASK4_ACCEL_TILT_ANGLE          (-6.0f)   // 降低起步补偿峰值，避免钢球被弹出
#define TASK4_DECEL_TILT_ANGLE          (14.5f)   // 减速补偿倾角；通常与起步补偿方向相反
#define TASK4_BALL_MAX_CM               (12.0f)
#define TASK4_STEP_FREQUENCY_HZ         (TASK1_STEP_FREQUENCY_HZ)

#undef TASK4_ACCEL_TIME_TICKS
#undef TASK4_DECEL_TIME_TICKS
#undef TASK4_ACCEL_TILT_ANGLE
#undef TASK4_DECEL_TILT_ANGLE
#define TASK4_ACCEL_TIME_TICKS           (200)
#define TASK4_DECEL_TIME_TICKS           (200)
#define TASK4_ACCEL_TILT_ANGLE           (-2.0f)
#define TASK4_DECEL_TILT_ANGLE           (2.0f)

// mode=5：环形线路行驶一圈，返回启停线后减速停车并继续平衡钢球。
#define TASK5_MODE_NUMBER               (5)
#define TASK5_GRAY_TARGET               (250.0f)
#define TASK5_MAX_TOTAL_TIME_TICKS      (3000)    // 10ms * 3000 = 30.00s
#define TASK5_ACCEL_TIME_TICKS          (150)
#define TASK5_DECEL_TIME_TICKS          (150)
#define TASK5_ACCEL_TILT_ANGLE          (-6.0f)
#define TASK5_DECEL_TILT_ANGLE          (15.0f)
#define TASK5_STEP_FREQUENCY_HZ         (TASK1_STEP_FREQUENCY_HZ)

#undef TASK5_ACCEL_TIME_TICKS
#undef TASK5_DECEL_TIME_TICKS
#undef TASK5_ACCEL_TILT_ANGLE
#undef TASK5_DECEL_TILT_ANGLE
#define TASK5_ACCEL_TIME_TICKS           (200)
#define TASK5_DECEL_TIME_TICKS           (200)
#define TASK5_ACCEL_TILT_ANGLE           (-2.0f)
#define TASK5_DECEL_TILT_ANGLE           (2.0f)

// mode=6：设置0cm角度
#define TASK6_MODE_NUMBER               (6)
#define TASK7_MODE_NUMBER               (7)
#define TASK6_TARGET_MIN_CM             (-11.0f)
#define TASK6_TARGET_MAX_CM             (11.0f)
#define TASK6_TARGET_COARSE_STEP_CM     (1.0f)
#define TASK6_TARGET_FINE_STEP_CM       (0.1f)
typedef enum
{
    TASK4_IDLE = 0,
    TASK4_WAIT_K230,
    TASK4_READY,
    TASK4_RUNNING,
    TASK4_HOLD,
    TASK4_DONE
} Task4_State;

typedef enum
{
    TASK3_IDLE = 0,
    TASK3_WAIT_K230,
    TASK3_READY,
    TASK3_MOVE_TO_POSITIVE,
    TASK3_CONTROL_TO_POSITIVE,
    TASK3_MOVE_TO_NEGATIVE,
    TASK3_HOLD_NEGATIVE,
    TASK3_DONE
} Task3_State;

typedef enum
{
    TASK6_SET_TARGET = 0,
    TASK6_READY,
    TASK6_RUNNING,
    TASK6_DONE
} Task6_State;

extern bool enable_task;
extern uint8 task_stop_flag;
extern volatile Task4_State task4_state;
extern volatile float task4_ball_position_cm;
extern volatile float task4_step_target_angle;
extern volatile float task3_absolute_target_angle;
extern float task4_gray_line_weight[GRAY_LINE_WEIGHT_NUM];
extern float task5_gray_line_weight[GRAY_LINE_WEIGHT_NUM];

void Task_Init (void);
void Task_Update (void);
void Task1_Init (void);
void Task1_Prepare (void);
void Task1_Cancel_Prepare (void);
uint8 Task1_Is_Ready (void);
uint8 Task1_Is_Stop_Requested (void);
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
uint8 Task3_Is_Stop_Requested (void);
void Task3_Start (void);
void Task3 (void);
void Task4_Init (void);
void Task4_Prepare (void);
void Task4_Cancel_Prepare (void);
uint8 Task4_Is_Ready (void);
uint8 Task4_Is_Stop_Requested (void);
void Task4_Start (void);
void Task4_Tick_10ms (void);
void Task4 (void);
void Task5_Init (void);
void Task5_Prepare (void);
void Task5_Cancel_Prepare (void);
uint8 Task5_Is_Ready (void);
uint8 Task5_Is_Stop_Requested (void);
void Task5_Start (void);
void Task5_Tick_10ms (void);
void Task5 (void);
void Task5_Set_Ball_Target_Position (float target_position_cm);
void Task6_Init (void);
void Task6_Prepare (void);
void Task6_Cancel (void);
uint8 Task6_Is_Ready (void);
uint8 Task6_Is_Stop_Requested (void);
void Task6_Start (void);
void Task6 (void);
void Task7_Init (void);
void Task7_Cancel (void);
void Task7 (void);

#endif
