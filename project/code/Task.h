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
#define TASK2_DECEL_START_TENTHS         (110U)  // 11.0s后开始减速
#define TASK2_DECEL_DURATION_TENTHS      (10U)   // 用1.0s降至目标速度
#define TASK2_DECEL_GRAY_TARGET          (300.0f)

// mode=1静止钢球平衡参数。
#define TASK1_MODE_NUMBER               (1)
#define TASK1_TARGET_POSITION_CM        (-5.0f)
#define TASK4_TARGET_POSITION_CM        (0.0f)
#define TASK1_BALL_ANGLE_PER_CM         (2.0f)
#define TASK1_ZERO_TARGET_OFFSET_ANGLE  (0.0f)    // 钢球位于0cm时的相对目标角度
#define TASK1_STEP_FREQUENCY_HZ         (2000)
#define TASK1_SAMPLE_PERIOD_S           (0.1f)    // K230位置和速度每0.1秒更新一次
#define TASK1_INTEGRAL_HISTORY_TICKS    (10U)     // 保存最近1秒的积分贡献
#define TASK1_INTEGRAL_KI_DEG_PER_CM_S  (0.6f)   // 位置积分每1cm*s补偿0.6度
#define TASK1_INTEGRAL_ERROR_LIMIT_CM   (1.7f)    // 太大误差调为1.7
#define TASK1_INTEGRAL_MAX_CM_S         (50.0f)   // 积分限幅，最大补偿约±15度
#define TASK1_INTEGRAL_MAX_DT_S         (0.2f)    // 丢帧时限制单次积分时间
#define TASK1_INTEGRAL_DEADBAND_CM      (0.4f)    // 误差进入±0.4cm后保持I项、不再累积
#define TASK1_INTEGRAL_CROSS_RESET_CM   (0.3f)    // 穿过目标并进入另一侧0.3cm后移除最近1秒积分

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
#define TASK3_POSITIVE_POSITION_CM      (4.0f)     // 正向急刹后的二次控制目标
#define TASK3_NEGATIVE_POSITION_CM      (-5.0f)
#define TASK3_POSITION_TOLERANCE_CM     (1.0f)
#define TASK3_TO_POSITIVE_ANGLE         (11.0f)    // 正向起步倾角，增大以保证钢球能接近+5cm
#define TASK3_TO_NEGATIVE_ANGLE         (-15.0f)   // 实测负角度使钢球向负半轴运动
#define TASK3_MOVE_FREQUENCY_HZ         (500)
#define TASK3_FIXED_TILT_ANGLE          (15.0f)
#define TASK3_FIRST_REVERSE_POSITION_CM (0.8f)
#define TASK3_SECOND_REVERSE_POSITION_CM (1.0f)
#define TASK3_QUADRATIC_START_CM        (-4.0f)
#define TASK3_QUADRATIC_TARGET_CM       (-5.0f)
#define TASK3_FLAG_POSITION_CM           (4.0f)
#define TASK3_RETURN_POSITION_CM         (2.0f)
#define TASK3_POSITIVE_BRAKE_POSITION_CM (1.0f)    // 正向固定倾角推进至此处后急刹
#define TASK3_POSITIVE_BRAKE_ANGLE      (-40.0f)   // 正向运动的反向急刹角度
#define TASK3_POSITIVE_REVERSE_POSITION_CM (4.0f)  // 过了这里折返
#define TASK3_POSITIVE_CONTROL_FREQUENCY_HZ (TASK1_STEP_FREQUENCY_HZ)
#define TASK3_BALL_VELOCITY_TARGET_PER_CM2 (1.5f)  // 偏离目标1cm时目标速度1.5cm/s
#define TASK3_BALL_VELOCITY_ANGLE_PER_CM_PER_S (1.5f) // 速度误差1cm/s，对应1.5度倾角
#define TASK8_MODE_NUMBER               (8)
#define TASK8_PARAMETER_STEP            (0.1f)
#define TASK8_PARAMETER_MIN             (0.0f)
#define TASK8_PARAMETER_MAX             (10.0f)
#define TASK3_NEGATIVE_HOLD_ANGLE_OFFSET (0.8f)    // 上调约1度，使负向平衡点由约-6cm回到-5cm
#define TASK3_NEGATIVE_HOLD_BASE_ANGLE   (1.0f)
#define TASK3_NEGATIVE_BRAKE_POSITION_CM (-1.0f)   // 到达-1cm时执行一次反向急停
#define TASK3_TO_NEGATIVE_BRAKE_ANGLE    (20.0f)   // 前往-5cm过程中急停的反向目标倾角
#define TASK3_BRAKE_FREQUENCY_HZ        (3000)    // 正、负方向急停的转动频率

#undef TASK3_NEGATIVE_BRAKE_POSITION_CM
#define TASK3_NEGATIVE_BRAKE_POSITION_CM  (-1.0f)
#undef TASK3_TO_POSITIVE_ANGLE
#undef TASK3_POSITIVE_REVERSE_POSITION_CM
#undef TASK3_NEGATIVE_HOLD_ANGLE_OFFSET
#define TASK3_TO_POSITIVE_ANGLE            (10.0f)
#define TASK3_POSITIVE_REVERSE_POSITION_CM (4.0f)
#define TASK3_NEGATIVE_HOLD_ANGLE_OFFSET  (3.0f)

// mode=4钢球平衡巡线测试参数
#define TASK4_MODE_NUMBER               (4)
#define TASK4_GRAY_TARGET               (200.0f)
#define TASK4_RUN_TIME_TICKS            (900)     // 10ms * 900 = 9.00s
#define TASK4_ACCEL_TIME_TICKS          (300)     // 3.0秒S曲线缓慢起步
#define TASK4_DECEL_TIME_TICKS          (0)
#define TASK4_ACCEL_TILT_ANGLE          (-8.0f)   // 起步补偿峰值，降低以避免钢球被弹出
#define TASK4_DECEL_TILT_ANGLE          (14.5f)   // 减速补偿倾角；通常与起步补偿方向相反
#define TASK4_BALL_MAX_CM               (12.0f)
#define TASK4_STEP_FREQUENCY_HZ         (TASK1_STEP_FREQUENCY_HZ)

#undef TASK4_ACCEL_TIME_TICKS
#undef TASK4_DECEL_TIME_TICKS
#undef TASK4_ACCEL_TILT_ANGLE
#undef TASK4_DECEL_TILT_ANGLE
#define TASK4_ACCEL_TIME_TICKS           (200)
#define TASK4_DECEL_TIME_TICKS           (200)
#define TASK4_ACCEL_TILT_ANGLE           (-1.0f)
#define TASK4_DECEL_TILT_ANGLE           (0.0f)
#undef TASK4_GRAY_TARGET
#define TASK4_GRAY_TARGET                (250.0f)
#define TASK4_5_MAX_ANGLE_DELTA_DEG       (8.0f)        // 防止震动导致小球飞出

// mode=5：环形线路行驶一圈，返回启停线后减速停车并继续平衡钢球。
#define TASK5_MODE_NUMBER               (5)
#define TASK5_GRAY_TARGET               (250.0f)
#define TASK5_MAX_TOTAL_TIME_TICKS      (3000)    // 10ms * 3000 = 30.00s
#define TASK5_ACCEL_TIME_TICKS          (150)
#define TASK5_DECEL_TIME_TICKS          (0)
#define TASK5_ACCEL_TILT_ANGLE          (-6.0f)
#define TASK5_DECEL_TILT_ANGLE          (15.0f)
#define TASK5_STEP_FREQUENCY_HZ         (TASK1_STEP_FREQUENCY_HZ)

#undef TASK5_ACCEL_TIME_TICKS
#undef TASK5_DECEL_TIME_TICKS
#undef TASK5_ACCEL_TILT_ANGLE
#undef TASK5_DECEL_TILT_ANGLE
#define TASK5_ACCEL_TIME_TICKS           (400)
#define TASK5_DECEL_TIME_TICKS           (0)
#define TASK5_ACCEL_TILT_ANGLE           (0.0f)
#define TASK5_DECEL_TILT_ANGLE           (2.0f)
#define TASK5_START_TILT_ANGLE           (-3.0f)
#define TASK5_START_TILT_TIME_TICKS      (10U)
#define TASK5_MAX_TARGET_ANGLE           (15.0f)

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
    TASK3_INITIAL_TO_NEGATIVE,
    TASK3_WAIT_RETURN_TO_POSITIVE,
    TASK3_INITIAL_TO_POSITIVE,
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
extern volatile uint8 task3_flag;
extern float task4_gray_line_weight[GRAY_LINE_WEIGHT_NUM];
extern float task5_gray_line_weight[GRAY_LINE_WEIGHT_NUM];

void Task_Init (void);
void Task_Update (void);
void Task2_Start (void);
void Task2_Update (void);
void Task2_Stop (void);
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
void Task8_Init (void);
void Task8_Cancel (void);
void Task8 (void);

extern float task1_ball_velocity_target_per_cm2;
extern float task1_ball_velocity_angle_per_cm_per_s;
extern float task3_ball_velocity_target_per_cm2;
extern float task3_ball_velocity_angle_per_cm_per_s;

#endif
