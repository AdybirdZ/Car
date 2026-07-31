#include "Task.h"
#include "Button.h"
#include "OLED.h"
#include "Light_and_Buzzer.h"
#include "Step_Encoder.h"

// 当前任务：车辆经过中间四路全黑的停车线时停车。
bool enable_task = true;
uint8 task_stop_flag = 0;
static uint8 task_stop_pending = 0;
static uint8 task_stop_delay_ticks = 0;
static uint8 task_stop_confirm_ticks = 0;

volatile Task4_State task4_state = TASK4_IDLE;
volatile float task4_ball_position_cm = 0.0f;
volatile float task4_step_target_angle = 0.0f;
float task4_gray_line_weight[GRAY_LINE_WEIGHT_NUM] =
{
    -1.1f, -0.8f, -0.5f, 0.2f, 0.2f, 0.5f, 0.8f, 1.1f
};
static volatile uint16 task4_run_ticks = 0;
static volatile uint8 task4_time_finished = 0;
static uint8 task4_stop_requested = 0;
static float task4_previous_gray_target = 0.0f;
static float task4_previous_gray_k = 0.0f;
static float task4_previous_gray_weight[GRAY_LINE_WEIGHT_NUM] = {0};
static uint8 task4_gray_weight_saved = 0;
static float task4_speed_cm_per_s = 0.0f;
static float task4_last_speed_position_cm = 0.0f;
static uint32 task4_last_speed_time_tenths = 0;
static uint8 task4_speed_valid = 0;
static volatile uint32 task4_speed_elapsed_tenths = 0;
static uint8 task4_speed_timer_ticks = 0;

static volatile Task4_State task5_state = TASK4_IDLE;
static volatile float task5_ball_position_cm = 0.0f;
static volatile float task5_step_target_angle = 0.0f;
float task5_gray_line_weight[GRAY_LINE_WEIGHT_NUM] =
{
    -1.1f, -0.8f, -0.5f, 0.2f, 0.2f, 0.5f, 0.8f, 1.1f
};
static volatile uint16 task5_run_ticks = 0;
static volatile uint16 task5_decel_ticks = 0;
static volatile uint8 task5_decelerating = 0;
static volatile uint8 task5_time_finished = 0;
static uint8 task5_stop_requested = 0;
static uint8 task5_stop_line_armed = 0;
static float task5_previous_gray_target = 0.0f;
static float task5_previous_gray_k = 0.0f;
static float task5_previous_gray_weight[GRAY_LINE_WEIGHT_NUM] = {0};
static uint8 task5_gray_weight_saved = 0;
static float task5_speed_cm_per_s = 0.0f;
static float task5_last_speed_position_cm = 0.0f;
static uint32 task5_last_speed_time_tenths = 0;
static uint8 task5_speed_valid = 0;

static volatile Task4_State task1_state = TASK4_IDLE;
static volatile float task1_ball_position_cm = 0.0f;
static volatile float task1_step_target_angle = 0.0f;
static float task1_speed_cm_per_s = 0.0f;
static float task1_last_speed_position_cm = 0.0f;
static uint32 task1_last_speed_time_tenths = 0;
static uint8 task1_speed_valid = 0;
static uint8 task1_stop_requested = 0;
static volatile Task3_State task3_state = TASK3_IDLE;
static volatile float task3_ball_position_cm = 0.0f;
static volatile float task3_step_target_angle = 0.0f;
static float task3_speed_cm_per_s = 0.0f;
static float task3_last_speed_position_cm = 0.0f;
static uint32 task3_last_speed_time_tenths = 0;
static uint8 task3_speed_valid = 0;
static uint8 task3_negative_brake_done = 0;
static uint8 task3_stop_requested = 0;
static uint8 task6_active = 0;
volatile float task3_absolute_target_angle = STEP_ENCODER_DEFAULT_STARTUP_TARGET_ANGLE;

/*
函数功能：检测A点启停线
参数：无
返回值：1表示左2、左1、右1、右2四路均检测到黑色，0表示不是启停线
*/
static uint8 Task_Is_Stop_Line (void)
{
    uint8 index = 0;
    uint8 black_count = 0;

    for(index = 0; index < GRAY_CHANNEL_NUM; index ++)
    {
        if(gray_line_black_level == gray_data[index])
        {
            black_count ++;
        }
    }

    return (black_count >= 3U);
}

/*
函数功能：任务模块初始化，清除停车锁存标志并恢复红外巡线输出
参数：无
*/
void Task_Init (void)
{
    task_stop_flag = 0;
    task_stop_pending = 0;
    task_stop_delay_ticks = 0;
    task_stop_confirm_ticks = 0;
}

/*
函数功能：刷新红外数据并检测A点启停线，检测到后立即停车且锁存结果
参数：无
说明：本函数应在车辆行驶期间被周期调用，例如每次主循环调用一次。
*/
void Task_Update (void)
{
    if(!enable_task || task_stop_flag)
    {
        return;
    }

    if(!task_stop_pending)
    {
        if(Task_Is_Stop_Line())
        {
            if(task_stop_confirm_ticks < TASK_STOP_CONFIRM_TICKS)
            {
                task_stop_confirm_ticks ++;
            }
            if(task_stop_confirm_ticks >= TASK_STOP_CONFIRM_TICKS)
            {
                task_stop_pending = 1;
                task_stop_delay_ticks = TASK_STOP_DELAY_TICKS;
            }
        }
        else
        {
            task_stop_confirm_ticks = 0;
        }
        return;
    }

    if(task_stop_delay_ticks > 0)
    {
        task_stop_delay_ticks --;
    }

    if(task_stop_delay_ticks == 0)
    {
        task_stop_flag = 1;
        enable_gray_line = false;
        Motor_PID_New_Stop();
        OLED_Stop_Time();
        // 停车动作完成后开启蜂鸣器；task_stop_flag锁存，避免重复触发。
        Buzz(0);
    }
}

static uint8 Task_Read_Ball_Position (float *position)
{
    char message[SERIAL_BUFFER_SIZE] = {0};
    float parsed_position = 0.0f;

    if(NULL == position
    || !Serial_Get_Message(message, SERIAL_BUFFER_SIZE)
    || !Serial_Parse_Signed_Float(message, &parsed_position)
    || parsed_position < -TASK4_BALL_MAX_CM
    || parsed_position > TASK4_BALL_MAX_CM)
    {
        return 0;
    }

    *position = parsed_position;
    OLED_Show_K230_Position(parsed_position);
    return 1;
}

static float Task_Limit (float value, float minimum, float maximum)
{
    if(value < minimum)
    {
        return minimum;
    }
    if(value > maximum)
    {
        return maximum;
    }
    return value;
}

static float Task4_Get_Acceleration_Tilt (void)
{
    if(task4_run_ticks >= TASK4_ACCEL_TIME_TICKS)
    {
        return 0.0f;
    }

    return TASK4_ACCEL_TILT_ANGLE
         * (float)(TASK4_ACCEL_TIME_TICKS - task4_run_ticks)
         / (float)TASK4_ACCEL_TIME_TICKS;
}

static float Task4_Get_Deceleration_Tilt (void)
{
    float progress;

    if(task4_run_ticks < (TASK4_RUN_TIME_TICKS - TASK4_DECEL_TIME_TICKS)
    || task4_run_ticks >= TASK4_RUN_TIME_TICKS)
    {
        return 0.0f;
    }

    progress = (float)(task4_run_ticks
             - (TASK4_RUN_TIME_TICKS - TASK4_DECEL_TIME_TICKS))
             / (float)TASK4_DECEL_TIME_TICKS;
    return TASK4_DECEL_TILT_ANGLE * 4.0f * progress * (1.0f - progress);
}

static void Task4_Apply_Gray_Line_Weight (void)
{
    uint8 index;

    for(index = 0; index < GRAY_LINE_WEIGHT_NUM; index++)
    {
        task4_previous_gray_weight[index] = gray_line_weight[index];
        gray_line_weight[index] = task4_gray_line_weight[index];
    }
    task4_gray_weight_saved = 1;
}

static void Task4_Restore_Gray_Line_Weight (void)
{
    uint8 index;

    if(!task4_gray_weight_saved)
    {
        return;
    }

    for(index = 0; index < GRAY_LINE_WEIGHT_NUM; index++)
    {
        gray_line_weight[index] = task4_previous_gray_weight[index];
    }
    task4_gray_weight_saved = 0;
}

static float Task5_Get_Acceleration_Tilt (void)
{
    if(task5_run_ticks >= TASK5_ACCEL_TIME_TICKS)
    {
        return 0.0f;
    }

    return TASK5_ACCEL_TILT_ANGLE
         * (float)(TASK5_ACCEL_TIME_TICKS - task5_run_ticks)
         / (float)TASK5_ACCEL_TIME_TICKS;
}

static float Task5_Get_Deceleration_Tilt (void)
{
    float progress;

    if(!task5_decelerating || task5_decel_ticks >= TASK5_DECEL_TIME_TICKS)
    {
        return 0.0f;
    }

    progress = (float)task5_decel_ticks / (float)TASK5_DECEL_TIME_TICKS;
    return TASK5_DECEL_TILT_ANGLE * 4.0f * progress * (1.0f - progress);
}

static void Task5_Apply_Gray_Line_Weight (void)
{
    uint8 index;

    for(index = 0; index < GRAY_LINE_WEIGHT_NUM; index++)
    {
        task5_previous_gray_weight[index] = gray_line_weight[index];
        gray_line_weight[index] = task5_gray_line_weight[index];
    }
    task5_gray_weight_saved = 1;
}

static void Task5_Restore_Gray_Line_Weight (void)
{
    uint8 index;

    if(!task5_gray_weight_saved)
    {
        return;
    }

    for(index = 0; index < GRAY_LINE_WEIGHT_NUM; index++)
    {
        gray_line_weight[index] = task5_previous_gray_weight[index];
    }
    task5_gray_weight_saved = 0;
}

// 每0.1秒用K230的新位置更新一次速度，避免串口帧率变化影响速度单位。
static void Task_Update_Ball_Speed (float position,
                                    float *speed,
                                    float *last_position,
                                    uint32 *last_time_tenths,
                                    uint8 *valid,
                                    uint32 current_time_tenths)
{
    float elapsed_s;

    if(NULL == speed || NULL == last_position || NULL == last_time_tenths || NULL == valid)
    {
        return;
    }

    if(!*valid)
    {
        *last_position = position;
        *last_time_tenths = current_time_tenths;
        *speed = 0.0f;
        *valid = 1;
        return;
    }

    if(current_time_tenths == *last_time_tenths)
    {
        return;
    }

    elapsed_s = (float)(current_time_tenths - *last_time_tenths)
              * TASK1_SAMPLE_PERIOD_S;
    if(elapsed_s > 0.0f)
    {
        *speed = (position - *last_position) / elapsed_s;
    }

    *last_position = position;
    *last_time_tenths = current_time_tenths;
}

void Task1_Init (void)
{
    task1_state = TASK4_IDLE;
    task1_ball_position_cm = 0.0f;
    task1_step_target_angle = 0.0f;
    task1_speed_cm_per_s = 0.0f;
    task1_speed_valid = 0;
    task1_stop_requested = 0;
}

// mode切换到1时启动K230，收到第一帧合法钢球位置后才允许A30启动。
void Task1_Prepare (void)
{
    if(TASK4_IDLE != task1_state)
    {
        return;
    }

    enable_k230_line = false;
    serial_rx_finish = 0;
    Serial_Send_Byte(K230_START_COMMAND);
    task1_state = TASK4_WAIT_K230;
    OLED_Show_Status("WAIT K230");
}

void Task1_Cancel_Prepare (void)
{
    if(TASK4_WAIT_K230 != task1_state && TASK4_READY != task1_state)
    {
        return;
    }

    Serial_Send_Byte(K230_STOP_COMMAND);
    task1_state = TASK4_IDLE;
    OLED_Show_Status("INIT DONE");
}

uint8 Task1_Is_Ready (void)
{
    return (TASK4_READY == task1_state) ? 1U : 0U;
}

uint8 Task1_Is_Stop_Requested (void)
{
    return task1_stop_requested;
}

// 蜂鸣1秒由main完成；此模式不启动巡线和车轮PID，仅启动钢球平衡。
void Task1_Start (void)
{
    if(TASK4_READY != task1_state)
    {
        return;
    }

    enable_task = false;
    enable_gray_line = false;
    Motor_PID_New_Stop();
    Step_To_Angle(0.0f, TASK4_STEP_FREQUENCY_HZ);
    task1_speed_valid = 0;
    task1_stop_requested = 0;
    while(Button_Get_Angle_Increase_Event())
    {
    }
    OLED_Start_Time();
    task1_state = TASK4_RUNNING;
    OLED_Show_Status("BALL BALANCE");
}

/*
函数功能：根据钢球位置和速度计算摆杆的相对目标角度
参数说明：
target_position_cm：钢球期望位置，单位cm
ball_position_cm：K230测得的当前钢球位置，单位cm
ball_speed_cm_per_s：钢球当前速度，正方向与K230位置正方向相同，单位cm/s
 返回值：步进电机相对目标角度，单位度
 说明：位置误差先换算成朝目标方向的目标速度；再根据目标速度和实测速度的差值计算倾角。
      按当前实测控制方向，速度差为正时应增大步进电机目标角度。
*/
float Task_Calculate_Velocity_Control_Angle (float target_position_cm,
                                             float ball_position_cm,
                                             float ball_speed_cm_per_s)
{
    float position_error_cm;
    float position_error_abs_cm;
    float target_speed_cm_per_s;
    float speed_error_cm_per_s;
    float target_angle;

    // 位置误差平方使钢球靠近目标点时自动显著降低目标速度，同时保留朝向目标点的方向。
    position_error_cm = target_position_cm - ball_position_cm;
    position_error_abs_cm = (position_error_cm < 0.0f)
                          ? -position_error_cm : position_error_cm;
    target_speed_cm_per_s = position_error_cm * position_error_abs_cm
                          * BALL_VELOCITY_TARGET_PER_CM2;
    target_speed_cm_per_s = Task_Limit(target_speed_cm_per_s,
                                       -BALL_VELOCITY_MAX_TARGET_SPEED,
                                       BALL_VELOCITY_MAX_TARGET_SPEED);
    speed_error_cm_per_s = target_speed_cm_per_s - ball_speed_cm_per_s;

    target_angle = TASK1_ZERO_TARGET_OFFSET_ANGLE
                 + target_position_cm * TASK3_BASE_ANGLE_PER_CM
                 + speed_error_cm_per_s * BALL_VELOCITY_ANGLE_PER_CM_PER_S;

    return Task_Limit(target_angle,
                      -BALL_VELOCITY_MAX_TARGET_ANGLE,
                      BALL_VELOCITY_MAX_TARGET_ANGLE);
}

/*
函数功能：根据指定的钢球目标位置，计算步进电机应到达的机械绝对角度
参数说明：target_position_cm为钢球目标位置，单位cm
返回值：0.0~360.0度范围内的机械绝对目标角度
说明：只在设置新的固定位置目标时调用，不随K230实时位置重复计算。
*/
float Task_Calculate_Absolute_Target_Angle (float target_position_cm)
{
    float relative_target_angle = TASK1_ZERO_TARGET_OFFSET_ANGLE
                                + target_position_cm * TASK3_BASE_ANGLE_PER_CM;

    return Step_Encoder_Relative_To_Absolute_Angle(relative_target_angle);
}

// mode=1运行函数：车辆始终静止，K230位置数据只用于保持钢球在中心附近。
void Task1 (void)
{
    float position;
    float normal_target_angle;

    if(TASK4_RUNNING == task1_state && Button_Get_Angle_Increase_Event())
    {
        task1_stop_requested = 1;
        task1_state = TASK4_DONE;
        OLED_Stop_Time();
        Serial_Send_Byte(K230_STOP_COMMAND);
        OLED_Show_Status("TASK1 STOPPED");
        return;
    }

    if(!Task_Read_Ball_Position(&position))
    {
        return;
    }
    task1_ball_position_cm = position;

    if(TASK4_WAIT_K230 == task1_state)
    {
        task1_state = TASK4_READY;
        OLED_Show_Status("TARGET AND K230 DONE");
    }
    else if(TASK4_RUNNING == task1_state)
    {
        Task_Update_Ball_Speed(task1_ball_position_cm,
                               &task1_speed_cm_per_s,
                               &task1_last_speed_position_cm,
                               &task1_last_speed_time_tenths,
                               &task1_speed_valid,
                               oled_elapsed_tenths);
        normal_target_angle = Task_Calculate_Velocity_Control_Angle(
                                  TASK1_TARGET_POSITION_CM,
                                  task1_ball_position_cm,
                                  task1_speed_cm_per_s);

        task1_step_target_angle = normal_target_angle;
        Step_To_Angle(task1_step_target_angle, TASK1_STEP_FREQUENCY_HZ);
    }
}

void Task3_Init (void)
{
    task3_state = TASK3_IDLE;
    task3_ball_position_cm = 0.0f;
    task3_step_target_angle = 0.0f;
    task3_speed_cm_per_s = 0.0f;
    task3_speed_valid = 0;
    task3_negative_brake_done = 0;
    task3_stop_requested = 0;
    task3_absolute_target_angle = Step_Encoder_Get_Startup_Target_Angle();
}

// mode切换到3时启动K230，收到第一帧合法位置后才允许A30启动。
void Task3_Prepare (void)
{
    if(TASK3_IDLE != task3_state)
    {
        return;
    }

    enable_k230_line = false;
    serial_rx_finish = 0;
    Serial_Send_Byte(K230_START_COMMAND);
    task3_state = TASK3_WAIT_K230;
    OLED_Show_Status("WAIT K230");
}

void Task3_Cancel_Prepare (void)
{
    if(TASK3_WAIT_K230 != task3_state && TASK3_READY != task3_state)
    {
        return;
    }

    Serial_Send_Byte(K230_STOP_COMMAND);
    task3_state = TASK3_IDLE;
    OLED_Show_Status("INIT DONE");
}

uint8 Task3_Is_Ready (void)
{
    return (TASK3_READY == task3_state) ? 1U : 0U;
}

uint8 Task3_Is_Stop_Requested (void)
{
    return task3_stop_requested;
}

// 蜂鸣1秒由main完成；车辆保持静止，先倾斜摆杆使钢球向+5cm运行。
void Task3_Start (void)
{
    if(TASK3_READY != task3_state)
    {
        return;
    }

    enable_task = false;
    enable_gray_line = false;
    Motor_PID_New_Stop();
    task3_step_target_angle = TASK3_TO_POSITIVE_ANGLE;
    task3_speed_cm_per_s = 0.0f;
    task3_speed_valid = 0;
    task3_negative_brake_done = 0;
    task3_stop_requested = 0;
    while(Button_Get_Angle_Increase_Event())
    {
    }
    Step_To_Angle(task3_step_target_angle, TASK3_MOVE_FREQUENCY_HZ);
    task3_state = TASK3_MOVE_TO_POSITIVE;
    OLED_Start_Time();
    OLED_Show_Status("BALL TO +5CM");
}

// mode=3运行函数：0cm到+5cm，再到-5cm并稳定在-5cm附近。
void Task3 (void)
{
    float position;

    if((TASK3_MOVE_TO_POSITIVE == task3_state
     || TASK3_CONTROL_TO_POSITIVE == task3_state
     || TASK3_MOVE_TO_NEGATIVE == task3_state
     || TASK3_HOLD_NEGATIVE == task3_state)
    && Button_Get_Angle_Increase_Event())
    {
        task3_stop_requested = 1;
        task3_state = TASK3_DONE;
        OLED_Stop_Time();
        Serial_Send_Byte(K230_STOP_COMMAND);
        OLED_Show_Status("TASK3 STOPPED");
        return;
    }

    if(!Task_Read_Ball_Position(&position))
    {
        return;
    }
    task3_ball_position_cm = position;

    if(TASK3_WAIT_K230 == task3_state)
    {
        task3_state = TASK3_READY;
        OLED_Show_Status("TARGET AND K230 DONE");
        return;
    }

    if(TASK3_MOVE_TO_POSITIVE == task3_state
    || TASK3_CONTROL_TO_POSITIVE == task3_state
    || TASK3_MOVE_TO_NEGATIVE == task3_state
    || TASK3_HOLD_NEGATIVE == task3_state)
    {
        Task_Update_Ball_Speed(task3_ball_position_cm,
                               &task3_speed_cm_per_s,
                               &task3_last_speed_position_cm,
                               &task3_last_speed_time_tenths,
                               &task3_speed_valid,
                               oled_elapsed_tenths);
    }

    if(TASK3_MOVE_TO_POSITIVE == task3_state)
    {
        if(task3_ball_position_cm
        >= TASK3_POSITIVE_BRAKE_POSITION_CM)
        {
            task3_step_target_angle = TASK3_POSITIVE_BRAKE_ANGLE;
            Step_To_Angle(task3_step_target_angle, TASK3_BRAKE_FREQUENCY_HZ);
            task3_state = TASK3_CONTROL_TO_POSITIVE;
            OLED_Show_Status("BRAKE +3CM");
        }
        return;
    }

    if(TASK3_CONTROL_TO_POSITIVE == task3_state)
    {
        // 以+5cm为二次函数目标减小正向速度；通过+4cm后立即开始返回。
        if(task3_ball_position_cm >= TASK3_POSITIVE_REVERSE_POSITION_CM)
        {
            task3_step_target_angle = TASK3_TO_NEGATIVE_ANGLE;
            Step_To_Angle(task3_step_target_angle, TASK3_MOVE_FREQUENCY_HZ);
            task3_state = TASK3_MOVE_TO_NEGATIVE;
            OLED_Show_Status("BALL TO -5CM");
            return;
        }

        task3_step_target_angle = Task_Calculate_Velocity_Control_Angle(
                                      TASK3_POSITIVE_POSITION_CM,
                                      task3_ball_position_cm,
                                      task3_speed_cm_per_s);
        Step_To_Angle(task3_step_target_angle,
                      TASK3_POSITIVE_CONTROL_FREQUENCY_HZ);
        return;
    }

    if(TASK3_MOVE_TO_NEGATIVE == task3_state)
    {
        if(!task3_negative_brake_done
        && task3_ball_position_cm <= TASK3_NEGATIVE_BRAKE_POSITION_CM)
        {
            // 钢球仍向负方向运动，因此先固定转到急刹角，再切换到-5cm的位置-速度控制。
            task3_negative_brake_done = 1;
            task3_absolute_target_angle =
                Step_Encoder_Relative_To_Absolute_Angle(
                    TASK3_NEGATIVE_HOLD_BASE_ANGLE);
            Step_To_Angle(TASK3_NEGATIVE_BRAKE_ANGLE, TASK3_BRAKE_FREQUENCY_HZ);
            task3_step_target_angle = Task_Calculate_Velocity_Control_Angle(
                                          TASK3_NEGATIVE_POSITION_CM,
                                          task3_ball_position_cm,
                                          task3_speed_cm_per_s)
                                    + TASK3_NEGATIVE_HOLD_ANGLE_OFFSET;
            Step_To_Angle(task3_step_target_angle, TASK3_BRAKE_FREQUENCY_HZ);
            task3_state = TASK3_HOLD_NEGATIVE;
            OLED_Show_Status("HOLD -5CM");
        }
        return;
    }

    if(TASK3_HOLD_NEGATIVE == task3_state)
    {
        task3_step_target_angle = Task_Calculate_Velocity_Control_Angle(
                                      TASK3_NEGATIVE_POSITION_CM,
                                      task3_ball_position_cm,
                                      task3_speed_cm_per_s)
                                + TASK3_NEGATIVE_HOLD_ANGLE_OFFSET;
        Step_To_Angle(task3_step_target_angle, TASK1_STEP_FREQUENCY_HZ);
    }
}

void Task4_Init (void)
{
    task4_state = TASK4_IDLE;
    task4_ball_position_cm = 0.0f;
    task4_step_target_angle = 0.0f;
    task4_run_ticks = 0;
    task4_time_finished = 0;
    task4_stop_requested = 0;
    task4_speed_cm_per_s = 0.0f;
    task4_speed_valid = 0;
    task4_speed_elapsed_tenths = 0;
    task4_speed_timer_ticks = 0;
}

// mode切换到4时发送小写s并立即允许A30启动，不等待K230首帧或钢球稳定判定。
void Task4_Prepare (void)
{
    if(TASK4_IDLE != task4_state)
    {
        return;
    }

    enable_k230_line = false;
    serial_rx_finish = 0;
    task4_previous_gray_target = gray_line_base_offset;
    task4_previous_gray_k = gray_line_k;
    Task4_Apply_Gray_Line_Weight();
    gray_line_base_offset = 0.0f;
    gray_line_k = 0.0f;
    Serial_Send_Byte(K230_START_COMMAND);
    task4_state = TASK4_READY;
    OLED_Show_Status("TASK4 READY");
}

void Task4_Cancel_Prepare (void)
{
    if(TASK4_WAIT_K230 != task4_state && TASK4_READY != task4_state)
    {
        return;
    }
    Serial_Send_Byte(K230_STOP_COMMAND);
    gray_line_base_offset = task4_previous_gray_target;
    gray_line_k = task4_previous_gray_k;
    Task4_Restore_Gray_Line_Weight();
    task4_state = TASK4_IDLE;
    OLED_Show_Status("INIT DONE");
}

uint8 Task4_Is_Ready (void)
{
    return (TASK4_READY == task4_state) ? 1U : 0U;
}

uint8 Task4_Is_Stop_Requested (void)
{
    return task4_stop_requested;
}

// 蜂鸣1秒由main完成；巡线从0速度开始，由10ms定时函数按S曲线缓慢提速。
void Task4_Start (void)
{
    if(TASK4_READY != task4_state)
    {
        return;
    }

    enable_task = false;
    enable_gray_line = true;
    gray_line_base_offset = 0.0f;
    gray_line_k = 0.0f;
    gray_line_correct_offset = 0.0f;
    gray_line_left_target = 0.0f;
    gray_line_right_target = 0.0f;
    Motor_PID_New_Start(0.0f, 0.0f);

    task4_run_ticks = 0;
    task4_time_finished = 0;
    task4_stop_requested = 0;
    task4_speed_cm_per_s = 0.0f;
    task4_speed_valid = 0;
    task4_speed_elapsed_tenths = 0;
    task4_speed_timer_ticks = 0;
    task4_state = TASK4_RUNNING;
    while(Button_Get_Angle_Increase_Event())
    {
    }
    Step_To_Angle(TASK4_ACCEL_TILT_ANGLE, TASK4_STEP_FREQUENCY_HZ);
    OLED_Start_Time();
    OLED_Show_Status("TASK4 RUNNING");
}

// OLED的10ms定时器调用；800次后立即关闭车轮，避免主循环阻塞导致超过8秒。
void Task4_Tick_10ms (void)
{
    float progress;

    if(TASK4_RUNNING != task4_state && TASK4_HOLD != task4_state)
    {
        return;
    }

    // 此时基只供钢球速度计算使用；OLED停表后仍需继续运行。
    task4_speed_timer_ticks ++;
    if(task4_speed_timer_ticks >= 10U)
    {
        task4_speed_timer_ticks = 0;
        task4_speed_elapsed_tenths ++;
    }

    if(TASK4_RUNNING != task4_state || task4_time_finished)
    {
        return;
    }

    task4_run_ticks ++;
    if(task4_run_ticks <= TASK4_ACCEL_TIME_TICKS)
    {
        progress = (float)task4_run_ticks / (float)TASK4_ACCEL_TIME_TICKS;
        gray_line_base_offset = TASK4_GRAY_TARGET
                              * (3.0f * progress * progress
                              - 2.0f * progress * progress * progress);
    }
    else if(task4_run_ticks >= (TASK4_RUN_TIME_TICKS - TASK4_DECEL_TIME_TICKS))
    {
        progress = (float)(TASK4_RUN_TIME_TICKS - task4_run_ticks)
                 / (float)TASK4_DECEL_TIME_TICKS;
        if(progress < 0.0f) progress = 0.0f;
        gray_line_base_offset = TASK4_GRAY_TARGET
                              * (3.0f * progress * progress
                              - 2.0f * progress * progress * progress);
    }
    else
    {
        gray_line_base_offset = TASK4_GRAY_TARGET;
    }
    gray_line_k = task4_previous_gray_k
                * gray_line_base_offset / TASK4_GRAY_TARGET;
    gray_line_left_target = gray_line_base_offset + gray_line_correct_offset;
    gray_line_right_target = gray_line_base_offset - gray_line_correct_offset;
    Motor_PID_New_Set_Targets(gray_line_left_target, gray_line_right_target);

    if(task4_run_ticks >= TASK4_RUN_TIME_TICKS)
    {
        task4_time_finished = 1;
        gray_line_base_offset = 0.0f;
        gray_line_k = 0.0f;
        enable_gray_line = false;
        Motor_PID_New_Stop();
    }
}

// mode=4运行函数：巡线并复用mode=1的位置-速度二次函数保持钢球在0cm附近。
void Task4 (void)
{
    float position;

    if((TASK4_RUNNING == task4_state || TASK4_HOLD == task4_state)
    && Button_Get_Angle_Increase_Event())
    {
        task4_stop_requested = 1;
        task4_state = TASK4_DONE;
        enable_gray_line = false;
        Motor_PID_New_Stop();
        OLED_Stop_Time();
        Serial_Send_Byte(K230_STOP_COMMAND);
        gray_line_k = task4_previous_gray_k;
        Task4_Restore_Gray_Line_Weight();
        OLED_Show_Status("TASK4 STOPPED");
        return;
    }

    if(TASK4_RUNNING == task4_state && task4_time_finished)
    {
        OLED_Stop_Time();
        task4_state = TASK4_HOLD;
        OLED_Show_Status("TASK4 HOLD");
    }

    if(Task_Read_Ball_Position(&position))
    {
        task4_ball_position_cm = position;
        if(TASK4_RUNNING == task4_state || TASK4_HOLD == task4_state)
        {
            Task_Update_Ball_Speed(task4_ball_position_cm,
                                   &task4_speed_cm_per_s,
                                   &task4_last_speed_position_cm,
                                   &task4_last_speed_time_tenths,
                                   &task4_speed_valid,
                                   task4_speed_elapsed_tenths);
            task4_step_target_angle = Task_Calculate_Velocity_Control_Angle(
                                          TASK1_TARGET_POSITION_CM,
                                          task4_ball_position_cm,
                                          task4_speed_cm_per_s)
                                    + Task4_Get_Acceleration_Tilt()
                                    + Task4_Get_Deceleration_Tilt();
            task4_step_target_angle = Task_Limit(
                                          task4_step_target_angle,
                                          -BALL_VELOCITY_MAX_TARGET_ANGLE,
                                          BALL_VELOCITY_MAX_TARGET_ANGLE);
            Step_To_Angle(task4_step_target_angle, TASK4_STEP_FREQUENCY_HZ);
        }
    }

    if(TASK4_RUNNING == task4_state && !task4_time_finished)
    {
        Gray_Line_Update_Target();
    }
}

void Task5_Init (void)
{
    task5_state = TASK4_IDLE;
    task5_ball_position_cm = 0.0f;
    task5_step_target_angle = 0.0f;
    task5_run_ticks = 0;
    task5_decel_ticks = 0;
    task5_decelerating = 0;
    task5_time_finished = 0;
    task5_stop_requested = 0;
    task5_stop_line_armed = 0;
    task5_speed_cm_per_s = 0.0f;
    task5_speed_valid = 0;
}

void Task5_Prepare (void)
{
    if(TASK4_IDLE != task5_state)
    {
        return;
    }

    enable_k230_line = false;
    serial_rx_finish = 0;
    task5_previous_gray_target = gray_line_base_offset;
    task5_previous_gray_k = gray_line_k;
    Task5_Apply_Gray_Line_Weight();
    gray_line_base_offset = 0.0f;
    gray_line_k = 0.0f;
    Serial_Send_Byte(K230_START_COMMAND);
    task5_state = TASK4_READY;
    OLED_Show_Status("TASK5 READY");
}

void Task5_Cancel_Prepare (void)
{
    if(TASK4_WAIT_K230 != task5_state && TASK4_READY != task5_state)
    {
        return;
    }

    Serial_Send_Byte(K230_STOP_COMMAND);
    gray_line_base_offset = task5_previous_gray_target;
    gray_line_k = task5_previous_gray_k;
    Task5_Restore_Gray_Line_Weight();
    task5_state = TASK4_IDLE;
    OLED_Show_Status("INIT DONE");
}

uint8 Task5_Is_Ready (void)
{
    return (TASK4_READY == task5_state) ? 1U : 0U;
}

uint8 Task5_Is_Stop_Requested (void)
{
    return task5_stop_requested;
}

void Task5_Start (void)
{
    if(TASK4_READY != task5_state)
    {
        return;
    }

    enable_task = false;
    enable_gray_line = true;
    gray_line_base_offset = 0.0f;
    gray_line_k = 0.0f;
    gray_line_correct_offset = 0.0f;
    gray_line_left_target = 0.0f;
    gray_line_right_target = 0.0f;
    Motor_PID_New_Start(0.0f, 0.0f);

    task5_run_ticks = 0;
    task5_decel_ticks = 0;
    task5_decelerating = 0;
    task5_time_finished = 0;
    task5_stop_requested = 0;
    task5_stop_line_armed = 0;
    task5_speed_cm_per_s = 0.0f;
    task5_speed_valid = 0;
    task5_state = TASK4_RUNNING;
    while(Button_Get_Angle_Increase_Event())
    {
    }
    Step_To_Angle(TASK5_ACCEL_TILT_ANGLE, TASK5_STEP_FREQUENCY_HZ);
    OLED_Start_Time();
    OLED_Show_Status("TASK5 RUNNING");
}

void Task5_Tick_10ms (void)
{
    float progress;

    if(TASK4_RUNNING != task5_state || task5_time_finished)
    {
        return;
    }

    task5_run_ticks ++;
    if(!task5_decelerating
    && task5_run_ticks >= (TASK5_MAX_TOTAL_TIME_TICKS - TASK5_DECEL_TIME_TICKS))
    {
        task5_decelerating = 1;
        task5_decel_ticks = 0;
    }

    if(task5_decelerating)
    {
        if(task5_decel_ticks < TASK5_DECEL_TIME_TICKS)
        {
            task5_decel_ticks ++;
        }
        progress = (float)(TASK5_DECEL_TIME_TICKS - task5_decel_ticks)
                 / (float)TASK5_DECEL_TIME_TICKS;
        gray_line_base_offset = TASK5_GRAY_TARGET
                              * (3.0f * progress * progress
                              - 2.0f * progress * progress * progress);
    }
    else if(task5_run_ticks <= TASK5_ACCEL_TIME_TICKS)
    {
        progress = (float)task5_run_ticks / (float)TASK5_ACCEL_TIME_TICKS;
        gray_line_base_offset = TASK5_GRAY_TARGET
                              * (3.0f * progress * progress
                              - 2.0f * progress * progress * progress);
    }
    else
    {
        gray_line_base_offset = TASK5_GRAY_TARGET;
    }

    gray_line_k = task5_previous_gray_k
                * gray_line_base_offset / TASK5_GRAY_TARGET;
    gray_line_left_target = gray_line_base_offset + gray_line_correct_offset;
    gray_line_right_target = gray_line_base_offset - gray_line_correct_offset;
    Motor_PID_New_Set_Targets(gray_line_left_target, gray_line_right_target);

    if(task5_decelerating && task5_decel_ticks >= TASK5_DECEL_TIME_TICKS)
    {
        task5_time_finished = 1;
        gray_line_base_offset = 0.0f;
        gray_line_k = 0.0f;
        enable_gray_line = false;
        Motor_PID_New_Stop();
    }
}

void Task5 (void)
{
    float position;

    if((TASK4_RUNNING == task5_state || TASK4_HOLD == task5_state)
    && Button_Get_Angle_Increase_Event())
    {
        task5_stop_requested = 1;
        task5_state = TASK4_DONE;
        enable_gray_line = false;
        Motor_PID_New_Stop();
        OLED_Stop_Time();
        Serial_Send_Byte(K230_STOP_COMMAND);
        gray_line_k = task5_previous_gray_k;
        Task5_Restore_Gray_Line_Weight();
        OLED_Show_Status("TASK5 STOPPED");
        return;
    }

    if(TASK4_RUNNING == task5_state && task5_time_finished)
    {
        OLED_Stop_Time();
        task5_state = TASK4_HOLD;
        OLED_Show_Status("TASK5 HOLD");
    }

    if(Task_Read_Ball_Position(&position))
    {
        task5_ball_position_cm = position;
        if(TASK4_RUNNING == task5_state || TASK4_HOLD == task5_state)
        {
            Task_Update_Ball_Speed(task5_ball_position_cm,
                                   &task5_speed_cm_per_s,
                                   &task5_last_speed_position_cm,
                                   &task5_last_speed_time_tenths,
                                   &task5_speed_valid,
                                   oled_elapsed_tenths);
            task5_step_target_angle = Task_Calculate_Velocity_Control_Angle(
                                          TASK1_TARGET_POSITION_CM,
                                          task5_ball_position_cm,
                                          task5_speed_cm_per_s)
                                    + Task5_Get_Acceleration_Tilt()
                                    + Task5_Get_Deceleration_Tilt();
            task5_step_target_angle = Task_Limit(
                                          task5_step_target_angle,
                                          -BALL_VELOCITY_MAX_TARGET_ANGLE,
                                          BALL_VELOCITY_MAX_TARGET_ANGLE);
            Step_To_Angle(task5_step_target_angle, TASK5_STEP_FREQUENCY_HZ);
        }
    }

    if(TASK4_RUNNING == task5_state && !task5_time_finished)
    {
        Gray_Line_Update_Target();

        if(!task5_stop_line_armed)
        {
            if(!Task_Is_Stop_Line())
            {
                task5_stop_line_armed = 1;
            }
        }
        else if(!task5_decelerating && Task_Is_Stop_Line())
        {
            task5_decelerating = 1;
            task5_decel_ticks = 0;
            OLED_Show_Status("TASK5 RETURN A");
        }
    }
}

void Task6_Init (void)
{
    task6_active = 0;
}

void Task6_Cancel (void)
{
    task6_active = 0;
}

/* mode=6：B0增加0.05度，A31减少0.05度，A30将当前初始化角度保存到Flash。 */
void Task6 (void)
{
    float target_angle;
    uint8 changed = 0;

    if(!task6_active)
    {
        enable_task = false;
        enable_gray_line = false;
        Motor_PID_New_Stop();
        task6_active = 1;
        target_angle = Step_Encoder_Get_Startup_Target_Angle();
        (void)Step_Encoder_Goto_Startup_Angle();
        OLED_Show_Step_Angle(target_angle);
    }

    target_angle = Step_Encoder_Get_Startup_Target_Angle();
    while(Button_Get_Angle_Increase_Event())
    {
        target_angle += TASK6_ANGLE_STEP_DEG;
        changed = 1;
    }
    while(Button_Get_Angle_Decrease_Event())
    {
        target_angle -= TASK6_ANGLE_STEP_DEG;
        changed = 1;
    }
    if(changed && Step_Encoder_Set_Startup_Target_Angle(target_angle))
    {
        (void)Step_Encoder_Goto_Startup_Angle();
        OLED_Show_Step_Angle(target_angle);
    }

    if(Button_Get_Start_Event())
    {
        (void)Step_Encoder_Save_Startup_Target_Angle();
        OLED_Show_Step_Angle(Step_Encoder_Get_Startup_Target_Angle());
    }
}
