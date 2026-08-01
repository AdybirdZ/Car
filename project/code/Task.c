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
static float ball_filter_position_cm = 0.0f;
static uint8 ball_filter_valid = 0;
static uint8 task2_active = 0;
static uint8 task2_stop_line_enabled = 0;
static float task2_start_gray_target = 0.0f;

typedef struct
{
    float contribution_cm_s;
    uint32 time_tenths;
} Task_Integral_History_Item;

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
static float task4_integral_error_cm_s = 0.0f;
static uint32 task4_last_integral_time_tenths = 0;
static uint8 task4_integral_time_valid = 0;
static float task4_last_integral_position_error_cm = 0.0f;
static uint8 task4_integral_position_error_valid = 0;
static Task_Integral_History_Item task4_integral_history[TASK1_INTEGRAL_HISTORY_TICKS];
static uint8 task4_integral_history_index = 0;
static uint8 task4_integral_history_count = 0;

static volatile Task4_State task5_state = TASK4_IDLE;
static volatile float task5_ball_position_cm = 0.0f;
static volatile float task5_step_target_angle = 0.0f;
float task5_gray_line_weight[GRAY_LINE_WEIGHT_NUM] =
{
    -2.5f, -1.8f, -1.0f, 0.2f, 0.2f, 1.0f, 1.8f, 2.5f
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
static volatile uint32 task5_speed_elapsed_tenths = 0;
static uint8 task5_speed_timer_ticks = 0;
static float task5_target_position_cm = 0.0f;
static float task5_integral_error_cm_s = 0.0f;
static uint32 task5_last_integral_time_tenths = 0;
static uint8 task5_integral_time_valid = 0;
static float task5_last_integral_position_error_cm = 0.0f;
static uint8 task5_integral_position_error_valid = 0;
static Task_Integral_History_Item task5_integral_history[TASK1_INTEGRAL_HISTORY_TICKS];
static uint8 task5_integral_history_index = 0;
static uint8 task5_integral_history_count = 0;

static volatile Task4_State task1_state = TASK4_IDLE;
static volatile float task1_ball_position_cm = 0.0f;
static volatile float task1_step_target_angle = 0.0f;
static float task1_speed_cm_per_s = 0.0f;
static float task1_last_speed_position_cm = 0.0f;
static uint32 task1_last_speed_time_tenths = 0;
static uint8 task1_speed_valid = 0;
static float task1_integral_error_cm_s = 0.0f;
static uint32 task1_last_integral_time_tenths = 0;
static uint8 task1_integral_time_valid = 0;
static float task1_last_integral_position_error_cm = 0.0f;
static uint8 task1_integral_position_error_valid = 0;
static Task_Integral_History_Item task1_integral_history[TASK1_INTEGRAL_HISTORY_TICKS];
static uint8 task1_integral_history_index = 0;
static uint8 task1_integral_history_count = 0;
static uint8 task1_stop_requested = 0;
static volatile Task3_State task3_state = TASK3_IDLE;
static volatile float task3_ball_position_cm = 0.0f;
static volatile float task3_step_target_angle = 0.0f;
static float task3_speed_cm_per_s = 0.0f;
static float task3_last_speed_position_cm = 0.0f;
static uint32 task3_last_speed_time_tenths = 0;
static uint8 task3_speed_valid = 0;
static uint8 task3_stop_requested = 0;
volatile uint8 task3_flag = 0;
static volatile Task6_State task6_state = TASK6_SET_TARGET;
static float task6_target_position_cm = 0.0f;
static uint8 task7_active = 0;
float task1_ball_velocity_target_per_cm2 = BALL_VELOCITY_TARGET_PER_CM2;
float task1_ball_velocity_angle_per_cm_per_s = BALL_VELOCITY_ANGLE_PER_CM_PER_S;
float task3_ball_velocity_target_per_cm2 = TASK3_BALL_VELOCITY_TARGET_PER_CM2;
float task3_ball_velocity_angle_per_cm_per_s = TASK3_BALL_VELOCITY_ANGLE_PER_CM_PER_S;
static uint8 task8_parameter_index = 0;
static uint8 task8_flash_loaded = 0;
static uint8 task8_display_required = 1;

static float Task_Limit_Task45_Angle_Change (float target, float previous)
{
    float delta = target - previous;
    if(delta > TASK4_5_MAX_ANGLE_DELTA_DEG)
    {
        return previous + TASK4_5_MAX_ANGLE_DELTA_DEG;
    }
    if(delta < -TASK4_5_MAX_ANGLE_DELTA_DEG)
    {
        return previous - TASK4_5_MAX_ANGLE_DELTA_DEG;
    }
    return target;
}

volatile float task3_absolute_target_angle = STEP_ENCODER_DEFAULT_STARTUP_TARGET_ANGLE;

#define TASK8_FLASH_SECTOR (5U)
#define TASK8_FLASH_PAGE   (0U)
#define TASK8_FLASH_MAGIC  (0x54415338UL)

typedef union
{
    float float_value;
    uint32 uint32_value;
} Task_Float_Value;

static void Task8_Load_Parameters (void)
{
    uint32 data[6];
    Task_Float_Value value;

    task8_flash_loaded = 1U;
    task1_ball_velocity_target_per_cm2 = BALL_VELOCITY_TARGET_PER_CM2;
    task1_ball_velocity_angle_per_cm_per_s = BALL_VELOCITY_ANGLE_PER_CM_PER_S;
    task3_ball_velocity_target_per_cm2 = TASK3_BALL_VELOCITY_TARGET_PER_CM2;
    task3_ball_velocity_angle_per_cm_per_s = TASK3_BALL_VELOCITY_ANGLE_PER_CM_PER_S;
    flash_read_page(TASK8_FLASH_SECTOR, TASK8_FLASH_PAGE, data, 6U);
    if(data[0] != TASK8_FLASH_MAGIC || data[5] != (TASK8_FLASH_MAGIC ^ data[1] ^ data[2] ^ data[3] ^ data[4]))
    {
        return;
    }
    value.uint32_value = data[1];
    if(value.float_value < TASK8_PARAMETER_MIN || value.float_value > TASK8_PARAMETER_MAX) return;
    task1_ball_velocity_target_per_cm2 = value.float_value;
    value.uint32_value = data[2];
    if(value.float_value < TASK8_PARAMETER_MIN || value.float_value > TASK8_PARAMETER_MAX) return;
    task1_ball_velocity_angle_per_cm_per_s = value.float_value;
    value.uint32_value = data[3];
    if(value.float_value < TASK8_PARAMETER_MIN || value.float_value > TASK8_PARAMETER_MAX) return;
    task3_ball_velocity_target_per_cm2 = value.float_value;
    value.uint32_value = data[4];
    if(value.float_value < TASK8_PARAMETER_MIN || value.float_value > TASK8_PARAMETER_MAX) return;
    task3_ball_velocity_angle_per_cm_per_s = value.float_value;
}

static uint8 Task8_Save_Parameters (void)
{
    uint32 data[6];
    Task_Float_Value value;
    data[0] = TASK8_FLASH_MAGIC;
    value.float_value = task1_ball_velocity_target_per_cm2; data[1] = value.uint32_value;
    value.float_value = task1_ball_velocity_angle_per_cm_per_s; data[2] = value.uint32_value;
    value.float_value = task3_ball_velocity_target_per_cm2; data[3] = value.uint32_value;
    value.float_value = task3_ball_velocity_angle_per_cm_per_s; data[4] = value.uint32_value;
    data[5] = TASK8_FLASH_MAGIC ^ data[1] ^ data[2] ^ data[3] ^ data[4];
    return (0U == flash_write_page(TASK8_FLASH_SECTOR, TASK8_FLASH_PAGE, data, 6U)) ? 1U : 0U;
}

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

void Task_Reset_Ball_Position_Filter (void)
{
    ball_filter_valid = 0;
    ball_filter_position_cm = 0.0f;
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
    if(!task8_flash_loaded)
    {
        Task8_Load_Parameters();
    }
}

/*
函数功能：刷新红外数据并检测A点启停线，检测到后立即停车且锁存结果
参数：无
说明：本函数应在车辆行驶期间被周期调用，例如每次主循环调用一次。
*/
void Task_Update (void)
{
    if(!enable_task || task_stop_flag || !task2_stop_line_enabled)
    {
        return;
    }

    if(!task_stop_pending)
    {
        if(Task_Is_Stop_Line())
        {
            task_stop_pending = 1;
            task_stop_delay_ticks = TASK_STOP_DELAY_TICKS;
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

/* Task4/5 use filtered K230 data; Task1/3 keep their tested raw response. */
static float Task_Filter_Ball_Position (float position)
{
    if(!ball_filter_valid)
    {
        ball_filter_position_cm = position;
        ball_filter_valid = 1;
    }
    else
    {
        ball_filter_position_cm += BALL_POSITION_FILTER_ALPHA
                                 * (position - ball_filter_position_cm);
    }

    return ball_filter_position_cm;
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

/* 限制单次步进目标变化量，避免噪声或任务切换造成摆杆角度突变。 */
static float Task_Limit_Angle_Step (float target,
                                    float *current,
                                    float maximum_step)
{
    float difference;

    if(NULL == current || maximum_step <= 0.0f)
    {
        return target;
    }

    difference = target - *current;
    if(difference > maximum_step)
    {
        difference = maximum_step;
    }
    else if(difference < -maximum_step)
    {
        difference = -maximum_step;
    }

    *current += difference;
    return *current;
}

/* Smooth a new angle command before applying the per-update step limit. */
static float Task4_Get_Acceleration_Tilt (void)
{
    float progress;

    if(task4_run_ticks >= TASK4_ACCEL_TIME_TICKS)
    {
        return 0.0f;
    }

    // 车轮起步S曲线的加速度在中段最大，补偿角也应从0平滑增大后回到0。
    progress = (float)task4_run_ticks / (float)TASK4_ACCEL_TIME_TICKS;
    return TASK4_ACCEL_TILT_ANGLE * 4.0f * progress * (1.0f - progress);
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
    if(task5_run_ticks < TASK5_START_TILT_TIME_TICKS)
    {
        return TASK5_START_TILT_ANGLE;
    }
    return 0.0f;
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

void Task2_Start (void)
{
    task2_start_gray_target = gray_line_base_offset;
    task2_active = 1U;
    task2_stop_line_enabled = 0U;
}

void Task2_Update (void)
{
    float ratio;

    if(!task2_active || oled_elapsed_tenths <= TASK2_DECEL_START_TENTHS)
    {
        return;
    }
    if(oled_elapsed_tenths >= (TASK2_DECEL_START_TENTHS + TASK2_DECEL_DURATION_TENTHS))
    {
        gray_line_base_offset = TASK2_DECEL_GRAY_TARGET;
        task2_stop_line_enabled = 1U;
        return;
    }

    ratio = (float)(oled_elapsed_tenths - TASK2_DECEL_START_TENTHS)
          / (float)TASK2_DECEL_DURATION_TENTHS;
    gray_line_base_offset = task2_start_gray_target
                          + (TASK2_DECEL_GRAY_TARGET - task2_start_gray_target) * ratio;
}

void Task2_Stop (void)
{
    if(task2_active)
    {
        gray_line_base_offset = task2_start_gray_target;
        task2_active = 0U;
        task2_stop_line_enabled = 0U;
    }
}

static void Task_Remove_Recent_Integral (float *integral_error_cm_s,
                                         Task_Integral_History_Item *history,
                                         uint8 history_count,
                                         uint32 current_time_tenths)
{
    float recent_contribution_cm_s = 0.0f;
    uint8 index;

    if(NULL == integral_error_cm_s || NULL == history)
    {
        return;
    }

    for(index = 0; index < history_count; index++)
    {
        if((current_time_tenths - history[index].time_tenths)
        <= TASK1_INTEGRAL_HISTORY_TICKS)
        {
            recent_contribution_cm_s += history[index].contribution_cm_s;
            history[index].contribution_cm_s = 0.0f;
        }
    }

    *integral_error_cm_s = Task_Limit(*integral_error_cm_s
                                     - recent_contribution_cm_s,
                                     -TASK1_INTEGRAL_MAX_CM_S,
                                     TASK1_INTEGRAL_MAX_CM_S);
}

/*
 * 只处理目标点附近的持续静态偏差；每个任务传入独立积分状态，
 * 因此任务切换时不会继承前一个任务的积分输出。
 */
static float Task_Update_Integral_Angle (float target_position_cm,
                                         float ball_position_cm,
                                         uint32 current_time_tenths,
                                         float *integral_error_cm_s,
                                         uint32 *last_time_tenths,
                                         uint8 *time_valid,
                                         float *last_position_error_cm,
                                         uint8 *position_error_valid,
                                         Task_Integral_History_Item *history,
                                         uint8 *history_index,
                                         uint8 *history_count)
{
    float error_cm;
    float elapsed_s;
    float contribution_cm_s;

    if(NULL == integral_error_cm_s || NULL == last_time_tenths || NULL == time_valid
    || NULL == last_position_error_cm || NULL == position_error_valid
    || NULL == history || NULL == history_index || NULL == history_count)
    {
        return 0.0f;
    }

    if(!*time_valid)
    {
        *last_time_tenths = current_time_tenths;
        *time_valid = 1;
        *last_position_error_cm = target_position_cm - ball_position_cm;
        *position_error_valid = 1;
        return *integral_error_cm_s * TASK1_INTEGRAL_KI_DEG_PER_CM_S;
    }

    if(current_time_tenths == *last_time_tenths)
    {
        return *integral_error_cm_s * TASK1_INTEGRAL_KI_DEG_PER_CM_S;
    }

    elapsed_s = (float)(current_time_tenths - *last_time_tenths)
              * TASK1_SAMPLE_PERIOD_S;
    *last_time_tenths = current_time_tenths;
    if(elapsed_s <= 0.0f)
    {
        return *integral_error_cm_s * TASK1_INTEGRAL_KI_DEG_PER_CM_S;
    }
    if(elapsed_s > TASK1_INTEGRAL_MAX_DT_S)
    {
        elapsed_s = TASK1_INTEGRAL_MAX_DT_S;
    }

    error_cm = target_position_cm - ball_position_cm;
    if(!*position_error_valid)
    {
        *last_position_error_cm = error_cm;
        *position_error_valid = 1;
    }
    else if((*last_position_error_cm > 0.0f
          && error_cm <= -TASK1_INTEGRAL_CROSS_RESET_CM)
         || (*last_position_error_cm < 0.0f
          && error_cm >= TASK1_INTEGRAL_CROSS_RESET_CM))
    {
        Task_Remove_Recent_Integral(integral_error_cm_s,
                                    history,
                                    *history_count,
                                    current_time_tenths);
    }

    // 只在明显位于目标一侧时更新侧别记录，避免穿过死区后漏掉过零清积分。
    if(error_cm <= -TASK1_INTEGRAL_CROSS_RESET_CM
    || error_cm >= TASK1_INTEGRAL_CROSS_RESET_CM)
    {
        *last_position_error_cm = error_cm;
    }

    if(error_cm > -TASK1_INTEGRAL_DEADBAND_CM
    && error_cm < TASK1_INTEGRAL_DEADBAND_CM)
    {
        return *integral_error_cm_s * TASK1_INTEGRAL_KI_DEG_PER_CM_S;
    }

    error_cm = Task_Limit(error_cm,
                          -TASK1_INTEGRAL_ERROR_LIMIT_CM,
                          TASK1_INTEGRAL_ERROR_LIMIT_CM);
    contribution_cm_s = error_cm * elapsed_s;
    *integral_error_cm_s += contribution_cm_s;
    *integral_error_cm_s = Task_Limit(*integral_error_cm_s,
                                      -TASK1_INTEGRAL_MAX_CM_S,
                                      TASK1_INTEGRAL_MAX_CM_S);

    history[*history_index].contribution_cm_s = contribution_cm_s;
    history[*history_index].time_tenths = current_time_tenths;
    *history_index = (uint8)((*history_index + 1U) % TASK1_INTEGRAL_HISTORY_TICKS);
    if(*history_count < TASK1_INTEGRAL_HISTORY_TICKS)
    {
        (*history_count)++;
    }

    return *integral_error_cm_s * TASK1_INTEGRAL_KI_DEG_PER_CM_S;
}

void Task1_Init (void)
{
    task1_state = TASK4_IDLE;
    task1_ball_position_cm = 0.0f;
    task1_step_target_angle = 0.0f;
    task1_speed_cm_per_s = 0.0f;
    task1_speed_valid = 0;
    task1_integral_error_cm_s = 0.0f;
    task1_integral_time_valid = 0;
    task1_integral_position_error_valid = 0;
    task1_integral_history_index = 0;
    task1_integral_history_count = 0;
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
    Task_Reset_Ball_Position_Filter();
    task1_speed_valid = 0;
    task1_integral_error_cm_s = 0.0f;
    task1_integral_time_valid = 0;
    task1_integral_position_error_valid = 0;
    task1_integral_history_index = 0;
    task1_integral_history_count = 0;
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
                          * task1_ball_velocity_target_per_cm2;
    target_speed_cm_per_s = Task_Limit(target_speed_cm_per_s,
                                       -BALL_VELOCITY_MAX_TARGET_SPEED,
                                       BALL_VELOCITY_MAX_TARGET_SPEED);
    speed_error_cm_per_s = target_speed_cm_per_s - ball_speed_cm_per_s;

    target_angle = TASK1_ZERO_TARGET_OFFSET_ANGLE
                 + speed_error_cm_per_s * task1_ball_velocity_angle_per_cm_per_s;

    return Task_Limit(target_angle,
                      -BALL_VELOCITY_MAX_TARGET_ANGLE,
                      BALL_VELOCITY_MAX_TARGET_ANGLE);
}

/* task3使用独立的二次位置-速度参数，避免影响task1、task4和task5。 */
static float Task3_Calculate_Velocity_Control_Angle (float target_position_cm,
                                                      float ball_position_cm,
                                                      float ball_speed_cm_per_s)
{
    float position_error_cm;
    float position_error_abs_cm;
    float target_speed_cm_per_s;
    float speed_error_cm_per_s;
    float target_angle;

    position_error_cm = target_position_cm - ball_position_cm;
    position_error_abs_cm = (position_error_cm < 0.0f)
                          ? -position_error_cm : position_error_cm;
    target_speed_cm_per_s = position_error_cm * position_error_abs_cm
                          * task3_ball_velocity_target_per_cm2;
    target_speed_cm_per_s = Task_Limit(target_speed_cm_per_s,
                                       -BALL_VELOCITY_MAX_TARGET_SPEED,
                                       BALL_VELOCITY_MAX_TARGET_SPEED);
    speed_error_cm_per_s = target_speed_cm_per_s - ball_speed_cm_per_s;

    target_angle = TASK1_ZERO_TARGET_OFFSET_ANGLE
                 + speed_error_cm_per_s * task3_ball_velocity_angle_per_cm_per_s;

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
    float relative_target_angle = TASK1_ZERO_TARGET_OFFSET_ANGLE;

    (void)target_position_cm;

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
        normal_target_angle += Task_Update_Integral_Angle(
                                   TASK1_TARGET_POSITION_CM,
                                   task1_ball_position_cm,
                                   oled_elapsed_tenths,
                                   &task1_integral_error_cm_s,
                                   &task1_last_integral_time_tenths,
                                   &task1_integral_time_valid,
                                   &task1_last_integral_position_error_cm,
                                   &task1_integral_position_error_valid,
                                   task1_integral_history,
                                   &task1_integral_history_index,
                                   &task1_integral_history_count);
        normal_target_angle = Task_Limit(normal_target_angle,
                                         -BALL_VELOCITY_MAX_TARGET_ANGLE,
                                         BALL_VELOCITY_MAX_TARGET_ANGLE);

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
    task3_stop_requested = 0;
    task3_flag = 0;
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
    task3_step_target_angle = TASK3_FIXED_TILT_ANGLE;
    task3_speed_cm_per_s = 0.0f;
    task3_speed_valid = 0;
    task3_stop_requested = 0;
    task3_flag = 0;
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
     || TASK3_INITIAL_TO_NEGATIVE == task3_state
     || TASK3_WAIT_RETURN_TO_POSITIVE == task3_state
     || TASK3_INITIAL_TO_POSITIVE == task3_state
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
     || TASK3_INITIAL_TO_NEGATIVE == task3_state
     || TASK3_WAIT_RETURN_TO_POSITIVE == task3_state
     || TASK3_INITIAL_TO_POSITIVE == task3_state
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
        if(task3_ball_position_cm >= TASK3_FIRST_REVERSE_POSITION_CM)
        {
            task3_step_target_angle = -TASK3_FIXED_TILT_ANGLE;
            Step_To_Angle(task3_step_target_angle, TASK3_MOVE_FREQUENCY_HZ);
            task3_state = TASK3_INITIAL_TO_NEGATIVE;
            OLED_Show_Status("BALL TO -1CM");
        }
        return;
    }

    if(TASK3_INITIAL_TO_NEGATIVE == task3_state)
    {
        if(task3_ball_position_cm >= TASK3_FLAG_POSITION_CM)
        {
            task3_flag = 1U;
            task3_state = TASK3_WAIT_RETURN_TO_POSITIVE;
            OLED_Show_Status("TASK3 FLAG = 1");
        }
        return;
    }

    if(TASK3_WAIT_RETURN_TO_POSITIVE == task3_state)
    {
        if(task3_ball_position_cm <= TASK3_RETURN_POSITION_CM)
        {
            task3_step_target_angle = TASK3_FIXED_TILT_ANGLE;
            Step_To_Angle(task3_step_target_angle, TASK3_MOVE_FREQUENCY_HZ);
            task3_state = TASK3_INITIAL_TO_POSITIVE;
            OLED_Show_Status("BALL TO -4CM");
        }
        return;
    }

    if(TASK3_INITIAL_TO_POSITIVE == task3_state)
    {
        if(task3_ball_position_cm <= TASK3_QUADRATIC_START_CM)
        {
            task3_state = TASK3_HOLD_NEGATIVE;
            OLED_Show_Status("NEGATIVE CONTROL");
        }
        return;
    }

    if(TASK3_CONTROL_TO_POSITIVE == task3_state)
    {
        // 以+5cm为二次函数目标减小正向速度；通过+4cm后立即开始返回。
        if(task3_ball_position_cm >= TASK3_QUADRATIC_TARGET_CM)
        {
            task3_step_target_angle = -TASK3_FIXED_TILT_ANGLE;
            Step_To_Angle(task3_step_target_angle, TASK3_MOVE_FREQUENCY_HZ);
            task3_state = TASK3_MOVE_TO_NEGATIVE;
            OLED_Show_Status("BALL TO -5CM");
            return;
        }

        task3_step_target_angle = Task3_Calculate_Velocity_Control_Angle(
                                       TASK3_QUADRATIC_TARGET_CM,
                                      task3_ball_position_cm,
                                      task3_speed_cm_per_s);
        Step_To_Angle(task3_step_target_angle,
                      TASK3_POSITIVE_CONTROL_FREQUENCY_HZ);
        return;
    }

    if(TASK3_MOVE_TO_NEGATIVE == task3_state)
    {
        if(task3_ball_position_cm
        <= TASK3_SECOND_REVERSE_POSITION_CM)
        {
            // 到达-1cm时一次性反向急停，随后始终由二次位置-速度函数控制。
            task3_step_target_angle = TASK3_FIXED_TILT_ANGLE;
            Step_To_Angle(task3_step_target_angle, TASK3_MOVE_FREQUENCY_HZ);
            task3_state = TASK3_HOLD_NEGATIVE;
            OLED_Show_Status("BALL HOLD -5CM");
        }
        return;
    }

    if(TASK3_HOLD_NEGATIVE == task3_state)
    {
        task3_step_target_angle = Task3_Calculate_Velocity_Control_Angle(
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
    task4_integral_error_cm_s = 0.0f;
    task4_integral_time_valid = 0;
    task4_integral_position_error_valid = 0;
    task4_integral_history_index = 0;
    task4_integral_history_count = 0;
    Task_Reset_Ball_Position_Filter();
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
    task4_integral_error_cm_s = 0.0f;
    task4_integral_time_valid = 0;
    task4_integral_position_error_valid = 0;
    task4_integral_history_index = 0;
    task4_integral_history_count = 0;
    Task_Reset_Ball_Position_Filter();
    task4_state = TASK4_RUNNING;
    while(Button_Get_Angle_Increase_Event())
    {
    }
    // 起步瞬间保持水平，后续补偿角由10ms定时器与车轮加速度同步输出。
    Step_To_Angle(0.0f, TASK4_STEP_FREQUENCY_HZ);
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
    float target_angle;

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
        task4_ball_position_cm = Task_Filter_Ball_Position(position);
        if(TASK4_RUNNING == task4_state || TASK4_HOLD == task4_state)
        {
            Task_Update_Ball_Speed(task4_ball_position_cm,
                                   &task4_speed_cm_per_s,
                                   &task4_last_speed_position_cm,
                                   &task4_last_speed_time_tenths,
                                   &task4_speed_valid,
                                   task4_speed_elapsed_tenths);
            target_angle = Task_Calculate_Velocity_Control_Angle(
                                          TASK4_TARGET_POSITION_CM,
                                          task4_ball_position_cm,
                                          task4_speed_cm_per_s)
                                     + Task_Update_Integral_Angle(
                                          TASK4_TARGET_POSITION_CM,
                                          task4_ball_position_cm,
                                          task4_speed_elapsed_tenths,
                                          &task4_integral_error_cm_s,
                                          &task4_last_integral_time_tenths,
                                          &task4_integral_time_valid,
                                          &task4_last_integral_position_error_cm,
                                          &task4_integral_position_error_valid,
                                          task4_integral_history,
                                          &task4_integral_history_index,
                                          &task4_integral_history_count)
                                     + Task4_Get_Acceleration_Tilt();
            target_angle = Task_Limit(target_angle,
                                      -BALL_VELOCITY_MAX_TARGET_ANGLE,
                                      BALL_VELOCITY_MAX_TARGET_ANGLE);
            task4_step_target_angle = Task_Limit_Task45_Angle_Change(
                                           target_angle,
                                           task4_step_target_angle);
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
    task5_speed_elapsed_tenths = 0;
    task5_speed_timer_ticks = 0;
    task5_target_position_cm = 0.0f;
    task5_integral_error_cm_s = 0.0f;
    task5_integral_time_valid = 0;
    task5_integral_position_error_valid = 0;
    task5_integral_history_index = 0;
    task5_integral_history_count = 0;
    Task_Reset_Ball_Position_Filter();
}

void Task5_Set_Ball_Target_Position (float target_position_cm)
{
    task5_target_position_cm = Task_Limit(target_position_cm,
                                          TASK6_TARGET_MIN_CM,
                                          TASK6_TARGET_MAX_CM);
}

void Task5_Prepare (void)
{
    if(TASK4_IDLE != task5_state)
    {
        return;
    }

    enable_k230_line = false;
    serial_rx_finish = 0;
    task5_target_position_cm = 0.0f;
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
    task5_speed_elapsed_tenths = 0;
    task5_speed_timer_ticks = 0;
    task5_integral_error_cm_s = 0.0f;
    task5_integral_time_valid = 0;
    task5_integral_position_error_valid = 0;
    task5_integral_history_index = 0;
    task5_integral_history_count = 0;
    Task_Reset_Ball_Position_Filter();
    task5_state = TASK4_RUNNING;
    while(Button_Get_Angle_Increase_Event())
    {
    }
    Step_To_Angle(0.0f, TASK5_STEP_FREQUENCY_HZ);
    OLED_Start_Time();
    OLED_Show_Status("TASK5 RUNNING");
}

void Task5_Tick_10ms (void)
{
    float progress;

    if(TASK4_RUNNING != task5_state && TASK4_HOLD != task5_state)
    {
        return;
    }

    // 此时基独立于OLED显示计时，停车后的钢球速度仍可继续更新。
    task5_speed_timer_ticks ++;
    if(task5_speed_timer_ticks >= 10U)
    {
        task5_speed_timer_ticks = 0;
        task5_speed_elapsed_tenths ++;
    }

    if(TASK4_RUNNING != task5_state || task5_time_finished)
    {
        return;
    }

    task5_run_ticks ++;
    if(task5_run_ticks <= TASK5_ACCEL_TIME_TICKS)
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

    if(task5_run_ticks >= TASK5_MAX_TOTAL_TIME_TICKS)
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
    float target_angle;

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
        task5_ball_position_cm = Task_Filter_Ball_Position(position);
        if(TASK4_RUNNING == task5_state || TASK4_HOLD == task5_state)
        {
            Task_Update_Ball_Speed(task5_ball_position_cm,
                                   &task5_speed_cm_per_s,
                                   &task5_last_speed_position_cm,
                                   &task5_last_speed_time_tenths,
                                   &task5_speed_valid,
                                   task5_speed_elapsed_tenths);
            target_angle = Task_Calculate_Velocity_Control_Angle(
                                          task5_target_position_cm,
                                          task5_ball_position_cm,
                                          task5_speed_cm_per_s)
                                    + Task_Update_Integral_Angle(
                                          task5_target_position_cm,
                                          task5_ball_position_cm,
                                          task5_speed_elapsed_tenths,
                                          &task5_integral_error_cm_s,
                                          &task5_last_integral_time_tenths,
                                          &task5_integral_time_valid,
                                          &task5_last_integral_position_error_cm,
                                          &task5_integral_position_error_valid,
                                          task5_integral_history,
                                          &task5_integral_history_index,
                                          &task5_integral_history_count)
                                     + Task5_Get_Acceleration_Tilt();
            target_angle = Task_Limit(target_angle,
                                      -BALL_VELOCITY_MAX_TARGET_ANGLE,
                                      BALL_VELOCITY_MAX_TARGET_ANGLE);
            target_angle = Task_Limit(target_angle,
                                      -TASK5_MAX_TARGET_ANGLE,
                                      TASK5_MAX_TARGET_ANGLE);
            task5_step_target_angle = Task_Limit_Task45_Angle_Change(
                                           target_angle,
                                           task5_step_target_angle);
            Step_To_Angle(task5_step_target_angle, TASK5_STEP_FREQUENCY_HZ);
        }
    }

    if(TASK4_RUNNING == task5_state && !task5_time_finished)
    {
        Gray_Line_Update_Target();

    }
}

void Task6_Init (void)
{
    task6_state = TASK6_SET_TARGET;
    task6_target_position_cm = 0.0f;
}

void Task6_Prepare (void)
{
    if(TASK6_SET_TARGET != task6_state)
    {
        return;
    }

    Task5_Prepare();
    if(Task5_Is_Ready())
    {
        task6_state = TASK6_READY;
        OLED_Show_Target_Position(task6_target_position_cm);
    }
}

void Task6_Cancel (void)
{
    if(TASK6_RUNNING != task6_state)
    {
        Task5_Cancel_Prepare();
        task6_state = TASK6_SET_TARGET;
    }
}

uint8 Task6_Is_Ready (void)
{
    return (TASK6_READY == task6_state) ? 1U : 0U;
}

uint8 Task6_Is_Stop_Requested (void)
{
    return (TASK6_RUNNING == task6_state && Task5_Is_Stop_Requested()) ? 1U : 0U;
}

void Task6_Start (void)
{
    if(TASK6_READY != task6_state)
    {
        return;
    }

    Task5_Set_Ball_Target_Position(task6_target_position_cm);
    Task5_Start();
    task6_state = TASK6_RUNNING;
}

/* mode=6：B0按1cm循环调整目标位置，A31按0.1cm精调，A30启动整圈巡线。 */
void Task6 (void)
{
    uint8 changed = 0;

    if(TASK6_RUNNING == task6_state)
    {
        Task5();
        return;
    }

    while(Button_Get_Angle_Increase_Event())
    {
        task6_target_position_cm += TASK6_TARGET_COARSE_STEP_CM;
        if(task6_target_position_cm > TASK6_TARGET_MAX_CM)
        {
            task6_target_position_cm = TASK6_TARGET_MIN_CM;
        }
        changed = 1;
    }
    while(Button_Get_Angle_Decrease_Event())
    {
        task6_target_position_cm += TASK6_TARGET_FINE_STEP_CM;
        if(task6_target_position_cm > TASK6_TARGET_MAX_CM)
        {
            task6_target_position_cm = TASK6_TARGET_MIN_CM;
        }
        changed = 1;
    }
    if(changed)
    {
        OLED_Show_Target_Position(task6_target_position_cm);
    }
}

void Task7_Init (void)
{
    task7_active = 0;
}

void Task7_Cancel (void)
{
    task7_active = 0;
}

/* mode=7：保留原mode=6的步进电机初始水平角标定和Flash保存功能。 */
void Task7 (void)
{
    float target_angle;
    uint8 changed = 0;

    if(!task7_active)
    {
        enable_task = false;
        enable_gray_line = false;
        Motor_PID_New_Stop();
        task7_active = 1;
        target_angle = Step_Encoder_Get_Startup_Target_Angle();
        (void)Step_Encoder_Goto_Startup_Angle_Mode7();
        OLED_Show_Step_Angle(target_angle);
    }

    target_angle = Step_Encoder_Get_Startup_Target_Angle();
    while(Button_Get_Angle_Increase_Event())
    {
        target_angle += TASK7_ANGLE_STEP_DEG;
        changed = 1;
    }
    while(Button_Get_Angle_Decrease_Event())
    {
        target_angle -= TASK7_ANGLE_STEP_DEG;
        changed = 1;
    }
    if(changed && Step_Encoder_Set_Startup_Target_Angle(target_angle))
    {
        (void)Step_Encoder_Goto_Startup_Angle_Mode7();
        OLED_Show_Step_Angle(target_angle);
    }

    if(Button_Get_Start_Event())
    {
        (void)Step_Encoder_Save_Startup_Target_Angle();
        OLED_Show_Step_Angle(Step_Encoder_Get_Startup_Target_Angle());
    }
}

void Task8_Init (void)
{
    task8_parameter_index = 0U;
    task8_display_required = 1U;
}

void Task8_Cancel (void)
{
    task8_parameter_index = 0U;
    task8_display_required = 1U;
}

static float *Task8_Current_Parameter (void)
{
    if(0U == task8_parameter_index) return &task1_ball_velocity_target_per_cm2;
    if(1U == task8_parameter_index) return &task1_ball_velocity_angle_per_cm_per_s;
    if(2U == task8_parameter_index) return &task3_ball_velocity_target_per_cm2;
    return &task3_ball_velocity_angle_per_cm_per_s;
}

void Task8 (void)
{
    float *parameter;
    uint8 changed = 0U;

    enable_task = false;
    enable_gray_line = false;
    Motor_PID_New_Stop();
    parameter = Task8_Current_Parameter();

    while(Button_Get_Angle_Increase_Event())
    {
        *parameter += TASK8_PARAMETER_STEP;
        if(*parameter > TASK8_PARAMETER_MAX) *parameter = TASK8_PARAMETER_MAX;
        changed = 1U;
    }
    while(Button_Get_Angle_Decrease_Event())
    {
        *parameter -= TASK8_PARAMETER_STEP;
        if(*parameter < TASK8_PARAMETER_MIN) *parameter = TASK8_PARAMETER_MIN;
        changed = 1U;
    }
    if(changed || task8_display_required)
    {
        OLED_Show_Ball_Control_Parameter(task8_parameter_index, *parameter);
        task8_display_required = 0U;
    }

    /* A30保存四个参数，并切换到下一个参数 */
    if(Button_Get_Start_Event())
    {
        if(Task8_Save_Parameters())
        {
            OLED_Show_Status("PARAM SAVED");
        }
        task8_parameter_index ++;
        if(task8_parameter_index >= 4U) task8_parameter_index = 0U;
        parameter = Task8_Current_Parameter();
        OLED_Show_Ball_Control_Parameter(task8_parameter_index, *parameter);
    }
}
