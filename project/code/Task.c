#include "Task.h"
#include "OLED.h"
#include "Light_and_Buzzer.h"
#include "Step_Encoder.h"

// 当前任务：车辆经过中间四路全黑的停车线时停车。
bool enable_task = true;
uint8 task_stop_flag = 0;
static uint8 task_stop_pending = 0;
static uint8 task_stop_delay_ticks = 0;

volatile Task4_State task4_state = TASK4_IDLE;
volatile float task4_ball_position_cm = 0.0f;
volatile float task4_step_target_angle = 0.0f;
static volatile uint16 task4_run_ticks = 0;
static volatile uint8 task4_time_finished = 0;
static float task4_previous_gray_target = 0.0f;
static volatile Task4_State task1_state = TASK4_IDLE;
static volatile float task1_ball_position_cm = 0.0f;
static volatile float task1_step_target_angle = 0.0f;
static int8 task1_brake_return_side = 0;
static float task1_speed_cm_per_s = 0.0f;
static float task1_last_speed_position_cm = 0.0f;
static uint32 task1_last_speed_time_tenths = 0;
static uint8 task1_speed_valid = 0;
static volatile Task3_State task3_state = TASK3_IDLE;
static volatile float task3_ball_position_cm = 0.0f;
static volatile float task3_step_target_angle = 0.0f;
volatile float task3_absolute_target_angle = STEP_ENCODER_STARTUP_TARGET_ANGLE;

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
        Buzz(1);
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
    return 1;
}

static float Task_Abs (float value)
{
    return (value < 0.0f) ? -value : value;
}

// 每0.1秒用K230的新位置更新一次速度，避免串口帧率变化影响速度单位。
static void Task1_Update_Speed (float position)
{
    uint32 current_time_tenths = oled_elapsed_tenths;
    float elapsed_s;

    if(!task1_speed_valid)
    {
        task1_last_speed_position_cm = position;
        task1_last_speed_time_tenths = current_time_tenths;
        task1_speed_cm_per_s = 0.0f;
        task1_speed_valid = 1;
        return;
    }

    if(current_time_tenths == task1_last_speed_time_tenths)
    {
        return;
    }

    elapsed_s = (float)(current_time_tenths - task1_last_speed_time_tenths)
              * TASK1_SAMPLE_PERIOD_S;
    if(elapsed_s > 0.0f)
    {
        task1_speed_cm_per_s = (position - task1_last_speed_position_cm) / elapsed_s;
    }

    task1_last_speed_position_cm = position;
    task1_last_speed_time_tenths = current_time_tenths;
}

void Task1_Init (void)
{
    task1_state = TASK4_IDLE;
    task1_ball_position_cm = 0.0f;
    task1_step_target_angle = 0.0f;
    task1_brake_return_side = 0;
    task1_speed_cm_per_s = 0.0f;
    task1_speed_valid = 0;
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
    task1_brake_return_side = 0;
    task1_speed_valid = 0;
    OLED_Start_Time();
    task1_state = TASK4_RUNNING;
    OLED_Show_Status("BALL BALANCE");
}

/*
函数功能：根据指定的钢球目标位置，计算步进电机应到达的机械绝对角度
参数说明：target_position_cm为钢球目标位置，单位cm
返回值：0.0~360.0度范围内的机械绝对目标角度
说明：只在设置新的固定位置目标时调用，不随K230实时位置重复计算。
*/
float Task_Calculate_Absolute_Target_Angle (float target_position_cm)
{
    float relative_target_angle = -target_position_cm * TASK3_BALL_ANGLE_PER_CM;

    return Step_Encoder_Relative_To_Absolute_Angle(relative_target_angle);
}

// mode=1运行函数：车辆始终静止，K230位置数据只用于保持钢球在中心附近。
void Task1 (void)
{
    float position;
    float normal_target_angle;
    float brake_target_angle;
    float brake_angle;

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
        Task1_Update_Speed(task1_ball_position_cm);
        normal_target_angle = TASK1_ZERO_TARGET_OFFSET_ANGLE
                            - task1_ball_position_cm * TASK1_BALL_ANGLE_PER_CM;

        if(task1_ball_position_cm <= -TASK1_BRAKE_TRIGGER_CM)
        {
            task1_brake_return_side = -1;
        }
        else if(task1_ball_position_cm >= TASK1_BRAKE_TRIGGER_CM)
        {
            task1_brake_return_side = 1;
        }
        else if(0 != task1_brake_return_side)
        {
            // 实测急刹方向：从负侧回到中心时在-1cm给负角度；从正侧回到中心时在+1cm给正角度。
            brake_angle = Task_Abs(task1_speed_cm_per_s)
                        * TASK1_BRAKE_ANGLE_PER_CM_PER_S;
            if(brake_angle > TASK1_BRAKE_MAX_ANGLE)
            {
                brake_angle = TASK1_BRAKE_MAX_ANGLE;
            }
            brake_target_angle = (task1_brake_return_side < 0)
                               ? -brake_angle : brake_angle;
            task1_brake_return_side = 0;
            Step_To_Angle(brake_target_angle, TASK1_BRAKE_FREQUENCY_HZ);
            Step_To_Angle(normal_target_angle, TASK1_BRAKE_FREQUENCY_HZ);
        }

        task1_step_target_angle = normal_target_angle;
        Step_To_Angle(task1_step_target_angle, TASK4_STEP_FREQUENCY_HZ);
    }
}

void Task3_Init (void)
{
    task3_state = TASK3_IDLE;
    task3_ball_position_cm = 0.0f;
    task3_step_target_angle = 0.0f;
    task3_absolute_target_angle = STEP_ENCODER_STARTUP_TARGET_ANGLE;
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
    Step_To_Angle(task3_step_target_angle, TASK3_MOVE_FREQUENCY_HZ);
    task3_state = TASK3_MOVE_TO_POSITIVE;
    OLED_Start_Time();
    OLED_Show_Status("BALL TO +5CM");
}

// mode=3运行函数：0cm到+5cm，再到-5cm并稳定在-5cm附近。
void Task3 (void)
{
    float position;
    float final_relative_angle;
    float position_error;

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

    if(TASK3_MOVE_TO_POSITIVE == task3_state)
    {
        if(task3_ball_position_cm
        >= (TASK3_POSITIVE_POSITION_CM - TASK3_POSITION_TOLERANCE_CM))
        {
            task3_step_target_angle = TASK3_TO_NEGATIVE_ANGLE;
            Step_To_Angle(task3_step_target_angle, TASK3_MOVE_FREQUENCY_HZ);
            task3_state = TASK3_MOVE_TO_NEGATIVE;
            OLED_Show_Status("BALL TO -5CM");
        }
        return;
    }

    if(TASK3_MOVE_TO_NEGATIVE == task3_state)
    {
        if(task3_ball_position_cm
        <= (TASK3_NEGATIVE_POSITION_CM + TASK3_POSITION_TOLERANCE_CM))
        {
            // -5cm的固定绝对基准角只在进入最终稳定阶段时计算一次。
            task3_absolute_target_angle =
                Task_Calculate_Absolute_Target_Angle(TASK3_NEGATIVE_POSITION_CM);
            task3_step_target_angle =
                -TASK3_NEGATIVE_POSITION_CM * TASK3_BALL_ANGLE_PER_CM;
            Step_To_Angle(task3_step_target_angle, TASK3_MOVE_FREQUENCY_HZ);
            task3_state = TASK3_HOLD_NEGATIVE;
            OLED_Show_Status("HOLD -5CM");
        }
        return;
    }

    if(TASK3_HOLD_NEGATIVE == task3_state)
    {
        final_relative_angle =
            -TASK3_NEGATIVE_POSITION_CM * TASK3_BALL_ANGLE_PER_CM;
        position_error = task3_ball_position_cm - TASK3_NEGATIVE_POSITION_CM;
        task3_step_target_angle = final_relative_angle
                                + position_error * TASK3_BALL_ANGLE_PER_CM;
        Step_To_Angle(task3_step_target_angle, TASK3_MOVE_FREQUENCY_HZ);
    }
}

void Task4_Init (void)
{
    task4_state = TASK4_IDLE;
    task4_ball_position_cm = 0.0f;
    task4_step_target_angle = 0.0f;
    task4_run_ticks = 0;
    task4_time_finished = 0;
}

// mode切换到4时发送小写s，收到第一帧合法#±XX.XX$后进入可启动状态。
void Task4_Prepare (void)
{
    if(TASK4_IDLE != task4_state)
    {
        return;
    }

    enable_k230_line = false;
    serial_rx_finish = 0;
    task4_previous_gray_target = gray_line_base_offset;
    gray_line_base_offset = TASK4_GRAY_TARGET;
    Serial_Send_Byte(K230_START_COMMAND);
    task4_state = TASK4_WAIT_K230;
    OLED_Show_Status("WAIT K230");
}

void Task4_Cancel_Prepare (void)
{
    if(TASK4_WAIT_K230 != task4_state && TASK4_READY != task4_state)
    {
        return;
    }
    Serial_Send_Byte(K230_STOP_COMMAND);
    gray_line_base_offset = task4_previous_gray_target;
    task4_state = TASK4_IDLE;
    OLED_Show_Status("INIT DONE");
}

uint8 Task4_Is_Ready (void)
{
    return (TASK4_READY == task4_state) ? 1U : 0U;
}

// 蜂鸣1秒由main完成；启动巡线后在0.1秒内顺时针倾斜20度，随后立即回正。
void Task4_Start (void)
{
    if(TASK4_READY != task4_state)
    {
        return;
    }

    enable_task = false;
    enable_gray_line = true;
    gray_line_base_offset = TASK4_GRAY_TARGET;
    Motor_PID_New_Start(TASK4_GRAY_TARGET, TASK4_GRAY_TARGET);

    Step_To_Angle(TASK4_LAUNCH_TILT_ANGLE, TASK4_LAUNCH_STEP_FREQUENCY_HZ);
    Step_To_Angle(0.0f, TASK4_LAUNCH_STEP_FREQUENCY_HZ);

    task4_run_ticks = 0;
    task4_time_finished = 0;
    task4_state = TASK4_RUNNING;
    OLED_Start_Time();
    OLED_Show_Status("TASK4 RUNNING");
}

// OLED的10ms定时器调用；800次后立即关闭车轮，避免主循环阻塞导致超过8秒。
void Task4_Tick_10ms (void)
{
    if(TASK4_RUNNING != task4_state || task4_time_finished)
    {
        return;
    }
    if(++task4_run_ticks >= TASK4_RUN_TIME_TICKS)
    {
        task4_time_finished = 1;
        enable_gray_line = false;
        Motor_PID_New_Stop();
    }
}

// mode=4运行函数：巡线并根据钢球位置按0.3度/cm的比例设置摆杆角度。
void Task4 (void)
{
    float position;

    if(TASK4_RUNNING == task4_state && task4_time_finished)
    {
        OLED_Stop_Time();
        Serial_Send_Byte(K230_STOP_COMMAND);
        Step_To_Angle(0.0f, TASK4_STEP_FREQUENCY_HZ);
        task4_state = TASK4_DONE;
        OLED_Show_Status("TASK4 DONE");
        return;
    }

    if(Task_Read_Ball_Position(&position))
    {
        task4_ball_position_cm = position;
        if(TASK4_WAIT_K230 == task4_state)
        {
            task4_state = TASK4_READY;
            OLED_Show_Status("TARGET AND K230 DONE");
        }
        else if(TASK4_RUNNING == task4_state)
        {
            // 正值表示球偏右；增大编码器角度使左侧降低，给钢球向左的负加速度。
            task4_step_target_angle = task4_ball_position_cm * TASK4_BALL_ANGLE_PER_CM;
            Step_To_Angle(task4_step_target_angle, TASK4_STEP_FREQUENCY_HZ);
        }
    }

    if(TASK4_RUNNING == task4_state)
    {
        Gray_Line_Update_Target();
    }
}
