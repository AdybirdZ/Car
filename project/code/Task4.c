#include "Task4.h"
#include "Serial.h"
#include "Gray_Line.h"
#include "Motor_PID_New.h"
#include "Step.h"
#include "OLED.h"
#include "Task.h"

volatile Task4_State task4_state = TASK4_IDLE;
volatile float task4_ball_position_cm = 0.0f;
volatile float task4_step_target_angle = 0.0f;
static volatile uint16 task4_run_ticks = 0;
static volatile uint8 task4_time_finished = 0;
static float task4_previous_gray_target = 0.0f;

static uint8 Task4_Read_Ball_Position (void)
{
    char message[SERIAL_BUFFER_SIZE] = {0};
    float position = 0.0f;

    if(!Serial_Get_Message(message, SERIAL_BUFFER_SIZE)
    || !Serial_Parse_Signed_Float(message, &position))
    {
        return 0;
    }
    if(position < -TASK4_BALL_MAX_CM || position > TASK4_BALL_MAX_CM)
    {
        return 0;
    }

    task4_ball_position_cm = position;

    // 球偏右为正；增大编码器角度使左侧降低，产生向左的负方向加速度。
    task4_step_target_angle = position * TASK4_BALL_ANGLE_PER_CM;
    Step_To_Angle(task4_step_target_angle, TASK4_STEP_FREQUENCY_HZ);
    return 1;
}

void Task4_Init (void)
{
    task4_state = TASK4_IDLE;
    task4_ball_position_cm = 0.0f;
    task4_step_target_angle = 0.0f;
    task4_run_ticks = 0;
    task4_time_finished = 0;
}

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
    Step_To_Angle(0.0f, TASK4_STEP_FREQUENCY_HZ);
    gray_line_base_offset = task4_previous_gray_target;
    task4_state = TASK4_IDLE;
    OLED_Show_Status("INIT DONE");
}

uint8 Task4_Is_Ready (void)
{
    return (TASK4_READY == task4_state) ? 1U : 0U;
}

void Task4_Start (void)
{
    if(TASK4_READY != task4_state)
    {
        return;
    }

    enable_task = false;
    enable_gray_line = true;
    task4_run_ticks = 0;
    task4_time_finished = 0;
    Motor_PID_New_Start(TASK4_GRAY_TARGET, TASK4_GRAY_TARGET);
    task4_state = TASK4_RUNNING;
    OLED_Show_Status("TASK4 RUNNING");
}

// 由10ms定时中断调用，保证到8.00s时立即关闭车轮，不受主循环阻塞影响。
void Task4_Tick_10ms (void)
{
    if(TASK4_RUNNING != task4_state || task4_time_finished)
    {
        return;
    }
    if(++task4_run_ticks >= (TASK4_RUN_TIME_TENTHS * 10U))
    {
        task4_time_finished = 1;
        enable_gray_line = false;
        Motor_PID_New_Stop();
    }
}

void Task4_Process (void)
{
    if(TASK4_RUNNING == task4_state && task4_time_finished)
    {
        OLED_Stop_Time();
        Serial_Send_Byte(K230_STOP_COMMAND);
        Step_To_Angle(0.0f, TASK4_STEP_FREQUENCY_HZ);
        task4_state = TASK4_DONE;
        OLED_Show_Status("TASK4 DONE");
        return;
    }

    if(TASK4_WAIT_K230 == task4_state || TASK4_READY == task4_state
    || TASK4_RUNNING == task4_state)
    {
        if(Task4_Read_Ball_Position() && TASK4_WAIT_K230 == task4_state)
        {
            task4_state = TASK4_READY;
            OLED_Show_Status("TARGET AND K230 DONE");
        }
    }

    if(TASK4_RUNNING != task4_state)
    {
        return;
    }

    Gray_Line_Update_Target();
}
