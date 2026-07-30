#include "Motor_PID_New.h"

Motor_PID_New_Struct motor_pid_new[2];
volatile bool enable_motor_pid_new = false;

static float Motor_PID_New_Limit (float value)
{
    if(value > PWM_MAX)
    {
        value = PWM_MAX;
    }
    else if(value < -PWM_MAX)
    {
        value = -PWM_MAX;
    }

    return value;
}

static void Motor_PID_New_Timer_Callback (uint32 event, void *ptr)
{
    (void)event;
    (void)ptr;
    Motor_PID_New_Process();
}

void Motor_PID_New_Init (void)
{
    uint8 motor;

    enable_motor_pid_new = false;

    for(motor = LEFT_MOTOR; motor <= RIGHT_MOTOR; motor ++)
    {
        motor_pid_new[motor].kp = MOTOR_PID_NEW_KP_DEFAULT;
        motor_pid_new[motor].ki = MOTOR_PID_NEW_KI_DEFAULT;
        motor_pid_new[motor].kd = MOTOR_PID_NEW_KD_DEFAULT;
        motor_pid_new[motor].target = MOTOR_PID_NEW_TARGET_DEFAULT;
        Motor_PID_New_Clear(motor);
    }

    pit_ms_init(MOTOR_PID_NEW_TIMER, MOTOR_PID_NEW_PERIOD_MS, Motor_PID_New_Timer_Callback, NULL);
}

void Motor_PID_New_Set (uint8 motor, float kp, float ki, float kd)
{
    if(motor > RIGHT_MOTOR)
    {
        return;
    }

    motor_pid_new[motor].kp = kp;
    motor_pid_new[motor].ki = ki;
    motor_pid_new[motor].kd = kd;
}

void Motor_PID_New_Set_Target (uint8 motor, float target)
{
    if(motor <= RIGHT_MOTOR)
    {
        motor_pid_new[motor].target = target;
    }
}

void Motor_PID_New_Set_Targets (float left_target, float right_target)
{
    Motor_PID_New_Set_Target(LEFT_MOTOR, left_target);
    Motor_PID_New_Set_Target(RIGHT_MOTOR, right_target);
}

void Motor_PID_New_Start (float left_target, float right_target)
{
    Motor_PID_New_Clear(LEFT_MOTOR);
    Motor_PID_New_Clear(RIGHT_MOTOR);
    Encoder_New_Clear();
    Motor_PID_New_Set_Targets(left_target, right_target);
    enable_motor_pid_new = true;
}

void Motor_PID_New_Stop (void)
{
    enable_motor_pid_new = false;
    Motor_PID_New_Clear(LEFT_MOTOR);
    Motor_PID_New_Clear(RIGHT_MOTOR);
    Encoder_New_Clear();
    Set_PWM(0.0f, LEFT_MOTOR);
    Set_PWM(0.0f, RIGHT_MOTOR);
}

void Motor_PID_New_Clear (uint8 motor)
{
    if(motor > RIGHT_MOTOR)
    {
        return;
    }

    motor_pid_new[motor].actual = 0.0f;
    motor_pid_new[motor].error = 0.0f;
    motor_pid_new[motor].last_error = 0.0f;
    motor_pid_new[motor].previous_error = 0.0f;
    motor_pid_new[motor].output = 0.0f;
}

float Motor_PID_New_Control (uint8 motor)
{
    Motor_PID_New_Struct *pid;
    float delta_output;

    if(motor > RIGHT_MOTOR)
    {
        return 0.0f;
    }

    pid = &motor_pid_new[motor];
    pid->actual = Encoder_New_Get_Speed(motor);
    pid->error = pid->target - pid->actual;

    delta_output = pid->kp * (pid->error - pid->last_error)
                 + pid->ki * pid->error
                 + pid->kd * (pid->error - 2.0f * pid->last_error + pid->previous_error);

    pid->output = Motor_PID_New_Limit(pid->output + delta_output);
    pid->previous_error = pid->last_error;
    pid->last_error = pid->error;

    // 速度过冲只能减小到 0，禁止 PID 在行驶中突然命令电机反转。
    if((pid->target > 0.0f && pid->output < 0.0f)
    || (pid->target < 0.0f && pid->output > 0.0f)
    || (0.0f == pid->target))
    {
        pid->output = 0.0f;
    }

    Set_PWM(pid->output, (int8)motor);
    return pid->output;
}

void Motor_PID_New_Process (void)
{
    Encoder_New_Update_Speed();

    if(!enable_motor_pid_new)
    {
        return;
    }

    Motor_PID_New_Control(LEFT_MOTOR);
    Motor_PID_New_Control(RIGHT_MOTOR);
}
