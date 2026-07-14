#include "Initialize.h"

void Init ()
{
    clock_init(SYSTEM_CLOCK_80M);
    debug_init();
    Light_and_Buzz_Init();

    if(enable_gray)
    {
        Gray_Init();
    }

    Task_Init();

    if(enable_position)
    {
        Position_Init();
    }

    if(enable_motor_output)
    {
        Motor_Init();
        Encoder_Init();
        Motor_PID_Structure_Init(&Motor_Left_PID, 0.02f, 0.026f, 0.012f, PWM_MAX, MOTOR_PID_INTEGRAL_MAX);
        Motor_PID_Structure_Init(&Motor_Right_PID, 0.03f, 0.028f, 0.016f, PWM_MAX, MOTOR_PID_INTEGRAL_MAX);
        Motor_PID_Target_Init(MOTOR_PID_TARGET_OFFSET);
        Angle_PID_Structure_Init(&Angle_PID, 0.075f, 0.075f, 0.08f, PWM_MAX, ANGLE_PID_INTEGRAL_MAX);
        absolute_encoder_get_location(LEFT_ENCODER_INDEX);
        absolute_encoder_get_location(RIGHT_ENCODER_INDEX);
    }

    if(enable_WIFI)
    {
        WIFI_Init();
    }

    pit_ms_init(PIT_TIM_G12, MOTOR_PID_PERIOD_MS, pit_handler, (void *)&pit_flag);
    interrupt_global_enable(0);
}
