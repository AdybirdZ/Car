#include "Initialize.h"

/*
函数功能：系统初始化，按顺序启动所有外设和模块，贯穿上电到主函数的整个准备阶段
参数：无
*/
void Init ()
{
    clock_init(SYSTEM_CLOCK_80M);
    debug_init();

    // K230必须在电机之前启动，否则K230无法启动
    if(enable_serial)
    {
        Serial_Init();
        system_delay_ms(K230_START_DELAY_MS);
        Serial_Send_Byte(K230_START_COMMAND);
    }

    Light_and_Buzz_Init();

    if(enable_gray && !enable_k230_line)
    {
        Gray_Init();
        Gray_Wait_First_Data();
    }

    //Task_Init();

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
        Motor_PID_Target_Init(enable_k230_line ? 0.0f : MOTOR_PID_TARGET_OFFSET);
        Angle_PID_Structure_Init(&Angle_PID, 10.0f, 0.0f, 0.0f, 10000.0f, ANGLE_PID_INTEGRAL_MAX);
        absolute_encoder_get_location(LEFT_ENCODER_INDEX);
        absolute_encoder_get_location(RIGHT_ENCODER_INDEX);
    }

    if(enable_WIFI)
    {
        WIFI_Init();
    }

    Gimbal_Init();

    Gimbal_Set_Multi_Position(GIMBAL_SERVO_1, GIMBAL_STARTUP_SERVO_1_ANGLE_X10);
    system_delay_ms(10);
    Gimbal_Set_Multi_Position(GIMBAL_SERVO_2, GIMBAL_SERVO_2_VERTICAL_ANGLE_X10);

    pit_ms_init(PIT_TIM_G12, MOTOR_PID_PERIOD_MS, pit_handler, (void *)&pit_flag);
    interrupt_global_enable(0);
}
