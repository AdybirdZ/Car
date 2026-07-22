#include "Initialize.h"

volatile uint8 init_current_module = 0;

/*
函数功能：初始化诊断提示，记录当前模块并输出串口日志
参数：
module：当前初始化模块编号
name：当前初始化模块名称
*/
static void Init_Module_Start (uint8 module, const char *name)
{
    uint8 count = 0;

    init_current_module = module;
    printf("\r\n[INIT %u] %s START\r\n", module, name);

#if INIT_DIAGNOSTIC_ENABLE
    for(count = 0; count < module; count++)
    {
        Buzz(1);
        system_delay_ms(100);
        Buzz(0);
        system_delay_ms(100);
    }
#endif
}

/*
函数功能：输出当前初始化模块完成日志
参数：
module：当前初始化模块编号
name：当前初始化模块名称
*/
static void Init_Module_Done (uint8 module, const char *name)
{
    printf("[INIT %u] %s DONE\r\n", module, name);
}

/*
函数功能：系统初始化，按顺序启动所有外设和模块，贯穿上电到主函数的整个准备阶段
参数：无
*/
void Init ()
{
    clock_init(SYSTEM_CLOCK_80M);
    debug_init();
    printf("\r\n[INIT] DEBUG DONE\r\n");

    // K230必须在电机之前启动，否则K230无法启动
    if(enable_serial)
    {
        Serial_Init();
        system_delay_ms(K230_START_DELAY_MS);
        Serial_Send_Byte(K230_START_COMMAND);
        system_delay_ms(INIT_MODULE_DELAY_MS);
    }

    Light_and_Buzz_Init();
    Init_Module_Start(INIT_MODULE_LIGHT, "LIGHT_BUZZER");
    Init_Module_Done(INIT_MODULE_LIGHT, "LIGHT_BUZZER");
    system_delay_ms(INIT_MODULE_DELAY_MS);

    if(enable_gray && !enable_k230_line)
    {
        Init_Module_Start(INIT_MODULE_GRAY, "GRAY");
        Gray_Init();
        system_delay_ms(INIT_MODULE_DELAY_MS);
        Gray_Wait_First_Data();
        Init_Module_Done(INIT_MODULE_GRAY, "GRAY");
        system_delay_ms(INIT_MODULE_DELAY_MS);
    }

    //Task_Init();

    if(enable_position)
    {
        Init_Module_Start(INIT_MODULE_POSITION, "IMU_POSITION");
        Position_Init();
        Init_Module_Done(INIT_MODULE_POSITION, "IMU_POSITION");
        system_delay_ms(INIT_MODULE_DELAY_MS);

        Init_Module_Start(INIT_MODULE_ACC, "ACC_CALIBRATION");
        Acc_PID_Init();
        Init_Module_Done(INIT_MODULE_ACC, "ACC_CALIBRATION");
        system_delay_ms(INIT_MODULE_DELAY_MS);
    }

    if(enable_motor_output)
    {
        Init_Module_Start(INIT_MODULE_MOTOR, "MOTOR");
        Motor_Init();
        Init_Module_Done(INIT_MODULE_MOTOR, "MOTOR");
        system_delay_ms(INIT_MODULE_DELAY_MS);

        Init_Module_Start(INIT_MODULE_ENCODER, "ENCODER");
        Encoder_Init();
        Init_Module_Done(INIT_MODULE_ENCODER, "ENCODER");
        system_delay_ms(INIT_MODULE_DELAY_MS);
        Motor_PID_Structure_Init(&Motor_Left_PID, 0.035f, 0.034f, 0.012f, PWM_MAX, MOTOR_PID_INTEGRAL_MAX);
        Motor_PID_Structure_Init(&Motor_Right_PID, 0.03f, 0.028f, 0.016f, PWM_MAX, MOTOR_PID_INTEGRAL_MAX);
        Motor_PID_Target_Init(enable_k230_line ? 0.0f : MOTOR_PID_TARGET_OFFSET);
        Angle_PID_Structure_Init(&Angle_PID, 15.0f, 0.0f, 0.0f, 10000.0f, ANGLE_PID_INTEGRAL_MAX);
        Straight_PID_Structure_Init(&Straight_PID, 10.0f, 0.0f, 0.0f, STRAIGHT_PID_OUTPUT_MAX, STRAIGHT_PID_INTEGRAL_MAX);
        absolute_encoder_get_location(LEFT_ENCODER_INDEX);
        absolute_encoder_get_location(RIGHT_ENCODER_INDEX);
        system_delay_ms(INIT_MODULE_DELAY_MS);
    }

    if(enable_WIFI)
    {
        Init_Module_Start(INIT_MODULE_WIFI, "WIFI");
        WIFI_Init();
        Init_Module_Done(INIT_MODULE_WIFI, "WIFI");
        system_delay_ms(INIT_MODULE_DELAY_MS);
    }

    Init_Module_Start(INIT_MODULE_GIMBAL, "GIMBAL");
    Gimbal_Init();
    Init_Module_Done(INIT_MODULE_GIMBAL, "GIMBAL");
    system_delay_ms(INIT_MODULE_DELAY_MS);

    Init_Module_Start(INIT_MODULE_GIMBAL_POS, "GIMBAL_POSITION");
    Gimbal_Set_Multi_Position(GIMBAL_SERVO_1, GIMBAL_STARTUP_SERVO_1_ANGLE_X10);        // 经测试，有时舵机启动错误，所以多启动几次
    system_delay_ms(30);
    Gimbal_Set_Multi_Position(GIMBAL_SERVO_1, GIMBAL_STARTUP_SERVO_1_ANGLE_X10);
    system_delay_ms(30);
    Gimbal_Set_Multi_Position(GIMBAL_SERVO_2, GIMBAL_SERVO_2_VERTICAL_ANGLE_X10);
    system_delay_ms(30);
    Gimbal_Set_Multi_Position(GIMBAL_SERVO_2, GIMBAL_SERVO_2_VERTICAL_ANGLE_X10);
    Init_Module_Done(INIT_MODULE_GIMBAL_POS, "GIMBAL_POSITION");
    system_delay_ms(INIT_MODULE_DELAY_MS);

    pit_ms_init(PIT_TIM_G12, MOTOR_PID_PERIOD_MS, pit_handler, (void *)&pit_flag);
    interrupt_global_enable(0);
    init_current_module = 0;
    printf("[INIT] ALL DONE\r\n");
}
