#include "Initialize.h"

void Init ()
{
    clock_init(SYSTEM_CLOCK_80M);                               // 时钟配置及系统初始化，务必保留！
    debug_init();					                            // 调试串口信息初始化
    Light_and_Buzz_Init();
    Motor_Init();
    Encoder_Init();
    WIFI_Init();
    Motor_PID_Structure_Init(&Motor_Left_PID, 0.02f, 0.016f, 0.012f, PWM_MAX, MOTOR_PID_INTEGRAL_MAX);
    Motor_PID_Structure_Init(&Motor_Right_PID, 0.03f, 0.024f, 0.016f, PWM_MAX, MOTOR_PID_INTEGRAL_MAX);
    Motor_PID_Target_Init(MOTOR_PID_TARGET_OFFSET);
    absolute_encoder_get_location(LEFT_ENCODER_INDEX);          // 防止上电后由于编码器位置不为零，产生一个很大的偏移值，造成不可预测的后果
    absolute_encoder_get_location(RIGHT_ENCODER_INDEX);         // 防止上电后由于编码器位置不为零，产生一个很大的偏移值，造成不可预测的后果
    pit_ms_init(PIT_TIM_G12, MOTOR_PID_PERIOD_MS, motor_pid_pit_handler, (void *)&pit_flag);
    interrupt_global_enable(0);                                 // 中断使能
}