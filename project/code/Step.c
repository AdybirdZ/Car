#include "Step.h"

uint32 step_frequency_hz = STEP_DEFAULT_FREQUENCY_HZ;
uint8 step_direction = STEP_DIRECTION_FORWARD;
bool step_enabled = false;
bool step_running = false;

/*
函数功能：初始化一个外置驱动板控制的步进电机
参数：无
说明：初始化后不输出脉冲，EN/SLEEP为低电平，驱动板处于休眠状态
*/
void Step_Init (void)
{
    gpio_init(STEP_DIR_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(STEP_EN_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    pwm_init(STEP_PWM_PIN, step_frequency_hz, 0);

    step_direction = STEP_DIRECTION_FORWARD;
    step_enabled = false;
    step_running = false;
}

/*
函数功能：设置步进脉冲频率，也就是每秒钟输出多少个脉冲，频率越大转速越快，修改频率时保持原来的运行或停止状态
参数：
frequency_hz：脉冲频率，单位Hz，必须大于0
*/
void Step_Set_Frequency (uint32 frequency_hz)
{
    if(frequency_hz == 0)
    {
        return;
    }

    step_frequency_hz = frequency_hz;
    pwm_init(STEP_PWM_PIN, step_frequency_hz, step_running ? STEP_PWM_DUTY : 0);
}

/*
函数功能：设置步进电机转动方向
参数：
direction：填入STEP_DIRECTION_FORWARD或STEP_DIRECTION_REVERSE
*/
void Step_Set_Direction (uint8 direction)
{
    if(STEP_DIRECTION_REVERSE == direction)
    {
        gpio_high(STEP_DIR_PIN);
        step_direction = STEP_DIRECTION_REVERSE;
    }
    else
    {
        gpio_low(STEP_DIR_PIN);
        step_direction = STEP_DIRECTION_FORWARD;
    }
}

/*
函数功能：使能步进电机驱动板，使能后电机将保持力矩并产生热量
参数：无
*/
void Step_Enable (void)
{
    gpio_high(STEP_EN_PIN);
    system_delay_ms(1);
    step_enabled = true;
}

/*
函数功能：失能步进电机驱动板
参数：无
*/
void Step_Disable (void)
{
    Step_Stop();                    // 停止脉冲
    gpio_low(STEP_EN_PIN);          // 将EN置低
    step_enabled = false;
}

/*
函数功能：开始连续输出步进脉冲
参数：无
*/
void Step_Start (void)
{
    if(!step_enabled)       // 若驱动板尚未使能，则先自动使能，再输出短高电平步进脉冲
    {
        Step_Enable();
    }

    pwm_set_duty(STEP_PWM_PIN, STEP_PWM_DUTY);
    step_running = true;
}

/*
函数功能：停止输出步进脉冲，但不失能驱动板，也就是说电机停止转动后仍保持力矩。需要释放电机时应调用Step_Disable
参数：无
*/
void Step_Stop (void)
{
    pwm_set_duty(STEP_PWM_PIN, 0);
    step_running = false;
}

/*
函数功能：让步进电机转动指定角度
参数说明：
angle_deg：目标角度，正数使用正转方向，负数使用反转方向
frequency_hz：STEP脉冲频率，单位Hz
说明：
函数根据电机整步数和驱动器细分数计算目标脉冲数，再根据脉冲频率计算运行时间。
函数结束时只停止STEP脉冲，不关闭驱动器，因此电机仍保持当前位置。
该函数为阻塞式函数，执行期间不会返回主循环。
*/
void Step_Move_Angle (float angle_deg, uint32 frequency_hz)
{
    float angle_abs = angle_deg;
    uint32 pulse_count;
    uint32 move_time_ms;

    if(angle_deg == 0.0f || frequency_hz == 0)
    {
        return;
    }

    if(angle_abs < 0.0f)
    {
        angle_abs = -angle_abs;
        Step_Set_Direction(STEP_DIRECTION_REVERSE);
    }
    else
    {
        Step_Set_Direction(STEP_DIRECTION_FORWARD);
    }

    // 四舍五入得到目标脉冲数，避免小角度计算结果被直接截断为0
    pulse_count = (uint32)((angle_abs * STEP_MOTOR_FULL_STEPS * STEP_MOTOR_MICROSTEP / 360.0f) + 0.5f);
    if(0 == pulse_count)
    {
        return;
    }

    Step_Set_Frequency(frequency_hz);

    // 运行时间 = 脉冲数 / 脉冲频率，向上取整以确保完成目标脉冲数
    move_time_ms = (pulse_count * 1000 + frequency_hz - 1) / frequency_hz;

    Step_Start();
    system_delay_ms(move_time_ms);
    Step_Stop();
}
