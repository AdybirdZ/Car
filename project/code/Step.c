#include "Step.h"

uint32 step_frequency_hz = STEP_DEFAULT_FREQUENCY_HZ;
uint8 step_direction = STEP_DIRECTION_FORWARD;
bool step_enabled = false;
bool step_running = false;
float step_current_angle = 0.0f;

/*
函数功能：根据STEP频率计算约5us高电平所需的PWM占空比
参数说明：frequency_hz为STEP脉冲频率
返回值：逐飞PWM接口使用的0到PWM_DUTY_MAX占空比数值
*/
uint32 Step_Get_PWM_Duty (uint32 frequency_hz)
{
    uint64 duty_value;

    duty_value = (uint64)frequency_hz * STEP_PULSE_WIDTH_US * PWM_DUTY_MAX;
    duty_value = (duty_value + 999999U) / 1000000U;

    if(duty_value < STEP_PWM_DUTY_MIN)
    {
        duty_value = STEP_PWM_DUTY_MIN;
    }
    if(duty_value > (PWM_DUTY_MAX / 2))
    {
        duty_value = PWM_DUTY_MAX / 2;
    }

    return (uint32)duty_value;
}

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
    step_current_angle = 0.0f;
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
    pwm_init(STEP_PWM_PIN, step_frequency_hz, step_running ? Step_Get_PWM_Duty(step_frequency_hz) : 0);
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

    pwm_set_duty(STEP_PWM_PIN, Step_Get_PWM_Duty(step_frequency_hz));
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
angle：目标角度，正数使用正转方向，负数使用反转方向
frequency_hz：STEP脉冲频率，单位Hz
说明：
函数根据电机整步数和驱动器细分数计算目标脉冲数，再根据脉冲频率计算运行时间。
函数结束时只停止STEP脉冲，不关闭驱动器，因此电机仍保持当前位置。
该函数为阻塞式函数，执行期间不会返回主循环。
*/
void Step_Move_Angle (float angle, uint32 frequency_hz)
{
    float angle_abs = angle;
    float moved_angle;
    uint32 pulse_count;
    uint32 move_time_ms;

    if(angle == 0.0f || frequency_hz == 0)
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

    // 按实际换算出的整数脉冲数更新软件位置，减小连续绝对定位时的累计误差。
    moved_angle = (float)pulse_count * 360.0f / (STEP_MOTOR_FULL_STEPS * STEP_MOTOR_MICROSTEP);
    step_current_angle += (angle < 0.0f) ? -moved_angle : moved_angle;
}

/*
函数功能：让步进电机以指定脉冲频率转到指定的绝对角度
参数说明：
angle：目标绝对角度，Step_Init后初始角度定义为0度
frequency_hz：STEP脉冲频率，单位Hz，数值越大转动越快
说明：函数根据“目标角度-当前软件记录角度”得到本次相对转角，完成后保持当前位置。
步进电机没有绝对位置反馈，断电、堵转或丢步后软件角度与真实角度会产生偏差。
*/
void Step_To_Angle (float angle, uint32 frequency_hz)
{
    float move_angle;

    if(0 == frequency_hz)
    {
        return;
    }

    move_angle = angle - step_current_angle;
    Step_Move_Angle(move_angle, frequency_hz);
}
