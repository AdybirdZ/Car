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
函数功能：设置步进脉冲频率，也就是每秒钟输出多少个脉冲
参数：
frequency_hz：脉冲频率，单位Hz，必须大于0
说明：频率越大转速越快；修改频率时保持原来的运行或停止状态
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
direction：STEP_DIRECTION_FORWARD或STEP_DIRECTION_REVERSE
说明：实际正反方向由电机接线决定，如与需要的方向相反可交换两个宏的使用
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
函数功能：使能步进电机驱动板
参数：无
说明：D36A官方例程将该脚称为SLEEP并置高使能；使能后电机将保持力矩并产生热量
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
说明：先停止脉冲，再将EN/SLEEP置低；失能后电机不再保持力矩，可降低发热
*/
void Step_Disable (void)
{
    Step_Stop();
    gpio_low(STEP_EN_PIN);
    step_enabled = false;
}

/*
函数功能：开始连续输出步进脉冲
参数：无
说明：若驱动板尚未使能，则先自动使能，再输出短高电平步进脉冲
*/
void Step_Start (void)
{
    if(!step_enabled)
    {
        Step_Enable();
    }

    pwm_set_duty(STEP_PWM_PIN, STEP_PWM_DUTY);
    step_running = true;
}

/*
函数功能：停止输出步进脉冲，但不失能驱动板
参数：无
说明：电机停止转动后仍保持力矩；需要释放电机时应调用Step_Disable
*/
void Step_Stop (void)
{
    pwm_set_duty(STEP_PWM_PIN, 0);
    step_running = false;
}
