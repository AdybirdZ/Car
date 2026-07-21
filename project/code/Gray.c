#include "Gray.h"

bool enable_gray = false;            // 注意：若此项为true则必须接好灰度传感器，否则会因为收不到第一组数据而卡死在初始化环节
bool gray_data_ready = false;
uint8 gray_value = 0;
uint8 gray_data[GRAY_CHANNEL_NUM] = {0};

/*
函数功能：灰度传感器初始化，配置CLK和DAT两个GPIO引脚
参数：无
*/
void Gray_Init ()
{
    if(!enable_gray)
    {
        return;
    }

    gpio_init(GRAY_DAT_PIN, GPI, GPIO_LOW, GPI_FLOATING_IN);
    gpio_init(GRAY_CLK_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
}

/*
函数功能：灰度传感器读取，用软件模拟时钟脉冲，逐位读取8个通道的数据
参数：无
*/
uint8 Gray_Read ()
{
    uint8 value = 0;
    uint8 i = 0;

    gpio_low(GRAY_CLK_PIN);

    for(i = 0; i < GRAY_CHANNEL_NUM; i++)
    {
        gpio_high(GRAY_CLK_PIN);
        system_delay_us(5);
        gpio_low(GRAY_CLK_PIN);

        value |= (gpio_get_level(GRAY_DAT_PIN) ? 1 : 0) << i;
    }

    return value;
}

/*
函数功能：灰度数据更新（读一次传感器，把8位数据拆成8个通道数组）
参数：无
*/
void Gray_Update ()
{
    uint8 i = 0;

    if(!enable_gray)
    {
        return;
    }

    gray_value = Gray_Read();

    if(GRAY_INVALID_VALUE != gray_value)
    {
        gray_data_ready = true;
    }

    for(i = 0; i < GRAY_CHANNEL_NUM; i++)
    {
        gray_data[i] = (gray_value >> i) & 0x01;
    }
}

/*
函数功能：反复读取直到收到第一帧有效数据，否则直接跳过
参数：无
*/
void Gray_Wait_First_Data ()        // 等待灰度传感器初始化完毕，否则上电后电机会短暂乱转
{
    if(!enable_gray)
    {
        gray_data_ready = true;
        return;
    }

    while(!gray_data_ready)
    {
        Gray_Update();
        system_delay_ms(5);
    }
}
