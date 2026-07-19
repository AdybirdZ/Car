#include "Light_and_Buzzer.h"

/*
函数功能：初始化灯光和蜂鸣器
参数：无
*/
void Light_and_Buzz_Init ()
{
    gpio_init(A14, GPO, 0, GPO_PUSH_PULL);  // 初始化GPIO A14 为输出 默认输出低电平
}

/*
函数功能：开关蜂鸣器
参数：
status：0=关闭，1=开启
*/
void Buzz (int status)
{
    if (status)
    {
        gpio_high(A14);
    }
    else
    {
        gpio_low(A14);
    }
}

/*
函数功能：开关照明灯
参数：
status：0=关闭，1=开启
*/
void Light (int status)
{
    if (status)
    {
        gpio_low(A14);
    }
    else
    {
        gpio_high(A14);
    }
}