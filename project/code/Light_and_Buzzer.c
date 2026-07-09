#include "Light_and_Buzzer.h"

void Light_and_Buzz_Init ()
{
    gpio_init(A14, GPO, 0, GPO_PUSH_PULL);  // 初始化GPIO A14 为输出 默认输出低电平
}

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