#include "Initialize.h"

void init ()
{
    gpio_init(A14, GPO, 0, GPO_PUSH_PULL);  //初始化蜂鸣器及LED引脚为输出
}