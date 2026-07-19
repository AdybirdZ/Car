#include "Encoder.h"

/*
函数功能：初始化左右两个绝对值编码器
参数：无
*/
void Encoder_Init ()
{
    absolute_encoder_init(LEFT_ENCODER_INDEX);
    absolute_encoder_init(RIGHT_ENCODER_INDEX);
}
