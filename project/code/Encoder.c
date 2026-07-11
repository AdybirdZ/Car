#include "Encoder.h"

void Encoder_Init ()
{
    absolute_encoder_init(LEFT_ENCODER_INDEX);
    absolute_encoder_init(RIGHT_ENCODER_INDEX);
}

float Encoder_Offset_To_Speed (int16 offset)
{
    return (float)offset * PWM_MAX / ENCODER_RESOLUTION;
}
