#include "Position.h"

float euler_angle[3] = {0, 0, 0};       // 下标0为横滚，下标1为俯仰，下标2为方向
bool enable_position = true;

void Position_Init ()
{
    if(!enable_position)
    {
        return;
    }

    while(imu660rc_init(IMU660RC_QUARTERNION_60HZ))
    {
        system_delay_ms(100);
    }

    Position_Update();
}

void Position_Update ()
{
    euler_angle[ROLL] = imu660rc_roll;
    euler_angle[PITCH] = imu660rc_pitch;
    euler_angle[YAW] = imu660rc_yaw + 180;
    if(euler_angle[YAW] >= 540.0f)
    {
        euler_angle[YAW] -= 360.0f;
    }
}
