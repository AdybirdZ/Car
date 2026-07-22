#include "Position.h"

float euler_angle[3] = {0, 0, 0};       // 下标0为横滚，下标1为俯仰，下标2为方向
bool enable_position = true;

/*
函数功能：IMU陀螺仪初始化
参数：无
*/
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

/*
函数功能：姿态数据更新，从IMU驱动读取最新的欧拉角，存入全局数组
参数：无
*/
void Position_Update ()
{
#if POSITION_POLLING_ENABLE
    // Read a quaternion only after IMU INT2 reports that a complete frame is ready.
    if(imu660rc_quarternion_ready)
    {
        imu660rc_quarternion_ready = 0;
        imu660rc_get_quarternion();
    }
#endif

    euler_angle[ROLL] = imu660rc_roll;
    euler_angle[PITCH] = imu660rc_pitch;
    euler_angle[YAW] = imu660rc_yaw;
}
