#include "Position.h"

float euler_angle[3] = {0, 0, 0};       // 下标0为横滚，下标1为俯仰，下标2为方向
bool enable_position = false;
bool position_init_ok = false;

#define POSITION_INIT_RETRY_MAX   (20)  // 每次失败等待 100 ms，最多等待约 2 s

/*
函数功能：IMU陀螺仪初始化
参数：无
*/
void Position_Init ()
{
    uint8 retry_count = 0;

    if(!enable_position)
    {
        return;
    }

    while(imu660rc_init(IMU660RC_QUARTERNION_60HZ))
    {
        retry_count ++;
        if(retry_count >= POSITION_INIT_RETRY_MAX)
        {
            /* IMU 缺失或通信失败时不能永久卡住整个系统。 */
            enable_position = false;
            position_init_ok = false;
            printf("[INIT] IMU660RC FAILED\r\n");
            return;
        }
        system_delay_ms(100);
    }

    position_init_ok = true;
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
