#include "Euler.h"
#include "math.h"

#ifndef M_PI
#define M_PI (3.14159265358979323846f)
#endif

#define EULER_RAD_TO_DEG (180.0f / M_PI)

Euler_Struct mpu6050_euler = {0};

/* 函数功能：把任意角度归一化到[-180, 180)范围。 */
float Euler_Normalize_180 (float angle)
{
    while(angle >= 180.0f)
    {
        angle -= 360.0f;
    }
    while(angle < -180.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

/* 函数功能：把任意角度归一化到[0, 360)范围，便于作为当前角度环的yaw输入。 */
float Euler_Normalize_360 (float angle)
{
    while(angle >= 360.0f)
    {
        angle -= 360.0f;
    }
    while(angle < 0.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

/*
函数功能：初始化欧拉角状态，不读取传感器，也不启动定时器。
*/
void Euler_Init (Euler_Struct *euler)
{
    if(NULL == euler)
    {
        return;
    }

    // 初始角先清零；第一次Euler_Update会用重力方向建立roll和pitch。
    euler->roll = 0.0f;
    euler->pitch = 0.0f;
    euler->yaw = 0.0f;
    euler->complementary_alpha = EULER_COMPLEMENTARY_ALPHA_DEFAULT;
    euler->initialized = 0;
}

/*
函数功能：设置互补滤波陀螺仪权重。
alpha越接近1，短期响应越平滑但漂移修正越慢；必须在[0,1]内。
*/
void Euler_Set_Complementary_Alpha (Euler_Struct *euler, float alpha)
{
    if(NULL == euler)
    {
        return;
    }

    if(alpha < 0.0f)
    {
        alpha = 0.0f;
    }
    else if(alpha > 1.0f)
    {
        alpha = 1.0f;
    }

    euler->complementary_alpha = alpha;
}

/* 函数功能：手工设置yaw基准；输入会被限制到[0,360)范围。 */
void Euler_Set_Yaw (Euler_Struct *euler, float yaw)
{
    if(NULL == euler)
    {
        return;
    }

    euler->yaw = Euler_Normalize_360(yaw);
}

/*
函数功能：只用重力加速度重新建立roll和pitch，并指定yaw初值。
前提：传感器应保持静止，否则线加速度会被错误解释为倾角。
*/
void Euler_Reset_From_Accel (Euler_Struct *euler, const MPU6050_Data_Struct *sensor, float yaw)
{
    float accel_yz;

    if(NULL == euler || NULL == sensor)
    {
        return;
    }

    // roll是Y、Z轴重力分量形成的夹角，atan2可保留完整象限信息。
    euler->roll = atan2f(sensor->accel_g[1], sensor->accel_g[2]) * EULER_RAD_TO_DEG;

    // pitch使用X轴与YZ合成分量计算；根号项始终非负，可避免象限歧义。
    accel_yz = sqrtf(sensor->accel_g[1] * sensor->accel_g[1]
                   + sensor->accel_g[2] * sensor->accel_g[2]);
    euler->pitch = atan2f(-sensor->accel_g[0], accel_yz) * EULER_RAD_TO_DEG;

    // 六轴MPU6050无法由重力确定航向，因此yaw只能使用调用者给出的基准。
    euler->yaw = Euler_Normalize_360(yaw);
    euler->initialized = 1;
}

/*
函数功能：根据一次MPU6050物理量采样更新欧拉角。
算法：角速度积分得到快速姿态；加速度计仅在合加速度接近1g时修正roll/pitch；yaw只积分Z轴角速度。
返回值：0更新成功，非0表示参数或采样周期无效。
*/
uint8 Euler_Update (Euler_Struct *euler, const MPU6050_Data_Struct *sensor, float dt_s)
{
    float accel_norm;
    float accel_roll;
    float accel_pitch;
    float accel_yz;
    float gyro_roll;
    float gyro_pitch;
    float roll_error;
    float pitch_error;

    if(NULL == euler || NULL == sensor)
    {
        return 1;
    }

    // dt必须使用秒。过大的dt通常意味着控制周期丢失，此时积分会产生危险角度跳变。
    if(dt_s <= 0.0f || dt_s > EULER_MAX_UPDATE_DT_S)
    {
        return 2;
    }

    // 第一次更新先用重力方向确定roll/pitch，避免从0度慢慢收敛。
    if(!euler->initialized)
    {
        Euler_Reset_From_Accel(euler, sensor, 0.0f);
    }

    // 对三个角速度分别积分；MPU6050_Update已经减去了静止零偏。
    gyro_roll = Euler_Normalize_180(euler->roll + sensor->gyro_dps[0] * dt_s);
    gyro_pitch = Euler_Normalize_180(euler->pitch + sensor->gyro_dps[1] * dt_s);
    euler->yaw = Euler_Normalize_360(euler->yaw + sensor->gyro_dps[2] * dt_s);

    // 计算加速度矢量模长，用来判断当前数据是否主要由重力构成。
    accel_norm = sqrtf(sensor->accel_g[0] * sensor->accel_g[0]
                     + sensor->accel_g[1] * sensor->accel_g[1]
                     + sensor->accel_g[2] * sensor->accel_g[2]);

    if(accel_norm >= EULER_ACCEL_NORM_MIN_G && accel_norm <= EULER_ACCEL_NORM_MAX_G)
    {
        // 车辆线加速度较小时，利用重力方向计算无长期漂移的roll/pitch参考角。
        accel_roll = atan2f(sensor->accel_g[1], sensor->accel_g[2]) * EULER_RAD_TO_DEG;
        accel_yz = sqrtf(sensor->accel_g[1] * sensor->accel_g[1]
                       + sensor->accel_g[2] * sensor->accel_g[2]);
        accel_pitch = atan2f(-sensor->accel_g[0], accel_yz) * EULER_RAD_TO_DEG;

        // 先计算最短角度误差，再叠加少量加速度修正，避免跨越±180度时错误平均。
        roll_error = Euler_Normalize_180(accel_roll - gyro_roll);
        pitch_error = Euler_Normalize_180(accel_pitch - gyro_pitch);
        euler->roll = Euler_Normalize_180(gyro_roll + (1.0f - euler->complementary_alpha) * roll_error);
        euler->pitch = Euler_Normalize_180(gyro_pitch + (1.0f - euler->complementary_alpha) * pitch_error);
    }
    else
    {
        // 急加速或振动时不相信加速度倾角，仅保留陀螺仪积分结果。
        euler->roll = gyro_roll;
        euler->pitch = gyro_pitch;
    }

    return 0;
}
