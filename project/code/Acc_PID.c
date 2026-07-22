#include "Acc_PID.h"

bool enable_acc_position = true;
volatile float acc_horizontal_x = 0.0f;
volatile float acc_horizontal_y = 0.0f;
volatile float acc_velocity_x = 0.0f;
volatile float acc_velocity_y = 0.0f;
volatile float acc_distance_x = 0.0f;
volatile float acc_distance_y = 0.0f;
volatile float acc_bias_x = 0.0f;
volatile float acc_bias_y = 0.0f;

static float acc_last_x = 0.0f;
static float acc_last_y = 0.0f;

/*
函数功能：使用欧拉角把IMU机体系三轴加速度转换到水平世界坐标系
参数：
body_x、body_y、body_z：IMU三轴加速度，单位为m/s^2
roll、pitch、yaw：IMU输出的欧拉角，单位为度
world_x、world_y：转换后的水平两轴加速度
*/
static void Acc_PID_To_Horizontal (float body_x, float body_y, float body_z,
                                   float roll, float pitch, float yaw,
                                   float *world_x, float *world_y)
{
    float roll_rad = roll * (PI / 180.0f);
    float pitch_rad = pitch * (PI / 180.0f);
    float yaw_rad = yaw * (PI / 180.0f);
    float cr = cosf(roll_rad);
    float sr = sinf(roll_rad);
    float cp = cosf(pitch_rad);
    float sp = sinf(pitch_rad);
    float cy = cosf(yaw_rad);
    float sy = sinf(yaw_rad);

    *world_x = cy * cp * body_x
             + (cy * sp * sr - sy * cr) * body_y
             + (cy * sp * cr + sy * sr) * body_z;

    *world_y = sy * cp * body_x
             + (sy * sp * sr + cy * cr) * body_y
             + (sy * sp * cr - cy * sr) * body_z;
}

/*
函数功能：清零水平两轴的加速度、速度和位移积分状态
参数：无
*/
void Acc_PID_Reset ()
{
    acc_horizontal_x = 0.0f;
    acc_horizontal_y = 0.0f;
    acc_velocity_x = 0.0f;
    acc_velocity_y = 0.0f;
    acc_distance_x = 0.0f;
    acc_distance_y = 0.0f;
    acc_last_x = 0.0f;
    acc_last_y = 0.0f;
}

/*
函数功能：初始化位移积分模块，车辆静止时采样并计算水平加速度零漂，调用期间车辆必须保持静止！！！
参数：无
*/
void Acc_PID_Init ()
{
    uint16 count = 0;
    float body_x = 0.0f;
    float body_y = 0.0f;
    float body_z = 0.0f;
    float world_x = 0.0f;
    float world_y = 0.0f;
    float bias_sum_x = 0.0f;
    float bias_sum_y = 0.0f;

    acc_bias_x = 0.0f;
    acc_bias_y = 0.0f;
    Acc_PID_Reset();

    for(count = 0; count < ACC_PID_CALIBRATION_COUNT; count++)
    {
        imu660rc_get_quarternion();

        body_x = imu660rc_acc_transition(imu660rc_acc_x) * ACC_PID_GRAVITY;
        body_y = imu660rc_acc_transition(imu660rc_acc_y) * ACC_PID_GRAVITY;
        body_z = imu660rc_acc_transition(imu660rc_acc_z) * ACC_PID_GRAVITY;

        Acc_PID_To_Horizontal(body_x, body_y, body_z,
                              imu660rc_roll, imu660rc_pitch, imu660rc_yaw,
                              &world_x, &world_y);

        bias_sum_x += world_x;
        bias_sum_y += world_y;
        system_delay_ms(ACC_PID_PERIOD_MS);
    }

    acc_bias_x = bias_sum_x / ACC_PID_CALIBRATION_COUNT;
    acc_bias_y = bias_sum_y / ACC_PID_CALIBRATION_COUNT;
}

/*
函数功能：更新水平两轴加速度，并依次积分得到速度和位移
参数：无
备注：本函数按20ms固定周期调用，速度单位为m/s，位移单位为m
*/
void Acc_PID_Update ()
{
    float body_x = 0.0f;
    float body_y = 0.0f;
    float body_z = 0.0f;
    float world_x = 0.0f;
    float world_y = 0.0f;
    float velocity_last_x = 0.0f;
    float velocity_last_y = 0.0f;

    if(!enable_acc_position)
    {
        return;
    }

    body_x = imu660rc_acc_transition(imu660rc_acc_x) * ACC_PID_GRAVITY;
    body_y = imu660rc_acc_transition(imu660rc_acc_y) * ACC_PID_GRAVITY;
    body_z = imu660rc_acc_transition(imu660rc_acc_z) * ACC_PID_GRAVITY;

    Acc_PID_To_Horizontal(body_x, body_y, body_z,
                          euler_angle[ROLL], euler_angle[PITCH], euler_angle[YAW],
                          &world_x, &world_y);

    acc_horizontal_x = world_x - acc_bias_x;
    acc_horizontal_y = world_y - acc_bias_y;

    velocity_last_x = acc_velocity_x;
    velocity_last_y = acc_velocity_y;

    acc_velocity_x += 0.5f * (acc_last_x + acc_horizontal_x) * ACC_PID_PERIOD_S;
    acc_velocity_y += 0.5f * (acc_last_y + acc_horizontal_y) * ACC_PID_PERIOD_S;

    acc_distance_x += 0.5f * (velocity_last_x + acc_velocity_x) * ACC_PID_PERIOD_S;
    acc_distance_y += 0.5f * (velocity_last_y + acc_velocity_y) * ACC_PID_PERIOD_S;

    acc_last_x = acc_horizontal_x;
    acc_last_y = acc_horizontal_y;
}
