#ifndef _EULER_H_
#define _EULER_H_

#include "MPU6050.h"

#define EULER_COMPLEMENTARY_ALPHA_DEFAULT    (0.98f)
#define EULER_ACCEL_NORM_MIN_G               (0.70f)
#define EULER_ACCEL_NORM_MAX_G               (1.30f)
#define EULER_MAX_UPDATE_DT_S                 (0.10f)

typedef struct
{
    float roll;
    float pitch;
    float yaw;
    float complementary_alpha;
    uint8 initialized;
} Euler_Struct;

extern Euler_Struct mpu6050_euler;

void Euler_Init (Euler_Struct *euler);
void Euler_Set_Complementary_Alpha (Euler_Struct *euler, float alpha);
void Euler_Set_Yaw (Euler_Struct *euler, float yaw);
void Euler_Reset_From_Accel (Euler_Struct *euler, const MPU6050_Data_Struct *sensor, float yaw);
uint8 Euler_Update (Euler_Struct *euler, const MPU6050_Data_Struct *sensor, float dt_s);
float Euler_Normalize_180 (float angle);
float Euler_Normalize_360 (float angle);

#endif
