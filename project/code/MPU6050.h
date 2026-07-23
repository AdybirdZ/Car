#ifndef _MPU6050_H_
#define _MPU6050_H_

#include "zf_common_headfile.h"

// 软件 IIC 接线：MPU6050 SCL->A1，SDA->A0，AD0->GND，器件七位地址为0x68。
#define MPU6050_SCL_PIN                 (A1)
#define MPU6050_SDA_PIN                 (A0)
#define MPU6050_IIC_ADDRESS             (0x68)
#define MPU6050_SOFT_IIC_DELAY          (50)

// MPU6050 寄存器地址。这里只列出本驱动实际使用的寄存器。
#define MPU6050_REG_SMPLRT_DIV          (0x19)
#define MPU6050_REG_CONFIG              (0x1A)
#define MPU6050_REG_GYRO_CONFIG         (0x1B)
#define MPU6050_REG_ACCEL_CONFIG        (0x1C)
#define MPU6050_REG_INT_PIN_CFG         (0x37)
#define MPU6050_REG_INT_ENABLE          (0x38)
#define MPU6050_REG_ACCEL_XOUT_H        (0x3B)
#define MPU6050_REG_TEMP_OUT_H          (0x41)
#define MPU6050_REG_GYRO_XOUT_H         (0x43)
#define MPU6050_REG_USER_CTRL           (0x6A)
#define MPU6050_REG_PWR_MGMT_1          (0x6B)
#define MPU6050_REG_WHO_AM_I            (0x75)

#define MPU6050_WHO_AM_I_VALUE          (0x68)

typedef enum
{
    MPU6050_OK = 0,
    MPU6050_ERROR_ID,
    MPU6050_ERROR_CONFIG,
    MPU6050_ERROR_PARAMETER
} MPU6050_Status_Enum;

// 枚举值直接对应 ACCEL_CONFIG 寄存器的 AFS_SEL[1:0] 位。
typedef enum
{
    MPU6050_ACCEL_RANGE_2G  = 0x00,
    MPU6050_ACCEL_RANGE_4G  = 0x08,
    MPU6050_ACCEL_RANGE_8G  = 0x10,
    MPU6050_ACCEL_RANGE_16G = 0x18
} MPU6050_Accel_Range_Enum;

// 枚举值直接对应 GYRO_CONFIG 寄存器的 FS_SEL[1:0] 位。
typedef enum
{
    MPU6050_GYRO_RANGE_250DPS  = 0x00,
    MPU6050_GYRO_RANGE_500DPS  = 0x08,
    MPU6050_GYRO_RANGE_1000DPS = 0x10,
    MPU6050_GYRO_RANGE_2000DPS = 0x18
} MPU6050_Gyro_Range_Enum;

typedef struct
{
    int16 accel_x;
    int16 accel_y;
    int16 accel_z;
    int16 temperature;
    int16 gyro_x;
    int16 gyro_y;
    int16 gyro_z;
} MPU6050_Raw_Data_Struct;

typedef struct
{
    MPU6050_Raw_Data_Struct raw;
    float accel_g[3];
    float gyro_dps[3];
    float temperature_c;
} MPU6050_Data_Struct;

extern MPU6050_Data_Struct mpu6050_data;
extern float mpu6050_gyro_bias_dps[3];

void MPU6050_Write_Reg (uint8 reg, uint8 data);
uint8 MPU6050_Read_Reg (uint8 reg);
void MPU6050_Read_Regs (uint8 reg, uint8 *buffer, uint8 length);

uint8 MPU6050_Check_ID (void);
uint8 MPU6050_Bus_Recovery (void);
uint8 MPU6050_Read_Reg_Retry (uint8 reg, uint8 *value, uint8 retry_count);
uint8 MPU6050_Verify_Reg (uint8 reg, uint8 mask, uint8 expected, uint8 retry_count);
uint8 MPU6050_Reset (void);
uint8 MPU6050_Set_Clock_Source (uint8 clock_source);
uint8 MPU6050_Set_Sample_Rate (uint16 sample_rate_hz);
uint8 MPU6050_Set_Accel_Range (MPU6050_Accel_Range_Enum range);
uint8 MPU6050_Set_Gyro_Range (MPU6050_Gyro_Range_Enum range);
uint8 MPU6050_Set_DLPF (uint8 dlpf_cfg);
uint8 MPU6050_Enable_DataReady_INT (uint8 enable);

void MPU6050_Read_Raw_Accel (MPU6050_Raw_Data_Struct *raw);
void MPU6050_Read_Raw_Gyro (MPU6050_Raw_Data_Struct *raw);
int16 MPU6050_Read_Raw_Temp (void);
void MPU6050_Read_All (MPU6050_Raw_Data_Struct *raw);
float MPU6050_Accel_To_G (int16 raw_value);
float MPU6050_Gyro_To_DPS (int16 raw_value);
float MPU6050_Temp_To_Celsius (int16 raw_value);
void MPU6050_Update (void);
uint8 MPU6050_Calibrate_Gyro (uint16 sample_count, uint16 sample_interval_ms);
void MPU6050_Set_Gyro_Bias (float x_dps, float y_dps, float z_dps);
uint8 MPU6050_Init (void);

#endif
