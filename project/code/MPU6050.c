#include "MPU6050.h"
#include "zf_driver_delay.h"
#include "zf_driver_gpio.h"
#include "zf_driver_soft_iic.h"

// 软件 IIC 对象保存从机地址、SCL/SDA 引脚以及位操作延时。
static soft_iic_info_struct mpu6050_iic;

// 保存当前量程，换算原始数据时必须使用与寄存器一致的灵敏度。
static MPU6050_Acc_Range_Enum mpu6050_acc_range = MPU6050_ACC_RANGE_4G;
static MPU6050_Gyro_Range_Enum mpu6050_gyro_range = MPU6050_GYRO_RANGE_500DPS;

// 对外提供最近一次完整采样以及静止校准得到的陀螺仪零偏。
MPU6050_Data_Struct mpu6050_data = {0};
float mpu6050_gyro_bias_dps[3] = {0.0f, 0.0f, 0.0f};

/*
函数功能：向 MPU6050 的一个8位寄存器写入一个字节。
参数说明：reg为寄存器地址，data为待写入数据。
*/
void MPU6050_Write_Reg (uint8 reg, uint8 data)
{
    // 逐飞库函数自动产生起始信号、器件地址、寄存器地址、数据和停止信号。
    soft_iic_write_8bit_register(&mpu6050_iic, reg, data);
}

/*
函数功能：读取 MPU6050 的一个8位寄存器。
返回值：寄存器当前内容。
*/
uint8 MPU6050_Read_Reg (uint8 reg)
{
    // 该函数内部先写寄存器地址，再发送重复起始信号并读取一个字节。
    return soft_iic_read_8bit_register(&mpu6050_iic, reg);
}

/*
函数功能：从指定寄存器开始连续读取多个字节。
备注：连续读取可保证三轴数据尽量来自同一采样时刻。
*/
void MPU6050_Read_Regs (uint8 reg, uint8 *buffer, uint8 length)
{
    // 空指针或长度为0时不访问总线，防止无效内存写入。
    if(NULL == buffer || 0 == length)
    {
        return;
    }

    // MPU6050 支持寄存器地址自动递增，因此可一次读取整个数据块。
    soft_iic_read_8bit_registers(&mpu6050_iic, reg, buffer, length);
}

/*
函数功能：检查 WHO_AM_I，确认总线上确实是 AD0 接地的 MPU6050。
返回值：MPU6050_OK表示正确，MPU6050_ERROR_ID表示未读到0x68。
*/
uint8 MPU6050_Check_ID (void)
{
    // WHO_AM_I 的正常值为7位 IIC 地址0x68。
    return (MPU6050_WHO_AM_I_VALUE == MPU6050_Read_Reg(MPU6050_REG_WHO_AM_I))
         ? MPU6050_OK
         : MPU6050_ERROR_ID;
}

/*
函数功能：尝试释放被从机拉住的软件 IIC 总线。
步骤：释放SDA，发送9个SCL脉冲，再手工产生一个STOP，最后重新初始化软件IIC。
*/
uint8 MPU6050_Bus_Recovery (void)
{
    uint8 pulse_count;

    // SCL 使用推挽输出，SDA 使用开漏输出；写高电平表示释放总线。
    gpio_init(MPU6050_SCL_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(MPU6050_SDA_PIN, GPO, GPIO_HIGH, GPO_OPEN_DTAIN);

    // 9个时钟脉冲可让停在一个字节中间的从机完成剩余位并释放SDA。
    for(pulse_count = 0; pulse_count < 9; pulse_count++)
    {
        gpio_low(MPU6050_SCL_PIN);
        system_delay_us(5);
        gpio_high(MPU6050_SCL_PIN);
        system_delay_us(5);
    }

    // SDA低且SCL高时再释放SDA，形成标准IIC停止条件。
    gpio_low(MPU6050_SDA_PIN);
    system_delay_us(5);
    gpio_high(MPU6050_SCL_PIN);
    system_delay_us(5);
    gpio_high(MPU6050_SDA_PIN);
    system_delay_us(5);

    // 恢复完成后重新填写逐飞软件IIC对象并配置GPIO模式。
    soft_iic_init(&mpu6050_iic, MPU6050_IIC_ADDRESS, MPU6050_SOFT_IIC_DELAY,
                  MPU6050_SCL_PIN, MPU6050_SDA_PIN);

    // 通过读取芯片ID判断恢复是否成功。
    return MPU6050_Check_ID();
}

/*
函数功能：重复读取一个静态寄存器，直到连续两次结果相同。
适用范围：WHO_AM_I和配置寄存器；不适合加速度、角速度等持续变化的数据寄存器。
*/
uint8 MPU6050_Read_Reg_Retry (uint8 reg, uint8 *value, uint8 retry_count)
{
    uint8 previous_value;
    uint8 current_value;

    if(NULL == value || 0 == retry_count)
    {
        return MPU6050_ERROR_PARAMETER;
    }

    // 保存第一次读取结果，后续每次读取都与前一次比较。
    previous_value = MPU6050_Read_Reg(reg);

    while(retry_count--)
    {
        current_value = MPU6050_Read_Reg(reg);
        if(current_value == previous_value)
        {
            // 连续两次一致后再把结果交给调用者，降低偶发通信错误被采用的概率。
            *value = current_value;
            return MPU6050_OK;
        }

        previous_value = current_value;
        system_delay_ms(1);
    }

    return MPU6050_ERROR_CONFIG;
}

/*
函数功能：重复读取静态配置寄存器，检查关键位是否与期望值一致。
用途：软件IIC底层接口不返回ACK状态，因此使用回读验证配置是否真正写入。
*/
uint8 MPU6050_Verify_Reg (uint8 reg, uint8 mask, uint8 expected, uint8 retry_count)
{
    uint8 read_value;

    // 至少执行一次读取，避免retry_count为0时完全不检查。
    do
    {
        read_value = MPU6050_Read_Reg(reg);
        if((read_value & mask) == (expected & mask))
        {
            return MPU6050_OK;
        }
        system_delay_ms(1);
    }while(retry_count--);

    return MPU6050_ERROR_CONFIG;
}

/*
函数功能：软件复位 MPU6050，并等待内部寄存器恢复稳定。
*/
uint8 MPU6050_Reset (void)
{
    // PWR_MGMT_1 的 DEVICE_RESET 位写1后芯片自动复位，该位随后自动清零。
    MPU6050_Write_Reg(MPU6050_REG_PWR_MGMT_1, 0x80);
    system_delay_ms(100);

    // 复位后再次检查ID，确认器件仍能正常通信。
    return MPU6050_Check_ID();
}

/*
函数功能：选择 MPU6050 系统时钟源。
clock_source常用值：0=内部振荡器，1=X轴陀螺PLL，2/3=Y/Z轴陀螺PLL。
*/
uint8 MPU6050_Set_Clock_Source (uint8 clock_source)
{
    // CLKSEL只有3位，超过7的参数没有意义。
    if(clock_source > 7)
    {
        return MPU6050_ERROR_PARAMETER;
    }

    // 只写时钟选择位，同时确保SLEEP位为0，使芯片保持唤醒状态。
    MPU6050_Write_Reg(MPU6050_REG_PWR_MGMT_1, clock_source & 0x07);
    return MPU6050_Verify_Reg(MPU6050_REG_PWR_MGMT_1, 0x47, clock_source & 0x07, 3);
}

/*
函数功能：设置采样率。DLPF开启时内部陀螺输出率为1000Hz。
计算关系：sample_rate = 1000 / (1 + SMPLRT_DIV)。
*/
uint8 MPU6050_Set_Sample_Rate (uint16 sample_rate_hz)
{
    uint16 divider;

    // 该接口按DLPF开启场景计算，限制到寄存器能稳定表达的4~1000Hz。
    if(sample_rate_hz < 4 || sample_rate_hz > 1000)
    {
        return MPU6050_ERROR_PARAMETER;
    }

    // 先做整数除法，再减1得到寄存器值；例如100Hz对应9。
    divider = (1000U / sample_rate_hz) - 1U;
    if(divider > 255U)
    {
        divider = 255U;
    }

    MPU6050_Write_Reg(MPU6050_REG_SMPLRT_DIV, (uint8)divider);
    return MPU6050_Verify_Reg(MPU6050_REG_SMPLRT_DIV, 0xFF, (uint8)divider, 3);
}

/*
函数功能：设置加速度计满量程，并同步更新物理量换算系数。
*/
uint8 MPU6050_Set_Acc_Range (MPU6050_Acc_Range_Enum range)
{
    if(range != MPU6050_ACC_RANGE_2G && range != MPU6050_ACC_RANGE_4G
    && range != MPU6050_ACC_RANGE_8G && range != MPU6050_ACC_RANGE_16G)
    {
        return MPU6050_ERROR_PARAMETER;
    }

    MPU6050_Write_Reg(MPU6050_REG_ACC_CONFIG, (uint8)range);
    if(MPU6050_OK != MPU6050_Verify_Reg(MPU6050_REG_ACC_CONFIG, 0x18, (uint8)range, 3))
    {
        return MPU6050_ERROR_CONFIG;
    }

    // 只有回读成功后才更新软件量程，避免换算系数与硬件不一致。
    mpu6050_acc_range = range;
    return MPU6050_OK;
}

/*
函数功能：设置陀螺仪满量程，并同步更新物理量换算系数。
*/
uint8 MPU6050_Set_Gyro_Range (MPU6050_Gyro_Range_Enum range)
{
    if(range != MPU6050_GYRO_RANGE_250DPS && range != MPU6050_GYRO_RANGE_500DPS
    && range != MPU6050_GYRO_RANGE_1000DPS && range != MPU6050_GYRO_RANGE_2000DPS)
    {
        return MPU6050_ERROR_PARAMETER;
    }

    MPU6050_Write_Reg(MPU6050_REG_GYRO_CONFIG, (uint8)range);
    if(MPU6050_OK != MPU6050_Verify_Reg(MPU6050_REG_GYRO_CONFIG, 0x18, (uint8)range, 3))
    {
        return MPU6050_ERROR_CONFIG;
    }

    mpu6050_gyro_range = range;
    return MPU6050_OK;
}

/*
函数功能：设置数字低通滤波器DLPF_CFG。
常用值4对应约20Hz带宽，适合小车姿态控制的初始测试。
*/
uint8 MPU6050_Set_DLPF (uint8 dlpf_cfg)
{
    if(dlpf_cfg > 6)
    {
        return MPU6050_ERROR_PARAMETER;
    }

    MPU6050_Write_Reg(MPU6050_REG_CONFIG, dlpf_cfg & 0x07);
    return MPU6050_Verify_Reg(MPU6050_REG_CONFIG, 0x07, dlpf_cfg, 3);
}

/*
函数功能：使能或关闭MPU6050 INT引脚的数据就绪中断。
*/
uint8 MPU6050_Enable_DataReady_INT (uint8 enable)
{
    // INT_ENABLE bit0为DATA_RDY_EN，其余中断在本驱动中保持关闭。
    uint8 register_value = enable ? 0x01 : 0x00;
    MPU6050_Write_Reg(MPU6050_REG_INT_ENABLE, register_value);
    return MPU6050_Verify_Reg(MPU6050_REG_INT_ENABLE, 0x01, register_value, 3);
}

/*
函数功能：连续读取三轴加速度原始值。
*/
void MPU6050_Read_Raw_Acc (MPU6050_Raw_Data_Struct *raw)
{
    uint8 buffer[6];

    if(NULL == raw)
    {
        return;
    }

    MPU6050_Read_Regs(MPU6050_REG_ACC_XOUT_H, buffer, 6);
    raw->acc_x = (int16)(((uint16)buffer[0] << 8) | buffer[1]);
    raw->acc_y = (int16)(((uint16)buffer[2] << 8) | buffer[3]);
    raw->acc_z = (int16)(((uint16)buffer[4] << 8) | buffer[5]);
}

/*
函数功能：连续读取三轴陀螺仪原始值。
*/
void MPU6050_Read_Raw_Gyro (MPU6050_Raw_Data_Struct *raw)
{
    uint8 buffer[6];

    if(NULL == raw)
    {
        return;
    }

    MPU6050_Read_Regs(MPU6050_REG_GYRO_XOUT_H, buffer, 6);
    raw->gyro_x = (int16)(((uint16)buffer[0] << 8) | buffer[1]);
    raw->gyro_y = (int16)(((uint16)buffer[2] << 8) | buffer[3]);
    raw->gyro_z = (int16)(((uint16)buffer[4] << 8) | buffer[5]);
}

/* 函数功能：单独读取温度传感器原始值，高字节在前。 */
int16 MPU6050_Read_Raw_Temp (void)
{
    uint8 buffer[2];

    MPU6050_Read_Regs(MPU6050_REG_TEMP_OUT_H, buffer, 2);
    return (int16)(((uint16)buffer[0] << 8) | buffer[1]);
}

/*
函数功能：从ACC_XOUT_H开始一次读取14字节，获得同一数据块内的加速度、温度和角速度。
*/
void MPU6050_Read_All (MPU6050_Raw_Data_Struct *raw)
{
    uint8 buffer[14];

    if(NULL == raw)
    {
        return;
    }

    // 单次突发读取比三次分开读取更快，也能减少轴数据跨采样周期的概率。
    MPU6050_Read_Regs(MPU6050_REG_ACC_XOUT_H, buffer, 14);

    // MPU6050所有传感器寄存器均按高字节在前、低字节在后排列。
    raw->acc_x    = (int16)(((uint16)buffer[0]  << 8) | buffer[1]);
    raw->acc_y    = (int16)(((uint16)buffer[2]  << 8) | buffer[3]);
    raw->acc_z    = (int16)(((uint16)buffer[4]  << 8) | buffer[5]);
    raw->temperature = (int16)(((uint16)buffer[6]  << 8) | buffer[7]);
    raw->gyro_x     = (int16)(((uint16)buffer[8]  << 8) | buffer[9]);
    raw->gyro_y     = (int16)(((uint16)buffer[10] << 8) | buffer[11]);
    raw->gyro_z     = (int16)(((uint16)buffer[12] << 8) | buffer[13]);
}

/* 函数功能：按照当前加速度量程把原始值转换为g。 */
float MPU6050_Acc_To_G (int16 raw_value)
{
    float sensitivity = 8192.0f;

    switch(mpu6050_acc_range)
    {
        case MPU6050_ACC_RANGE_2G:  sensitivity = 16384.0f; break;
        case MPU6050_ACC_RANGE_4G:  sensitivity = 8192.0f;  break;
        case MPU6050_ACC_RANGE_8G:  sensitivity = 4096.0f;  break;
        case MPU6050_ACC_RANGE_16G: sensitivity = 2048.0f;  break;
        default: break;
    }

    return (float)raw_value / sensitivity;
}

/* 函数功能：按照当前陀螺仪量程把原始值转换为度每秒。 */
float MPU6050_Gyro_To_DPS (int16 raw_value)
{
    float sensitivity = 65.5f;

    switch(mpu6050_gyro_range)
    {
        case MPU6050_GYRO_RANGE_250DPS:  sensitivity = 131.0f; break;
        case MPU6050_GYRO_RANGE_500DPS:  sensitivity = 65.5f;  break;
        case MPU6050_GYRO_RANGE_1000DPS: sensitivity = 32.8f;  break;
        case MPU6050_GYRO_RANGE_2000DPS: sensitivity = 16.4f;  break;
        default: break;
    }

    return (float)raw_value / sensitivity;
}

/* 函数功能：按照数据手册公式将温度ADC值换算为摄氏度。 */
float MPU6050_Temp_To_Celsius (int16 raw_value)
{
    return ((float)raw_value / 340.0f) + 36.53f;
}

/*
函数功能：刷新全局mpu6050_data，并应用静止校准得到的陀螺仪零偏。
*/
void MPU6050_Update (void)
{
    MPU6050_Read_All(&mpu6050_data.raw);

    mpu6050_data.acc_g[0] = MPU6050_Acc_To_G(mpu6050_data.raw.acc_x);
    mpu6050_data.acc_g[1] = MPU6050_Acc_To_G(mpu6050_data.raw.acc_y);
    mpu6050_data.acc_g[2] = MPU6050_Acc_To_G(mpu6050_data.raw.acc_z);

    // 从角速度中减去零偏，使静止时三个轴尽可能接近0度每秒。
    mpu6050_data.gyro_dps[0] = MPU6050_Gyro_To_DPS(mpu6050_data.raw.gyro_x) - mpu6050_gyro_bias_dps[0];
    mpu6050_data.gyro_dps[1] = MPU6050_Gyro_To_DPS(mpu6050_data.raw.gyro_y) - mpu6050_gyro_bias_dps[1];
    mpu6050_data.gyro_dps[2] = MPU6050_Gyro_To_DPS(mpu6050_data.raw.gyro_z) - mpu6050_gyro_bias_dps[2];

    mpu6050_data.temperature_c = MPU6050_Temp_To_Celsius(mpu6050_data.raw.temperature);
}

/*
函数功能：车辆完全静止时采集多组陀螺仪数据，计算三个轴的平均零偏。
注意：校准期间任何转动都会被误认为零偏，导致后续角度持续漂移。
*/
uint8 MPU6050_Calibrate_Gyro (uint16 sample_count, uint16 sample_interval_ms)
{
    uint16 sample_index;
    MPU6050_Raw_Data_Struct raw = {0};
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;

    if(0 == sample_count)
    {
        return MPU6050_ERROR_PARAMETER;
    }

    // 校准开始前先清除旧零偏，避免旧值影响本次原始角速度平均值。
    MPU6050_Set_Gyro_Bias(0.0f, 0.0f, 0.0f);

    for(sample_index = 0; sample_index < sample_count; sample_index++)
    {
        MPU6050_Read_Raw_Gyro(&raw);
        sum_x += MPU6050_Gyro_To_DPS(raw.gyro_x);
        sum_y += MPU6050_Gyro_To_DPS(raw.gyro_y);
        sum_z += MPU6050_Gyro_To_DPS(raw.gyro_z);
        system_delay_ms(sample_interval_ms);
    }

    // 算术平均能削弱随机噪声，得到静止状态下的固定偏置。
    mpu6050_gyro_bias_dps[0] = sum_x / (float)sample_count;
    mpu6050_gyro_bias_dps[1] = sum_y / (float)sample_count;
    mpu6050_gyro_bias_dps[2] = sum_z / (float)sample_count;

    return MPU6050_OK;
}

/* 函数功能：手工设置三个轴的陀螺仪零偏，便于保存和恢复标定结果。 */
void MPU6050_Set_Gyro_Bias (float x_dps, float y_dps, float z_dps)
{
    mpu6050_gyro_bias_dps[0] = x_dps;
    mpu6050_gyro_bias_dps[1] = y_dps;
    mpu6050_gyro_bias_dps[2] = z_dps;
}

/*
函数功能：初始化MPU6050为100Hz采样、约20Hz低通、加速度±4g、陀螺仪±500dps。
返回值：0成功；非0表示ID、配置或参数错误。
*/
uint8 MPU6050_Init (void)
{
    // 首先初始化软件IIC对象；这里传入的是7位地址0x68，不是8位地址0xD0。
    soft_iic_init(&mpu6050_iic, MPU6050_IIC_ADDRESS, MPU6050_SOFT_IIC_DELAY,
                  MPU6050_SCL_PIN, MPU6050_SDA_PIN);
    system_delay_ms(100);

    // 第一次ID检查失败时尝试恢复总线，恢复后仍失败才向上层报告。
    if(MPU6050_OK != MPU6050_Check_ID() && MPU6050_OK != MPU6050_Bus_Recovery())
    {
        return MPU6050_ERROR_ID;
    }

    if(MPU6050_OK != MPU6050_Reset())
    {
        return MPU6050_ERROR_ID;
    }

    // 选择X轴陀螺仪PLL作为系统时钟，稳定性通常优于内部RC振荡器。
    if(MPU6050_OK != MPU6050_Set_Clock_Source(1))
    {
        return MPU6050_ERROR_CONFIG;
    }

    // 关闭MPU6050对辅助IIC主机的控制；XDA/XCL在本项目中不使用。
    MPU6050_Write_Reg(MPU6050_REG_USER_CTRL, 0x00);
    MPU6050_Write_Reg(MPU6050_REG_INT_PIN_CFG, 0x00);

    if(MPU6050_OK != MPU6050_Set_DLPF(4))
    {
        return MPU6050_ERROR_CONFIG;
    }
    if(MPU6050_OK != MPU6050_Set_Sample_Rate(100))
    {
        return MPU6050_ERROR_CONFIG;
    }
    if(MPU6050_OK != MPU6050_Set_Acc_Range(MPU6050_ACC_RANGE_4G))
    {
        return MPU6050_ERROR_CONFIG;
    }
    if(MPU6050_OK != MPU6050_Set_Gyro_Range(MPU6050_GYRO_RANGE_500DPS))
    {
        return MPU6050_ERROR_CONFIG;
    }
    if(MPU6050_OK != MPU6050_Enable_DataReady_INT(0))
    {
        return MPU6050_ERROR_CONFIG;
    }

    // 丢弃配置切换后的短暂过渡数据，再读取第一组完整样本。
    system_delay_ms(50);
    MPU6050_Update();

    return MPU6050_OK;
}
