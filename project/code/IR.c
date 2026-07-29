#include "IR.h"

bool enable_ir = true;
uint8 ir_value = 0;
uint8 ir_data[IR_CHANNEL_NUM] = {0};

// 数组下标0到7依次对应车辆从左到右的IR1到IR8。
static const gpio_pin_enum ir_pin[IR_CHANNEL_NUM] =
{
    IR_CHANNEL_1_PIN,
    IR_CHANNEL_2_PIN,
    IR_CHANNEL_3_PIN,
    IR_CHANNEL_4_PIN,
    IR_CHANNEL_5_PIN,
    IR_CHANNEL_6_PIN,
    IR_CHANNEL_7_PIN,
    IR_CHANNEL_8_PIN
};

/*
函数功能：初始化八路并行红外巡线传感器的GPIO输入
参数：无
说明：使用内部上拉可避免传感器未连接时输入悬空；传感器输出电压不得超过单片机IO允许电压。
*/
void IR_Init (void)
{
    uint8 i = 0;

    if(!enable_ir)
    {
        return;
    }

    for(i = 0; i < IR_CHANNEL_NUM; i ++)
    {
        gpio_init(ir_pin[i], GPI, GPIO_HIGH, GPI_PULL_UP);
    }

    IR_Update();
}

/*
函数功能：同时读取八路红外传感器，并将结果打包成一个8位数
参数：无
返回值：bit0到bit7依次表示IR1到IR8的实际输入电平，1为高电平，0为低电平
*/
uint8 IR_Read (void)
{
    uint8 i = 0;
    uint8 value = 0;

    if(!enable_ir)
    {
        return 0;
    }

    for(i = 0; i < IR_CHANNEL_NUM; i ++)
    {
        if(GPIO_HIGH == gpio_get_level(ir_pin[i]))
        {
            value |= (uint8)(1U << i);
        }
    }

    return value;
}

/*
函数功能：刷新八路红外传感器数据
参数：无
说明：ir_value保存打包结果，ir_data[0]到ir_data[7]保存各通道的实际高低电平。
*/
void IR_Update (void)
{
    uint8 i = 0;

    if(!enable_ir)
    {
        return;
    }

    ir_value = IR_Read();

    for(i = 0; i < IR_CHANNEL_NUM; i ++)
    {
        ir_data[i] = (ir_value >> i) & 0x01U;
    }
}
