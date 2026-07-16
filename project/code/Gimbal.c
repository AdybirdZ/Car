#include "Gimbal.h"

/*
以下宏定义来自轮趣科技云台资料：
通信格式：0x7A + 电机地址 + 功能码 + 数据 + BCC异或校验 + 0x7B
两轴默认地址：0x01、0x02
使能：0x06，设置模式：0x00，设置速度：0x01，设置多圈位置：0x02，请求反馈：0x0E 
多圈角度单位为0.1°
反馈帧固定为9字节，第4至第7个数据为舵机返回的数据
*/

#define GIMBAL_FRAME_HEADER           (0x7A)        // 帧头
#define GIMBAL_FRAME_TAIL             (0x7B)        // 帧尾

// 以下为可选择的向舵机发送的命令
#define GIMBAL_COMMAND_MODE           (0x00)
#define GIMBAL_COMMAND_SPEED          (0x01)
#define GIMBAL_COMMAND_MULTI_POSITION (0x02)
#define GIMBAL_COMMAND_DISABLE        (0x05)
#define GIMBAL_COMMAND_ENABLE         (0x06)
#define GIMBAL_COMMAND_FEEDBACK       (0x0E)
#define GIMBAL_RX_FRAME_SIZE          (9)
#define GIMBAL_TX_BUFFER_SIZE         (12)

typedef struct
{
    int32 servo_1_angle_x10;
    int32 servo_2_angle_x10;
}Gimbal_Test_Step_Struct;

Gimbal_Feedback_Struct gimbal_feedback[2] = {0};

static uint8 gimbal_rx_buffer[GIMBAL_RX_FRAME_SIZE] = {0};
static uint8 gimbal_rx_index = 0;

static const Gimbal_Test_Step_Struct gimbal_test_steps[] =
{
    {   0,    0},
    { 150,    0},       // 15°，0°
    {   0,    0},
    {-150,    0},       // -15°，0°
    {   0,    0},
    {   0,  100},       // 0°，10°
    {   0,    0},
    {   0, -100}        // 0°，-10°
};

static uint8 Gimbal_Calculate_BCC (const uint8 *data, uint8 length)         // 异或校验函数
{
    uint8 index = 0;
    uint8 bcc = 0;

    for(index = 0; index < length; index++)
    {
        bcc ^= data[index];
    }

    return bcc;
}

static void Gimbal_Send_Command (uint8 servo, uint8 command, const uint8 *data, uint8 data_length)
{
    uint8 frame[GIMBAL_TX_BUFFER_SIZE] = {0};
    uint8 index = 0;
    uint8 data_index = 0;

    if(data_length > (GIMBAL_TX_BUFFER_SIZE - 5))       // 除数据外，还要发送的占5位，所以减去5
    {
        return;
    }

    frame[index++] = GIMBAL_FRAME_HEADER;
    frame[index++] = servo;
    frame[index++] = command;

    for(data_index = 0; data_index < data_length; data_index++)
    {
        frame[index++] = data[data_index];
    }

    frame[index] = Gimbal_Calculate_BCC(frame, index);
    index++;
    frame[index++] = GIMBAL_FRAME_TAIL;
    uart_write_buffer(GIMBAL_UART_INDEX, frame, index);
}

static void Gimbal_Parse_Feedback_Frame (void)          // 解析读取来的舵机数据
{
    uint8 servo_index = 0;
    uint8 feedback_type = gimbal_rx_buffer[2];
    uint32 raw_data = 0;

    if(gimbal_rx_buffer[8] != GIMBAL_FRAME_TAIL || Gimbal_Calculate_BCC(gimbal_rx_buffer, 7) != gimbal_rx_buffer[7])
    {
        return;
    }

    if(GIMBAL_SERVO_1 == gimbal_rx_buffer[1])
    {
        servo_index = 0;
    }
    else if(GIMBAL_SERVO_2 == gimbal_rx_buffer[1])
    {
        servo_index = 1;
    }
    else
    {
        return;
    }

    raw_data = ((uint32)gimbal_rx_buffer[3] << 24) | ((uint32)gimbal_rx_buffer[4] << 16) | ((uint32)gimbal_rx_buffer[5] << 8) | ((uint32)gimbal_rx_buffer[6]);

    switch(feedback_type)
    {
        case GIMBAL_FEEDBACK_SPEED:
        {
            gimbal_feedback[servo_index].speed = (int32)raw_data;
        }break;

        case GIMBAL_FEEDBACK_MULTI_ANGLE:
        {
            gimbal_feedback[servo_index].multi_angle_x10 = (int32)raw_data;
        }break;

        case GIMBAL_FEEDBACK_SINGLE_ANGLE:
        {
            gimbal_feedback[servo_index].single_angle_x10 = (uint16)raw_data;
        }break;

        case GIMBAL_FEEDBACK_ACCELERATION:
        {
            gimbal_feedback[servo_index].acceleration = (int32)raw_data;
        }break;

        case GIMBAL_FEEDBACK_VOLTAGE:
        {
            gimbal_feedback[servo_index].voltage_x100 = (uint16)raw_data;
        }break;

        default:
        {
            return;
        }
    }

    gimbal_feedback[servo_index].data_ready = 1;
}

static void Gimbal_Parse_Byte (uint8 data)              // 将接收到的字节利用解析数据函数逐个解析
{
    if(gimbal_rx_index == 0)
    {
        if(GIMBAL_FRAME_HEADER == data)
        {
            gimbal_rx_buffer[gimbal_rx_index++] = data;
        }
        return;
    }

    gimbal_rx_buffer[gimbal_rx_index++] = data;

    if(GIMBAL_RX_FRAME_SIZE <= gimbal_rx_index)
    {
        Gimbal_Parse_Feedback_Frame();
        gimbal_rx_index = 0;
    }
}

static void Gimbal_UART_Callback (uint32 state, void *ptr)
{
    uint8 data = 0;

    (void)ptr;

    if(UART_INTERRUPT_STATE_RX != state)
    {
        return;
    }

    while(uart_query_byte(GIMBAL_UART_INDEX, &data))
    {
        Gimbal_Parse_Byte(data);
    }
}

void Gimbal_Enable (uint8 servo)
{
    Gimbal_Send_Command(servo, GIMBAL_COMMAND_ENABLE, NULL, 0);
}

void Gimbal_Disable (uint8 servo)
{
    Gimbal_Send_Command(servo, GIMBAL_COMMAND_DISABLE, NULL, 0);
}

void Gimbal_Set_Mode (uint8 servo, uint16 mode)
{
    uint8 data[2] = {0};

    data[0] = (uint8)(mode >> 8);
    data[1] = (uint8)mode;
    Gimbal_Send_Command(servo, GIMBAL_COMMAND_MODE, data, 2);
}

void Gimbal_Set_Speed (uint8 servo, int16 speed_rpm)
{
    uint16 raw_speed = (uint16)speed_rpm;
    uint8 data[2] = {0};

    data[0] = (uint8)(raw_speed >> 8);
    data[1] = (uint8)raw_speed;
    Gimbal_Send_Command(servo, GIMBAL_COMMAND_SPEED, data, 2);
}

void Gimbal_Set_Multi_Position (uint8 servo, int32 angle_x10)
{
    uint32 raw_angle = (uint32)angle_x10;
    uint8 data[4] = {0};

    data[0] = (uint8)(raw_angle >> 24);
    data[1] = (uint8)(raw_angle >> 16);
    data[2] = (uint8)(raw_angle >> 8);
    data[3] = (uint8)raw_angle;
    Gimbal_Send_Command(servo, GIMBAL_COMMAND_MULTI_POSITION, data, 4);
}

void Gimbal_Request_Feedback (uint8 servo, uint8 feedback_type)
{
    uint8 data[1] = {feedback_type};

    Gimbal_Send_Command(servo, GIMBAL_COMMAND_FEEDBACK, data, 1);
}

void Gimbal_Init (void)
{
    uart_init(GIMBAL_UART_INDEX, GIMBAL_UART_BAUD, GIMBAL_UART_TX_PIN, GIMBAL_UART_RX_PIN);
    uart_set_callback(GIMBAL_UART_INDEX, Gimbal_UART_Callback, NULL);
    uart_set_interrupt_config(GIMBAL_UART_INDEX, UART_INTERRUPT_CONFIG_RX_ENABLE);

    uart_write_byte(GIMBAL_UART_INDEX, 0x00);
    system_delay_ms(1500);

    Gimbal_Enable(GIMBAL_SERVO_1);
    system_delay_ms(2);
    Gimbal_Enable(GIMBAL_SERVO_2);
    system_delay_ms(2);

    Gimbal_Set_Mode(GIMBAL_SERVO_1, GIMBAL_MODE_MULTI_POSITION);
    system_delay_ms(2);
    Gimbal_Set_Mode(GIMBAL_SERVO_2, GIMBAL_MODE_MULTI_POSITION);
    system_delay_ms(2);

    Gimbal_Set_Speed(GIMBAL_SERVO_1, GIMBAL_TEST_SPEED_RPM);
    system_delay_ms(10);
    Gimbal_Set_Speed(GIMBAL_SERVO_2, GIMBAL_TEST_SPEED_RPM);
    system_delay_ms(10);

    printf("\r\nF32C云台测试开始：使用UART1的B4、B5，波特率115200\r\n");
}

void Gimbal_Test_Process (void)
{
    static uint8 test_step = 0;
    static uint8 baseline_ready = 0;
    static int32 servo_1_baseline_x10 = 0;
    static int32 servo_2_baseline_x10 = 0;
    const Gimbal_Test_Step_Struct *step = &gimbal_test_steps[test_step];
    int32 servo_1_target_x10 = 0;
    int32 servo_2_target_x10 = 0;

    if(!baseline_ready)
    {
        gimbal_feedback[0].data_ready = 0;
        gimbal_feedback[1].data_ready = 0;
        Gimbal_Request_Feedback(GIMBAL_SERVO_1, GIMBAL_FEEDBACK_MULTI_ANGLE);       // 获取角度，作为基线
        system_delay_ms(10);                                                        // 防止两个数据返回时间太接近
        Gimbal_Request_Feedback(GIMBAL_SERVO_2, GIMBAL_FEEDBACK_MULTI_ANGLE);
        system_delay_ms(30);                                                        // 等待两个数据完全返回完毕

        if(gimbal_feedback[0].data_ready && gimbal_feedback[1].data_ready)
        {
            servo_1_baseline_x10 = gimbal_feedback[0].multi_angle_x10;
            servo_2_baseline_x10 = gimbal_feedback[1].multi_angle_x10;
            baseline_ready = 1;
            printf("基线: 下方舵机=%ld，上方舵机=%ld（单位为0.1°）\r\n", (long)servo_1_baseline_x10, (long)servo_2_baseline_x10);
        }
        else
        {
            printf("等待初始化完毕\r\n");
            system_delay_ms(500);
        }
        return;
    }

    servo_1_target_x10 = servo_1_baseline_x10 + step->servo_1_angle_x10;
    servo_2_target_x10 = servo_2_baseline_x10 + step->servo_2_angle_x10;

    Gimbal_Set_Multi_Position(GIMBAL_SERVO_1, servo_1_target_x10);
    system_delay_ms(10);
    Gimbal_Set_Multi_Position(GIMBAL_SERVO_2, servo_2_target_x10);

    printf("步骤%u：偏移量: 下面舵机=%ld，上面舵机=%ld（单位为0.1°）\r\n", test_step, (long)step->servo_1_angle_x10, (long)step->servo_2_angle_x10);

    system_delay_ms(GIMBAL_TEST_STEP_DELAY_MS);

    gimbal_feedback[0].data_ready = 0;
    gimbal_feedback[1].data_ready = 0;
    Gimbal_Request_Feedback(GIMBAL_SERVO_1, GIMBAL_FEEDBACK_MULTI_ANGLE);
    system_delay_ms(10);
    Gimbal_Request_Feedback(GIMBAL_SERVO_2, GIMBAL_FEEDBACK_MULTI_ANGLE);
    system_delay_ms(20);

    if(gimbal_feedback[0].data_ready)
    {
        printf("返回值：下面舵机: %ld（单位为0.1°）\r\n", (long)gimbal_feedback[0].multi_angle_x10);
    }
    if(gimbal_feedback[1].data_ready)
    {
        printf("返回值：上面舵机: %ld（单位为0.1°）\r\n", (long)gimbal_feedback[1].multi_angle_x10);
    }

    test_step++;
    if((sizeof(gimbal_test_steps) / sizeof(gimbal_test_steps[0])) <= test_step)
    {
        test_step = 0;
    }
}
