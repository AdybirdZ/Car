#include "Serial.h"
#include "Motor_PID.h"
#include <math.h>

bool enable_serial = true;
bool enable_k230_line = true;

static char serial_parse_buffer[SERIAL_BUFFER_SIZE] = {0};  // 注意static关键字，这是解析临时存储数组，一帧可能不完整，外部不能直接访问
static uint8 serial_receiving = 0;
static uint16 serial_parse_index = 0;

volatile uint8 serial_rx_finish = 0;
volatile uint16 serial_rx_length = 0;
char serial_rx_buffer[SERIAL_BUFFER_SIZE] = {0};        // 接收到的完整的一帧数据存在这里，且去除了帧头帧尾

volatile int16 k230_line_raw = 0;
volatile int16 k230_line_error = 0;
volatile int16 k230_line_tens = 0;
volatile uint8 k230_line_online = 0;
float k230_line_weight = 0.0f;
float k230_line_correct_offset = 0.0f;
float k230_line_left_target = 0.0f;
float k230_line_right_target = 0.0f;
static uint16 k230_line_timeout_count = 0;

volatile uint8 right_flag = 0;
volatile uint8 left_flag = 0;
volatile uint8 both_flag = 0;
volatile uint8 stop_flag = 0;

volatile uint8 k230_number_a = 0;
volatile uint8 k230_number_b = 0;
volatile uint8 k230_number_a_ready = 0;
volatile uint8 k230_number_b_ready = 0;

/*
函数功能：严格比较两个以 '\0' 结束的字符串是否完全相同，包括字符大小写
参数：
*left、*right：需要比较的两个字符串
返回值：1 表示完全相同，0 表示不同
*/
static uint8 Serial_String_Equal (const char *left, const char *right)
{
    uint16 index = 0;

    if(NULL == left || NULL == right)
    {
        return 0;
    }

    while(left[index] != '\0' && right[index] != '\0')
    {
        if(left[index] != right[index])
        {
            return 0;
        }
        index ++;
    }

    return (left[index] == '\0' && right[index] == '\0') ? 1 : 0;
}

/*
函数功能：解析 K230 返回的路口字符串，并置对应的事件标志位
参数：
*message：已经去掉帧头#和帧尾$的字符串
返回值：1表示识别到RIGHT、LEFT、BOTH或STOP，0表示不是路口字符串
*/
static uint8 Serial_Parse_Road_Command (const char *message)
{
    if(Serial_String_Equal(message, "RIGHT"))
    {
        right_flag = 1;
        return 1;
    }

    if(Serial_String_Equal(message, "LEFT"))
    {
        left_flag = 1;
        return 1;
    }

    if(Serial_String_Equal(message, "BOTH"))
    {
        both_flag = 1;
        return 1;
    }

    if(Serial_String_Equal(message, "STOP"))
    {
        stop_flag = 1;
        return 1;
    }

    return 0;
}

/*
函数功能：解析K230返回的数字识别结果，数据格式为#AX$或#BX$，其中X必须是字符0到9
参数：
*message：已经去掉帧头#和帧尾$的字符串
返回值：1表示收到正确的A/B数字结果，0表示格式不正确
*/
static uint8 Serial_Parse_Number_Result (const char *message)
{
    uint8 number = 0;

    if(NULL == message || message[0] == '\0' || message[1] < '0' || message[1] > '9' || message[2] != '\0')
    {
        return 0;
    }

    number = (uint8)(message[1] - '0');

    if(message[0] == 'A')
    {
        k230_number_a = number;
        k230_number_a_ready = 1;
        return 1;
    }

    if(message[0] == 'B')
    {
        k230_number_b = number;
        k230_number_b_ready = 1;
        return 1;
    }

    return 0;
}

/*
函数功能：将字符串解析为带符号的数字
参数：
*str：待解析的字符串
*result：将解析好的带符号的数字存储的位置
*/
static uint8 Serial_Parse_Signed_Int (const char *str, int16 *result)
{
    uint16 index = 0;
    int32 value = 0;
    int32 sign = 1;

    if(NULL == str || NULL == result)
    {
        return 0;
    }

    if('-' == str[index])
    {
        sign = -1;
        index ++;
    }
    else if('+' == str[index])
    {
        index ++;
    }

    if(str[index] < '0' || str[index] > '9')
    {
        return 0;
    }

    while(str[index] >= '0' && str[index] <= '9')
    {
        value = value * 10 + (str[index] - '0');        // 秦九韶算法
        if(value > 32767)
        {
            return 0;
        }
        index ++;
    }

    if('\0' != str[index])
    {
        return 0;
    }

    *result = (int16)(value * sign);
    return 1;
}

/*
函数功能：停止使用K230巡线功能
参数：无
*/
static void Serial_Stop_K230_Line (void)
{
    k230_line_online = 0;
    k230_line_weight = 0.0f;
    k230_line_correct_offset = 0.0f;
    k230_line_left_target = 0.0f;
    k230_line_right_target = 0.0f;
    motor_target_offset[LEFT_MOTOR] = 0.0f;
    motor_target_offset[RIGHT_MOTOR] = 0.0f;
}

/*
函数功能：将K230返回的数据进行处理
参数：
raw_value：串口返回的夹在#和$之间的带符号数值
*/
static void Serial_Update_K230_Line (int16 raw_value)
{
    int16 line_error = raw_value;
    int16 value_tens = 0;
    int16 abs_value_tens = 0;
    float normalized = 0.0f;
    float magnitude = 0.0f;

    value_tens = line_error / 10;               // 除以十后的精度足够用于速度差修正

    abs_value_tens = (value_tens < 0) ? -value_tens : value_tens;
    normalized = (float)abs_value_tens / K230_LINE_NORMALIZE_SCALE;
    magnitude = K230_LINE_WEIGHT_K * powf(normalized, 1.5f);                                     // 权重拟合为1.5次函数曲线

    // K230负值需要向左修正，正值需要向右修正，推理可知，权重与偏差同号
    if(value_tens > 0)
    {
        k230_line_weight = magnitude;
    }
    else if(value_tens < 0)
    {
        k230_line_weight = -magnitude;
    }
    else
    {
        k230_line_weight = 0.0f;
    }

    k230_line_raw = raw_value;
    k230_line_error = line_error;
    k230_line_tens = value_tens;
    k230_line_correct_offset = k230_line_weight * K230_LINE_CORRECT_K;
    k230_line_left_target = MOTOR_PID_TARGET_OFFSET + k230_line_correct_offset;
    k230_line_right_target = MOTOR_PID_TARGET_OFFSET - k230_line_correct_offset;

    motor_target_offset[LEFT_MOTOR] = k230_line_left_target;
    motor_target_offset[RIGHT_MOTOR] = k230_line_right_target;
    k230_line_timeout_count = 0;
    k230_line_online = 1;
}

/*
函数功能：中断回调函数中调用，将数据从解析缓存数组转移到接收数组中，保证接收数组是完整一帧数据
参数：无
*/
static void Serial_Save_Message (void)
{
    uint16 index = 0;

    for(index = 0; index <= serial_parse_index && index < SERIAL_BUFFER_SIZE; index++)
    {
        serial_rx_buffer[index] = serial_parse_buffer[index];
    }

    serial_rx_length = serial_parse_index;
    serial_rx_finish = 1;
}

/*
函数功能：接收K230发来的一个个字节的数据，存入解析缓存数组中
参数：
data：K230发来的一个字节的数据
*/
static void Serial_Parse_Byte (uint8 data)
{
    if(SERIAL_START_CHAR == data)
    {
        serial_receiving = 1;
        serial_parse_index = 0;
        serial_parse_buffer[0] = '\0';
    }
    else if(SERIAL_END_CHAR == data && serial_receiving)
    {
        serial_parse_buffer[serial_parse_index] = '\0';
        Serial_Save_Message();
        serial_receiving = 0;
        serial_parse_index = 0;
    }
    else if(serial_receiving)
    {
        if(serial_parse_index < (SERIAL_BUFFER_SIZE - 1))
        {
            serial_parse_buffer[serial_parse_index ++] = (char)data;
        }
        else
        {
            serial_receiving = 0;
            serial_parse_index = 0;
        }
    }
}

/*
函数功能：串口中断回调函数
参数：
state：中断函数类型
*ptr：注册回调时传入的用户指针
*/
static void Serial_Callback (uint32 state, void *ptr)
{
    uint8 data = 0;

    (void)ptr;

    if(UART_INTERRUPT_STATE_RX != state)
    {
        return;
    }

    while(uart_query_byte(SERIAL_INDEX, &data))
    {
        Serial_Parse_Byte(data);
    }
}

/*
函数功能：串口初始化
参数：无
*/
void Serial_Init (void)
{
    if(!enable_serial)
    {
        return;
    }

    uart_init(SERIAL_INDEX, SERIAL_BAUD, SERIAL_TX_PIN, SERIAL_RX_PIN);
    uart_set_callback(SERIAL_INDEX, Serial_Callback, NULL);
    uart_set_interrupt_config(SERIAL_INDEX, UART_INTERRUPT_CONFIG_RX_ENABLE);
}

/*
函数功能：处理串口接收到的K230数据
参数：无
*/
void Serial_Process (void)
{
    char message[SERIAL_BUFFER_SIZE] = {0};
    int16 raw_value = 0;

    if(!enable_serial)
    {
        return;
    }

    if(Serial_Get_Message(message, SERIAL_BUFFER_SIZE))
    {
        if(Serial_Parse_Road_Command(message))
        {
            return;
        }

        if(Serial_Parse_Number_Result(message))
        {
            return;
        }

        if(enable_k230_line && Serial_Parse_Signed_Int(message, &raw_value))
        {
            Serial_Update_K230_Line(raw_value);
            return;
        }
    }

    if(!enable_k230_line)           // 只有巡线数据才需要超时停车保护
    {
        return;
    }

    if(k230_line_timeout_count < K230_LINE_TIMEOUT_COUNT)           // 超时保护
    {
        k230_line_timeout_count ++;
    }
    else
    {
        Serial_Stop_K230_Line();
    }
}

/*
函数功能：发送一个字节
参数：
data：要发送的字节
*/
void Serial_Send_Byte (uint8 data)
{
    if(!enable_serial)
    {
        return;
    }

    uart_write_byte(SERIAL_INDEX, data);
}

/*
函数功能：发送首字节为#，末字节为$，中间可自定义的字符串（没用，因为这是接收K230发来的信息格式，一开始搞错了）
参数：
*str：要发送的字符串的首地址指针
*/
void Serial_Send_Message (const char *str)
{
    if(!enable_serial || NULL == str)
    {
        return;
    }

    Serial_Send_Byte(SERIAL_START_CHAR);
    uart_write_string(SERIAL_INDEX, str);
    Serial_Send_Byte(SERIAL_END_CHAR);
}

/*
函数功能：把K230返回的数据（暂存在了缓存中）存进指定数组中
参数：
*buffer：指定数组的首地址
buffer_size：数组大小
*/
uint8 Serial_Get_Message (char *buffer, uint16 buffer_size)
{
    uint16 index = 0;
    uint16 copy_length = 0;

    if(!enable_serial || NULL == buffer || 0 == buffer_size || !serial_rx_finish)
    {
        return 0;
    }

    uart_set_interrupt_config(SERIAL_INDEX, UART_INTERRUPT_CONFIG_RX_DISABLE);

    copy_length = serial_rx_length;
    if(copy_length > (buffer_size - 1))
    {
        copy_length = buffer_size - 1;
    }

    for(index = 0; index < copy_length; index ++)
    {
        buffer[index] = serial_rx_buffer[index];
    }
    buffer[copy_length] = '\0';

    serial_rx_finish = 0;

    uart_set_interrupt_config(SERIAL_INDEX, UART_INTERRUPT_CONFIG_RX_ENABLE);

    return 1;
}

/*
函数功能：一次性清除所有路口识别事件标志
参数：无
*/
void Serial_Clear_Road_Flags (void)
{
    right_flag = 0;
    left_flag = 0;
    both_flag = 0;
    stop_flag = 0;
}

/*
函数功能：向K230发送小写字符a，请求A处数字识别，收到#AX$后由Serial_Process函数得到结果
*/
void Serial_Request_Number_A (void)
{
    k230_number_a_ready = 0;            // 先清除旧的标志位
    Serial_Send_Byte('a');
}

/*
函数功能：向K230发送小写字符b，请求B处数字识别，收到#BX$后由Serial_Process函数得到结果
*/
void Serial_Request_Number_B (void)
{
    k230_number_b_ready = 0;            // 先清除旧的标志位
    Serial_Send_Byte('b');
}

/*
函数功能：读取最近一次A处数字识别结果
参数：
*number：保存识别结果数字的变量的地址
返回值：1表示读到了新结果（指的是结果更新了，数字与上次识别到的一样也指新结果），0表示没有新结果或传进来的*number为空
*/
uint8 Serial_Get_Number_A (uint8 *number)
{
    if(number == NULL || !k230_number_a_ready)
    {
        return 0;
    }

    *number = k230_number_a;
    k230_number_a_ready = 0;
    return 1;
}

/*
函数功能：读取最近一次B处数字识别结果
参数：
*number：保存识别结果数字的变量的地址
返回值：1表示读到了新结果（指的是结果更新了，数字与上次识别到的一样也指新结果），0表示没有新结果或传进来的*number为空
*/
uint8 Serial_Get_Number_B (uint8 *number)
{
    if(number == NULL || !k230_number_b_ready)
    {
        return 0;
    }

    *number = k230_number_b;
    k230_number_b_ready = 0;
    return 1;
}
