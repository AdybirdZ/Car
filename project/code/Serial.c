#include "Serial.h"
#include "Motor_PID.h"

bool enable_serial = true;
bool enable_k230_line = true;
volatile uint8 serial_rx_finish = 0;
volatile uint16 serial_rx_length = 0;
char serial_rx_buffer[SERIAL_BUFFER_SIZE] = {0};

volatile int16 k230_line_raw = 0;
volatile int16 k230_line_tens = 0;
volatile uint8 k230_line_online = 0;
float k230_line_weight = 0.0f;
float k230_line_correct_offset = 0.0f;
float k230_line_left_target = 0.0f;
float k230_line_right_target = 0.0f;

static char serial_parse_buffer[SERIAL_BUFFER_SIZE] = {0};
static uint8 serial_receiving = 0;
static uint16 serial_parse_index = 0;
static uint16 k230_line_timeout_count = 0;

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
        value = value * 10 + (str[index] - '0');
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

static void Serial_Update_K230_Line (int16 raw_value)
{
    int16 value_tens = raw_value / 10;
    int16 abs_value_tens = 0;
    float normalized = 0.0f;
    float square = 0.0f;
    float magnitude = 0.0f;

    if(value_tens > K230_LINE_INPUT_LIMIT)
    {
        value_tens = K230_LINE_INPUT_LIMIT;
    }
    else if(value_tens < -K230_LINE_INPUT_LIMIT)
    {
        value_tens = -K230_LINE_INPUT_LIMIT;
    }

    abs_value_tens = (value_tens < 0) ? -value_tens : value_tens;
    normalized = (float)abs_value_tens / (float)K230_LINE_INPUT_LIMIT;
    square = normalized * normalized;
    magnitude = K230_LINE_WEIGHT_LIMIT * square * square;

    // K230 正值表示黑线偏左，因此对应负权重；负值对应正权重。
    if(value_tens > 0)
    {
        k230_line_weight = -magnitude;
    }
    else if(value_tens < 0)
    {
        k230_line_weight = magnitude;
    }
    else
    {
        k230_line_weight = 0.0f;
    }

    k230_line_raw = raw_value;
    k230_line_tens = value_tens;
    k230_line_correct_offset = k230_line_weight * K230_LINE_CORRECT_K;
    k230_line_left_target = MOTOR_PID_TARGET_OFFSET + k230_line_correct_offset;
    k230_line_right_target = MOTOR_PID_TARGET_OFFSET - k230_line_correct_offset;

    motor_target_offset[LEFT_MOTOR] = k230_line_left_target;
    motor_target_offset[RIGHT_MOTOR] = k230_line_right_target;
    k230_line_timeout_count = 0;
    k230_line_online = 1;
}

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

void Serial_Process (void)
{
    char message[SERIAL_BUFFER_SIZE] = {0};
    int16 raw_value = 0;

    if(!enable_serial || !enable_k230_line)
    {
        return;
    }

    if(Serial_Get_Message(message, SERIAL_BUFFER_SIZE))
    {
        if(Serial_Parse_Signed_Int(message, &raw_value))
        {
            Serial_Update_K230_Line(raw_value);
            return;
        }
    }

    if(k230_line_timeout_count < K230_LINE_TIMEOUT_COUNT)
    {
        k230_line_timeout_count ++;
    }
    else
    {
        Serial_Stop_K230_Line();
    }
}

void Serial_Send_Byte (uint8 data)
{
    if(!enable_serial)
    {
        return;
    }

    uart_write_byte(SERIAL_INDEX, data);
}

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
