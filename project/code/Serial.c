#include "Serial.h"

bool enable_serial = true;
volatile uint8 serial_rx_finish = 0;
volatile uint16 serial_rx_length = 0;
char serial_rx_buffer[SERIAL_BUFFER_SIZE] = {0};

static char serial_parse_buffer[SERIAL_BUFFER_SIZE] = {0};
static uint8 serial_receiving = 0;
static uint16 serial_parse_index = 0;

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
    // UART2 RX interrupt handles receiving.
}

void Serial_Send_Message (const char *str)
{
    if(!enable_serial || NULL == str)
    {
        return;
    }

    uart_write_byte(SERIAL_INDEX, SERIAL_START_CHAR);
    uart_write_string(SERIAL_INDEX, str);
    uart_write_byte(SERIAL_INDEX, SERIAL_END_CHAR);
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
