#ifndef _SERIAL_H_
#define _SERIAL_H_

#include "zf_common_headfile.h"

#define SERIAL_INDEX                  (UART_2)
#define SERIAL_TX_PIN                 (UART2_TX_B15)
#define SERIAL_RX_PIN                 (UART2_RX_B16)
#define SERIAL_BAUD                   (115200)
#define SERIAL_START_CHAR             ('#')
#define SERIAL_END_CHAR               ('$')
#define SERIAL_BUFFER_SIZE            (64)
#define K230_START_COMMAND            ('s')
#define K230_STOP_COMMAND             ('q')
#define K230_START_DELAY_MS           (5000)

/* K230 directly returns a signed lateral error: negative=left, positive=right. */
#define K230_LINE_NORMALIZE_SCALE     (30.0f)
#define K230_LINE_WEIGHT_K            (9.0f)
#define K230_LINE_CORRECT_K           (40.0f)
#define K230_LINE_TIMEOUT_COUNT       (100)

extern bool enable_serial;
extern bool enable_k230_line;
extern volatile uint8 serial_rx_finish;
extern volatile uint16 serial_rx_length;
extern char serial_rx_buffer[SERIAL_BUFFER_SIZE];
extern volatile int16 k230_line_raw;
extern volatile int16 k230_line_error;
extern volatile int16 k230_line_tens;
extern volatile uint8 k230_line_online;
extern float k230_line_weight;
extern float k230_line_correct_offset;
extern float k230_line_left_target;
extern float k230_line_right_target;

extern volatile uint8 right_flag;
extern volatile uint8 left_flag;
extern volatile uint8 both_flag;
extern volatile uint8 stop_flag;

extern volatile uint8 k230_number_a;
extern volatile uint8 k230_number_b;
extern volatile uint8 k230_number_a_ready;
extern volatile uint8 k230_number_b_ready;

void Serial_Init (void);
void Serial_Process (void);
void Serial_Send_Byte (uint8 data);
void Serial_Send_Message (const char *str);
uint8 Serial_Get_Message (char *buffer, uint16 buffer_size);
void Serial_Clear_Road_Flags (void);
void Serial_Request_Number_A (void);
void Serial_Request_Number_B (void);
uint8 Serial_Get_Number_A (uint8 *number);
uint8 Serial_Get_Number_B (uint8 *number);

#endif
