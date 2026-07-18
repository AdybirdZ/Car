#ifndef _SERIAL_H_
#define _SERIAL_H_

#include "zf_common_headfile.h"

#define SERIAL_INDEX             (UART_2)
#define SERIAL_TX_PIN            (UART2_TX_B15)
#define SERIAL_RX_PIN            (UART2_RX_B16)
#define SERIAL_BAUD              (115200)
#define SERIAL_START_CHAR        ('#')                  // K230返回数据第一位为#
#define SERIAL_END_CHAR          ('$')                  // K230返回数据最后一位为$
#define SERIAL_BUFFER_SIZE       (64)
#define K230_START_COMMAND       ('s')                  // 启动K230发送s
#define K230_START_DELAY_MS      (5000)
#define K230_LINE_INPUT_LIMIT    (30)                   // 300以外的值被丢弃，300除以10为30
#define K230_LINE_WEIGHT_LIMIT   (9.0f)                 // 与灰度传感器巡线一样，权重最大为9
#define K230_LINE_CORRECT_K      (40.0f)                // 与灰度传感器巡线一样，k为40
#define K230_LINE_TIMEOUT_COUNT  (25)

extern bool enable_serial;
extern bool enable_k230_line;
extern volatile uint8 serial_rx_finish;
extern volatile uint16 serial_rx_length;
extern char serial_rx_buffer[SERIAL_BUFFER_SIZE];
extern volatile int16 k230_line_raw;
extern volatile int16 k230_line_tens;
extern volatile uint8 k230_line_online;
extern float k230_line_weight;
extern float k230_line_correct_offset;
extern float k230_line_left_target;
extern float k230_line_right_target;

void Serial_Init (void);
void Serial_Process (void);
void Serial_Send_Byte (uint8 data);
void Serial_Send_Message (const char *str);
uint8 Serial_Get_Message (char *buffer, uint16 buffer_size);

#endif
