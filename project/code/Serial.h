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
#define K230_STOP_COMMAND        ('q')                  // 停止K230发送q
#define K230_START_DELAY_MS      (5000)
#define K230_DUAL_LINE_CENTER_MODE (1)                  // 1=K230 返回双线中点横坐标，0=K230 直接返回带符号横向偏差
#define K230_DUAL_LINE_TARGET_X  (160)                  // 画面宽度为 320 时的道路中心横坐标；应与 K230 输出坐标系一致
#define K230_LINE_INPUT_LIMIT    (30)                   // 300以外的值被丢弃，300除以10为30
#define K230_LINE_WEIGHT_LIMIT   (9.0f)                 // 与灰度传感器巡线一样，权重最大为9
#define K230_LINE_CORRECT_K      (40.0f)                // 与灰度传感器巡线一样，k为40
#define K230_LINE_TIMEOUT_COUNT  (100)                  // 主循环每Xms调用一次Serial_Process，（这个数乘上X）ms超时停车

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

// K230路口识别事件标志位。收到对应的大写字符串后置1，下次调用前会自动清零。
extern volatile uint8 right_flag;
extern volatile uint8 left_flag;
extern volatile uint8 both_flag;
extern volatile uint8 stop_flag;

// A、B处数字识别结果，数值范围为0到9。ready置1表示已收到新结果。
extern volatile uint8 k230_number_a;
extern volatile uint8 k230_number_b;

// K230数字识别事件标志位。收到数字后置1，下次调用前会自动清零。
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
