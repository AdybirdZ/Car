#ifndef _SERIAL_H_
#define _SERIAL_H_

#include "zf_common_headfile.h"

#define SERIAL_INDEX             (UART_2)
#define SERIAL_TX_PIN            (UART2_TX_B15)
#define SERIAL_RX_PIN            (UART2_RX_B16)
#define SERIAL_BAUD              (115200)
#define SERIAL_START_CHAR        ('#')
#define SERIAL_END_CHAR          ('$')
#define SERIAL_BUFFER_SIZE       (64)

extern bool enable_serial;
extern volatile uint8 serial_rx_finish;
extern volatile uint16 serial_rx_length;
extern char serial_rx_buffer[SERIAL_BUFFER_SIZE];

void Serial_Init (void);
void Serial_Process (void);
void Serial_Send_Message (const char *str);
uint8 Serial_Get_Message (char *buffer, uint16 buffer_size);

#endif
