#ifndef _GIMBAL_H_
#define _GIMBAL_H_

#include "zf_common_headfile.h"

#define GIMBAL_TEST_MODE              (1)

#define GIMBAL_UART_INDEX             (UART_1)
#define GIMBAL_UART_TX_PIN            (UART1_TX_B4)
#define GIMBAL_UART_RX_PIN            (UART1_RX_B5)
#define GIMBAL_UART_BAUD              (115200)

#define GIMBAL_SERVO_1                (0x01)            // 下面的舵机
#define GIMBAL_SERVO_2                (0x02)            // 上面的舵机
#define GIMBAL_TEST_SPEED_RPM         (30)
#define GIMBAL_TEST_STEP_DELAY_MS     (2500)

#define GIMBAL_MODE_SPEED             (0x0000)
#define GIMBAL_MODE_MULTI_POSITION    (0x0001)
#define GIMBAL_MODE_SINGLE_POSITION   (0x0002)

//以下为可以选择的让舵机返回的数据
#define GIMBAL_FEEDBACK_SPEED         (0x00)
#define GIMBAL_FEEDBACK_MULTI_ANGLE   (0x01)
#define GIMBAL_FEEDBACK_SINGLE_ANGLE  (0x02)
#define GIMBAL_FEEDBACK_ACCELERATION  (0x03)
#define GIMBAL_FEEDBACK_VOLTAGE       (0x04)

typedef struct
{
    volatile int32 speed;
    volatile int32 multi_angle_x10;
    volatile uint16 single_angle_x10;
    volatile int32 acceleration;
    volatile uint16 voltage_x100;
    volatile uint8 data_ready;
} Gimbal_Feedback_Struct;

extern Gimbal_Feedback_Struct gimbal_feedback[2];

void Gimbal_Init (void);
void Gimbal_Enable (uint8 servo);
void Gimbal_Disable (uint8 servo);
void Gimbal_Set_Mode (uint8 servo, uint16 mode);
void Gimbal_Set_Speed (uint8 servo, int16 speed_rpm);
void Gimbal_Set_Multi_Position (uint8 servo, int32 angle_x10);
void Gimbal_Request_Feedback (uint8 servo, uint8 feedback_type);
void Gimbal_Test_Process (void);

#endif
