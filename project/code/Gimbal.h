#ifndef _GIMBAL_H_
#define _GIMBAL_H_

#include "zf_common_headfile.h"

// 多圈位置单位：0.1°
#define GIMBAL_STARTUP_SERVO_1_ANGLE_X10    (0)
#define GIMBAL_SERVO_2_STARTUP_OFFSET_X10   (300)       // 2号舵机初始化目标为刚上电角度+？°
#define GIMBAL_SERVO_2_LIFT_ANGLE_X10        (2000)       // 上抬20.0°
#define GIMBAL_SERVO_2_RETURN_ANGLE_X10      (1000)       // 上抬后向下回10.0°，最终相对初始化目标上抬10.0°
#define GIMBAL_SERVO_2_ACTION_DELAY_MS       (1000)
#define GIMBAL_STARTUP_FEEDBACK_RETRY       (3)
#define GIMBAL_STARTUP_FEEDBACK_WAIT_MS     (50)

#define GIMBAL_UART_INDEX             (UART_1)
#define GIMBAL_UART_TX_PIN            (UART1_TX_B4)
#define GIMBAL_UART_RX_PIN            (UART1_RX_B5)
#define GIMBAL_UART_BAUD              (115200)

#define GIMBAL_SERVO_1                (0x01)            // 下面的舵机
#define GIMBAL_SERVO_2                (0x02)            // 上面的舵机
#define GIMBAL_TEST_SPEED_RPM         (30)
#define GIMBAL_TEST_STEP_DELAY_MS     (2500)

// 以下为可选的舵机模式
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
extern bool enable_gimbal;                              // false 时跳过云台 UART 和上电位置命令，可在未接云台时正常启动
extern volatile int32 gimbal_servo_2_startup_angle_x10;
extern volatile int32 gimbal_servo_2_startup_target_x10;
extern volatile uint8 gimbal_servo_2_startup_ready;

void Gimbal_Init (void);
void Gimbal_Enable (uint8 servo);
void Gimbal_Disable (uint8 servo);
void Gimbal_Set_Mode (uint8 servo, uint16 mode);
void Gimbal_Set_Speed (uint8 servo, int16 speed_rpm);
void Gimbal_Set_Multi_Position (uint8 servo, int32 angle_x10);
void Gimbal_Request_Feedback (uint8 servo, uint8 feedback_type);
uint8 Gimbal_Set_Servo_2_Startup_Position (void);
void Gimbal_Test_Process (void);

#endif
