#ifndef _INITIALIZE_H_
#define _INITIALIZE_H_

#include "zf_common_headfile.h"
#include "isr.h"
#include "Light_and_Buzzer.h"
#include "Encoder.h"
#include "Gray.h"
#include "Gray_Line.h"
#include "Motor.h"
#include "Motor_PID.h"
#include "Angle_PID.h"
#include "Straight_PID.h"
#include "Acc_PID.h"
#include "Action.h"
#include "Task.h"
#include "Position.h"
#include "WIFI.h"
#include "Serial.h"
#include "Gimbal.h"
#include "MPU6050.h"
#include "Euler.h"

#define INIT_MODULE_DELAY_MS      (100)       // 不同模块初始化之间的等待时间，单位为毫秒
#define INIT_DIAGNOSTIC_ENABLE    (1)         // 1=初始化时输出日志并蜂鸣报码，0=关闭诊断提示
#define INIT_POWER_ON_RESET_DELAY_MS (2000)   // 首次上电后等待电源稳定，再自动执行一次软件POR复位

#define INIT_MODULE_LIGHT         (1)
#define INIT_MODULE_GRAY          (2)
#define INIT_MODULE_POSITION      (3)
#define INIT_MODULE_ACC           (4)
#define INIT_MODULE_MOTOR         (5)
#define INIT_MODULE_ENCODER       (6)
#define INIT_MODULE_WIFI          (7)
#define INIT_MODULE_GIMBAL        (8)
#define INIT_MODULE_GIMBAL_POS    (9)

extern volatile uint8 init_current_module;

void Init ();

#endif
