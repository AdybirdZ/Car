#ifndef _OLED_H_
#define _OLED_H_

#include "zf_common_headfile.h"
#include "zf_device_ips200pro.h"

#define OLED_MODE_MIN                   (1)
#define OLED_MODE_MAX                   (8)
#define OLED_TIMER_PERIOD_MS            (10)
#define OLED_TENTH_SECOND_TICKS         (100 / OLED_TIMER_PERIOD_MS)
#define OLED_TIMER_PIT                  (PIT_TIM_G12)

extern uint8 mode;
extern volatile uint32 oled_elapsed_tenths;

void OLED_Init (void);
void OLED_Start_Time (void);
void OLED_Stop_Time (void);
void OLED_Show_Time (void);
void OLED_Show_Mode (void);
void OLED_Show_Status (const char *status);
void OLED_Show_Step_Angle (float angle);
void OLED_Show_Target_Position (float position_cm);
void OLED_Show_K230_Position (float position_cm);
void OLED_Show_Ball_Control_Parameter (uint8 index, float value);
void OLED_Process (void);

#endif
