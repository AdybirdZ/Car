#ifndef _BUTTON_H_
#define _BUTTON_H_

#include "zf_common_headfile.h"

// A30作为整车启动键，B1保留为模式选择键；两个按键均为低电平按下。
#define BUTTON_START_PIN                 (A30)
#define BUTTON_MODE_PIN                  (B1)
#define BUTTON_SCAN_PERIOD_MS            (10)
#define BUTTON_DEBOUNCE_COUNT            (2)

void Button_Init (void);
void Button_Scan_10ms (void);
bool Button_Get_Press_Event (void);
bool Button_Get_Start_Event (void);

#endif
