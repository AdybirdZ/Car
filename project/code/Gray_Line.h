#ifndef _GRAY_LINE_H_
#define _GRAY_LINE_H_

#include "Gray.h"
#include "Motor_PID.h"
#include "Motor_PID_New.h"

#define GRAY_LINE_WEIGHT_NUM    (GRAY_CHANNEL_NUM)
#define GRAY_LINE_LOST_SEARCH_BASE_OFFSET (40.0f)
#define GRAY_LINE_LOST_SEARCH_STEP_OFFSET (5.0f)
#define GRAY_LINE_LOST_SEARCH_MAX_OFFSET  (180.0f)
#define GRAY_LINE_LOST_HOLD_COUNT         (20U)

extern volatile bool enable_gray_line;
extern uint8 gray_line_black_level;
extern uint8 gray_line_found;

extern float gray_line_weight[GRAY_LINE_WEIGHT_NUM];
extern float gray_line_k;
extern float gray_line_base_offset;
extern float gray_line_error;
extern float gray_line_correct_offset;
extern float gray_line_left_target;
extern float gray_line_right_target;

void Gray_Line_Update_Target ();

#endif
