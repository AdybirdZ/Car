#ifndef _IR_LINE_H_
#define _IR_LINE_H_

#include "IR.h"
#include "Motor_PID.h"

#define IR_LINE_WEIGHT_NUM    (IR_CHANNEL_NUM)

extern volatile bool enable_ir_line;
extern uint8 ir_line_active_level;
extern uint8 ir_line_found;

extern float ir_line_weight[IR_LINE_WEIGHT_NUM];
extern float ir_line_k;
extern float ir_line_base_offset;
extern float ir_line_error;
extern float ir_line_correct_offset;
extern float ir_line_left_target;
extern float ir_line_right_target;

void IR_Line_Update_Target (void);

#endif
