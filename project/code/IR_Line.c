#include "IR_Line.h"

volatile bool enable_ir_line = true;
uint8 ir_line_active_level = IR_ACTIVE_LEVEL;
uint8 ir_line_found = 0;

// IR1到IR8必须按车辆从左到右排列，负权重表示黑线在车体左侧，正权重表示在右侧
float ir_line_weight[IR_LINE_WEIGHT_NUM] = {-9.0f, -6.0f, -1.5f, 0.0f, 0.0f, 1.5f, 6.0f, 9.0f};
float ir_line_k = 40.0f;
float ir_line_base_offset = MOTOR_PID_TARGET_OFFSET;
float ir_line_error = 0.0f;
float ir_line_correct_offset = 0.0f;
float ir_line_left_target = MOTOR_PID_TARGET_OFFSET;
float ir_line_right_target = MOTOR_PID_TARGET_OFFSET;

/*
函数功能：返回value的绝对值
参数：
value：任意浮点数
*/
static float IR_Line_Abs (float value)
{
    return (value < 0.0f) ? -value : value;
}

/*
函数功能：红外巡线目标更新，依次完成传感器读取、黑线查找、差速计算和电机目标写入
参数：无
*/
void IR_Line_Update_Target (void)
{
    uint8 i = 0;
    float abs_weight = 0.0f;
    float max_abs_weight = -1.0f;

    if(!enable_ir_line || !enable_ir)
    {
        motor_target_offset[LEFT_MOTOR] = ir_line_base_offset;
        motor_target_offset[RIGHT_MOTOR] = ir_line_base_offset;
        return;
    }

    IR_Update();

    ir_line_found = 0;
    ir_line_error = 0.0f;

    for(i = 0; i < IR_LINE_WEIGHT_NUM; i ++)
    {
        if(ir_data[i] == ir_line_active_level)
        {
            abs_weight = IR_Line_Abs(ir_line_weight[i]);

            if(abs_weight > max_abs_weight)         // 多个通道同时检测到黑线时采用离车体中心最远通道的权重
            {
                max_abs_weight = abs_weight;
                ir_line_error = ir_line_weight[i];
                ir_line_found = 1;
            }
        }
    }

    if(!ir_line_found)
    {
        ir_line_error = 0.0f;
    }

    ir_line_correct_offset = ir_line_error * ir_line_k;
    ir_line_left_target = ir_line_base_offset + ir_line_correct_offset;
    ir_line_right_target = ir_line_base_offset - ir_line_correct_offset;

    motor_target_offset[LEFT_MOTOR] = ir_line_left_target;
    motor_target_offset[RIGHT_MOTOR] = ir_line_right_target;
}
