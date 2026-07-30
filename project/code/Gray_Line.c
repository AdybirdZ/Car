#include "Gray_Line.h"

volatile bool enable_gray_line = true;
uint8 gray_line_black_level = 0;
uint8 gray_line_found = 0;

float gray_line_weight[GRAY_LINE_WEIGHT_NUM] = {-4.0f, -3.0f, -1.8f, 0.2f, 0.2f, 1.8f, 3.0f, 4.0f};     // 各个通道的权重
float gray_line_k = 65.0f;
float gray_line_base_offset = 470.0f;       // 这是PID目标速度
float gray_line_error = 0.0f;
float gray_line_correct_offset = 0.0f;
float gray_line_left_target = 470.0f;
float gray_line_right_target = 470.0f;

/*
函数功能：返回value的绝对值
参数：
value：任意浮点数
*/
static float Gray_Line_Abs (float value)
{
    return (value < 0.0f) ? -value : value;
}

/*
函数功能：灰度巡线目标更新：读传感器→找黑线→计算差速→写入电机目标
参数：无
*/
void Gray_Line_Update_Target ()
{
    uint8 i = 0;
    float abs_weight = 0.0f;
    float max_abs_weight = -1.0f;

    if(!enable_gray_line)
    {
        motor_target_offset[LEFT_MOTOR] = gray_line_base_offset;
        motor_target_offset[RIGHT_MOTOR] = gray_line_base_offset;
        Motor_PID_New_Set_Targets(gray_line_base_offset, gray_line_base_offset);
        return;
    }

    Gray_Update();

    gray_line_found = 0;
    gray_line_error = 0.0f;

    for(i = 0; i < GRAY_LINE_WEIGHT_NUM; i ++)
    {
        if(gray_data[i] == gray_line_black_level)
        {
            abs_weight = Gray_Line_Abs(gray_line_weight[i]);

            if(abs_weight > max_abs_weight)
            {
                max_abs_weight = abs_weight;
                gray_line_error = gray_line_weight[i];
                gray_line_found = 1;
            }
        }
    }

    if(!gray_line_found)
    {
        // 8 路全白时保留上一次有效巡线计算的左右轮目标速度。
        return;
    }

    gray_line_correct_offset = gray_line_error * gray_line_k;
    gray_line_left_target = gray_line_base_offset + gray_line_correct_offset;
    gray_line_right_target = gray_line_base_offset - gray_line_correct_offset;

    motor_target_offset[LEFT_MOTOR] = gray_line_left_target;
    motor_target_offset[RIGHT_MOTOR] = gray_line_right_target;
    Motor_PID_New_Set_Targets(gray_line_left_target, gray_line_right_target);
}
