#include "Gray_Line.h"

volatile bool enable_gray_line = true;
uint8 gray_line_black_level = 0;
uint8 gray_line_found = 0;

float gray_line_weight[GRAY_LINE_WEIGHT_NUM] = {-5.0f, -3.8f, -2.4f, 0.2f, 0.2f, 2.4f, 3.8f, 5.0f};     // 各个通道的权重
float gray_line_k = 65.0f;
float gray_line_base_offset = 470.0f;       // 这是PID目标速度
float gray_line_error = 0.0f;
float gray_line_correct_offset = 0.0f;
float gray_line_left_target = 470.0f;
float gray_line_right_target = 470.0f;
static int8 gray_line_last_direction = 0;
static uint16 gray_line_lost_count = 0;

/*
函数功能：返回value的绝对值
参数：
value：任意浮点数
*/
/*
函数功能：灰度巡线目标更新：读传感器→找黑线→计算差速→写入电机目标
参数：无
*/
void Gray_Line_Update_Target ()
{
    uint8 i = 0;
    uint8 black_count = 0;
    float weight_sum = 0.0f;

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
            weight_sum += gray_line_weight[i];
            black_count ++;
        }
    }

    if(black_count > 0U)
    {
        gray_line_error = weight_sum / (float)black_count;
        gray_line_found = 1;
    }

    if(!gray_line_found)
    {
        float search_offset;
        if(gray_line_last_direction == 0) return;
        if(gray_line_lost_count < 0xFFFFU) gray_line_lost_count ++;
        if(gray_line_lost_count <= GRAY_LINE_LOST_HOLD_COUNT)
        {
            /* 丢线初期保持上一帧转向修正，仅同步当前基础速度。 */
            gray_line_left_target = gray_line_base_offset + gray_line_correct_offset;
            gray_line_right_target = gray_line_base_offset - gray_line_correct_offset;
            motor_target_offset[LEFT_MOTOR] = gray_line_left_target;
            motor_target_offset[RIGHT_MOTOR] = gray_line_right_target;
            Motor_PID_New_Set_Targets(gray_line_left_target, gray_line_right_target);
            return;
        }
        search_offset = GRAY_LINE_LOST_SEARCH_BASE_OFFSET;
        search_offset += (float)(gray_line_lost_count - GRAY_LINE_LOST_HOLD_COUNT)
                       * GRAY_LINE_LOST_SEARCH_STEP_OFFSET;
        if(search_offset > GRAY_LINE_LOST_SEARCH_MAX_OFFSET)
        {
            search_offset = GRAY_LINE_LOST_SEARCH_MAX_OFFSET;
        }
        gray_line_correct_offset = (float)gray_line_last_direction * search_offset;
        gray_line_left_target = gray_line_base_offset + gray_line_correct_offset;
        gray_line_right_target = gray_line_base_offset - gray_line_correct_offset;
        motor_target_offset[LEFT_MOTOR] = gray_line_left_target;
        motor_target_offset[RIGHT_MOTOR] = gray_line_right_target;
        Motor_PID_New_Set_Targets(gray_line_left_target, gray_line_right_target);
        return;
    }

    gray_line_lost_count = 0;
    if(gray_line_error > 0.0f) gray_line_last_direction = 1;
    else if(gray_line_error < 0.0f) gray_line_last_direction = -1;

    gray_line_correct_offset = gray_line_error * gray_line_k;
    gray_line_left_target = gray_line_base_offset + gray_line_correct_offset;
    gray_line_right_target = gray_line_base_offset - gray_line_correct_offset;

    motor_target_offset[LEFT_MOTOR] = gray_line_left_target;
    motor_target_offset[RIGHT_MOTOR] = gray_line_right_target;
    Motor_PID_New_Set_Targets(gray_line_left_target, gray_line_right_target);
}
