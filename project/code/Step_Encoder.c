#include "Step_Encoder.h"
#include "Step.h"

volatile int32 step_encoder_count = 0;
volatile int32 step_encoder_zero_count = 0;
volatile uint32 step_encoder_z_count = 0;
volatile float step_encoder_initial_absolute_angle = 0.0f;
volatile uint8 step_encoder_initial_absolute_valid = 0;
float step_encoder_startup_target_angle = STEP_ENCODER_DEFAULT_STARTUP_TARGET_ANGLE;

#define STEP_ENCODER_CONFIG_FLASH_SECTOR      (5U)
#define STEP_ENCODER_CONFIG_FLASH_PAGE        (1U)
#define STEP_ENCODER_CONFIG_MAGIC             (0x53544550UL)

typedef union
{
    float float_value;
    uint32 uint32_value;
} Step_Encoder_Flash_Value;

static volatile uint8 step_encoder_last_state = 0;
static Step_Encoder_Status step_encoder_status = STEP_ENCODER_PWM_ERROR;

static void Step_Encoder_Load_Startup_Target_Angle (void)
{
    uint32 data[3];
    Step_Encoder_Flash_Value angle_data;

    step_encoder_startup_target_angle = STEP_ENCODER_DEFAULT_STARTUP_TARGET_ANGLE;
    flash_read_page(STEP_ENCODER_CONFIG_FLASH_SECTOR,
                    STEP_ENCODER_CONFIG_FLASH_PAGE,
                    data,
                    3U);
    angle_data.uint32_value = data[1];
    if(data[0] == STEP_ENCODER_CONFIG_MAGIC
    && data[2] == (STEP_ENCODER_CONFIG_MAGIC ^ data[1])
    && angle_data.float_value >= 0.0f
    && angle_data.float_value < 360.0f)
    {
        step_encoder_startup_target_angle = angle_data.float_value;
    }
}

float Step_Encoder_Get_Startup_Target_Angle (void)
{
    return step_encoder_startup_target_angle;
}

uint8 Step_Encoder_Set_Startup_Target_Angle (float angle)
{
    if(angle < 0.0f || angle >= 360.0f)
    {
        return 0;
    }
    step_encoder_startup_target_angle = angle;
    return 1;
}

uint8 Step_Encoder_Save_Startup_Target_Angle (void)
{
    uint32 data[3];
    Step_Encoder_Flash_Value angle_data;

    angle_data.float_value = step_encoder_startup_target_angle;
    data[0] = STEP_ENCODER_CONFIG_MAGIC;
    data[1] = angle_data.uint32_value;
    data[2] = STEP_ENCODER_CONFIG_MAGIC ^ data[1];
    return (0U == flash_write_page(STEP_ENCODER_CONFIG_FLASH_SECTOR,
                                   STEP_ENCODER_CONFIG_FLASH_PAGE,
                                   data,
                                   3U)) ? 1U : 0U;
}

// A/B四倍频状态表
static void Step_Encoder_AB_Callback (uint32 event, void *ptr)
{
    static const int8 transition[16] =
    {
         0,  1, -1,  0,
        -1,  0,  0,  1,
         1,  0,  0, -1,
         0, -1,  1,  0
    };
    uint8 state;

    (void)event;
    (void)ptr;
    state = (uint8)((gpio_get_level(STEP_ENCODER_A_PIN) << 1)
                  | gpio_get_level(STEP_ENCODER_B_PIN));
    step_encoder_count += (int32)(transition[(step_encoder_last_state << 2) | state]
                                * STEP_ENCODER_COUNT_SIGN);
    step_encoder_last_state = state;
}

// Z每转一圈产生一次脉冲，仅用于观察和校验
static void Step_Encoder_Z_Callback (uint32 event, void *ptr)
{
    (void)event;
    (void)ptr;
    step_encoder_z_count ++;
}

static uint8 Step_Encoder_Wait_Level (uint8 level, uint32 timeout_us)
{
    while(timeout_us--)
    {
        if(gpio_get_level(STEP_ENCODER_PWM_PIN) == level)
        {
            return 1;
        }
        system_delay_us(1);
    }
    return 0;
}

// 读取一帧PWM。关闭中断可避免其他中断拉长高、低电平的软件计时。
static uint8 Step_Encoder_Read_Absolute_Angle_Once (float *angle)
{
    uint32 high_us = 0;
    uint32 low_us = 0;
    uint32 period_us;
    uint32 key;

    if(NULL == angle)
    {
        return 0;
    }

    key = __get_PRIMASK();
    __disable_irq();
    if(!Step_Encoder_Wait_Level(GPIO_LOW, 10000)
    || !Step_Encoder_Wait_Level(GPIO_HIGH, 10000))
    {
        if(0 == key) __enable_irq();
        return 0;
    }
    while(gpio_get_level(STEP_ENCODER_PWM_PIN) && high_us < 10000)
    {
        high_us ++;
        system_delay_us(1);
    }
    while(!gpio_get_level(STEP_ENCODER_PWM_PIN) && low_us < 10000)
    {
        low_us ++;
        system_delay_us(1);
    }
    if(0 == key) __enable_irq();

    period_us = high_us + low_us;
    if(0 == high_us || 0 == low_us || period_us < 100)
    {
        return 0;
    }

    // MT6816/MS42CG：PWM占空比对应单圈位置码，对应的是0~360度绝对角度（掉电不丢失，因为检测磁轴位置）
    *angle = (((float)high_us / (float)period_us) * 4115.0f - 1.0f) * 360.0f / 4115.0f;
    if(*angle < 0.0f) *angle = 0.0f;
    if(*angle >= 360.0f) *angle -= 360.0f;
    return 1;
}

// 连续读取多帧并取中值；帧间离散过大时判定PWM不稳定，不允许电机动作。
uint8 Step_Encoder_Read_Absolute_Angle (float *angle)
{
    float sample[STEP_ENCODER_PWM_SAMPLE_COUNT];
    float reference;
    float temp;
    uint8 i;
    uint8 j;

    if(NULL == angle)
    {
        return 0;
    }
    for(i = 0; i < STEP_ENCODER_PWM_SAMPLE_COUNT; i++)
    {
        if(!Step_Encoder_Read_Absolute_Angle_Once(&sample[i]))
        {
            return 0;
        }
    }

    // 先围绕第一帧展开，解决359度与0度在数值上相差很大的问题。
    reference = sample[0];
    for(i = 1; i < STEP_ENCODER_PWM_SAMPLE_COUNT; i++)
    {
        while(sample[i] - reference > 180.0f) sample[i] -= 360.0f;
        while(sample[i] - reference < -180.0f) sample[i] += 360.0f;
    }
    for(i = 0; i < STEP_ENCODER_PWM_SAMPLE_COUNT - 1; i++)
    {
        for(j = (uint8)(i + 1); j < STEP_ENCODER_PWM_SAMPLE_COUNT; j++)
        {
            if(sample[j] < sample[i])
            {
                temp = sample[i];
                sample[i] = sample[j];
                sample[j] = temp;
            }
        }
    }
    if(sample[STEP_ENCODER_PWM_SAMPLE_COUNT - 1] - sample[0]
       > STEP_ENCODER_PWM_MAX_SPREAD_DEG)
    {
        return 0;
    }

    *angle = sample[STEP_ENCODER_PWM_SAMPLE_COUNT / 2];
    while(*angle >= 360.0f) *angle -= 360.0f;
    while(*angle < 0.0f) *angle += 360.0f;
    return 1;
}

/*
函数功能：检查步进电机当前绝对角度是否超过初始化目标角度的正方向安全上限
返回值：1表示超过上限，0表示未超过或本次绝对角度读取失败
说明：直接按绝对角度数值比较，仅检查“当前角度 > 初始化角度 + 20度”的异常情况。
*/
uint8 Step_Encoder_Is_Above_Prestart_Limit (void)
{
    float current_angle;

    if(!Step_Encoder_Read_Absolute_Angle(&current_angle))
    {
        return 0;
    }

    return (current_angle > (step_encoder_startup_target_angle
                           + STEP_ENCODER_PRESTART_MAX_POSITIVE_OFFSET_DEG)) ? 1U : 0U;
}

static float Step_Encoder_Shortest_Angle_Error (float target, float actual)
{
    float error = target - actual;
    while(error > 180.0f) error -= 360.0f;
    while(error <= -180.0f) error += 360.0f;
    return error;
}

uint8 Step_Encoder_Init (void)
{
    float initial_angle;

    Step_Encoder_Load_Startup_Target_Angle();
    gpio_init(STEP_ENCODER_PWM_PIN, GPI, GPIO_LOW, GPI_PULL_DOWN);
    step_encoder_count = 0;
    step_encoder_zero_count = 0;
    step_encoder_z_count = 0;
    step_encoder_initial_absolute_angle = 0.0f;
    step_encoder_initial_absolute_valid = 0;

    gpio_init(STEP_ENCODER_A_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(STEP_ENCODER_B_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    step_encoder_last_state = (uint8)((gpio_get_level(STEP_ENCODER_A_PIN) << 1)
                                    | gpio_get_level(STEP_ENCODER_B_PIN));

    exti_init(STEP_ENCODER_A_PIN, EXTI_TRIGGER_BOTH, Step_Encoder_AB_Callback, NULL);
    exti_init(STEP_ENCODER_B_PIN, EXTI_TRIGGER_BOTH, Step_Encoder_AB_Callback, NULL);
    exti_init(STEP_ENCODER_Z_PIN, EXTI_TRIGGER_RISING, Step_Encoder_Z_Callback, NULL);

    if(!Step_Encoder_Read_Absolute_Angle(&initial_angle))
    {
        step_encoder_status = STEP_ENCODER_PWM_ERROR;
        return 0;
    }

    step_encoder_initial_absolute_angle = initial_angle;
    step_encoder_initial_absolute_valid = 1;

    // 用PWM单圈绝对角度建立A/B当前位置，后续闭环全部由更实时的A/B反馈完成
    step_encoder_count = (int32)(initial_angle * STEP_ENCODER_COUNTS_PER_REV / 360.0f + 0.5f);
    step_encoder_zero_count = step_encoder_count;
    step_encoder_status = STEP_ENCODER_OK;
    return 1;
}

float Step_Encoder_Get_Relative_Angle (void)
{
    int32 count;
    uint32 key = __get_PRIMASK();
    __disable_irq();
    count = step_encoder_count - step_encoder_zero_count;
    if(0 == key) __enable_irq();
    return (float)count * 360.0f / STEP_ENCODER_COUNTS_PER_REV;
}

/*
函数功能：将上电定位后的相对目标角度换算为机械绝对目标角度
参数说明：relative_angle为Step_To_Angle使用的相对角度
返回值：0.0~360.0度范围内的机械绝对目标角度
说明：本函数只计算目标值；实时实测值仍应使用Step_Encoder_Read_Absolute_Angle读取。
*/
float Step_Encoder_Relative_To_Absolute_Angle (float relative_angle)
{
    float absolute_angle = step_encoder_startup_target_angle + relative_angle;

    while(absolute_angle >= 360.0f)
    {
        absolute_angle -= 360.0f;
    }
    while(absolute_angle < 0.0f)
    {
        absolute_angle += 360.0f;
    }
    return absolute_angle;
}

void Step_Encoder_Set_Zero (void)
{
    uint32 key = __get_PRIMASK();
    __disable_irq();
    step_encoder_zero_count = step_encoder_count;
    if(0 == key) __enable_irq();
}

Step_Encoder_Status Step_Encoder_Move_To_Relative_Angle (float angle, uint32 frequency_hz)
{
    uint32 cycle;
    uint32 no_move_pulses = 0;
    uint8 settled = 0;
    int32 target_pulse_count;
    int32 target_count;

    if(0 == frequency_hz)
    {
        return STEP_ENCODER_TIMEOUT;
    }

    // 目标先对齐到可实际到达的微步，避免要求电机停在两个STEP脉冲之间。
    target_pulse_count = (int32)(angle * STEP_MOTOR_FULL_STEPS
                               * STEP_MOTOR_MICROSTEP / 360.0f
                               + ((angle >= 0.0f) ? 0.5f : -0.5f));
    target_count = (int32)((float)target_pulse_count
                         * STEP_ENCODER_COUNTS_PER_REV
                         / (STEP_MOTOR_FULL_STEPS * STEP_MOTOR_MICROSTEP)
                         + ((target_pulse_count >= 0) ? 0.5f : -0.5f));
    for(cycle = 0; cycle < STEP_ENCODER_MAX_CONTROL_CYCLES; cycle++)
    {
        int32 current_count;
        int32 error;
        int32 before_count;
        int32 movement;
        uint32 error_abs;
        uint32 pulses;

        current_count = (int32)(Step_Encoder_Get_Relative_Angle()
                              * STEP_ENCODER_COUNTS_PER_REV / 360.0f);
        error = target_count - current_count;
        error_abs = (error >= 0) ? (uint32)error : (uint32)(-error);
        if(error_abs <= STEP_ENCODER_TOLERANCE_COUNT)
        {
            if(++settled >= 3)
            {
                step_encoder_status = STEP_ENCODER_OK;
                step_current_angle = Step_Encoder_Get_Relative_Angle();
                return step_encoder_status;
            }
            system_delay_ms(2);
            continue;
        }
        settled = 0;
        // 取最接近的脉冲数，不能向上取整；否则小误差会多走一步并反向来回修正。
        pulses = (uint32)((error_abs * STEP_MOTOR_FULL_STEPS * STEP_MOTOR_MICROSTEP
                         + (uint32)(STEP_ENCODER_COUNTS_PER_REV / 2.0f))
                         / (uint32)STEP_ENCODER_COUNTS_PER_REV);
        if(0 == pulses) pulses = 1;
        if(pulses > STEP_ENCODER_MAX_BURST_PULSES) pulses = STEP_ENCODER_MAX_BURST_PULSES;

        Step_Set_Direction((error > 0) ? STEP_DIRECTION_FORWARD : STEP_DIRECTION_REVERSE);
        before_count = current_count;
        Step_Output_Pulses(pulses, frequency_hz);
        system_delay_ms(1);
        current_count = (int32)(Step_Encoder_Get_Relative_Angle()
                              * STEP_ENCODER_COUNTS_PER_REV / 360.0f);
        movement = current_count - before_count;
        if((error > 0 && movement < 0) || (error < 0 && movement > 0))
        {
            step_encoder_status = STEP_ENCODER_DIRECTION_ERROR;
            return step_encoder_status;
        }
        if(0 == movement)
        {
            no_move_pulses += pulses;
            if(no_move_pulses >= STEP_ENCODER_NO_MOVE_PULSE_LIMIT)
            {
                step_encoder_status = STEP_ENCODER_NO_FEEDBACK;
                return step_encoder_status;
            }
        }
        else
        {
            no_move_pulses = 0;
        }
    }

    step_encoder_status = STEP_ENCODER_TIMEOUT;
    return step_encoder_status;
}

Step_Encoder_Status Step_Encoder_Goto_Startup_Angle (void)
{
    float current_angle;
    float next_angle;
    float error;
    float movement;
    uint32 cycle;
    uint32 pulses;
    uint32 no_move_pulses = 0;
    uint8 settled = 0;
    uint8 forward_increases_angle = 1;
    uint8 direction_known = 0;
    uint8 direction;

    if(!Step_Encoder_Read_Absolute_Angle(&current_angle))
    {
        step_encoder_status = STEP_ENCODER_PWM_ERROR;
        return step_encoder_status;
    }

    // 初始化定位直接闭环使用PWM绝对角度。每发一小段脉冲都重新测量，
    // 同时自动判断DIR正转究竟使绝对角度增加还是减小。
    for(cycle = 0; cycle < STEP_ENCODER_STARTUP_MAX_CYCLES; cycle++)
    {
        error = Step_Encoder_Shortest_Angle_Error(step_encoder_startup_target_angle,
                                                  current_angle);
        if(error <= STEP_ENCODER_STARTUP_TOLERANCE_DEG
        && error >= -STEP_ENCODER_STARTUP_TOLERANCE_DEG)
        {
            if(++settled >= 3)
            {
                Step_Encoder_Set_Zero();
                step_current_angle = 0.0f;
                step_encoder_status = STEP_ENCODER_OK;
                return step_encoder_status;
            }
            if(!Step_Encoder_Read_Absolute_Angle(&current_angle))
            {
                step_encoder_status = STEP_ENCODER_PWM_ERROR;
                return step_encoder_status;
            }
            continue;
        }
        settled = 0;

        pulses = (uint32)(((error >= 0.0f ? error : -error)
                          * STEP_MOTOR_FULL_STEPS * STEP_MOTOR_MICROSTEP / 360.0f) + 0.5f);
        if(0 == pulses) pulses = 1;
        if(pulses > STEP_ENCODER_STARTUP_MAX_BURST) pulses = STEP_ENCODER_STARTUP_MAX_BURST;

        if(!direction_known)
        {
            direction = STEP_DIRECTION_FORWARD;
        }
        else if(error > 0.0f)
        {
            direction = forward_increases_angle ? STEP_DIRECTION_FORWARD : STEP_DIRECTION_REVERSE;
        }
        else
        {
            direction = forward_increases_angle ? STEP_DIRECTION_REVERSE : STEP_DIRECTION_FORWARD;
        }

        Step_Set_Direction(direction);
        Step_Output_Pulses(pulses, STEP_ENCODER_STARTUP_FREQUENCY_HZ);
        if(!Step_Encoder_Read_Absolute_Angle(&next_angle))
        {
            Step_Stop();
            step_encoder_status = STEP_ENCODER_PWM_ERROR;
            return step_encoder_status;
        }

        movement = Step_Encoder_Shortest_Angle_Error(next_angle, current_angle);
        if(movement < 0.0f) movement = -movement;
        if(movement < 0.05f)
        {
            no_move_pulses += pulses;
            if(no_move_pulses >= STEP_ENCODER_NO_MOVE_PULSE_LIMIT)
            {
                Step_Stop();
                step_encoder_status = STEP_ENCODER_NO_FEEDBACK;
                return step_encoder_status;
            }
        }
        else
        {
            float signed_movement = Step_Encoder_Shortest_Angle_Error(next_angle, current_angle);
            no_move_pulses = 0;
            if(!direction_known && STEP_DIRECTION_FORWARD == direction)
            {
                forward_increases_angle = (signed_movement > 0.0f) ? 1 : 0;
                direction_known = 1;
            }
        }
        current_angle = next_angle;
    }

    Step_Stop();
    step_encoder_status = STEP_ENCODER_TIMEOUT;
    return step_encoder_status;
}

Step_Encoder_Status Step_Encoder_Get_Status (void)
{
    return step_encoder_status;
}
