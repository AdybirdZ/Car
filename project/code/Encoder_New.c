#include "Encoder_New.h"

volatile uint32 encoder_new_count[2] = {0, 0};
volatile float encoder_new_speed[2] = {0.0f, 0.0f};

static void Encoder_New_Left_Callback (uint32 event, void *ptr)
{
    (void)event;
    (void)ptr;
    encoder_new_count[MOTOR_NEW_LEFT] ++;
}

static void Encoder_New_Right_Callback (uint32 event, void *ptr)
{
    (void)event;
    (void)ptr;
    encoder_new_count[MOTOR_NEW_RIGHT] ++;
}

static float Encoder_New_Count_To_Speed (uint32 count, uint8 motor)
{
    float speed;
    uint8 direction;

    speed = (float)count / ENCODER_NEW_PULSES_PER_REV
          * ENCODER_NEW_PI * ENCODER_NEW_WHEEL_DIAMETER_MM
          * (1000.0f / ENCODER_NEW_SAMPLE_PERIOD_MS);

    // 当前只对 A 相计数，方向符号由正在执行的电机方向决定。
    direction = Motor_New_Get_Direction(motor);
    if(MOTOR_NEW_REVERSE == direction)
    {
        speed = -speed;
    }
    else if(MOTOR_NEW_STOP == direction)
    {
        speed = 0.0f;
    }

    return speed;
}

void Encoder_New_Init (void)
{
    encoder_new_count[MOTOR_NEW_LEFT] = 0;
    encoder_new_count[MOTOR_NEW_RIGHT] = 0;
    encoder_new_speed[MOTOR_NEW_LEFT] = 0.0f;
    encoder_new_speed[MOTOR_NEW_RIGHT] = 0.0f;

    gpio_init(ENCODER_NEW_LEFT_B_PIN, GPI, GPIO_LOW, GPI_PULL_DOWN);
    gpio_init(ENCODER_NEW_RIGHT_B_PIN, GPI, GPIO_LOW, GPI_PULL_DOWN);

    exti_init(ENCODER_NEW_LEFT_A_PIN, EXTI_TRIGGER_RISING, Encoder_New_Left_Callback, NULL);
    exti_init(ENCODER_NEW_RIGHT_A_PIN, EXTI_TRIGGER_RISING, Encoder_New_Right_Callback, NULL);
}

void Encoder_New_Update_Speed (void)
{
    uint32 left_count = encoder_new_count[MOTOR_NEW_LEFT];
    uint32 right_count = encoder_new_count[MOTOR_NEW_RIGHT];

    encoder_new_count[MOTOR_NEW_LEFT] = 0;
    encoder_new_count[MOTOR_NEW_RIGHT] = 0;

    encoder_new_speed[MOTOR_NEW_LEFT] = Encoder_New_Count_To_Speed(left_count, MOTOR_NEW_LEFT);
    encoder_new_speed[MOTOR_NEW_RIGHT] = Encoder_New_Count_To_Speed(right_count, MOTOR_NEW_RIGHT);
}

void Encoder_New_Clear (void)
{
    encoder_new_count[MOTOR_NEW_LEFT] = 0;
    encoder_new_count[MOTOR_NEW_RIGHT] = 0;
    encoder_new_speed[MOTOR_NEW_LEFT] = 0.0f;
    encoder_new_speed[MOTOR_NEW_RIGHT] = 0.0f;
}

float Encoder_New_Get_Speed (uint8 motor)
{
    if(motor > MOTOR_NEW_RIGHT)
    {
        return 0.0f;
    }

    return encoder_new_speed[motor];
}
