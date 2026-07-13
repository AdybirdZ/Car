#include "Gray.h"

bool enable_gray = true;
uint8 gray_value = 0;
uint8 gray_data[GRAY_CHANNEL_NUM] = {0};

void Gray_Init ()
{
    if(!enable_gray)
    {
        return;
    }

    gpio_init(GRAY_DAT_PIN, GPI, GPIO_LOW, GPI_FLOATING_IN);
    gpio_init(GRAY_CLK_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
}

uint8 Gray_Read ()
{
    uint8 value = 0;
    uint8 i = 0;

    gpio_low(GRAY_CLK_PIN);

    for(i = 0; i < GRAY_CHANNEL_NUM; i++)
    {
        gpio_high(GRAY_CLK_PIN);
        system_delay_us(5);
        gpio_low(GRAY_CLK_PIN);

        value |= (gpio_get_level(GRAY_DAT_PIN) ? 1 : 0) << i;
    }

    return value;
}

void Gray_Update ()
{
    uint8 i = 0;

    if(!enable_gray)
    {
        return;
    }

    gray_value = Gray_Read();

    for(i = 0; i < GRAY_CHANNEL_NUM; i++)
    {
        gray_data[i] = (gray_value >> i) & 0x01;
    }
}
