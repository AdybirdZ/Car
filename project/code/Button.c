#include "Button.h"

static uint8 mode_button_stable_pressed = 0;
static uint8 mode_button_debounce_count = 0;
static uint8 start_button_stable_pressed = 0;
static uint8 start_button_debounce_count = 0;
static volatile uint8 mode_button_press_event_count = 0;
static volatile uint8 start_button_press_event_count = 0;

/*
函数功能：初始化A30启动按键和B1模式按键输入
参数说明：无
说明：板载按键释放时为高电平，按下时为低电平，因此使用内部上拉输入。
*/
void Button_Init (void)
{
    gpio_init(BUTTON_START_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(BUTTON_MODE_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    mode_button_stable_pressed = 0;
    mode_button_debounce_count = 0;
    start_button_stable_pressed = 0;
    start_button_debounce_count = 0;
    mode_button_press_event_count = 0;
    start_button_press_event_count = 0;
}

/*
函数功能：按10ms周期扫描B1按键并完成软件消抖
参数说明：无
说明：连续两次读到新状态后才确认状态变化，每次有效按下只产生一个事件。
*/
void Button_Scan_10ms (void)
{
    uint8 mode_pressed = (GPIO_LOW == gpio_get_level(BUTTON_MODE_PIN)) ? 1 : 0;
    uint8 start_pressed = (GPIO_LOW == gpio_get_level(BUTTON_START_PIN)) ? 1 : 0;

    if(mode_pressed == mode_button_stable_pressed)
    {
        mode_button_debounce_count = 0;
    }
    else
    {
        mode_button_debounce_count ++;
        if(mode_button_debounce_count >= BUTTON_DEBOUNCE_COUNT)
        {
            mode_button_debounce_count = 0;
            mode_button_stable_pressed = mode_pressed;
            if(mode_pressed && mode_button_press_event_count < 255)
            {
                mode_button_press_event_count ++;
            }
        }
    }

    if(start_pressed == start_button_stable_pressed)
    {
        start_button_debounce_count = 0;
    }
    else
    {
        start_button_debounce_count ++;
        if(start_button_debounce_count >= BUTTON_DEBOUNCE_COUNT)
        {
            start_button_debounce_count = 0;
            start_button_stable_pressed = start_pressed;
            if(start_pressed && start_button_press_event_count < 255)
            {
                start_button_press_event_count ++;
            }
        }
    }
}

/*
函数功能：读取并清除一个按键按下事件
返回值：true表示检测到一次有效按下，false表示当前没有待处理事件
*/
bool Button_Get_Press_Event (void)
{
    if(0 == mode_button_press_event_count)
    {
        return false;
    }

    mode_button_press_event_count --;
    return true;
}

/*
函数功能：读取并清除一个整车启动按键事件
返回值：true表示A30启动键产生了一次有效按下，false表示没有启动事件
*/
bool Button_Get_Start_Event (void)
{
    if(0 == start_button_press_event_count)
    {
        return false;
    }

    start_button_press_event_count --;
    return true;
}
