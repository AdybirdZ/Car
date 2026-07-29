#include "OLED.h"
#include "Button.h"

uint8 mode = OLED_MODE_MIN;
volatile uint32 oled_elapsed_tenths = 0;

static volatile uint8 oled_refresh_required = 0;
static uint8 oled_timer_tick_count = 0;

/*
函数功能：OLED计时和按键扫描的10ms定时中断回调
参数说明：event为中断事件，ptr为用户指针，本函数均不使用
说明：中断中只更新时间和按键状态，不进行耗时的OLED SPI通信。
*/
static void OLED_Timer_Callback (uint32 event, void *ptr)
{
    (void)event;
    (void)ptr;

    Button_Scan_10ms();

    oled_timer_tick_count ++;
    if(oled_timer_tick_count >= OLED_TENTH_SECOND_TICKS)
    {
        oled_timer_tick_count = 0;
        oled_elapsed_tenths ++;
        oled_refresh_required = 1;
    }
}

/*
函数功能：初始化OLED并显示初始界面
参数说明：无
说明：计时器不在这里启动，必须在整个Init函数即将结束时调用OLED_Start_Time。
*/
void OLED_Init (void)
{
    oled_init();
    oled_clear();
    oled_set_font(OLED_6X8_FONT);

    mode = OLED_MODE_MIN;
    oled_elapsed_tenths = 0;
    oled_refresh_required = 1;

    oled_show_string(0, 0, "INIT DONE");
    OLED_Show_Time();
    OLED_Show_Mode();
}

/*
函数功能：从0.0秒开始计时
参数说明：无
说明：该函数应作为Init函数的最后一个操作，确保初始化耗时不计入显示时间。
*/
void OLED_Start_Time (void)
{
    oled_elapsed_tenths = 0;
    oled_timer_tick_count = 0;
    oled_refresh_required = 1;
    pit_ms_init(OLED_TIMER_PIT, OLED_TIMER_PERIOD_MS, OLED_Timer_Callback, NULL);
}

/*
函数功能：显示Init结束后的运行时间，精度为0.1秒
参数说明：无
*/
void OLED_Show_Time (void)
{
    uint32 elapsed_tenths = oled_elapsed_tenths;

    oled_show_string(0, 2, "Time:");
    oled_show_uint(36, 2, elapsed_tenths / 10, 6);
    oled_show_string(72, 2, ".");
    oled_show_uint(78, 2, elapsed_tenths % 10, 1);
    oled_show_string(84, 2, " s ");
}

/*
函数功能：显示当前mode值
参数说明：无
*/
void OLED_Show_Mode (void)
{
    oled_show_string(0, 4, "Mode:");
    oled_show_uint(36, 4, mode, 1);
    oled_show_string(42, 4, " ");
}

/*
函数功能：处理按键事件并刷新OLED显示
参数说明：无
说明：应在主循环中反复调用，OLED的SPI写操作不会放在定时中断中执行。
*/
void OLED_Process (void)
{
    uint8 mode_changed = 0;

    while(Button_Get_Press_Event())
    {
        mode ++;
        if(mode > OLED_MODE_MAX)
        {
            mode = OLED_MODE_MIN;
        }
        mode_changed = 1;
    }

    if(oled_refresh_required)
    {
        oled_refresh_required = 0;
        OLED_Show_Time();
    }

    if(mode_changed)
    {
        OLED_Show_Mode();
    }
}
