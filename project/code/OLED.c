#include "OLED.h"
#include "Button.h"
#include "Task.h"

uint8 mode = OLED_MODE_MIN;
volatile uint32 oled_elapsed_tenths = 0;

static volatile uint8 oled_refresh_required = 0;
static uint8 oled_timer_tick_count = 0;
static bool oled_time_running = false;
static bool ips200pro_ready = false;
static uint16 ips200pro_page_id = 0;
static uint16 ips200pro_status_label_id = 0;
static uint16 ips200pro_time_label_id = 0;
static uint16 ips200pro_mode_label_id = 0;
static uint16 ips200pro_k230_label_id = 0;

static void OLED_Timer_Callback (uint32 event, void *ptr)
{
    (void)event;
    (void)ptr;

    Button_Scan_10ms();
    Task4_Tick_10ms();

    if(oled_time_running)
    {
        oled_timer_tick_count ++;
        if(oled_timer_tick_count >= OLED_TENTH_SECOND_TICKS)
        {
            oled_timer_tick_count = 0;
            oled_elapsed_tenths ++;
            oled_refresh_required = 1;
        }
    }
}

void OLED_Init (void)
{
    mode = OLED_MODE_MIN;
    oled_elapsed_tenths = 0;
    oled_timer_tick_count = 0;
    oled_time_running = false;
    oled_refresh_required = 1;
    ips200pro_ready = false;

    ips200pro_page_id = ips200pro_init("Car", IPS200PRO_TITLE_TOP, 30);
    if(0U == ips200pro_page_id)
    {
        return;
    }

    ips200pro_page_switch(ips200pro_page_id, PAGE_ANIM_OFF);

    ips200pro_status_label_id = ips200pro_label_create(10, 10, 220, 30);
    ips200pro_time_label_id = ips200pro_label_create(10, 55, 220, 35);
    ips200pro_mode_label_id = ips200pro_label_create(10, 100, 220, 35);
    ips200pro_k230_label_id = ips200pro_label_create(10, 145, 220, 35);

    if(0U == ips200pro_status_label_id
    || 0U == ips200pro_time_label_id
    || 0U == ips200pro_mode_label_id
    || 0U == ips200pro_k230_label_id)
    {
        return;
    }

    ips200pro_set_font(ips200pro_status_label_id, FONT_SIZE_20);
    ips200pro_set_font(ips200pro_time_label_id, FONT_SIZE_20);
    ips200pro_set_font(ips200pro_mode_label_id, FONT_SIZE_20);
    ips200pro_set_font(ips200pro_k230_label_id, FONT_SIZE_20);
    ips200pro_set_backlight(255);

    ips200pro_ready = true;
    ips200pro_label_printf(ips200pro_status_label_id, "INIT DONE");
    OLED_Show_Time();
    OLED_Show_Mode();
    ips200pro_label_printf(ips200pro_k230_label_id, "K230: --.-- cm");
}

void OLED_Start_Time (void)
{
    oled_elapsed_tenths = 0;
    oled_timer_tick_count = 0;
    oled_time_running = true;
    oled_refresh_required = 1;
    pit_ms_init(OLED_TIMER_PIT, OLED_TIMER_PERIOD_MS, OLED_Timer_Callback, NULL);
}

void OLED_Stop_Time (void)
{
    oled_time_running = false;
    OLED_Show_Time();
}

void OLED_Show_Time (void)
{
    uint32 elapsed_tenths = oled_elapsed_tenths;

    if(!ips200pro_ready)
    {
        return;
    }

    ips200pro_label_printf(ips200pro_time_label_id,
                           "Time: %u.%u s",
                           (unsigned int)(elapsed_tenths / 10U),
                           (unsigned int)(elapsed_tenths % 10U));
}

void OLED_Show_Mode (void)
{
    if(!ips200pro_ready)
    {
        return;
    }

    ips200pro_label_printf(ips200pro_mode_label_id, "Mode: %u", (unsigned int)mode);
}

void OLED_Show_Status (const char *status)
{
    if(!ips200pro_ready || NULL == status)
    {
        return;
    }
    ips200pro_label_printf(ips200pro_status_label_id, "%s", status);
}

void OLED_Show_Step_Angle (float angle)
{
    uint16 angle_x100;

    if(!ips200pro_ready)
    {
        return;
    }
    angle_x100 = (uint16)(angle * 100.0f + 0.5f);
    ips200pro_label_printf(ips200pro_status_label_id,
                           "Step: %u.%02u deg",
                           (unsigned int)(angle_x100 / 100U),
                           (unsigned int)(angle_x100 % 100U));
}

void OLED_Show_K230_Position (float position_cm)
{
    int32 position_x100;
    uint32 magnitude_x100;
    char sign = '+';

    if(!ips200pro_ready)
    {
        return;
    }

    if(position_cm < 0.0f)
    {
        position_x100 = (int32)(position_cm * 100.0f - 0.5f);
    }
    else
    {
        position_x100 = (int32)(position_cm * 100.0f + 0.5f);
    }
    if(position_x100 < 0)
    {
        sign = '-';
        magnitude_x100 = (uint32)(-position_x100);
    }
    else
    {
        magnitude_x100 = (uint32)position_x100;
    }

    ips200pro_label_printf(ips200pro_k230_label_id,
                           "K230: %c%u.%02u cm",
                           sign,
                           (unsigned int)(magnitude_x100 / 100U),
                           (unsigned int)(magnitude_x100 % 100U));
}

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
