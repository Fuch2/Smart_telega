#include "app_timebase.h"

static uint32_t g_now_ms = 0u;

void app_timebase_set_now_ms(uint32_t now_ms)
{
    g_now_ms = now_ms;
}

uint32_t app_timebase_now_ms(void)
{
    return g_now_ms;
}
