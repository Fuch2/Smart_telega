#include "ws2812_if.h"
#include "ws2812_stub_backend.h"
#include "diag.h"

#include <stddef.h>

static bool g_ws_initialized = false;
static ws2812_error_t g_last_error = WS2812_ERR_NOT_INITIALIZED;

bool ws2812_init(void)
{
    if (!ws2812_stub_backend_init()) {
        g_ws_initialized = false;
        g_last_error = WS2812_ERR_BACKEND;
        return false;
    }

    g_ws_initialized = true;
    g_last_error = WS2812_ERR_NONE;
    return true;
}

bool ws2812_show(const uint8_t *grb, uint16_t led_count)
{
    ws2812_stub_status_t st;

    if (!g_ws_initialized) {
        g_last_error = WS2812_ERR_NOT_INITIALIZED;
        return false;
    }

    if (grb == NULL) {
        g_last_error = WS2812_ERR_NULL_PTR;
        return false;
    }

    if (led_count == 0u) {
        g_last_error = WS2812_ERR_BAD_LED_COUNT;
        return false;
    }

    st = ws2812_stub_backend_show(grb, led_count);
    if (st == WS2812_STUB_OK) {
        g_last_error = WS2812_ERR_NONE;
        return true;
    }

    g_last_error = WS2812_ERR_BACKEND;
    diag_inc_ws2812_update_errors();
    diag_fault_latch(DIAG_FAULT_WS2812);
    return false;
}

ws2812_error_t ws2812_get_last_error(void)
{
    return g_last_error;
}

void ws2812_if_debug_reset_state(void)
{
    g_ws_initialized = false;
    g_last_error = WS2812_ERR_NOT_INITIALIZED;
}
