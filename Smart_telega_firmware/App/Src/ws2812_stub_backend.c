#include "ws2812_stub_backend.h"

#include <string.h>

typedef struct
{
    bool initialized;
    bool force_fail;

    uint16_t last_led_count;
    uint16_t last_frame_size;
    uint8_t last_frame[WS2812_STUB_MAX_FRAME_BYTES];

    ws2812_stub_status_t last_status;
} ws2812_stub_ctx_t;

static ws2812_stub_ctx_t g_stub;

bool ws2812_stub_backend_init(void)
{
    (void)memset(&g_stub, 0, sizeof(g_stub));
    g_stub.initialized = true;
    g_stub.last_status = WS2812_STUB_OK;
    return true;
}

ws2812_stub_status_t ws2812_stub_backend_show(const uint8_t *grb, uint16_t led_count)
{
    uint16_t bytes;

    if (!g_stub.initialized) {
        g_stub.last_status = WS2812_STUB_ERR_NOT_INITIALIZED;
        return g_stub.last_status;
    }

    if (g_stub.force_fail) {
        g_stub.last_status = WS2812_STUB_ERR_INJECTED_FAIL;
        return g_stub.last_status;
    }

    if (grb == 0) {
        g_stub.last_status = WS2812_STUB_ERR_NULL_PTR;
        return g_stub.last_status;
    }

    if ((led_count == 0u) || (led_count > WS2812_STUB_MAX_LED_COUNT)) {
        g_stub.last_status = WS2812_STUB_ERR_BAD_LED_COUNT;
        return g_stub.last_status;
    }

    bytes = (uint16_t)(led_count * WS2812_STUB_BYTES_PER_LED);

    g_stub.last_led_count = led_count;
    g_stub.last_frame_size = bytes;
    (void)memcpy(g_stub.last_frame, grb, bytes);

    g_stub.last_status = WS2812_STUB_OK;
    return g_stub.last_status;
}

uint16_t ws2812_stub_backend_get_last_led_count(void)
{
    return g_stub.last_led_count;
}

uint16_t ws2812_stub_backend_get_last_frame_size(void)
{
    return g_stub.last_frame_size;
}

const uint8_t *ws2812_stub_backend_get_last_frame_ptr(void)
{
    return g_stub.last_frame;
}

ws2812_stub_status_t ws2812_stub_backend_get_last_status(void)
{
    return g_stub.last_status;
}

void ws2812_stub_backend_set_force_fail(bool enable)
{
    g_stub.force_fail = enable;
}
