#ifndef WS2812_STUB_BACKEND_H
#define WS2812_STUB_BACKEND_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define WS2812_STUB_MAX_LED_COUNT   (128u)
#define WS2812_STUB_BYTES_PER_LED   (3u)
#define WS2812_STUB_MAX_FRAME_BYTES (WS2812_STUB_MAX_LED_COUNT * WS2812_STUB_BYTES_PER_LED)

typedef enum
{
    WS2812_STUB_OK = 0,
    WS2812_STUB_ERR_NOT_INITIALIZED,
    WS2812_STUB_ERR_NULL_PTR,
    WS2812_STUB_ERR_BAD_LED_COUNT,
    WS2812_STUB_ERR_INJECTED_FAIL,
} ws2812_stub_status_t;

bool ws2812_stub_backend_init(void);
ws2812_stub_status_t ws2812_stub_backend_show(const uint8_t *grb, uint16_t led_count);

/* Debug inspection */
uint16_t ws2812_stub_backend_get_last_led_count(void);
uint16_t ws2812_stub_backend_get_last_frame_size(void); /* bytes */
const uint8_t *ws2812_stub_backend_get_last_frame_ptr(void);
ws2812_stub_status_t ws2812_stub_backend_get_last_status(void);

/* Error injection control */
void ws2812_stub_backend_set_force_fail(bool enable);

#ifdef __cplusplus
}
#endif

#endif /* WS2812_STUB_BACKEND_H */
