#ifndef WS2812_IF_H
#define WS2812_IF_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    WS2812_ERR_NONE = 0,
    WS2812_ERR_NOT_INITIALIZED,
    WS2812_ERR_NULL_PTR,
    WS2812_ERR_BAD_LED_COUNT,
    WS2812_ERR_BACKEND,
} ws2812_error_t;

/* Init abstraction and selected backend (MVP: stub backend). */
bool ws2812_init(void);

/* Push GRB frame:
 * - grb points to 3*led_count bytes (GRB order).
 * - led_count is number of physical LEDs.
 */
bool ws2812_show(const uint8_t *grb, uint16_t led_count);

/* Returns last interface/backend error. */
ws2812_error_t ws2812_get_last_error(void);

void ws2812_if_debug_reset_state(void);

#ifdef __cplusplus
}
#endif

#endif /* WS2812_IF_H */
