#include "led_mapper.h"

#include <string.h>

const uint8_t g_led_logical_to_physical[LED_LOGICAL_SLOTS] = {
    0u,  2u,  4u,  6u,  8u, 10u, 12u, 14u,
    16u, 18u, 20u, 22u, 24u, 26u, 28u, 30u,
    32u, 34u, 36u, 38u, 40u, 42u, 44u, 46u
};

bool led_mapper_logical_to_physical(uint8_t logical_slot, uint8_t *out_physical_index)
{
    uint8_t p;

    if (out_physical_index == NULL) {
        return false;
    }
    if (logical_slot >= LED_LOGICAL_SLOTS) {
        return false;
    }

    p = g_led_logical_to_physical[logical_slot];
    if (p == LED_MAPPER_INVALID_INDEX || p >= LED_PHYSICAL_COUNT) {
        return false;
    }

    *out_physical_index = p;
    return true;
}

bool led_mapper_set_physical_grb(uint8_t *physical_grb_buf,
                                 uint16_t buf_size,
                                 uint8_t physical_index,
                                 uint8_t r,
                                 uint8_t g,
                                 uint8_t b)
{
    uint32_t off;

    if (physical_grb_buf == NULL) {
        return false;
    }
    if (buf_size < LED_PHYSICAL_BUF_SIZE) {
        return false;
    }
    if (physical_index >= LED_PHYSICAL_COUNT) {
        return false;
    }

    off = (uint32_t)physical_index * LED_BYTES_PER_PIXEL;

    /* WS2812 byte order: G,R,B */
    physical_grb_buf[off + 0u] = g;
    physical_grb_buf[off + 1u] = r;
    physical_grb_buf[off + 2u] = b;

    return true;
}

void led_mapper_clear_physical(uint8_t *physical_grb_buf, uint16_t buf_size)
{
    if (physical_grb_buf == NULL) {
        return;
    }
    if (buf_size < LED_PHYSICAL_BUF_SIZE) {
        return;
    }

    (void)memset(physical_grb_buf, 0, LED_PHYSICAL_BUF_SIZE);
}
