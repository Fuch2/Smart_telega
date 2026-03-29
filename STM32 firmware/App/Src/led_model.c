#include "led_model.h"
#include "ws2812_if.h"

#include <string.h>

typedef struct
{
    led_logical_state_t staged;
    led_logical_state_t applied;
    uint8_t physical_grb[LED_PHYSICAL_BUF_SIZE];
} led_model_ctx_t;

static led_model_ctx_t g_led;

static bool build_physical_from_state(const led_logical_state_t *state,
                                      uint8_t *out_grb_buf,
                                      uint16_t buf_size)
{
    uint8_t i;
    uint8_t p;

    if (state == NULL || out_grb_buf == NULL) {
        return false;
    }
    if (buf_size < LED_PHYSICAL_BUF_SIZE) {
        return false;
    }

    led_mapper_clear_physical(out_grb_buf, buf_size);

    for (i = 0u; i < LED_LOGICAL_SLOTS; i++) {
        led_color24_t c = state->slot[i];
        uint8_t r = LED_COLOR24_R(c);
        uint8_t g = LED_COLOR24_G(c);
        uint8_t b = LED_COLOR24_B(c);

        if (!led_mapper_logical_to_physical(i, &p)) {
            return false;
        }
        if (!led_mapper_set_physical_grb(out_grb_buf, buf_size, p, r, g, b)) {
            return false;
        }
    }

    return true;
}

void led_model_init(void)
{
    led_model_reset();
}

void led_model_reset(void)
{
    (void)memset(&g_led, 0, sizeof(g_led));
}

bool led_model_set_slot(uint8_t logical_slot, led_color24_t color)
{
    if (logical_slot >= LED_LOGICAL_SLOTS) {
        return false;
    }

    g_led.staged.slot[logical_slot] = (color & 0x00FFFFFFu);
    return true;
}

bool led_model_set_slots_bulk(uint8_t start_slot, const led_color24_t *colors, uint8_t count)
{
    uint8_t i;

    if (colors == NULL) {
        return false;
    }
    if (count == 0u) {
        return true;
    }
    if (start_slot >= LED_LOGICAL_SLOTS) {
        return false;
    }
    if ((uint16_t)start_slot + (uint16_t)count > LED_LOGICAL_SLOTS) {
        return false;
    }

    for (i = 0u; i < count; i++) {
        g_led.staged.slot[start_slot + i] = (colors[i] & 0x00FFFFFFu);
    }
    return true;
}

bool led_model_set_full_frame(const led_color24_t *colors24)
{
    uint8_t i;

    if (colors24 == NULL) {
        return false;
    }

    (void)memcpy(g_led.staged.slot, colors24, sizeof(g_led.staged.slot));

    for (i = 0u; i < LED_LOGICAL_SLOTS; i++) {
        g_led.staged.slot[i] &= 0x00FFFFFFu;
    }
    return true;
}

void led_model_clear_staged(void)
{
    (void)memset(&g_led.staged, 0, sizeof(g_led.staged));
}

bool led_model_build_physical_from_staged(uint8_t *out_grb_buf, uint16_t buf_size)
{
    return build_physical_from_state(&g_led.staged, out_grb_buf, buf_size);
}

bool led_model_apply(void)
{
    bool ok;

    ok = build_physical_from_state(&g_led.staged, g_led.physical_grb, (uint16_t)sizeof(g_led.physical_grb));
    if (!ok) {
        return false;
    }

    ok = ws2812_show(g_led.physical_grb, LED_PHYSICAL_COUNT);
    if (!ok) {
        /* last valid displayed state preserved */
        return false;
    }

    g_led.applied = g_led.staged;
    return true;
}

void led_model_get_staged(led_logical_state_t *out_state)
{
    if (out_state == NULL) {
        return;
    }
    *out_state = g_led.staged;
}

void led_model_get_applied(led_logical_state_t *out_state)
{
    if (out_state == NULL) {
        return;
    }
    *out_state = g_led.applied;
}

bool led_model_get_slot_staged(uint8_t logical_slot, led_color24_t *out_color)
{
    if (out_color == NULL) {
        return false;
    }
    if (logical_slot >= LED_LOGICAL_SLOTS) {
        return false;
    }

    *out_color = g_led.staged.slot[logical_slot];
    return true;
}

bool led_model_get_slot_applied(uint8_t logical_slot, led_color24_t *out_color)
{
    if (out_color == NULL) {
        return false;
    }
    if (logical_slot >= LED_LOGICAL_SLOTS) {
        return false;
    }

    *out_color = g_led.applied.slot[logical_slot];
    return true;
}
