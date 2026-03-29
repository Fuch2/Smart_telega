#ifndef LED_MODEL_H
#define LED_MODEL_H

#include <stdint.h>
#include <stdbool.h>

#include "led_mapper.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef uint32_t led_color24_t;

#define LED_COLOR24_RGB(r,g,b) \
    ((((uint32_t)(r) & 0xFFu) << 16) | (((uint32_t)(g) & 0xFFu) << 8) | ((uint32_t)(b) & 0xFFu))
#define LED_COLOR24_R(c) ((uint8_t)(((c) >> 16) & 0xFFu))
#define LED_COLOR24_G(c) ((uint8_t)(((c) >> 8) & 0xFFu))
#define LED_COLOR24_B(c) ((uint8_t)((c) & 0xFFu))

typedef struct
{
    led_color24_t slot[LED_LOGICAL_SLOTS];
} led_logical_state_t;

void led_model_init(void);
void led_model_reset(void);

bool led_model_set_slot(uint8_t logical_slot, led_color24_t color);
bool led_model_set_slots_bulk(uint8_t start_slot, const led_color24_t *colors, uint8_t count);
bool led_model_set_full_frame(const led_color24_t *colors24);
void led_model_clear_staged(void);

bool led_model_build_physical_from_staged(uint8_t *out_grb_buf, uint16_t buf_size);

/* Now uses ws2812_if internally (no callback argument). */
bool led_model_apply(void);

void led_model_get_staged(led_logical_state_t *out_state);
void led_model_get_applied(led_logical_state_t *out_state);

bool led_model_get_slot_staged(uint8_t logical_slot, led_color24_t *out_color);
bool led_model_get_slot_applied(uint8_t logical_slot, led_color24_t *out_color);

#ifdef __cplusplus
}
#endif

#endif /* LED_MODEL_H */
