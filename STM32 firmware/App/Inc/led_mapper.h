#ifndef LED_MAPPER_H
#define LED_MAPPER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LED_LOGICAL_SLOTS        (24u)
#define LED_PHYSICAL_COUNT       (48u)   /* 0,2,4,...,46 используются */
#define LED_BYTES_PER_PIXEL      (3u)    /* GRB */
#define LED_PHYSICAL_BUF_SIZE    (LED_PHYSICAL_COUNT * LED_BYTES_PER_PIXEL)

/* Invalid index sentinel for table entries if needed */
#define LED_MAPPER_INVALID_INDEX (0xFFu)

/* Compile-time mapping table:
 * logical slot i -> physical LED index
 * Default mapping uses even indices: 0,2,4,...,46
 */
extern const uint8_t g_led_logical_to_physical[LED_LOGICAL_SLOTS];

bool led_mapper_logical_to_physical(uint8_t logical_slot, uint8_t *out_physical_index);

/* Set one physical pixel in GRB buffer */
bool led_mapper_set_physical_grb(uint8_t *physical_grb_buf,
                                 uint16_t buf_size,
                                 uint8_t physical_index,
                                 uint8_t r,
                                 uint8_t g,
                                 uint8_t b);

/* Clear full physical buffer to OFF */
void led_mapper_clear_physical(uint8_t *physical_grb_buf, uint16_t buf_size);

#ifdef __cplusplus
}
#endif

#endif /* LED_MAPPER_H */
