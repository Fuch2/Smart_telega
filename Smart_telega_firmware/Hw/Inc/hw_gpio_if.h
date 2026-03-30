#ifndef HW_GPIO_IF_H
#define HW_GPIO_IF_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HW_GPIO_LOGICAL_COUNT  (24u)

void hw_gpio_if_init(void);

/* Читает "электрический уровень" линии:
 * out_level_high = true  -> уровень 1
 * out_level_high = false -> уровень 0
 */
bool hw_gpio_if_read_logical(uint8_t logical_index, bool *out_level_high);

/* Host/debug hook: задать тестовый уровень логического канала.
 * На target можно оставить реализацию no-op/false по флагу.
 */
bool hw_gpio_if_debug_set_level(uint8_t logical_index, bool level_high);

#ifdef __cplusplus
}
#endif

#endif /* HW_GPIO_IF_H */
