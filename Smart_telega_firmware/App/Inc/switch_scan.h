#ifndef SWITCH_SCAN_H
#define SWITCH_SCAN_H

#include <stdint.h>
#include <stdbool.h>

#include "switch_snapshot.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Полярность:
 * bit=1 в ACTIVE_HIGH -> активное состояние это GPIO=1
 * bit=1 в ACTIVE_LOW  -> активное состояние это GPIO=0
 * Должны покрывать все 24 канала без пересечений.
 */
#ifndef SWITCH_ACTIVE_HIGH_MASK
#define SWITCH_ACTIVE_HIGH_MASK   (0x00000000u)
#endif

#ifndef SWITCH_ACTIVE_LOW_MASK
#define SWITCH_ACTIVE_LOW_MASK    (SWITCH_CHANNEL_MASK24)
#endif

void switch_scan_init(void);

/* Возвращает mask "логически активных" каналов (младшие 24 бита). */
uint32_t switch_scan_get_raw_mask(void);

/* Helper: прочитать один логический канал (0..23) */
bool switch_scan_get_raw_channel(uint8_t logical_index, bool *out_active);

#ifdef __cplusplus
}
#endif

#endif /* SWITCH_SCAN_H */
