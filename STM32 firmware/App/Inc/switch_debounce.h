#ifndef SWITCH_DEBOUNCE_H
#define SWITCH_DEBOUNCE_H

#include <stdint.h>
#include <stdbool.h>

#include "switch_snapshot.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SWITCH_DEBOUNCE_MS         (50u)
#define SWITCH_STABLE_CONFIRM_MS   (1000u)

void switch_debounce_init(void);
void switch_debounce_reset(void);

/* Вызывать периодически из superloop:
 * raw_mask - сырое состояние (младшие 24 бита)
 * now_ms   - монотонное время в миллисекундах (uint32_t, допускается wrap-around)
 */
void switch_debounce_process(uint32_t raw_mask, uint32_t now_ms);

void switch_debounce_get_snapshot(switch_snapshot_t *out_snapshot);

/* Optional helper для одного канала (0..23):
 * debounced/stable/unstable по логическому индексу
 */
bool switch_debounce_get_channel_state(uint8_t logical_index,
                                       bool *out_debounced,
                                       bool *out_stable,
                                       bool *out_unstable);

#ifdef __cplusplus
}
#endif

#endif /* SWITCH_DEBOUNCE_H */
