#ifndef SWITCH_SNAPSHOT_H
#define SWITCH_SNAPSHOT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SWITCH_CHANNEL_COUNT   (24u)
#define SWITCH_CHANNEL_MASK24  (0x00FFFFFFu)

/* Normative snapshot semantics (frozen architecture):
 * - debounced_bits
 * - stable_bits
 * - unstable_bits = debounced_bits XOR stable_bits
 */
typedef struct
{
    uint32_t debounced_bits;
    uint32_t stable_bits;
    uint32_t unstable_bits;
} switch_snapshot_t;

#ifdef __cplusplus
}
#endif

#endif /* SWITCH_SNAPSHOT_H */
