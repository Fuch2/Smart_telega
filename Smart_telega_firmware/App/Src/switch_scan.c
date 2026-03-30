#include "switch_scan.h"
#include "hw_gpio_if.h"

#define STATIC_ASSERT(COND, NAME) typedef char static_assert_##NAME[(COND) ? 1 : -1]

STATIC_ASSERT(((SWITCH_ACTIVE_HIGH_MASK | SWITCH_ACTIVE_LOW_MASK) & SWITCH_CHANNEL_MASK24) == SWITCH_CHANNEL_MASK24,
              polarity_masks_must_cover_all_24_channels);

STATIC_ASSERT((SWITCH_ACTIVE_HIGH_MASK & SWITCH_ACTIVE_LOW_MASK) == 0u,
              polarity_masks_must_not_overlap);

void switch_scan_init(void)
{
    hw_gpio_if_init();
}

uint32_t switch_scan_get_raw_mask(void)
{
    uint32_t active_mask = 0u;
    uint8_t i;

    for (i = 0u; i < SWITCH_CHANNEL_COUNT; i++) {
        bool level_high = false;
        bool ok = hw_gpio_if_read_logical(i, &level_high);

        if (!ok) {
            continue; /* безопасно считаем как неактивный */
        }

        if ((SWITCH_ACTIVE_HIGH_MASK & (1u << i)) != 0u) {
            if (level_high) {
                active_mask |= (1u << i);
            }
        } else {
            /* ACTIVE_LOW */
            if (!level_high) {
                active_mask |= (1u << i);
            }
        }
    }

    return (active_mask & SWITCH_CHANNEL_MASK24);
}

bool switch_scan_get_raw_channel(uint8_t logical_index, bool *out_active)
{
    bool level_high = false;
    bool ok;

    if ((out_active == 0) || (logical_index >= SWITCH_CHANNEL_COUNT)) {
        return false;
    }

    ok = hw_gpio_if_read_logical(logical_index, &level_high);
    if (!ok) {
        return false;
    }

    if ((SWITCH_ACTIVE_HIGH_MASK & (1u << logical_index)) != 0u) {
        *out_active = level_high;
    } else {
        *out_active = !level_high;
    }

    return true;
}
