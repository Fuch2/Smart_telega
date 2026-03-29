#include "switch_debounce.h"
#include "diag.h"

#include <string.h>

#define DEBOUNCE_MS   (50u)
#define STABLE_MS     (1000u)

typedef struct
{
    switch_snapshot_t snap;
    uint32_t t_candidate[SWITCH_CHANNEL_COUNT];
    uint32_t t_debounced[SWITCH_CHANNEL_COUNT];
    uint32_t candidate_bits;
    bool initialized;
} switch_debounce_ctx_t;

static switch_debounce_ctx_t g_db;

static bool elapsed_ge(uint32_t now, uint32_t since, uint32_t delta)
{
    return ((uint32_t)(now - since) >= delta);
}

void switch_debounce_init(void)
{
    switch_debounce_reset();
    g_db.initialized = true;
}

void switch_debounce_reset(void)
{
    memset(&g_db, 0, sizeof(g_db));
    g_db.initialized = true;
}

void switch_debounce_process(uint32_t raw_bits, uint32_t now_ms)
{
    uint8_t i;
    uint32_t bit;
    uint32_t prev_unstable;
    uint32_t new_unstable_edges;

    raw_bits &= SWITCH_CHANNEL_MASK24;

    if (!g_db.initialized) {
        switch_debounce_init();
    }

    for (i = 0u; i < SWITCH_CHANNEL_COUNT; i++) {
        bit = (1u << i);

        if (((raw_bits ^ g_db.candidate_bits) & bit) != 0u) {
            if ((raw_bits & bit) != 0u) {
                g_db.candidate_bits |= bit;
            } else {
                g_db.candidate_bits &= ~bit;
            }
            g_db.t_candidate[i] = now_ms;
        }

        if (((g_db.snap.debounced_bits ^ g_db.candidate_bits) & bit) != 0u) {
            if (elapsed_ge(now_ms, g_db.t_candidate[i], DEBOUNCE_MS)) {
                if ((g_db.candidate_bits & bit) != 0u) {
                    g_db.snap.debounced_bits |= bit;
                } else {
                    g_db.snap.debounced_bits &= ~bit;
                }
                g_db.t_debounced[i] = now_ms;
            }
        }

        if (((g_db.snap.stable_bits ^ g_db.snap.debounced_bits) & bit) != 0u) {
            if (elapsed_ge(now_ms, g_db.t_debounced[i], STABLE_MS)) {
                if ((g_db.snap.debounced_bits & bit) != 0u) {
                    g_db.snap.stable_bits |= bit;
                } else {
                    g_db.snap.stable_bits &= ~bit;
                }
            }
        }
    }

    prev_unstable = g_db.snap.unstable_bits;
    g_db.snap.unstable_bits = (g_db.snap.debounced_bits ^ g_db.snap.stable_bits) & SWITCH_CHANNEL_MASK24;

    new_unstable_edges = g_db.snap.unstable_bits & (~prev_unstable);
    if (new_unstable_edges != 0u) {
        diag_inc_switch_unstable_count();
    }
}

void switch_debounce_get_snapshot(switch_snapshot_t *out_snapshot)
{
    if (out_snapshot == NULL) {
        return;
    }
    *out_snapshot = g_db.snap;
}
