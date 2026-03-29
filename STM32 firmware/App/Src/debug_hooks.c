#include "debug_hooks.h"

#if DEBUG_TRACE_ENABLE
#include <stdio.h>
#endif

typedef struct
{
    bool heartbeat_level;
    uint32_t last_toggle_ms;
    bool initialized;
} debug_hooks_ctx_t;

static debug_hooks_ctx_t g_dbg;

void debug_hooks_init(void)
{
    g_dbg.heartbeat_level = false;
    g_dbg.last_toggle_ms = 0u;
    g_dbg.initialized = true;
}

void debug_hooks_heartbeat_tick(uint32_t now_ms)
{
    if (!g_dbg.initialized) {
        debug_hooks_init();
    }

    if ((uint32_t)(now_ms - g_dbg.last_toggle_ms) >= 1000u) {
        g_dbg.heartbeat_level = !g_dbg.heartbeat_level;
        g_dbg.last_toggle_ms = now_ms;
    }
}

bool debug_hooks_get_heartbeat_level(void)
{
    return g_dbg.heartbeat_level;
}

#if DEBUG_TRACE_ENABLE
void debug_trace_event_u32(const char *tag, uint32_t v)
{
    printf("[TRACE] %s=%lu\n", (tag != 0) ? tag : "null", (unsigned long)v);
}
#endif

void debug_hooks_watchdog_kick(void)
{
    /* watchdog hook exists, intentionally no-op in MVP (OFF by default) */
}
