#ifndef DEBUG_HOOKS_H
#define DEBUG_HOOKS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef DEBUG_TRACE_ENABLE
#define DEBUG_TRACE_ENABLE 0
#endif

void debug_hooks_init(void);

/* Heartbeat hook: call periodically with monotonic ms tick.
 * Toggles internal heartbeat every 1000 ms.
 */
void debug_hooks_heartbeat_tick(uint32_t now_ms);
bool debug_hooks_get_heartbeat_level(void);

#if DEBUG_TRACE_ENABLE
void debug_trace_event_u32(const char *tag, uint32_t v);
#else
static inline void debug_trace_event_u32(const char *tag, uint32_t v)
{
    (void)tag; (void)v;
}
#endif

/* Watchdog hook present, OFF by default */
void debug_hooks_watchdog_kick(void);

#ifdef __cplusplus
}
#endif

#endif /* DEBUG_HOOKS_H */
