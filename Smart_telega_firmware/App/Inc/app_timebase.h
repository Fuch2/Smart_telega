#ifndef APP_TIMEBASE_H
#define APP_TIMEBASE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void app_timebase_set_now_ms(uint32_t now_ms);
uint32_t app_timebase_now_ms(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_TIMEBASE_H */
