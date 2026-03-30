#ifndef APP_MAINLOOP_H
#define APP_MAINLOOP_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint32_t heartbeat_period_ms;   /* default: 1000 */
    uint32_t switch_scan_period_ms; /* default: 1..5 */
    uint32_t debounce_period_ms;    /* default: 1 */
} app_mainloop_cfg_t;

/* Инициализация всех прикладных подсистем + отправка READY event */
void app_mainloop_init(const app_mainloop_cfg_t *cfg);

/* Один шаг superloop (неблокирующий) */
void app_mainloop_step(void);

/* Бесконечный цикл (опционально использовать из main) */
void app_mainloop_run_forever(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_MAINLOOP_H */
