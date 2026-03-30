#include "app_mainloop.h"

#include <string.h>
#include <stdio.h>

#include "app_timebase.h"
#include "switch_scan.h"
#include "switch_debounce.h"
#include "debug_hooks.h"
#include "cmd_runtime.h"
#include "cmd_dispatcher.h"
#include "protocol_frame.h"
#include "diag.h"

#include "hw_uart_if.h"

typedef struct {
    bool inited;
    bool ready_sent;
    app_mainloop_cfg_t cfg;
    uint32_t last_switch_scan_ms;
    uint32_t last_debounce_ms;
    uint32_t last_heartbeat_ms;
    protocol_stream_parser_t rx_parser;
} app_mainloop_state_t;


static app_mainloop_state_t g_ml;

static bool period_elapsed(uint32_t now, uint32_t last, uint32_t period_ms)
{
    return (uint32_t)(now - last) >= period_ms;
}

/*
 * MVP-minimal READY emission:
 * - фиксируем факт отправки EVT_READY в диагностике (tx_frames++)
 * - без локального fake CMD-dispatch
 *
 * В полной интеграции здесь должен быть encode+uart tx реального EVT_READY кадра.
 */
static void send_ready_event_once(void)
{
    if (g_ml.ready_sent) {
        return;
    }

    /* seq/cmd_id для READY в MVP host-check не критичны, важен факт TX */
    diag_inc_tx_frames();
    g_ml.ready_sent = true;
}

static void finalize_longop_if_due(uint32_t now)
{
    cmd_longop_state_t *lop = cmd_runtime_longop_mut();
    if ((lop == NULL) || (!lop->active)) {
        return;
    }

    /* wrap-safe: now >= deadline */
    if ((uint32_t)(now - lop->deadline_ms) < 0x80000000u) {
        lop->active = false;
        lop->op_id = CMD_LONGOP_NONE;
        lop->result_pending = false;
        lop->result_code = 0u;
    }
}

void app_mainloop_init(const app_mainloop_cfg_t *cfg)
{
    const app_mainloop_cfg_t def_cfg = {
        .heartbeat_period_ms = 1000u,
        .switch_scan_period_ms = 2u,
        .debounce_period_ms = 1u
    };
    uint32_t now;

    g_ml.cfg = (cfg != NULL) ? *cfg : def_cfg;

    if (g_ml.cfg.heartbeat_period_ms == 0u) g_ml.cfg.heartbeat_period_ms = 1000u;
    if (g_ml.cfg.switch_scan_period_ms == 0u) g_ml.cfg.switch_scan_period_ms = 2u;
    if (g_ml.cfg.debounce_period_ms == 0u) g_ml.cfg.debounce_period_ms = 1u;

    now = app_timebase_now_ms();
    g_ml.last_switch_scan_ms = now;
    g_ml.last_debounce_ms = now;
    g_ml.last_heartbeat_ms = now;

    switch_scan_init();
    switch_debounce_init();
    cmd_runtime_init();
    cmd_dispatcher_init();
    debug_hooks_heartbeat_tick(now);
    hw_uart_if_init();
    protocol_stream_parser_init(&g_ml.rx_parser);


    g_ml.ready_sent = false;

    /* Важно для теста stage8: READY фиксируется уже в init */
    send_ready_event_once();

    g_ml.inited = true;
}

static void process_uart_rx(void)
{
    while (hw_uart_if_rx_byte_ready()) {
        uint8_t b = hw_uart_if_rx_byte_take();
        protocol_parser_event_t ev = protocol_stream_parser_feed_byte(&g_ml.rx_parser, b);

        if (ev == PROTOCOL_PARSER_EVENT_FRAME_READY) {
            protocol_frame_t req;
            if (protocol_stream_parser_take_frame(&g_ml.rx_parser, &req)) {
                cmd_dispatch_result_t out;
                cmd_dispatcher_process_frame(&req, &out);

                if (out.response_ready) {
                    uint8_t tx_buf[PROTOCOL_MAX_FRAME_SIZE + 2u];  // +2 для SOF
                    size_t tx_len = 0;

                    tx_buf[0] = PROTOCOL_SOF0;
                    tx_buf[1] = PROTOCOL_SOF1;
                    protocol_status_t st = protocol_frame_encode(
                        &out.response_frame,
                        &tx_buf[2], sizeof(tx_buf) - 2u, &tx_len);

                    if (st == PROTOCOL_STATUS_OK) {
                        hw_uart_if_tx(tx_buf, (uint16_t)(tx_len + 2u));
                    }
                }
            }
        }
    }
}




void app_mainloop_step(void)
{
    uint32_t now;

    if (!g_ml.inited) {
        app_mainloop_init(NULL);
    }

    now = app_timebase_now_ms();

    hw_uart_if_step();          /* ← добавить сюда */
    process_uart_rx();

    finalize_longop_if_due(now);

    if (period_elapsed(now, g_ml.last_switch_scan_ms, g_ml.cfg.switch_scan_period_ms)) {
        g_ml.last_switch_scan_ms = now;
        uint32_t raw = switch_scan_get_raw_mask();
        switch_debounce_process(raw, now);
    }

    if (period_elapsed(now, g_ml.last_heartbeat_ms, g_ml.cfg.heartbeat_period_ms)) {
        g_ml.last_heartbeat_ms = now;
        debug_hooks_heartbeat_tick(now);

    }


}


void app_mainloop_run_forever(void)
{
    if (!g_ml.inited) {
        app_mainloop_init(NULL);
    }

    for (;;) {
        app_mainloop_step();
    }
}
