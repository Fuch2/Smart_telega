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
    uint32_t last_stable_bits;
    bool has_switch_snapshot;
    protocol_stream_parser_t rx_parser;
} app_mainloop_state_t;


static app_mainloop_state_t g_ml;

static bool period_elapsed(uint32_t now, uint32_t last, uint32_t period_ms)
{
    return (uint32_t)(now - last) >= period_ms;
}

static bool send_event_frame(uint8_t event_id,
                             const uint8_t *payload,
                             uint16_t payload_len)
{
    protocol_frame_t evt;
    uint8_t tx_buf[PROTOCOL_MAX_FRAME_SIZE + 2u];
    size_t encoded_len = 0u;
    protocol_status_t st;

    if (payload_len > PROTOCOL_MAX_PAYLOAD) {
        return false;
    }

    memset(&evt, 0, sizeof(evt));
    evt.protocol_version = PROTOCOL_VERSION_V1;
    evt.frame_type = PROTOCOL_FRAME_TYPE_EVT;
    evt.seq = 0u;
    evt.command_id = event_id;
    evt.payload_length = payload_len;

    if ((payload != NULL) && (payload_len > 0u)) {
        memcpy(evt.payload, payload, payload_len);
    }

    tx_buf[0] = PROTOCOL_SOF0;
    tx_buf[1] = PROTOCOL_SOF1;
    st = protocol_frame_encode(&evt, &tx_buf[2], sizeof(tx_buf) - 2u, &encoded_len);
    if (st != PROTOCOL_STATUS_OK) {
        return false;
    }

    hw_uart_if_tx(tx_buf, (uint16_t)(encoded_len + 2u));
    diag_inc_tx_frames();
    return true;
}

static void send_ready_event_once(void)
{
    if (g_ml.ready_sent) {
        return;
    }

    if (send_event_frame(PROTOCOL_EVT_READY, NULL, 0u)) {
        g_ml.ready_sent = true;
    }
}

static void send_switch_changed_events(uint32_t changed_bits,
                                       uint32_t stable_bits)
{
    uint8_t channel;

    for (channel = 0u; channel < SWITCH_CHANNEL_COUNT; channel++) {
        const uint32_t bit = (1u << channel);
        uint8_t payload[2];

        if ((changed_bits & bit) == 0u) {
            continue;
        }

        payload[0] = channel;
        payload[1] = ((stable_bits & bit) != 0u) ? 1u : 0u;
        (void)send_event_frame(PROTOCOL_EVT_SWITCH_CHANGED,
                               payload,
                               (uint16_t)sizeof(payload));
    }
}

static void emit_switch_events_from_snapshot(void)
{
    switch_snapshot_t snap;
    uint32_t changed_bits;

    switch_debounce_get_snapshot(&snap);

    if (!g_ml.has_switch_snapshot) {
        g_ml.last_stable_bits = snap.stable_bits;
        g_ml.has_switch_snapshot = true;
        return;
    }

    changed_bits = (g_ml.last_stable_bits ^ snap.stable_bits) &
                   SWITCH_CHANNEL_MASK24;
    if (changed_bits == 0u) {
        return;
    }

    g_ml.last_stable_bits = snap.stable_bits;
    send_switch_changed_events(changed_bits, snap.stable_bits);
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
    g_ml.last_stable_bits = 0u;
    g_ml.has_switch_snapshot = false;

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
        emit_switch_events_from_snapshot();
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
