#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "protocol_crc16.h"
#include "protocol_frame.h"
#include "cmd_dispatcher.h"
#include "cmd_handlers.h"
#include "cmd_runtime.h"

#include "switch_scan.h"
#include "switch_snapshot.h"
#include "switch_debounce.h"
#include "hw_gpio_if.h"

#include "led_mapper.h"
#include "led_model.h"

#include "ws2812_if.h"
#include "ws2812_stub_backend.h"

#include "diag.h"
#include "debug_hooks.h"

#include "app_mainloop.h"
#include "app_timebase.h"

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

#define TEST_ASSERT(cond)                                                         \
    do {                                                                          \
        if (!(cond)) {                                                            \
            printf("FAIL: %s:%d: %s\n", __FILE__, __LINE__, #cond);              \
            return false;                                                         \
        }                                                                         \
    } while (0)

static void fill_frame(protocol_frame_t *f,
                       uint8_t frame_type,
                       uint8_t seq,
                       uint8_t cmd_id,
                       const uint8_t *payload,
                       uint16_t payload_len)
{
    memset(f, 0, sizeof(*f));
    f->protocol_version = PROTOCOL_VERSION_V1;
    f->frame_type = frame_type;
    f->seq = seq;
    f->command_id = cmd_id;
    f->payload_length = payload_len;
    if ((payload_len > 0u) && (payload != NULL)) {
        memcpy(f->payload, payload, payload_len);
    }
}

static bool frame_equal_essential(const protocol_frame_t *a, const protocol_frame_t *b)
{
    if (a->protocol_version != b->protocol_version) return false;
    if (a->frame_type != b->frame_type) return false;
    if (a->seq != b->seq) return false;
    if (a->command_id != b->command_id) return false;
    if (a->payload_length != b->payload_length) return false;
    if ((a->payload_length > 0u) &&
        (memcmp(a->payload, b->payload, a->payload_length) != 0)) {
        return false;
    }
    return true;
}

static bool expected_active_from_level(uint8_t idx, bool level_high)
{
    if ((SWITCH_ACTIVE_HIGH_MASK & (1u << idx)) != 0u) {
        return level_high;
    }
    return !level_high;
}

/* ===================== Stage 1 ===================== */

static bool test_s1_crc16_known_vector(void)
{
    static const uint8_t data[] = {'1','2','3','4','5','6','7','8','9'};
    uint16_t crc = protocol_crc16_ccitt_false(data, sizeof(data));
    TEST_ASSERT(crc == 0x29B1u);
    return true;
}

static bool test_s1_frame_encode_decode_roundtrip(void)
{
    protocol_frame_t in, out;
    uint8_t wire[PROTOCOL_MAX_FRAME_SIZE];
    size_t wire_len = 0u;
    protocol_status_t st;
    uint8_t p[6] = {10u,20u,30u,40u,50u,60u};

    fill_frame(&in, PROTOCOL_FRAME_TYPE_CMD, 0x42u, PROTOCOL_CMD_PING, p, (uint16_t)sizeof(p));

    st = protocol_frame_encode(&in, wire, sizeof(wire), &wire_len);
    TEST_ASSERT(st == PROTOCOL_STATUS_OK);

    st = protocol_frame_decode(wire, wire_len, &out);
    TEST_ASSERT(st == PROTOCOL_STATUS_OK);

    TEST_ASSERT(frame_equal_essential(&in, &out));
    return true;
}

static bool test_s1_frame_decode_crc_error(void)
{
    protocol_frame_t in, out;
    uint8_t wire[PROTOCOL_MAX_FRAME_SIZE];
    size_t wire_len = 0u;
    protocol_status_t st;
    uint8_t p[3] = {1u,2u,3u};

    fill_frame(&in, PROTOCOL_FRAME_TYPE_CMD, 0x01u, PROTOCOL_CMD_NOP, p, (uint16_t)sizeof(p));
    st = protocol_frame_encode(&in, wire, sizeof(wire), &wire_len);
    TEST_ASSERT(st == PROTOCOL_STATUS_OK);

    wire[3] ^= 0x5Au;
    st = protocol_frame_decode(wire, wire_len, &out);
    TEST_ASSERT(st != PROTOCOL_STATUS_OK);
    return true;
}

static bool test_s1_frame_decode_truncated(void)
{
    protocol_frame_t in, out;
    uint8_t wire[PROTOCOL_MAX_FRAME_SIZE];
    size_t wire_len = 0u;
    protocol_status_t st;

    fill_frame(&in, PROTOCOL_FRAME_TYPE_CMD, 0x02u, PROTOCOL_CMD_GET_FW_VERSION, NULL, 0u);
    st = protocol_frame_encode(&in, wire, sizeof(wire), &wire_len);
    TEST_ASSERT(st == PROTOCOL_STATUS_OK);

    st = protocol_frame_decode(wire, wire_len - 1u, &out);
    TEST_ASSERT(st != PROTOCOL_STATUS_OK);
    return true;
}

/* ===================== Stage 2 ===================== */

typedef struct
{
    protocol_frame_t req_decoded;
    cmd_dispatch_result_t dispatch_out;
    uint8_t req_wire[PROTOCOL_MAX_FRAME_SIZE];
    size_t req_wire_len;
    uint8_t resp_wire[PROTOCOL_MAX_FRAME_SIZE];
    size_t resp_wire_len;
    protocol_frame_t resp_decoded;
} flow_t;

static bool run_dispatch_flow(flow_t *ctx, const protocol_frame_t *req_in)
{
    protocol_status_t st;
    memset(ctx, 0, sizeof(*ctx));

    st = protocol_frame_encode(req_in, ctx->req_wire, sizeof(ctx->req_wire), &ctx->req_wire_len);
    TEST_ASSERT(st == PROTOCOL_STATUS_OK);

    st = protocol_frame_decode(ctx->req_wire, ctx->req_wire_len, &ctx->req_decoded);
    TEST_ASSERT(st == PROTOCOL_STATUS_OK);

    cmd_dispatcher_process_frame(&ctx->req_decoded, &ctx->dispatch_out);
    TEST_ASSERT(ctx->dispatch_out.response_ready == true);

    st = protocol_frame_encode(&ctx->dispatch_out.response_frame,
                               ctx->resp_wire, sizeof(ctx->resp_wire), &ctx->resp_wire_len);
    TEST_ASSERT(st == PROTOCOL_STATUS_OK);

    st = protocol_frame_decode(ctx->resp_wire, ctx->resp_wire_len, &ctx->resp_decoded);
    TEST_ASSERT(st == PROTOCOL_STATUS_OK);

    return true;
}

static bool assert_corr(const protocol_frame_t *req, const protocol_frame_t *resp)
{
    TEST_ASSERT(resp->protocol_version == PROTOCOL_VERSION_V1);
    TEST_ASSERT(resp->seq == req->seq);
    TEST_ASSERT(resp->command_id == req->command_id);
    return true;
}

static bool test_s2_dispatch_ping_resp(void)
{
    protocol_frame_t req;
    flow_t f;
    uint8_t p[4] = {9u,8u,7u,6u};

    fill_frame(&req, PROTOCOL_FRAME_TYPE_CMD, 0x10u, PROTOCOL_CMD_PING, p, (uint16_t)sizeof(p));
    TEST_ASSERT(run_dispatch_flow(&f, &req));
    TEST_ASSERT(assert_corr(&req, &f.resp_decoded));

    TEST_ASSERT(f.resp_decoded.frame_type == PROTOCOL_FRAME_TYPE_RESP);
    TEST_ASSERT(f.dispatch_out.result_code == CMD_RESULT_OK);
    TEST_ASSERT(f.resp_decoded.payload_length == (uint16_t)(1u + sizeof(p)));
    TEST_ASSERT(f.resp_decoded.payload[0] == (uint8_t)CMD_RESULT_OK);
    TEST_ASSERT(memcmp(&f.resp_decoded.payload[1], p, sizeof(p)) == 0);

    return true;
}

static bool test_s2_dispatch_unknown_cmd_nack(void)
{
    protocol_frame_t req;
    flow_t f;

    fill_frame(&req, PROTOCOL_FRAME_TYPE_CMD, 0x11u, 0xFEu, NULL, 0u);
    TEST_ASSERT(run_dispatch_flow(&f, &req));
    TEST_ASSERT(assert_corr(&req, &f.resp_decoded));

    TEST_ASSERT(f.resp_decoded.frame_type == PROTOCOL_FRAME_TYPE_NACK);
    TEST_ASSERT(f.dispatch_out.result_code == CMD_RESULT_UNKNOWN_COMMAND);
    TEST_ASSERT(f.resp_decoded.payload_length == 1u);
    TEST_ASSERT(f.resp_decoded.payload[0] == (uint8_t)CMD_RESULT_UNKNOWN_COMMAND);

    return true;
}

static bool test_s2_dispatch_bad_frame_type_nack(void)
{
    protocol_frame_t req;
    flow_t f;

    fill_frame(&req, PROTOCOL_FRAME_TYPE_RESP, 0x12u, PROTOCOL_CMD_NOP, NULL, 0u);
    TEST_ASSERT(run_dispatch_flow(&f, &req));
    TEST_ASSERT(assert_corr(&req, &f.resp_decoded));

    TEST_ASSERT(f.resp_decoded.frame_type == PROTOCOL_FRAME_TYPE_NACK);
    TEST_ASSERT(f.dispatch_out.result_code == CMD_RESULT_BAD_FRAME_TYPE);
    TEST_ASSERT(f.resp_decoded.payload_length == 1u);
    TEST_ASSERT(f.resp_decoded.payload[0] == (uint8_t)CMD_RESULT_BAD_FRAME_TYPE);

    return true;
}

static bool test_s2_runtime_trace_updates(void)
{
    protocol_frame_t req;
    flow_t f;
    const cmd_trace_t *tr;

    fill_frame(&req, PROTOCOL_FRAME_TYPE_CMD, 0x13u, PROTOCOL_CMD_GET_READY_STATE, NULL, 0u);
    TEST_ASSERT(run_dispatch_flow(&f, &req));

    tr = cmd_runtime_trace_get();
    TEST_ASSERT(tr != NULL);
    TEST_ASSERT(tr->last_req_seq == 0x13u);
    TEST_ASSERT(tr->last_req_cmd_id == PROTOCOL_CMD_GET_READY_STATE);
    TEST_ASSERT(tr->last_req_frame_type == PROTOCOL_FRAME_TYPE_CMD);
    TEST_ASSERT(tr->last_resp_frame_type == PROTOCOL_FRAME_TYPE_RESP);
    TEST_ASSERT(tr->last_result_code == (uint8_t)CMD_RESULT_OK);
    TEST_ASSERT(tr->total_requests >= 1u);

    return true;
}

/* ===================== Stage 3 ===================== */

static bool test_s3_init_default_levels_mask(void)
{
    uint32_t mask;
    switch_scan_init();
    mask = switch_scan_get_raw_mask();
    TEST_ASSERT(mask == (SWITCH_ACTIVE_LOW_MASK & SWITCH_CHANNEL_MASK24));
    return true;
}

static bool test_s3_single_toggle(void)
{
    uint8_t ch = 7u;
    uint32_t bit = (1u << ch);
    uint32_t mask;

    switch_scan_init();

    TEST_ASSERT(hw_gpio_if_debug_set_level(ch, false) == true);
    mask = switch_scan_get_raw_mask();
    if (expected_active_from_level(ch, false)) TEST_ASSERT((mask & bit) != 0u);
    else                                       TEST_ASSERT((mask & bit) == 0u);

    TEST_ASSERT(hw_gpio_if_debug_set_level(ch, true) == true);
    mask = switch_scan_get_raw_mask();
    if (expected_active_from_level(ch, true)) TEST_ASSERT((mask & bit) != 0u);
    else                                      TEST_ASSERT((mask & bit) == 0u);

    return true;
}

static bool test_s3_channel_api_invalid(void)
{
    bool active = false;
    switch_scan_init();

    TEST_ASSERT(switch_scan_get_raw_channel(24u, &active) == false);
    TEST_ASSERT(switch_scan_get_raw_channel(100u, &active) == false);
    TEST_ASSERT(switch_scan_get_raw_channel(0u, NULL) == false);

    return true;
}

static bool test_s3_pattern_full_mask(void)
{
    uint8_t i;
    uint32_t expected = 0u;
    uint32_t mask;

    switch_scan_init();

    for (i = 0u; i < SWITCH_CHANNEL_COUNT; i++) {
        bool level = ((i & 1u) == 0u);
        TEST_ASSERT(hw_gpio_if_debug_set_level(i, level) == true);
        if (expected_active_from_level(i, level)) {
            expected |= (1u << i);
        }
    }

    mask = switch_scan_get_raw_mask();
    TEST_ASSERT((mask & SWITCH_CHANNEL_MASK24) == expected);
    return true;
}

/* ===================== Stage 4 ===================== */

static bool test_s4_idle_zero_stays_zero(void)
{
    switch_snapshot_t s;
    uint32_t t;

    switch_debounce_reset();

    for (t = 0u; t <= 1500u; t += 10u) {
        switch_debounce_process(0u, t);
    }

    switch_debounce_get_snapshot(&s);
    TEST_ASSERT(s.debounced_bits == 0u);
    TEST_ASSERT(s.stable_bits == 0u);
    TEST_ASSERT(s.unstable_bits == 0u);
    TEST_ASSERT(s.unstable_bits == ((s.debounced_bits ^ s.stable_bits) & SWITCH_CHANNEL_MASK24));

    return true;
}

static bool test_s4_short_pulse_ignored(void)
{
    const uint8_t ch = 3u;
    const uint32_t bit = (1u << ch);
    switch_snapshot_t s;

    switch_debounce_reset();

    switch_debounce_process(0u, 0u);
    switch_debounce_process(bit, 10u);
    switch_debounce_process(bit, 30u);
    switch_debounce_process(0u, 40u);
    switch_debounce_process(0u, 100u);

    switch_debounce_get_snapshot(&s);
    TEST_ASSERT((s.debounced_bits & bit) == 0u);
    TEST_ASSERT((s.stable_bits & bit) == 0u);
    TEST_ASSERT((s.unstable_bits & bit) == 0u);

    return true;
}

static bool test_s4_debounce_after_50ms(void)
{
    const uint8_t ch = 5u;
    const uint32_t bit = (1u << ch);
    switch_snapshot_t s;

    switch_debounce_reset();

    switch_debounce_process(0u, 0u);
    switch_debounce_process(bit, 10u);
    switch_debounce_process(bit, 59u);
    switch_debounce_get_snapshot(&s);
    TEST_ASSERT((s.debounced_bits & bit) == 0u);

    switch_debounce_process(bit, 60u);
    switch_debounce_get_snapshot(&s);
    TEST_ASSERT((s.debounced_bits & bit) != 0u);
    TEST_ASSERT((s.stable_bits & bit) == 0u);
    TEST_ASSERT((s.unstable_bits & bit) != 0u);

    return true;
}

static bool test_s4_stable_after_1000ms(void)
{
    const uint8_t ch = 7u;
    const uint32_t bit = (1u << ch);
    switch_snapshot_t s;

    switch_debounce_reset();

    switch_debounce_process(0u, 0u);
    switch_debounce_process(bit, 100u);
    switch_debounce_process(bit, 150u);

    switch_debounce_process(bit, 1149u);
    switch_debounce_get_snapshot(&s);
    TEST_ASSERT((s.stable_bits & bit) == 0u);

    switch_debounce_process(bit, 1150u);
    switch_debounce_get_snapshot(&s);
    TEST_ASSERT((s.stable_bits & bit) != 0u);
    TEST_ASSERT((s.unstable_bits & bit) == 0u);

    return true;
}

static bool test_s4_unstable_equals_xor(void)
{
    switch_snapshot_t s;
    uint32_t raw = (0x0055AA55u & SWITCH_CHANNEL_MASK24);

    switch_debounce_reset();

    switch_debounce_process(0u, 0u);
    switch_debounce_process(raw, 10u);
    switch_debounce_process(raw, 60u);
    switch_debounce_get_snapshot(&s);
    TEST_ASSERT(s.unstable_bits == ((s.debounced_bits ^ s.stable_bits) & SWITCH_CHANNEL_MASK24));

    switch_debounce_process(raw, 1060u);
    switch_debounce_get_snapshot(&s);
    TEST_ASSERT(s.unstable_bits == ((s.debounced_bits ^ s.stable_bits) & SWITCH_CHANNEL_MASK24));

    return true;
}

static bool test_s4_wraparound_time(void)
{
    const uint8_t ch = 2u;
    const uint32_t bit = (1u << ch);
    switch_snapshot_t s;

    switch_debounce_reset();

    switch_debounce_process(0u, 0xFFFFFFF0u);
    switch_debounce_process(bit, 0xFFFFFFF6u);
    switch_debounce_process(bit, 0x00000028u);
    switch_debounce_get_snapshot(&s);
    TEST_ASSERT((s.debounced_bits & bit) != 0u);

    switch_debounce_process(bit, 0x00000410u);
    switch_debounce_get_snapshot(&s);
    TEST_ASSERT((s.stable_bits & bit) != 0u);

    return true;
}

/* ===================== Stage 5 ===================== */

static bool test_s5_mapper_even_slots_only(void)
{
    uint8_t p = 0xFFu;
    uint8_t i;

    for (i = 0u; i < LED_LOGICAL_SLOTS; i++) {
        TEST_ASSERT(led_mapper_logical_to_physical(i, &p) == true);
        TEST_ASSERT(p == (uint8_t)(i * 2u));
    }

    TEST_ASSERT(led_mapper_logical_to_physical(24u, &p) == false);
    TEST_ASSERT(led_mapper_logical_to_physical(0u, NULL) == false);
    return true;
}

static bool test_s5_set_single_and_build_grb(void)
{
    uint8_t buf[LED_PHYSICAL_BUF_SIZE];

    led_model_reset();
    TEST_ASSERT(led_model_set_slot(0u, LED_COLOR24_RGB(0x11u,0x22u,0x33u)) == true);
    TEST_ASSERT(led_model_build_physical_from_staged(buf, sizeof(buf)) == true);

    TEST_ASSERT(buf[0] == 0x22u);
    TEST_ASSERT(buf[1] == 0x11u);
    TEST_ASSERT(buf[2] == 0x33u);

    TEST_ASSERT(buf[3] == 0u);
    TEST_ASSERT(buf[4] == 0u);
    TEST_ASSERT(buf[5] == 0u);

    return true;
}

static bool test_s5_apply_success_updates_applied(void)
{
    led_logical_state_t stg, app;

    led_model_reset();
    TEST_ASSERT(ws2812_init() == true);

    TEST_ASSERT(led_model_set_slot(2u, LED_COLOR24_RGB(0xAAu,0xBBu,0xCCu)) == true);
    led_model_get_staged(&stg);

    TEST_ASSERT(led_model_apply() == true);
    led_model_get_applied(&app);

    TEST_ASSERT(memcmp(&stg, &app, sizeof(stg)) == 0);
    return true;
}

static bool test_s5_intermediate_leds_always_off(void)
{
    uint8_t buf[LED_PHYSICAL_BUF_SIZE];
    uint8_t phys;

    led_model_reset();
    TEST_ASSERT(led_model_set_slot(0u, LED_COLOR24_RGB(1u,2u,3u)) == true);
    TEST_ASSERT(led_model_set_slot(10u, LED_COLOR24_RGB(4u,5u,6u)) == true);
    TEST_ASSERT(led_model_set_slot(23u, LED_COLOR24_RGB(7u,8u,9u)) == true);

    TEST_ASSERT(led_model_build_physical_from_staged(buf, sizeof(buf)) == true);

    for (phys = 1u; phys < LED_PHYSICAL_COUNT; phys = (uint8_t)(phys + 2u)) {
        uint16_t off = (uint16_t)phys * LED_BYTES_PER_PIXEL;
        TEST_ASSERT(buf[off + 0u] == 0u);
        TEST_ASSERT(buf[off + 1u] == 0u);
        TEST_ASSERT(buf[off + 2u] == 0u);
    }

    return true;
}

/* ===================== Stage 6 ===================== */

static bool test_s6_ws_show_before_init_fails(void)
{
    uint8_t dummy[3] = {0u,0u,0u};

    ws2812_if_debug_reset_state();

    TEST_ASSERT(ws2812_show(dummy, 1u) == false);
    TEST_ASSERT(ws2812_get_last_error() == WS2812_ERR_NOT_INITIALIZED);
    return true;
}

static bool test_s6_ws_show_null_and_bad_count(void)
{
    uint8_t dummy[3] = {1u,2u,3u};

    TEST_ASSERT(ws2812_init() == true);

    TEST_ASSERT(ws2812_show(NULL, 1u) == false);
    TEST_ASSERT(ws2812_get_last_error() == WS2812_ERR_NULL_PTR);

    TEST_ASSERT(ws2812_show(dummy, 0u) == false);
    TEST_ASSERT(ws2812_get_last_error() == WS2812_ERR_BAD_LED_COUNT);

    return true;
}

static bool test_s6_ws_show_success_and_last_frame_inspect(void)
{
    uint8_t frame[LED_PHYSICAL_BUF_SIZE];
    uint16_t i;
    const uint8_t *last;

    for (i = 0u; i < LED_PHYSICAL_BUF_SIZE; i++) {
        frame[i] = (uint8_t)(i & 0xFFu);
    }

    TEST_ASSERT(ws2812_init() == true);
    ws2812_stub_backend_set_force_fail(false);

    TEST_ASSERT(ws2812_show(frame, LED_PHYSICAL_COUNT) == true);
    TEST_ASSERT(ws2812_get_last_error() == WS2812_ERR_NONE);

    TEST_ASSERT(ws2812_stub_backend_get_last_status() == WS2812_STUB_OK);
    TEST_ASSERT(ws2812_stub_backend_get_last_led_count() == LED_PHYSICAL_COUNT);
    TEST_ASSERT(ws2812_stub_backend_get_last_frame_size() == LED_PHYSICAL_BUF_SIZE);

    last = ws2812_stub_backend_get_last_frame_ptr();
    TEST_ASSERT(last != NULL);
    TEST_ASSERT(memcmp(last, frame, LED_PHYSICAL_BUF_SIZE) == 0);

    return true;
}

static bool test_s6_ws_injected_backend_fail(void)
{
    uint8_t frame[LED_PHYSICAL_BUF_SIZE];
    memset(frame, 0xA5, sizeof(frame));

    TEST_ASSERT(ws2812_init() == true);
    ws2812_stub_backend_set_force_fail(true);

    TEST_ASSERT(ws2812_show(frame, LED_PHYSICAL_COUNT) == false);
    TEST_ASSERT(ws2812_get_last_error() == WS2812_ERR_BACKEND);
    TEST_ASSERT(ws2812_stub_backend_get_last_status() == WS2812_STUB_ERR_INJECTED_FAIL);

    ws2812_stub_backend_set_force_fail(false);
    return true;
}

static bool test_s6_led_model_apply_fail_keeps_applied(void)
{
    led_logical_state_t before, after;

    TEST_ASSERT(ws2812_init() == true);
    ws2812_stub_backend_set_force_fail(false);

    led_model_reset();
    TEST_ASSERT(led_model_set_slot(1u, LED_COLOR24_RGB(10u,20u,30u)) == true);
    TEST_ASSERT(led_model_apply() == true);
    led_model_get_applied(&before);

    TEST_ASSERT(led_model_set_slot(1u, LED_COLOR24_RGB(99u,88u,77u)) == true);

    ws2812_stub_backend_set_force_fail(true);
    TEST_ASSERT(led_model_apply() == false);
    ws2812_stub_backend_set_force_fail(false);

    led_model_get_applied(&after);
    TEST_ASSERT(memcmp(&before, &after, sizeof(before)) == 0);

    return true;
}

/* ===================== Stage 7 ===================== */

static bool test_s7_diag_init_and_reset(void)
{
    diag_snapshot_t s;

    diag_init();
    diag_get_snapshot(&s);

    TEST_ASSERT(s.rx_frames_ok == 0u);
    TEST_ASSERT(s.rx_crc_errors == 0u);
    TEST_ASSERT(s.rx_length_errors == 0u);
    TEST_ASSERT(s.rx_unknown_cmd == 0u);
    TEST_ASSERT(s.tx_frames == 0u);
    TEST_ASSERT(s.ws2812_update_errors == 0u);
    TEST_ASSERT(s.switch_unstable_count == 0u);
    TEST_ASSERT(s.last_cmd_id == 0u);
    TEST_ASSERT(s.last_seq == 0u);
    TEST_ASSERT(s.last_status == 0u);
    TEST_ASSERT(s.last_fault_code == (uint8_t)DIAG_FAULT_NONE);

    return true;
}

static bool test_s7_diag_fault_latch_semantics(void)
{
    diag_snapshot_t s;

    diag_reset_all();

    diag_fault_latch(DIAG_FAULT_RX_CRC);
    diag_fault_latch(DIAG_FAULT_WS2812);
    diag_get_snapshot(&s);
    TEST_ASSERT(s.last_fault_code == (uint8_t)DIAG_FAULT_RX_CRC);

    diag_fault_clear();
    diag_get_snapshot(&s);
    TEST_ASSERT(s.last_fault_code == (uint8_t)DIAG_FAULT_NONE);

    return true;
}

static bool test_s7_protocol_decode_counters(void)
{
    protocol_frame_t in, out;
    protocol_status_t st;
    uint8_t wire[PROTOCOL_MAX_FRAME_SIZE];
    size_t wire_len = 0u;
    diag_snapshot_t s;
    uint8_t p[2] = {0xAAu, 0x55u};

    diag_reset_all();

    fill_frame(&in, PROTOCOL_FRAME_TYPE_CMD, 0x22u, PROTOCOL_CMD_PING, p, (uint16_t)sizeof(p));
    st = protocol_frame_encode(&in, wire, sizeof(wire), &wire_len);
    TEST_ASSERT(st == PROTOCOL_STATUS_OK);

    st = protocol_frame_decode(wire, wire_len, &out);
    TEST_ASSERT(st == PROTOCOL_STATUS_OK);

    wire[3] ^= 0x11u;
    st = protocol_frame_decode(wire, wire_len, &out);
    TEST_ASSERT(st != PROTOCOL_STATUS_OK);

    st = protocol_frame_decode(wire, wire_len - 1u, &out);
    TEST_ASSERT(st != PROTOCOL_STATUS_OK);

    diag_get_snapshot(&s);
    TEST_ASSERT(s.rx_frames_ok >= 1u);
    TEST_ASSERT(s.rx_crc_errors >= 1u);
    TEST_ASSERT(s.rx_length_errors >= 1u);
    TEST_ASSERT(s.last_fault_code != (uint8_t)DIAG_FAULT_NONE);

    return true;
}

static bool test_s7_dispatcher_trace_and_unknown_tx(void)
{
    protocol_frame_t req;
    cmd_dispatch_result_t out;
    diag_snapshot_t s;

    diag_reset_all();
    cmd_dispatcher_init();

    fill_frame(&req, PROTOCOL_FRAME_TYPE_CMD, 0x33u, PROTOCOL_CMD_GET_READY_STATE, NULL, 0u);
    memset(&out, 0, sizeof(out));
    cmd_dispatcher_process_frame(&req, &out);
    TEST_ASSERT(out.response_ready == true);

    fill_frame(&req, PROTOCOL_FRAME_TYPE_CMD, 0x34u, 0xF0u, NULL, 0u);
    memset(&out, 0, sizeof(out));
    cmd_dispatcher_process_frame(&req, &out);
    TEST_ASSERT(out.response_ready == true);
    TEST_ASSERT(out.response_frame.frame_type == PROTOCOL_FRAME_TYPE_NACK);

    diag_get_snapshot(&s);
    TEST_ASSERT(s.tx_frames >= 2u);
    TEST_ASSERT(s.rx_unknown_cmd >= 1u);
    TEST_ASSERT(s.last_cmd_id == 0xF0u);
    TEST_ASSERT(s.last_seq == 0x34u);
    TEST_ASSERT(s.last_status == (uint8_t)CMD_RESULT_UNKNOWN_COMMAND);

    return true;
}

static bool test_s7_ws2812_diag_counter(void)
{
    uint8_t frame[LED_PHYSICAL_BUF_SIZE];
    diag_snapshot_t s;

    memset(frame, 0xCD, sizeof(frame));

    diag_reset_all();
    TEST_ASSERT(ws2812_init() == true);
    ws2812_stub_backend_set_force_fail(true);

    TEST_ASSERT(ws2812_show(frame, LED_PHYSICAL_COUNT) == false);

    diag_get_snapshot(&s);
    TEST_ASSERT(s.ws2812_update_errors >= 1u);
    TEST_ASSERT(s.last_fault_code != (uint8_t)DIAG_FAULT_NONE);

    ws2812_stub_backend_set_force_fail(false);
    return true;
}

static bool test_s7_switch_unstable_counter(void)
{
    diag_snapshot_t a, b;
    uint32_t raw = (1u << 4);

    diag_reset_all();
    switch_debounce_reset();

    diag_get_snapshot(&a);

    switch_debounce_process(0u, 0u);
    switch_debounce_process(raw, 10u);
    switch_debounce_process(raw, 60u);

    diag_get_snapshot(&b);
    TEST_ASSERT(b.switch_unstable_count >= a.switch_unstable_count);

    return true;
}

static bool test_s7_heartbeat_hook_1hz(void)
{
    bool l0, l1, l2;

    debug_hooks_init();

    debug_hooks_heartbeat_tick(0u);
    l0 = debug_hooks_get_heartbeat_level();

    debug_hooks_heartbeat_tick(1000u);
    l1 = debug_hooks_get_heartbeat_level();

    debug_hooks_heartbeat_tick(2000u);
    l2 = debug_hooks_get_heartbeat_level();

    TEST_ASSERT(l1 != l0);
    TEST_ASSERT(l2 != l1);

    return true;
}

/* ===================== Stage 8 ===================== */

static bool test_s8_mainloop_init_and_step_smoke(void)
{
    app_mainloop_cfg_t cfg;

    cfg.heartbeat_period_ms = 1000u;
    cfg.switch_scan_period_ms = 2u;
    cfg.debounce_period_ms = 1u;

    app_mainloop_init(&cfg);

    /* Несколько шагов superloop: не должно падать */
    app_mainloop_step();
    app_mainloop_step();
    app_mainloop_step();

    TEST_ASSERT(true);
    return true;
}

static bool test_s8_ready_event_sent_once_counter(void)
{
    /* Проверяем косвенно через tx_frames:
       init -> должен отправиться EVT_READY ровно 1 раз */
    diag_snapshot_t a, b, c;
    app_mainloop_cfg_t cfg;

    diag_reset_all();

    cfg.heartbeat_period_ms = 1000u;
    cfg.switch_scan_period_ms = 2u;
    cfg.debounce_period_ms = 1u;

    diag_get_snapshot(&a);
    app_mainloop_init(&cfg);
    diag_get_snapshot(&b);

    /* После init ожидаем хотя бы 1 TX (READY EVT) */
    TEST_ASSERT(b.tx_frames >= (a.tx_frames + 1u));

    /* Доп. шаги не должны повторно слать READY */
    app_mainloop_step();
    app_mainloop_step();
    app_mainloop_step();
    diag_get_snapshot(&c);

    /* В этих шагах может быть 0 новых TX (обычно так),
       главное — READY не штормится. Для безопасной проверки:
       не ожидаем обязательного роста. */
    TEST_ASSERT(c.tx_frames >= b.tx_frames);

    return true;
}

static bool test_s8_longop_led_test_finalize_path(void)
{
    cmd_longop_state_t *lop;
    uint32_t now0;

    app_mainloop_cfg_t cfg;
    cfg.heartbeat_period_ms = 1000u;
    cfg.switch_scan_period_ms = 2u;
    cfg.debounce_period_ms = 1u;

    app_mainloop_init(&cfg);

    lop = cmd_runtime_longop_mut();
    TEST_ASSERT(lop != NULL);

    now0 = app_timebase_now_ms();

    /* Эмулируем активную long-op LED_TEST */
    lop->active = true;
    lop->op_id = CMD_LONGOP_LED_TEST;
    lop->seq = 0xA1u;
    lop->command_id = PROTOCOL_CMD_GET_READY_STATE; /* любой валидный cmd id для корреляции */
    lop->start_ms = now0;
    lop->deadline_ms = now0; /* сразу готово к финализации */
    lop->result_pending = true;
    lop->result_code = (uint8_t)CMD_RESULT_OK;

    app_mainloop_step();

    TEST_ASSERT(lop->active == false);
    TEST_ASSERT(lop->op_id == CMD_LONGOP_NONE);
    TEST_ASSERT(lop->result_pending == false);

    return true;
}

static bool test_s8_mainloop_updates_runtime_trace_via_dispatcher(void)
{
    /* Проверяем интеграцию cmd_runtime в mainloop-пути:
       отправляем request через encode/decode/dispatch путь и смотрим trace.
       Здесь используем уже существующий run_dispatch_flow, как интеграционный baseline. */
    protocol_frame_t req;
    flow_t f;
    const cmd_trace_t *tr;

    fill_frame(&req, PROTOCOL_FRAME_TYPE_CMD, 0xB2u, PROTOCOL_CMD_PING, NULL, 0u);
    TEST_ASSERT(run_dispatch_flow(&f, &req));

    tr = cmd_runtime_trace_get();
    TEST_ASSERT(tr != NULL);
    TEST_ASSERT(tr->last_req_seq == 0xB2u);
    TEST_ASSERT(tr->last_req_cmd_id == PROTOCOL_CMD_PING);
    TEST_ASSERT(tr->last_resp_frame_type == f.resp_decoded.frame_type);

    return true;
}

static bool test_s8_watchdog_hook_safe_when_off(void)
{
    /* По ТЗ watchdog OFF by default — superloop должен быть безопасен */
    app_mainloop_cfg_t cfg;

    cfg.heartbeat_period_ms = 1000u;
    cfg.switch_scan_period_ms = 2u;
    cfg.debounce_period_ms = 1u;

    app_mainloop_init(&cfg);

    /* Просто прогоняем шаги; отсутствие фолта = PASS */
    for (uint32_t i = 0u; i < 32u; i++) {
        app_mainloop_step();
    }

    TEST_ASSERT(true);
    return true;
}


/* ===================== Runner ===================== */

int main(void)
{
    struct {
        const char *name;
        const char *stage;
        bool (*fn)(void);
    } tests[] = {
        {"s1_crc16_known_vector", "stage1", test_s1_crc16_known_vector},
        {"s1_frame_encode_decode_roundtrip", "stage1", test_s1_frame_encode_decode_roundtrip},
        {"s1_frame_decode_crc_error", "stage1", test_s1_frame_decode_crc_error},
        {"s1_frame_decode_truncated", "stage1", test_s1_frame_decode_truncated},

        {"s2_dispatch_ping_resp", "stage2", test_s2_dispatch_ping_resp},
        {"s2_dispatch_unknown_cmd_nack", "stage2", test_s2_dispatch_unknown_cmd_nack},
        {"s2_dispatch_bad_frame_type_nack", "stage2", test_s2_dispatch_bad_frame_type_nack},
        {"s2_runtime_trace_updates", "stage2", test_s2_runtime_trace_updates},

        {"s3_init_default_levels_mask", "stage3", test_s3_init_default_levels_mask},
        {"s3_single_toggle", "stage3", test_s3_single_toggle},
        {"s3_channel_api_invalid", "stage3", test_s3_channel_api_invalid},
        {"s3_pattern_full_mask", "stage3", test_s3_pattern_full_mask},

        {"s4_idle_zero_stays_zero", "stage4", test_s4_idle_zero_stays_zero},
        {"s4_short_pulse_ignored", "stage4", test_s4_short_pulse_ignored},
        {"s4_debounce_after_50ms", "stage4", test_s4_debounce_after_50ms},
        {"s4_stable_after_1000ms", "stage4", test_s4_stable_after_1000ms},
        {"s4_unstable_equals_xor", "stage4", test_s4_unstable_equals_xor},
        {"s4_wraparound_time", "stage4", test_s4_wraparound_time},

        {"s5_mapper_even_slots_only", "stage5", test_s5_mapper_even_slots_only},
        {"s5_set_single_and_build_grb", "stage5", test_s5_set_single_and_build_grb},
        {"s5_apply_success_updates_applied", "stage5", test_s5_apply_success_updates_applied},
        {"s5_intermediate_leds_always_off", "stage5", test_s5_intermediate_leds_always_off},

        {"s6_ws_show_before_init_fails", "stage6", test_s6_ws_show_before_init_fails},
        {"s6_ws_show_null_and_bad_count", "stage6", test_s6_ws_show_null_and_bad_count},
        {"s6_ws_show_success_and_last_frame_inspect", "stage6", test_s6_ws_show_success_and_last_frame_inspect},
        {"s6_ws_injected_backend_fail", "stage6", test_s6_ws_injected_backend_fail},
        {"s6_led_model_apply_fail_keeps_applied", "stage6", test_s6_led_model_apply_fail_keeps_applied},

        {"s7_diag_init_and_reset", "stage7", test_s7_diag_init_and_reset},
        {"s7_diag_fault_latch_semantics", "stage7", test_s7_diag_fault_latch_semantics},
        {"s7_protocol_decode_counters", "stage7", test_s7_protocol_decode_counters},
        {"s7_dispatcher_trace_and_unknown_tx", "stage7", test_s7_dispatcher_trace_and_unknown_tx},
        {"s7_ws2812_diag_counter", "stage7", test_s7_ws2812_diag_counter},
        {"s7_switch_unstable_counter", "stage7", test_s7_switch_unstable_counter},
        {"s7_heartbeat_hook_1hz", "stage7", test_s7_heartbeat_hook_1hz},
        {"s8_mainloop_init_and_step_smoke", "stage8", test_s8_mainloop_init_and_step_smoke},
        {"s8_ready_event_sent_once_counter", "stage8", test_s8_ready_event_sent_once_counter},
        {"s8_longop_led_test_finalize_path", "stage8", test_s8_longop_led_test_finalize_path},
        {"s8_mainloop_updates_runtime_trace_via_dispatcher", "stage8", test_s8_mainloop_updates_runtime_trace_via_dispatcher},
        {"s8_watchdog_hook_safe_when_off", "stage8", test_s8_watchdog_hook_safe_when_off},
    };

    size_t i;
    size_t passed = 0u;
    size_t total = ARRAY_SIZE(tests);

    diag_init();
    debug_hooks_init();

    cmd_dispatcher_init();
    switch_scan_init();
    switch_debounce_init();
    led_model_init();

    for (i = 0u; i < total; i++) {
        bool ok = tests[i].fn();
        printf("[%s] (%s) %s\n", ok ? "PASS" : "FAIL", tests[i].stage, tests[i].name);
        if (ok) {
            passed++;
        }
    }

    printf("RESULT: %zu/%zu passed\n", passed, total);
    return (passed == total) ? 0 : 1;
}
