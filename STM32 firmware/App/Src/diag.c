#include "diag.h"
#include <string.h>

typedef struct
{
    diag_snapshot_t s;
} diag_ctx_t;

static diag_ctx_t g_diag;

void diag_init(void)
{
    diag_reset_all();
}

void diag_reset_all(void)
{
    (void)memset(&g_diag, 0, sizeof(g_diag));
    g_diag.s.last_fault_code = (uint8_t)DIAG_FAULT_NONE;
}

void diag_reset_counters(void)
{
    g_diag.s.rx_frames_ok = 0u;
    g_diag.s.rx_crc_errors = 0u;
    g_diag.s.rx_length_errors = 0u;
    g_diag.s.rx_unknown_cmd = 0u;
    g_diag.s.tx_frames = 0u;
    g_diag.s.ws2812_update_errors = 0u;
    g_diag.s.switch_unstable_count = 0u;
}

void diag_inc_rx_frames_ok(void)            { g_diag.s.rx_frames_ok++; }
void diag_inc_rx_crc_errors(void)           { g_diag.s.rx_crc_errors++; }
void diag_inc_rx_length_errors(void)        { g_diag.s.rx_length_errors++; }
void diag_inc_rx_unknown_cmd(void)          { g_diag.s.rx_unknown_cmd++; }
void diag_inc_tx_frames(void)               { g_diag.s.tx_frames++; }
void diag_inc_ws2812_update_errors(void)    { g_diag.s.ws2812_update_errors++; }
void diag_inc_switch_unstable_count(void)   { g_diag.s.switch_unstable_count++; }

void diag_trace_last_command(uint8_t cmd_id, uint8_t seq, uint8_t status)
{
    g_diag.s.last_cmd_id = cmd_id;
    g_diag.s.last_seq = seq;
    g_diag.s.last_status = status;
}

void diag_fault_latch(diag_fault_code_t code)
{
    if (g_diag.s.last_fault_code == (uint8_t)DIAG_FAULT_NONE) {
        g_diag.s.last_fault_code = (uint8_t)code;
    }
}

void diag_fault_clear(void)
{
    g_diag.s.last_fault_code = (uint8_t)DIAG_FAULT_NONE;
}

void diag_get_snapshot(diag_snapshot_t *out_snapshot)
{
    if (out_snapshot == NULL) {
        return;
    }
    *out_snapshot = g_diag.s;
}
