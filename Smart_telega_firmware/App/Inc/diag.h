#ifndef DIAG_H
#define DIAG_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    DIAG_FAULT_NONE = 0,
    DIAG_FAULT_RX_CRC = 1,
    DIAG_FAULT_RX_LENGTH = 2,
    DIAG_FAULT_UNKNOWN_CMD = 3,
    DIAG_FAULT_WS2812 = 4,
} diag_fault_code_t;

typedef struct
{
    /* counters */
    uint32_t rx_frames_ok;
    uint32_t rx_crc_errors;
    uint32_t rx_length_errors;
    uint32_t rx_unknown_cmd;
    uint32_t tx_frames;
    uint32_t ws2812_update_errors;
    uint32_t switch_unstable_count;

    /* trace */
    uint8_t  last_cmd_id;
    uint8_t  last_seq;
    uint8_t  last_status;

    /* latched fault */
    uint8_t  last_fault_code;
} diag_snapshot_t;

void diag_init(void);
void diag_reset_all(void);
void diag_reset_counters(void);

void diag_inc_rx_frames_ok(void);
void diag_inc_rx_crc_errors(void);
void diag_inc_rx_length_errors(void);
void diag_inc_rx_unknown_cmd(void);
void diag_inc_tx_frames(void);
void diag_inc_ws2812_update_errors(void);
void diag_inc_switch_unstable_count(void);

void diag_trace_last_command(uint8_t cmd_id, uint8_t seq, uint8_t status);

/* Latch semantics: sets fault only if currently NONE */
void diag_fault_latch(diag_fault_code_t code);
void diag_fault_clear(void);

void diag_get_snapshot(diag_snapshot_t *out_snapshot);

#ifdef __cplusplus
}
#endif

#endif /* DIAG_H */
