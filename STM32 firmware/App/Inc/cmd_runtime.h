#ifndef CMD_RUNTIME_H
#define CMD_RUNTIME_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    CMD_LONGOP_NONE = 0,
    CMD_LONGOP_LED_TEST = 1
} cmd_longop_id_t;

typedef struct
{
    bool active;
    cmd_longop_id_t op_id;
    uint8_t seq;
    uint8_t command_id;
    uint32_t start_ms;
    uint32_t deadline_ms;
    bool result_pending;
    uint8_t result_code;
} cmd_longop_state_t;

typedef struct
{
    uint8_t last_req_seq;
    uint8_t last_req_cmd_id;
    uint8_t last_req_frame_type;
    uint8_t last_result_code;
    uint8_t last_resp_frame_type; /* RESP or NACK */
    uint32_t total_requests;
    uint32_t total_resp;
    uint32_t total_nack;
} cmd_trace_t;

void cmd_runtime_init(void);

cmd_longop_state_t* cmd_runtime_longop_mut(void);
const cmd_longop_state_t* cmd_runtime_longop_get(void);

void cmd_runtime_trace_req(uint8_t frame_type, uint8_t seq, uint8_t cmd_id);
void cmd_runtime_trace_resp(uint8_t frame_type, uint8_t result_code);

const cmd_trace_t* cmd_runtime_trace_get(void);

#ifdef __cplusplus
}
#endif

#endif /* CMD_RUNTIME_H */
