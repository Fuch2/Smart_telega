#include "cmd_runtime.h"
#include "protocol_frame.h"


#include <string.h>

static cmd_longop_state_t g_longop;
static cmd_trace_t g_trace;

void cmd_runtime_init(void)
{
    (void)memset(&g_longop, 0, sizeof(g_longop));
    (void)memset(&g_trace, 0, sizeof(g_trace));
}

cmd_longop_state_t* cmd_runtime_longop_mut(void)
{
    return &g_longop;
}

const cmd_longop_state_t* cmd_runtime_longop_get(void)
{
    return &g_longop;
}

void cmd_runtime_trace_req(uint8_t frame_type, uint8_t seq, uint8_t cmd_id)
{
    g_trace.last_req_frame_type = frame_type;
    g_trace.last_req_seq = seq;
    g_trace.last_req_cmd_id = cmd_id;
    g_trace.total_requests++;
}

void cmd_runtime_trace_resp(uint8_t frame_type, uint8_t result_code)
{
    g_trace.last_resp_frame_type = frame_type;
    g_trace.last_result_code = result_code;

    if (frame_type == PROTOCOL_FRAME_TYPE_RESP) {
        g_trace.total_resp++;
    } else if (frame_type == PROTOCOL_FRAME_TYPE_NACK) {
        g_trace.total_nack++;
    }
}


const cmd_trace_t* cmd_runtime_trace_get(void)
{
    return &g_trace;
}
