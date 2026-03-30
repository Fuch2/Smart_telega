#include "cmd_dispatcher.h"
#include "cmd_handlers.h"
#include "cmd_runtime.h"
#include "diag.h"

#include <string.h>
#include <stdint.h>

#define RESP_PAYLOAD_CAP   ((uint16_t)sizeof(((protocol_frame_t*)0)->payload))

static void build_reply(cmd_dispatch_result_t *out,
                        const protocol_frame_t *req,
                        uint8_t frame_type,
                        uint8_t result_code,
                        const uint8_t *extra_payload,
                        uint16_t extra_len)
{
    uint16_t copy_len = 0u;

    memset(&out->response_frame, 0, sizeof(out->response_frame));
    out->response_frame.protocol_version = PROTOCOL_VERSION_V1;
    out->response_frame.frame_type       = frame_type;
    out->response_frame.seq              = req->seq;
    out->response_frame.command_id       = req->command_id;
    out->result_code                     = result_code;

    if ((extra_payload != NULL) && (extra_len > 0u)) {
        uint16_t max = RESP_PAYLOAD_CAP;
        copy_len = (extra_len <= max) ? extra_len : max;
        memcpy(out->response_frame.payload, extra_payload, copy_len);
    }

    out->response_frame.payload_length = copy_len;
    out->response_ready = true;
}


void cmd_dispatcher_init(void)
{
    cmd_runtime_init();
    cmd_handlers_init();
}

void cmd_dispatcher_process_frame(const protocol_frame_t *req, cmd_dispatch_result_t *out)
{
    cmd_handler_result_t hres;

    if ((req == NULL) || (out == NULL)) {
        return;
    }

    memset(out, 0, sizeof(*out));

    /* trace входящего запроса */
    cmd_runtime_trace_req(req->frame_type, req->seq, req->command_id);

    if (req->frame_type != PROTOCOL_FRAME_TYPE_CMD) {
        build_reply(out, req, PROTOCOL_FRAME_TYPE_NACK,
                    (uint8_t)CMD_RESULT_BAD_FRAME_TYPE, NULL, 0u);

        cmd_runtime_trace_resp(out->response_frame.frame_type, out->result_code);

        diag_inc_tx_frames();
        diag_trace_last_command(req->command_id, req->seq, out->result_code);
        return;
    }

    switch (req->command_id) {
    /* ── Команды с реальной реализацией ── */
    case PROTOCOL_CMD_NOP:
    case PROTOCOL_CMD_PING:
    case PROTOCOL_CMD_GET_FW_VERSION:
    case PROTOCOL_CMD_GET_READY_STATE:
    case PROTOCOL_CMD_GET_SWITCH_SNAPSHOT:          /* ✅ было GET_SWITCH_STATE */
        memset(&hres, 0, sizeof(hres));

        cmd_handlers_execute(req->command_id,
                             req->payload,
                             req->payload_length,
                             &hres);

        build_reply(out, req, PROTOCOL_FRAME_TYPE_RESP,
                    (uint8_t)hres.result,
                    hres.resp_payload,
                    hres.resp_payload_len);
        break;

    /* ── Заглушки: команды известны, но не реализованы ── */
    case PROTOCOL_CMD_GET_SWITCH_DELTA:
    case PROTOCOL_CMD_LED_SET_SLOT:
    case PROTOCOL_CMD_LED_SET_BULK:
    case PROTOCOL_CMD_LED_CLEAR_ALL:
    case PROTOCOL_CMD_LED_APPLY:
    case PROTOCOL_CMD_LED_TEST:
    case PROTOCOL_CMD_LED_SET_MAP_VER:
    case PROTOCOL_CMD_GET_DIAG:
    case PROTOCOL_CMD_RESET_DIAG:
    case PROTOCOL_CMD_SET_POLL_HINT:
        build_reply(out, req, PROTOCOL_FRAME_TYPE_RESP,
                    (uint8_t)CMD_RESULT_UNSUPPORTED, NULL, 0u);
        break;

    /* ── Неизвестная команда ── */
    default:
        diag_inc_rx_unknown_cmd();
        diag_fault_latch(DIAG_FAULT_UNKNOWN_CMD);

        build_reply(out, req, PROTOCOL_FRAME_TYPE_NACK,
                    (uint8_t)CMD_RESULT_UNKNOWN_COMMAND, NULL, 0u);
        break;
}


    if (out->response_ready) {
        cmd_runtime_trace_resp(out->response_frame.frame_type, out->result_code);

        diag_inc_tx_frames();
        diag_trace_last_command(req->command_id, req->seq, out->result_code);
    }
}
