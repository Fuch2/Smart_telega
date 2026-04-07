#include "cmd_handlers.h"
#include "protocol_frame.h"
#include "switch_debounce.h"


#include <string.h>

/* Static FW metadata for MVP */
#define FW_VERSION_MAJOR   (0u)
#define FW_VERSION_MINOR   (1u)
#define FW_VERSION_PATCH   (0u)
#define FW_VERSION_BUILD   (1u)

/* После app_mainloop_init() прикладной слой готов принимать команды. */
#define READY_STATE_READY (1u)

static void result_set(cmd_handler_result_t *r, cmd_result_code_t code)
{
    if (r == NULL) {
        return;
    }
    r->result = code;
    r->resp_payload_len = 0u;
}

static void handle_nop(const uint8_t *req_payload, uint16_t req_len, cmd_handler_result_t *r)
{
    (void)req_payload;
    if (req_len != 0u) {
        result_set(r, CMD_RESULT_BAD_PAYLOAD);
        return;
    }
    result_set(r, CMD_RESULT_OK);
}

static void handle_ping(const uint8_t *req_payload, uint16_t req_len, cmd_handler_result_t *r)
{
    if ((r == NULL) || (req_payload == NULL && req_len > 0u)) {
        return;
    }
    if (req_len > 128u) {
        result_set(r, CMD_RESULT_BAD_PAYLOAD);
        return;
    }

    r->result = CMD_RESULT_OK;
    r->resp_payload_len = req_len;
    if (req_len > 0u) {
        (void)memcpy(r->resp_payload, req_payload, req_len);
    }
}

static void handle_get_fw_version(const uint8_t *req_payload, uint16_t req_len, cmd_handler_result_t *r)
{
    (void)req_payload;
    if (req_len != 0u) {
        result_set(r, CMD_RESULT_BAD_PAYLOAD);
        return;
    }

    /* payload: major, minor, patch, build */
    r->result = CMD_RESULT_OK;
    r->resp_payload[0] = FW_VERSION_MAJOR;
    r->resp_payload[1] = FW_VERSION_MINOR;
    r->resp_payload[2] = FW_VERSION_PATCH;
    r->resp_payload[3] = FW_VERSION_BUILD;
    r->resp_payload_len = 4u;
}

static void handle_get_ready_state(const uint8_t *req_payload, uint16_t req_len, cmd_handler_result_t *r)
{
    (void)req_payload;
    if (req_len != 0u) {
        result_set(r, CMD_RESULT_BAD_PAYLOAD);
        return;
    }

    /* payload: ready_state(1 byte) */
    r->result = CMD_RESULT_OK;
    r->resp_payload[0] = READY_STATE_READY;
    r->resp_payload_len = 1u;
}

static void handle_get_switch_snapshot(const uint8_t *req_payload, uint16_t req_len,
                                       cmd_handler_result_t *r)
{
    switch_snapshot_t snap;
    (void)req_payload;

    if (req_len != 0u) {
        result_set(r, CMD_RESULT_BAD_PAYLOAD);
        return;
    }

    switch_debounce_get_snapshot(&snap);

    r->resp_payload[0] = (uint8_t)( snap.debounced_bits        & 0xFFu);
    r->resp_payload[1] = (uint8_t)((snap.debounced_bits >>  8u) & 0xFFu);
    r->resp_payload[2] = (uint8_t)((snap.debounced_bits >> 16u) & 0xFFu);
    r->resp_payload_len = 3u;
    r->result = CMD_RESULT_OK;
}

void cmd_handlers_init(void)
{
    /* no-op for now */
}


void cmd_handlers_execute(uint8_t command_id,
                          const uint8_t *req_payload,
                          uint16_t req_payload_len,
                          cmd_handler_result_t *out_result)
{
    if (out_result == NULL) {
        return;
    }

    result_set(out_result, CMD_RESULT_INTERNAL_ERROR);

    switch (command_id) {
    case PROTOCOL_CMD_NOP:
        handle_nop(req_payload, req_payload_len, out_result);
        break;

    case PROTOCOL_CMD_PING:
        handle_ping(req_payload, req_payload_len, out_result);
        break;

    case PROTOCOL_CMD_GET_FW_VERSION:
        handle_get_fw_version(req_payload, req_payload_len, out_result);
        break;

    case PROTOCOL_CMD_GET_READY_STATE:
        handle_get_ready_state(req_payload, req_payload_len, out_result);
        break;

    case PROTOCOL_CMD_GET_SWITCH_SNAPSHOT:
        handle_get_switch_snapshot(req_payload, req_payload_len, out_result);
        break;

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
        result_set(out_result, CMD_RESULT_UNSUPPORTED);
        break;

    default:
        result_set(out_result, CMD_RESULT_UNKNOWN_COMMAND);
        break;
}

}
