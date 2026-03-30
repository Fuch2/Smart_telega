#ifndef CMD_HANDLERS_H
#define CMD_HANDLERS_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Unified command result codes for dispatcher */
typedef enum
{
    CMD_RESULT_OK = 0x00,
    CMD_RESULT_BAD_FRAME_TYPE = 0x01,
    CMD_RESULT_UNKNOWN_COMMAND = 0x02,
    CMD_RESULT_BAD_PAYLOAD = 0x03,
    CMD_RESULT_UNSUPPORTED = 0x04,
    CMD_RESULT_BUSY = 0x05,
    CMD_RESULT_INTERNAL_ERROR = 0x06
} cmd_result_code_t;

typedef struct
{
    cmd_result_code_t result;
    uint8_t resp_payload[128];
    uint16_t resp_payload_len;
} cmd_handler_result_t;

void cmd_handlers_init(void);

/* Dispatch table entry point */
void cmd_handlers_execute(uint8_t command_id,
                          const uint8_t *req_payload,
                          uint16_t req_payload_len,
                          cmd_handler_result_t *out_result);

#ifdef __cplusplus
}
#endif

#endif /* CMD_HANDLERS_H */
