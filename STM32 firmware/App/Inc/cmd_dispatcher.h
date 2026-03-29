#ifndef CMD_DISPATCHER_H
#define CMD_DISPATCHER_H

#include <stdbool.h>
#include <stdint.h>

#include "protocol_frame.h"
#include "cmd_handlers.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    bool response_ready;
    protocol_frame_t response_frame;
    cmd_result_code_t result_code;
} cmd_dispatch_result_t;

void cmd_dispatcher_init(void);

/* Input: decoded request frame (from protocol parser)
 * Output: response frame (RESP or NACK) ready for protocol_frame_encode + UART TX
 */
void cmd_dispatcher_process_frame(const protocol_frame_t *req,
                                  cmd_dispatch_result_t *out_result);

#ifdef __cplusplus
}
#endif

#endif /* CMD_DISPATCHER_H */
