#ifndef PROTOCOL_FRAME_H
#define PROTOCOL_FRAME_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Protocol v1 framing constants */
#define PROTOCOL_SOF0                  (0xAAu)
#define PROTOCOL_SOF1                  (0x55u)
#define PROTOCOL_VERSION_V1            (0x01u)

/* Frame type values (protocol v1) */
#define PROTOCOL_FRAME_TYPE_CMD        (0x01u)
#define PROTOCOL_FRAME_TYPE_RESP       (0x02u)
#define PROTOCOL_FRAME_TYPE_ACK        (0x03u)
#define PROTOCOL_FRAME_TYPE_NACK       (0x04u)
#define PROTOCOL_FRAME_TYPE_EVT        (0x05u)

/* Command IDs used by dispatcher stage */
#define PROTOCOL_CMD_PING              (0x01u)
#define PROTOCOL_CMD_GET_READY_STATE   (0x02u)
#define PROTOCOL_CMD_GET_FW_VERSION    (0x03u)

#define PROTOCOL_CMD_GET_SWITCH_SNAPSHOT (0x10u)
#define PROTOCOL_CMD_GET_SWITCH_DELTA    (0x11u)

#define PROTOCOL_CMD_LED_SET_SLOT        (0x20u)
#define PROTOCOL_CMD_LED_SET_BULK        (0x21u)
#define PROTOCOL_CMD_LED_CLEAR_ALL       (0x22u)
#define PROTOCOL_CMD_LED_APPLY           (0x23u)
#define PROTOCOL_CMD_LED_TEST            (0x24u)
#define PROTOCOL_CMD_LED_SET_MAP_VER     (0x25u)

#define PROTOCOL_CMD_GET_DIAG            (0x30u)
#define PROTOCOL_CMD_RESET_DIAG          (0x31u)

#define PROTOCOL_CMD_SET_POLL_HINT       (0x40u)
#define PROTOCOL_CMD_NOP                 (0x7Fu)
#define PROTOCOL_MAX_PAYLOAD           (128u)

/* Header fields from protocol */
#define PROTOCOL_HEADER_SIZE_NO_SOF    (6u)  /* ver, type, seq, cmd, len_le(2) */
#define PROTOCOL_CRC_SIZE              (2u)
#define PROTOCOL_SOF_SIZE              (2u)

#define PROTOCOL_MIN_FRAME_SIZE \
    (PROTOCOL_SOF_SIZE + PROTOCOL_HEADER_SIZE_NO_SOF + PROTOCOL_CRC_SIZE)

#define PROTOCOL_MAX_FRAME_SIZE \
    (PROTOCOL_MIN_FRAME_SIZE + PROTOCOL_MAX_PAYLOAD)

/* Frame container */
typedef struct
{
    uint8_t  protocol_version;
    uint8_t  frame_type;
    uint8_t  seq;
    uint8_t  command_id;
    uint16_t payload_length; /* LE on wire */
    uint8_t  payload[PROTOCOL_MAX_PAYLOAD];
} protocol_frame_t;

/* Generic return status for frame codec operations */
typedef enum
{
    PROTOCOL_STATUS_OK = 0,
    PROTOCOL_STATUS_NULL_ARG,
    PROTOCOL_STATUS_BUF_TOO_SMALL,
    PROTOCOL_STATUS_BAD_SOF,
    PROTOCOL_STATUS_UNSUPPORTED_VERSION,
    PROTOCOL_STATUS_BAD_LENGTH,
    PROTOCOL_STATUS_BAD_CRC
} protocol_status_t;

/* Stream parser result event */
typedef enum
{
    PROTOCOL_PARSER_EVENT_NONE = 0,
    PROTOCOL_PARSER_EVENT_FRAME_READY,
    PROTOCOL_PARSER_EVENT_BAD_SOF,
    PROTOCOL_PARSER_EVENT_BAD_LENGTH,
    PROTOCOL_PARSER_EVENT_BAD_CRC,
    PROTOCOL_PARSER_EVENT_UNSUPPORTED_VERSION
} protocol_parser_event_t;

/* Parser-level diagnostic counters */
typedef struct
{
    uint32_t bytes_in;
    uint32_t frames_ok;
    uint32_t err_bad_sof;
    uint32_t err_bad_length;
    uint32_t err_bad_crc;
    uint32_t err_unsupported_version;
    uint32_t resync_count;
} protocol_parser_counters_t;

typedef struct
{
    /* internal state */
    uint8_t  state;
    uint8_t  hdr[PROTOCOL_HEADER_SIZE_NO_SOF];
    uint8_t  hdr_pos;
    uint16_t payload_pos;
    uint16_t payload_len;
    uint8_t  crc_bytes[2];
    uint8_t  crc_pos;

    /* frame-in-progress raw storage (without SOF): header + payload */
    uint8_t  work[PROTOCOL_HEADER_SIZE_NO_SOF + PROTOCOL_MAX_PAYLOAD];

    /* completed frame */
    protocol_frame_t out_frame;
    bool out_frame_ready;

    protocol_parser_counters_t counters;
} protocol_stream_parser_t;

/* Complete-buffer codec */
protocol_status_t protocol_frame_encode(
    const protocol_frame_t *frame,
    uint8_t *out_buf,
    size_t out_buf_size,
    size_t *out_frame_size);

protocol_status_t protocol_frame_decode(
    const uint8_t *buf,
    size_t buf_len,
    protocol_frame_t *out_frame);

/* Stream parser API */
void protocol_stream_parser_init(protocol_stream_parser_t *p);
void protocol_stream_parser_reset(protocol_stream_parser_t *p);

protocol_parser_event_t protocol_stream_parser_feed_byte(
    protocol_stream_parser_t *p,
    uint8_t byte);

bool protocol_stream_parser_take_frame(
    protocol_stream_parser_t *p,
    protocol_frame_t *out_frame);

const protocol_parser_counters_t* protocol_stream_parser_get_counters(
    const protocol_stream_parser_t *p);

#ifdef __cplusplus
}
#endif

#endif /* PROTOCOL_FRAME_H */
