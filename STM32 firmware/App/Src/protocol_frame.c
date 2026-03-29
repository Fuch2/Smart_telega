#include "protocol_frame.h"
#include "protocol_crc16.h"
#include "diag.h"

#include <string.h>
#include <stddef.h>

#define PROTOCOL_HEADER_SIZE   (6u)  /* ver, type, seq, cmd, len_lo, len_hi */
#define PROTOCOL_CRC_SIZE      (2u)

static uint16_t rd_u16_le(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static void wr_u16_le(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

protocol_status_t protocol_frame_encode(const protocol_frame_t *in,
                                        uint8_t *out_buf,
                                        size_t out_buf_size,
                                        size_t *out_len)
{
    size_t total_len;
    uint16_t crc;
    size_t max_payload = sizeof(((protocol_frame_t*)0)->payload);

    if ((in == NULL) || (out_buf == NULL) || (out_len == NULL)) {
        /* В твоем baseline нет BAD_PARAM -> используем существующий статус */
        return PROTOCOL_STATUS_BAD_LENGTH;
    }

    if ((size_t)in->payload_length > max_payload) {
        return PROTOCOL_STATUS_BAD_LENGTH;
    }

    total_len = (size_t)PROTOCOL_HEADER_SIZE +
                (size_t)in->payload_length +
                (size_t)PROTOCOL_CRC_SIZE;

    if ((total_len > PROTOCOL_MAX_FRAME_SIZE) || (total_len > out_buf_size)) {
        return PROTOCOL_STATUS_BAD_LENGTH;
    }

    out_buf[0] = in->protocol_version;
    out_buf[1] = in->frame_type;
    out_buf[2] = in->seq;
    out_buf[3] = in->command_id;
    wr_u16_le(&out_buf[4], in->payload_length);

    if (in->payload_length > 0u) {
        memcpy(&out_buf[6], in->payload, in->payload_length);
    }

    crc = protocol_crc16_ccitt_false(out_buf, (size_t)PROTOCOL_HEADER_SIZE + (size_t)in->payload_length);
    wr_u16_le(&out_buf[PROTOCOL_HEADER_SIZE + in->payload_length], crc);

    *out_len = total_len;
    return PROTOCOL_STATUS_OK;
}

protocol_status_t protocol_frame_decode(const uint8_t *in_buf,
                                        size_t in_len,
                                        protocol_frame_t *out)
{
    uint16_t payload_len;
    size_t expected_len;
    uint16_t rx_crc;
    uint16_t calc_crc;
    size_t max_payload = sizeof(((protocol_frame_t*)0)->payload);

    if ((in_buf == NULL) || (out == NULL)) {
        return PROTOCOL_STATUS_BAD_LENGTH;
    }

    if (in_len < (size_t)(PROTOCOL_HEADER_SIZE + PROTOCOL_CRC_SIZE)) {
        diag_inc_rx_length_errors();
        diag_fault_latch(DIAG_FAULT_RX_LENGTH);
        return PROTOCOL_STATUS_BAD_LENGTH;
    }

    payload_len = rd_u16_le(&in_buf[4]);

    if ((size_t)payload_len > max_payload) {
        diag_inc_rx_length_errors();
        diag_fault_latch(DIAG_FAULT_RX_LENGTH);
        return PROTOCOL_STATUS_BAD_LENGTH;
    }

    expected_len = (size_t)PROTOCOL_HEADER_SIZE + (size_t)payload_len + (size_t)PROTOCOL_CRC_SIZE;
    if (in_len != expected_len) {
        diag_inc_rx_length_errors();
        diag_fault_latch(DIAG_FAULT_RX_LENGTH);
        return PROTOCOL_STATUS_BAD_LENGTH;
    }

    rx_crc = rd_u16_le(&in_buf[PROTOCOL_HEADER_SIZE + payload_len]);
    calc_crc = protocol_crc16_ccitt_false(in_buf, (size_t)PROTOCOL_HEADER_SIZE + (size_t)payload_len);

    if (rx_crc != calc_crc) {
        diag_inc_rx_crc_errors();
        diag_fault_latch(DIAG_FAULT_RX_CRC);
        return PROTOCOL_STATUS_BAD_CRC;
    }

    out->protocol_version = in_buf[0];
    out->frame_type = in_buf[1];
    out->seq = in_buf[2];
    out->command_id = in_buf[3];
    out->payload_length = payload_len;

    if (payload_len > 0u) {
        memcpy(out->payload, &in_buf[6], payload_len);
    }

    diag_inc_rx_frames_ok();
    return PROTOCOL_STATUS_OK;
}
