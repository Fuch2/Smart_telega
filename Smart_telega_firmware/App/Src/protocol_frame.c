#include "protocol_frame.h"
#include "protocol_crc16.h"
#include "diag.h"

#include <string.h>
#include <stddef.h>

#define PROTOCOL_HEADER_SIZE   (6u)  /* ver, type, seq, cmd, len_lo, len_hi */
#define PROTOCOL_CRC_SIZE      (2u)

#define PS_WAIT_SOF0    0u
#define PS_WAIT_SOF1    1u
#define PS_READ_HDR     2u
#define PS_READ_PAYLOAD 3u
#define PS_READ_CRC     4u

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

void protocol_stream_parser_init(protocol_stream_parser_t *p)
{
    if (p == NULL) { return; }
    memset(p, 0, sizeof(*p));
    p->state = PS_WAIT_SOF0;
}

void protocol_stream_parser_reset(protocol_stream_parser_t *p)
{
    if (p == NULL) { return; }
    /* Сохраняем счётчики, сбрасываем только состояние */
    protocol_parser_counters_t saved = p->counters;
    memset(p, 0, sizeof(*p));
    p->counters = saved;
    p->state = PS_WAIT_SOF0;
}

protocol_parser_event_t protocol_stream_parser_feed_byte(
    protocol_stream_parser_t *p, uint8_t byte)
{
    if (p == NULL) { return PROTOCOL_PARSER_EVENT_NONE; }

    p->counters.bytes_in++;
    p->out_frame_ready = false;

    switch (p->state) {

    /* ── SOF ──────────────────────────────────────────────── */
    case PS_WAIT_SOF0:
        if (byte == PROTOCOL_SOF0) {
            p->state = PS_WAIT_SOF1;
        } else {
            p->counters.err_bad_sof++;
        }
        break;

    case PS_WAIT_SOF1:
        if (byte == PROTOCOL_SOF1) {
            p->hdr_pos    = 0u;
            p->payload_pos = 0u;
            p->crc_pos    = 0u;
            p->state      = PS_READ_HDR;
        } else if (byte == PROTOCOL_SOF0) {
            /* Остаёмся — возможно начало нового SOF */
            p->counters.err_bad_sof++;
            p->counters.resync_count++;
        } else {
            p->counters.err_bad_sof++;
            p->counters.resync_count++;
            p->state = PS_WAIT_SOF0;
        }
        break;

    /* ── Header: ver, type, seq, cmd, len_lo, len_hi ─────── */
    case PS_READ_HDR:
        p->hdr[p->hdr_pos] = byte;
        p->work[p->hdr_pos] = byte;   /* work[] = header + payload для CRC */
        p->hdr_pos++;

        if (p->hdr_pos >= PROTOCOL_HEADER_SIZE_NO_SOF) {
            p->payload_len = rd_u16_le(&p->hdr[4]);

            if (p->payload_len > PROTOCOL_MAX_PAYLOAD) {
                p->counters.err_bad_length++;
                diag_inc_rx_length_errors();
                diag_fault_latch(DIAG_FAULT_RX_LENGTH);
                p->counters.resync_count++;
                p->state = PS_WAIT_SOF0;
                return PROTOCOL_PARSER_EVENT_BAD_LENGTH;
            }

            /* version check */
            if (p->hdr[0] != PROTOCOL_VERSION_V1) {
                p->counters.err_unsupported_version++;
                p->counters.resync_count++;
                p->state = PS_WAIT_SOF0;
                return PROTOCOL_PARSER_EVENT_UNSUPPORTED_VERSION;
            }

            if (p->payload_len == 0u) {
                p->state = PS_READ_CRC;
            } else {
                p->payload_pos = 0u;
                p->state = PS_READ_PAYLOAD;
            }
        }
        break;

    /* ── Payload ──────────────────────────────────────────── */
    case PS_READ_PAYLOAD:
        p->work[PROTOCOL_HEADER_SIZE_NO_SOF + p->payload_pos] = byte;
        p->payload_pos++;

        if (p->payload_pos >= p->payload_len) {
            p->state = PS_READ_CRC;
        }
        break;

    /* ── CRC: 2 байта LE ──────────────────────────────────── */
    case PS_READ_CRC:
        p->crc_bytes[p->crc_pos] = byte;
        p->crc_pos++;

        if (p->crc_pos >= PROTOCOL_CRC_SIZE) {
            uint16_t rx_crc   = rd_u16_le(p->crc_bytes);
            uint16_t calc_crc = protocol_crc16_ccitt_false(
                p->work,
                (size_t)PROTOCOL_HEADER_SIZE_NO_SOF + (size_t)p->payload_len);

            if (rx_crc != calc_crc) {
                p->counters.err_bad_crc++;
                diag_inc_rx_crc_errors();
                diag_fault_latch(DIAG_FAULT_RX_CRC);
                p->counters.resync_count++;
                p->state = PS_WAIT_SOF0;
                return PROTOCOL_PARSER_EVENT_BAD_CRC;
            }

            /* ✅ Фрейм валиден — заполняем out_frame */
            p->out_frame.protocol_version = p->hdr[0];
            p->out_frame.frame_type       = p->hdr[1];
            p->out_frame.seq              = p->hdr[2];
            p->out_frame.command_id       = p->hdr[3];
            p->out_frame.payload_length   = p->payload_len;

            if (p->payload_len > 0u) {
                memcpy(p->out_frame.payload,
                       &p->work[PROTOCOL_HEADER_SIZE_NO_SOF],
                       p->payload_len);
            }

            p->out_frame_ready = true;
            p->counters.frames_ok++;
            diag_inc_rx_frames_ok();
            p->state = PS_WAIT_SOF0;
            return PROTOCOL_PARSER_EVENT_FRAME_READY;
        }
        break;

    default:
        p->state = PS_WAIT_SOF0;
        break;
    }

    return PROTOCOL_PARSER_EVENT_NONE;
}

bool protocol_stream_parser_take_frame(
    protocol_stream_parser_t *p, protocol_frame_t *out_frame)
{
    if ((p == NULL) || (out_frame == NULL)) { return false; }
    if (!p->out_frame_ready) { return false; }

    *out_frame = p->out_frame;
    p->out_frame_ready = false;
    return true;
}

const protocol_parser_counters_t* protocol_stream_parser_get_counters(
    const protocol_stream_parser_t *p)
{
    return (p != NULL) ? &p->counters : NULL;
}
