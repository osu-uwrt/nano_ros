#include "nros_stream_parser.h"

#include <string.h>
#include <stdbool.h>

static void get_u16(const uint8_t *buf, uint16_t *out) {
    *out = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
}

static void put_u16(uint8_t *buf, uint16_t v) {
    buf[0] = v & 0xFF;
    buf[1] = (v >> 8) & 0xFF;
}
static void put_u32(uint8_t *buf, uint32_t v) {
    buf[0] = v & 0xFF; buf[1] = (v >> 8) & 0xFF;
    buf[2] = (v >> 16) & 0xFF; buf[3] = (v >> 24) & 0xFF;
}

static void reset_to_hunt(nros_stream_parser_t *p, bool count_as_resync) {
    if (count_as_resync) p->resync_count++;
    p->state = HUNT_SYNC;
    p->header_bytes_have = 0;
}

uint16_t crc16_update(uint16_t crc, uint8_t byte) {
    crc ^= (uint16_t)byte << 8;
    for (int i = 0; i < 8; i++) {
        crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ NROS_CRC16_POLY)
                              : (uint16_t)(crc << 1);
    }
    return crc;
}

uint16_t crc16_buf(const uint8_t *data, size_t len) {
    uint16_t crc = NROS_CRC16_INIT;
    for (size_t i = 0; i < len; i++) crc = crc16_update(crc, data[i]);
    return crc;
}

void nros_stream_parser_init(nros_stream_parser_t *p) {
    memset(p, 0, sizeof(*p));
    p->state = HUNT_SYNC;
}

void nros_stream_parser_feed(nros_stream_parser_t *p, const uint8_t data[], size_t len, nros_frame_cb cb, void *user_data) {
    size_t i = 0;
    while (i < len) {
        uint8_t byte = data[i++];

        switch (p->state) {

        case HUNT_SYNC: 
            // 2 byte sliding window to grab onto 2 byte magic number
            p->header_buf[0] = p->header_buf[1];
            p->header_buf[1] = byte;
            if (p->header_buf[0] == NROS_MAGIC0 && p->header_buf[1] == NROS_MAGIC1) {
                // got sync
                p->header_bytes_have = 2;
                p->state = WAIT_HEADER;
            }
            break;

        case WAIT_HEADER:

            p->header_buf[p->header_bytes_have++] = byte;
            if (p->header_bytes_have < sizeof(p->header_buf)) {
                break;
            }
            // have enough bytes for the header
            memcpy(&p->header, p->header_buf, sizeof(p->header));
            if (p->header.length > NROS_BUFFER_SIZE) {
                // sync checked out but length is trash, reset
                memmove(p->header_buf, p->header_buf + 1, sizeof(p->header_buf) - 1);
                p->header_bytes_have = sizeof(p->header_buf) - 1;
                reset_to_hunt(p, true);
                // re seed magic number window
                p->header_buf[0] = p->header_buf[sizeof(p->header_buf) - 2];
                p->header_buf[1] = byte;
                break;
            }
            p->running_crc = crc16_buf((uint8_t *)&p->header, sizeof(p->header));
            p->payload_bytes_have = 0;
            p->crc_bytes_have = 0;
            p->state = (p->header.length == 0) ? WAIT_CRC : WAIT_PAYLOAD;
            break;

        case WAIT_PAYLOAD:

            p->payload_buf[p->payload_bytes_have++] = byte;
            p->running_crc = crc16_update(p->running_crc, byte);
            if (p->payload_bytes_have < p->header.length) {
                break;
            }
            p->state = WAIT_CRC;
            break;

        case WAIT_CRC:

            p->crc_buf[p->crc_bytes_have++] = byte;
            if (p->crc_bytes_have < 2) {
                break;
            }

            uint16_t received_crc;
            get_u16(p->crc_buf, &received_crc);

            if (received_crc != p->running_crc) {
                p->crc_fail_count++;
                reset_to_hunt(p, false);
                break;
            }

            cb(&p->header, p->payload_buf, user_data);
            reset_to_hunt(p, false);
            break;

        }
    }
}

size_t nros_stream_encode_frame(const nros_frame_header_t *header, const uint8_t *payload, size_t payload_len, uint8_t *out_buf, size_t out_buf_cap) {
    size_t total = sizeof(nros_frame_header_t) + payload_len + NROS_CRC_LEN;
    if (out_buf_cap < total) {
        return 0;
    }

    uint8_t *cursor = out_buf;
    cursor[0] = NROS_MAGIC0;
    cursor[1] = NROS_MAGIC1;
    cursor[2] = header->type;
    cursor[3] = header->topic_id;
    put_u16(cursor + 4, header->length);

    cursor += sizeof(nros_frame_header_t);

    if (payload_len > 0 && payload != NULL) {
        memcpy(cursor, payload, payload_len);
        cursor += payload_len;
    }

    uint16_t crc = crc16_buf(out_buf, sizeof(nros_frame_header_t) + payload_len);
    put_u16(cursor, crc);

    return total;
}