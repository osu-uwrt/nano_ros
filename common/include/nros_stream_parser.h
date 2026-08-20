#ifndef NROS_PARSER_H
#define NROS_PARSER_H

#include <stdint.h>
#include <stddef.h>

#include "protocol.h"

#define NROS_CRC16_POLY 0x1021
#define NROS_CRC16_INIT 0xFFFF
#define NROS_CRC_LEN 2

typedef enum {
    HUNT_SYNC,
    WAIT_HEADER,
    WAIT_PAYLOAD,
    WAIT_CRC,
} parse_state_t;

typedef struct {
    parse_state_t state;

    uint8_t header_buf[sizeof(nros_frame_header_t)];
    size_t header_bytes_have;

    nros_frame_header_t header;
    uint16_t running_crc;

    uint8_t payload_buf[NROS_BUFFER_SIZE];
    size_t payload_bytes_have;

    uint8_t crc_buf[2];
    size_t crc_bytes_have;

    size_t resync_count;
    size_t crc_fail_count;
} nros_stream_parser_t;


typedef void (*nros_frame_cb)(const nros_frame_header_t *hdr, const uint8_t payload[], void *user_data);

void nros_stream_parser_init(nros_stream_parser_t *p);

// Feed however many bytes you have (1 or 1000, doesn't matter).
// Internally hunts for sync, validates, buffers payload, fires cb on success.
void nros_stream_parser_feed(nros_stream_parser_t *p, const uint8_t data[], size_t len, nros_frame_cb cb, void *user_data);

size_t nros_stream_encode_frame(const nros_frame_header_t *hdr, const uint8_t *payload, size_t payload_len, uint8_t *out_buf, size_t out_buf_cap);

uint16_t crc16_update(uint16_t crc, uint8_t byte);
uint16_t crc16_buf(const uint8_t *data, size_t len);

#endif // NROS_PARSER_H