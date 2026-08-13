#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

typedef enum {
    NROS_PACKET_REGISTER = 0x01, // register topics or reregister if coming from host
    NROS_PACKET_DATA     = 0x02, // pub / sub data
    NROS_PACKET_ACK      = 0x03, // with status
    NROS_PACKET_REGISTER_DONE = 0x04, // marks registration as done
} nros_packet_type_t;

typedef struct {
    uint8_t type; // assigned to with the packet type enum
    uint8_t topic_id;
    uint16_t length;
} __attribute__((packed)) nros_frame_header_t;

// add flag for stating pub/sub/service
typedef struct {
    uint8_t topic_id;
    uint8_t qos;
    uint8_t type_name_len;
    uint8_t topic_name_len;
    // type name and topic name are strings appended after this on the wire
} __attribute__((packed)) nros_registration_payload_t;

typedef struct {
    uint8_t topic_id;    
    uint8_t status;      // 0 = ok, nonzero = error code (name collision, unsupported qos, etc.)
} __attribute__((packed)) nros_ack_payload_t;

#endif // PROTOCOL_H