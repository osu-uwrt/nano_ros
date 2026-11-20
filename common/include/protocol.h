#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

#define NROS_MAGIC0 0xAA
#define NROS_MAGIC1 0x55
#define NROS_BUFFER_SIZE 1024 // a whole frame (header, payload, CRC) has to fit in this
#define NROS_CRC_LEN 2
#define NROS_MAX_STR_SIZE 128 // longest topic name or type name a board can register

// Checked at compile time on both sides: the mcu and the agent are built by different
// compilers for different CPUs but read the same bytes
#ifdef __cplusplus
#define NROS_STATIC_ASSERT(cond, msg) static_assert(cond, msg)
#else
#define NROS_STATIC_ASSERT(cond, msg) _Static_assert(cond, msg)
#endif

// Payload structs are copied straight onto the wire, which assumes a little-endian CPU (the RP2040
// and x86 / arm64 hosts all are)
#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "nano_ros copies protocol structs onto the wire as-is and needs a little-endian CPU"
#endif

typedef enum {
    NROS_PACKET_REGISTER = 0x01, // register topics or reregister if coming from host
    NROS_PACKET_DATA     = 0x02, // pub / sub data
    NROS_PACKET_ACK      = 0x03, // with status
    NROS_PACKET_REGISTER_DONE = 0x04, // marks registration as done
    NROS_PACKET_DISCOVER = 0x05, // from agent to tell mcu to give up its topics
} nros_packet_type_t;

typedef struct {
    uint8_t magic[2];
    uint8_t type; // assigned to with the packet type enum
   // uint8_t client_id;
    uint8_t topic_id;
    uint16_t length;
} __attribute__((packed)) nros_frame_header_t;

NROS_STATIC_ASSERT(sizeof(nros_frame_header_t) == 6, "nros_frame_header_t must be 6 bytes on the wire");

// The biggest payload one frame can carry
#define NROS_MAX_PAYLOAD (NROS_BUFFER_SIZE - sizeof(nros_frame_header_t) - NROS_CRC_LEN)

// QoS profile sent in nros_registration_payload_t.qos. The agent maps each value
// to an rclcpp::QoS profile. Values are part of the wire protocol: append only.
typedef enum {
    NROS_QOS_DEFAULT         = 0x00, // rclcpp default: reliable, volatile, keep last 10
    NROS_QOS_SENSOR_DATA     = 0x01, // rclcpp::SensorDataQoS: best effort, volatile, keep last 5
    NROS_QOS_RELIABLE        = 0x02, // default with explicit reliable reliability
    NROS_QOS_BEST_EFFORT     = 0x03, // default with best effort reliability
    NROS_QOS_TRANSIENT_LOCAL = 0x04, // reliable, transient local (latched), keep last 1
    NROS_QOS_SYSTEM_DEFAULT  = 0x05, // rclcpp::SystemDefaultsQoS (whatever the RMW picks)
    NROS_QOS_COUNT,                  // number of valid values, not a QoS
} nros_qos_t;

// status byte in nros_ack_payload_t
typedef enum {
    NROS_ACK_OK              = 0x00,
    NROS_ACK_UNSUPPORTED_QOS = 0x01,
    NROS_ACK_MALFORMED       = 0x02,
} nros_ack_status_t;

// add flag for stating pub/sub/service
typedef struct {
    uint8_t topic_id;
    uint8_t qos; // nros_qos_t
    uint8_t type_name_len;
    uint8_t topic_name_len;
    // type name and topic name are strings appended after this on the wire
} __attribute__((packed)) nros_registration_payload_t;

NROS_STATIC_ASSERT(sizeof(nros_registration_payload_t) == 4, "nros_registration_payload_t must be 4 bytes on the wire");
NROS_STATIC_ASSERT(NROS_MAX_STR_SIZE <= UINT8_MAX, "registration sends name lengths as uint8_t");
NROS_STATIC_ASSERT(sizeof(nros_registration_payload_t) + 2 * NROS_MAX_STR_SIZE <= NROS_MAX_PAYLOAD,
                   "a registration with the longest names has to fit in one frame");

typedef struct {
    uint8_t topic_id;    
    uint8_t status;      // nros_ack_status_t: 0 = ok, nonzero = error code
} __attribute__((packed)) nros_ack_payload_t;

NROS_STATIC_ASSERT(sizeof(nros_ack_payload_t) == 2, "nros_ack_payload_t must be 2 bytes on the wire");

#endif // PROTOCOL_H