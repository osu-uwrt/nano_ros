#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

#define NROS_MAGIC0 0xAA
#define NROS_MAGIC1 0x55
#define NROS_BUFFER_SIZE 1024

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

typedef struct {
    uint8_t topic_id;    
    uint8_t status;      // nros_ack_status_t: 0 = ok, nonzero = error code
} __attribute__((packed)) nros_ack_payload_t;

#endif // PROTOCOL_H