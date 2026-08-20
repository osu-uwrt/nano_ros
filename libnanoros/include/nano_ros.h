#ifndef NANO_ROS_H
#define NANO_ROS_H

#include <ucdr/microcdr.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef struct {
    bool (*init)(void *ctx);
    bool (*open)();
    bool (*close)();
    size_t (*write)(const uint8_t *payload, uint16_t len, void *user_data); // MUST report how many bytes actually written
    size_t (*read)(uint8_t *buffer, uint16_t len, void *user_data); // MUST report how many bytes were read
    void *user_data;  // optional per-transport context, can be NULL
} nros_transport_t;

typedef struct {
    uint8_t qos;
    uint8_t topic_id;
    const char *topic_name;
    const char *topic_type;
    void (*cb)(ucdrBuffer *buf);
} nros_topic_t;

typedef bool (*nros_serialize_fn)(ucdrBuffer* buf, const void* msg);

// QoS byte
// bit 0:    reliable (0 = fire-and-forget, 1 = ACK+retry to broker)
// bit 1:    request replay-on-subscribe (broker should cache for late joiners)
// bits 2-4: retry depth (0-7, only relevant if bit0=1)
// bits 5-7: reserved
typedef uint8_t nros_qos_flags_t;

extern const nros_topic_t nros_topics[];
extern const uint8_t nros_topic_count;

bool nros_init(nros_transport_t *t, bool packet_oriented);
void nros_spin();

bool nros_publish(uint8_t topic_id, nros_serialize_fn fn, const void* msg);

#endif // NANO_ROS_H