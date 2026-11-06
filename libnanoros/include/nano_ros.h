#ifndef NANO_ROS_H
#define NANO_ROS_H

#include <ucdr/microcdr.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "protocol.h"

typedef struct {
    bool (*init)(void *ctx);
    bool (*open)();
    bool (*close)();
    size_t (*write)(const uint8_t *payload, uint16_t len, void *user_data); // MUST report how many bytes actually written
    size_t (*read)(uint8_t *buffer, uint16_t len, void *user_data); // MUST report how many bytes were read
    void *user_data;  // optional per-transport context, can be NULL
} nros_transport_t;

typedef struct {
    uint8_t qos; // nros_qos_t, see protocol.h
    uint8_t topic_id;
    const char *topic_name;
    const char *topic_type;
    void (*cb)(ucdrBuffer *buf);
} nros_topic_t;

typedef bool (*nros_serialize_fn)(ucdrBuffer* buf, const void* msg);

// Board health, published by the agent on /diagnostics
typedef struct {
    uint32_t faults;      // board-defined bitmask, nonzero shows as an error
    uint8_t reset_reason; // nros_reset_reason_t, why the board last reset
    const char *version;  // firmware version or git hash, can be NULL
} nros_status_t;

typedef uint32_t (*nros_millis_fn)(void);
typedef void (*nros_status_fill_fn)(nros_status_t *status);

extern const nros_topic_t nros_topics[];
extern const uint8_t nros_topic_count;
extern const char nros_board_name[]; // [board] name from ros.toml

bool nros_init(nros_transport_t *t, bool use_nros_framing);
void nros_spin();

bool nros_publish(uint8_t topic_id, nros_serialize_fn fn, const void* msg);

// Makes nros_spin send a status report about once a second while connected. millis is the board's
// clock in milliseconds since boot. fill is called before each report to set faults, reset reason
// and version; it can be NULL, which reports just uptime and dropped publishes.
void nros_enable_status(nros_millis_fn millis, nros_status_fill_fn fill);

#endif // NANO_ROS_H