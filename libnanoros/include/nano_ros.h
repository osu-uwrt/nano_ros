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
typedef uint32_t (*nros_millis_fn)(void);
typedef void (*nros_connection_cb)(bool connected);

extern const nros_topic_t nros_topics[];
extern const uint8_t nros_topic_count;

bool nros_init(nros_transport_t *t, bool use_nros_framing);
void nros_spin();

bool nros_publish(uint8_t topic_id, nros_serialize_fn fn, const void* msg);

// Lets the library time out on its own: it re-announces itself while unregistered, gives up on a
// registration the agent stopped answering, and notices when the agent goes away, then reconnects
// when the agent is back. millis is the board's clock in milliseconds since boot. Without a clock
// the board still registers whenever the agent sends DISCOVER, but never notices a lost agent.
void nros_set_clock(nros_millis_fn millis);

// True once the agent has accepted every topic, false again if the agent goes away
bool nros_is_connected(void);

// Called with true when the board finishes registering and false when it loses the agent
void nros_set_connection_cb(nros_connection_cb cb);

#endif // NANO_ROS_H