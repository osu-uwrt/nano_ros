#include "nano_ros.h"
#include "protocol.h"
#include "nros_stream_parser.h"
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>

#define NROS_BUFFER_SIZE 1024
#define NROS_TOPIC_ALLOC_SIZE 256
#define NROS_MAX_STR_SIZE 128

typedef enum {
    RX_WAIT_HEADER,
    RX_WAIT_PAYLOAD,
} nros_rx_state_t;

typedef enum {
    TX_IDLE,
    TX_SENDING_HEADER,
    TX_SENDING_PAYLOAD,
    TX_SENDING_FRAME,
} nros_tx_state_t;

typedef struct {
    uint8_t topic_idx;
    bool need_registration;
    bool registration_complete;
} nros_registration_ctx_t;

// typedef struct {
// } nros_registration_flags_t;

typedef struct {
    nros_tx_state_t state;

    nros_frame_header_t header;

    size_t header_bytes_sent;
    size_t payload_bytes_sent;
    size_t payload_len;

    uint8_t topic_registration_idx;

    bool is_registration;
    bool send_done;
    bool sending_reg_done_frame; // for completion of registration

    uint8_t buffer[NROS_BUFFER_SIZE];
    uint8_t payload_buffer[NROS_BUFFER_SIZE]; // used by library to set up a payload before fully framing in buffer

    //new 
    size_t frame_len;
    size_t bytes_sent;
} nros_tx_ctx_t;

typedef struct {
    nros_rx_state_t state;
    nros_frame_header_t header;
    size_t bytes_have;
    uint8_t buffer[NROS_BUFFER_SIZE];
    bool awaiting_ack;
} nros_rx_ctx_t;

static nros_transport_t *transport;
static nros_topic_t const *topic_map[NROS_TOPIC_ALLOC_SIZE] = { 0 };
typedef enum {
    LINK_DISCONNECTED, // not registered, announcing with HELLO
    LINK_REGISTERING,  // sending REGISTER frames
    LINK_CONNECTED,    // agent accepted every topic
} nros_link_state_t;

#define NROS_HELLO_PERIOD_MS 1000  // how often to announce while unregistered
#define NROS_ACK_TIMEOUT_MS 1000   // give up on a registration when an ACK takes longer than this
#define NROS_AGENT_TIMEOUT_MS 2000 // agent pings every 500 ms, so this is several missed pings

static nros_link_state_t link_state = LINK_DISCONNECTED;
static nros_millis_fn clock_ms;
static nros_connection_cb connection_cb;
static bool discover_pending;      // DISCOVER arrived while the transmitter was busy
static bool hello_pending = true;  // announce as soon as we can after boot
static uint32_t last_rx_ms;
static uint32_t last_hello_ms;
static uint32_t ack_wait_start_ms;

static uint32_t now_ms() {
    return clock_ms ? clock_ms() : 0;
}

static void set_link_state(nros_link_state_t state) {
    if (state == link_state) {
        return;
    }
    bool was_connected = link_state == LINK_CONNECTED;
    link_state = state;
    if (state == LINK_CONNECTED) {
        last_rx_ms = now_ms();
    }
    if (state == LINK_DISCONNECTED) {
        last_hello_ms = now_ms();
        hello_pending = true;
    }
    bool connected = state == LINK_CONNECTED;
    if (connection_cb && connected != was_connected) {
        connection_cb(connected);
    }
}

static nros_rx_ctx_t rx = { .state = RX_WAIT_HEADER };
static nros_tx_ctx_t tx = { .state = TX_IDLE };
static nros_registration_ctx_t registration = { 0 };
static nros_stream_parser_t parser = { 0 };

static const uint8_t CDR_ENCAP_HEADER[4] = { 0x00, 0x01, 0x00, 0x00 };

// static void load_next_registration_frame() {
//     if (tx.sending_reg_done_frame) {
//         printf("done reg\n");
//         tx.header = (nros_frame_header_t){ .magic[0] = NROS_MAGIC0, .magic[1] = NROS_MAGIC1, .type = NROS_PACKET_REGISTER_DONE, .topic_id = 0, .length = 0 };
//         tx.payload_len = 0;
//     } else {
//         printf("loading reg frame\n");
//         const nros_topic_t *topic = &nros_topics[tx.topic_registration_idx];
//         size_t type_name_len = strlen(topic->topic_type);
//         size_t topic_name_len = strlen(topic->topic_name);
//         //assert(type_len <= UINT8_MAX && name_len <= UINT8_MAX);
        
//         nros_registration_payload_t *reg = (nros_registration_payload_t *)tx.buffer;
//         reg->topic_id = topic->topic_id;
//         reg->qos = topic->qos;
//         reg->type_name_len = type_name_len;
//         reg->topic_name_len = topic_name_len;

//         uint8_t *cursor = tx.buffer + sizeof(nros_registration_payload_t);
//         memcpy(cursor, topic->topic_type, type_name_len);
//         cursor += type_name_len;
//         memcpy(cursor, topic->topic_name, topic_name_len);
//         cursor += topic_name_len;

//         tx.payload_len = cursor - tx.buffer;
//         tx.header = (nros_frame_header_t){ .magic = NROS_MAGIC, .type = NROS_PACKET_REGISTER, .topic_id = topic->topic_id, .length = tx.payload_len };
//         rx.awaiting_ack = true;
//     }
//     tx.header_bytes_sent = 0;
//     tx.payload_bytes_sent = 0;
//     tx.state = TX_SENDING_HEADER;
// }

// needs more error checking 
static void load_next_registration_frame() {
    if (tx.sending_reg_done_frame) {
        printf("done reg\n");
        tx.header.type = NROS_PACKET_REGISTER_DONE;
        tx.header.length = 0;
        tx.header.topic_id = 0;
        tx.frame_len = nros_stream_encode_frame(&tx.header, NULL, 0, tx.buffer, NROS_BUFFER_SIZE);
        if (tx.frame_len == 0) {
            printf("failed to encode register frame");
        }

        tx.bytes_sent = 0;
        tx.state = TX_SENDING_FRAME;
    } else {
        printf("loading reg frame\n");
        const nros_topic_t *topic = &nros_topics[tx.topic_registration_idx];

        size_t name_len = strlen(topic->topic_name);
        size_t type_len = strlen(topic->topic_type);

        if (type_len > NROS_MAX_STR_SIZE || name_len > NROS_MAX_STR_SIZE) {
            printf("STRING TOO BIG for topic id[%d]\n", topic->topic_id);
            return;
        }

        size_t payload_size = sizeof(nros_registration_payload_t) + name_len + type_len;
        //uint8_t payload_buf[sizeof(nros_registration_payload_t) + 2 * NROS_MAX_STR_SIZE];
        nros_registration_payload_t p = {
            .qos = topic->qos,
            .topic_id = topic->topic_id,
            .topic_name_len = name_len,
            .type_name_len = type_len,
        };
        //memcpy(payload_buf, &p, sizeof(nros_registration_payload_t));
        memcpy(tx.payload_buffer, &p, sizeof(nros_registration_payload_t));

        uint8_t *cursor = tx.payload_buffer + sizeof(nros_registration_payload_t);
        memcpy(cursor, topic->topic_type, type_len);
        cursor += type_len;
        memcpy(cursor, topic->topic_name, name_len);

        tx.header.length = payload_size;
        tx.header.topic_id = topic->topic_id;
        tx.header.type = NROS_PACKET_REGISTER;
        tx.frame_len = nros_stream_encode_frame(&tx.header, tx.payload_buffer, payload_size, tx.buffer, NROS_BUFFER_SIZE);
        if (tx.frame_len == 0) {
            printf("frame too big for topic id [%d], not sending \n", topic->topic_id);
        }
        tx.state = TX_SENDING_FRAME;
        tx.bytes_sent = 0;
        rx.awaiting_ack = true;
        ack_wait_start_ms = now_ms();
    }
}

static bool register_topics_start() {
    if (tx.state != TX_IDLE) {
        return false;
    }

    tx.is_registration = true;
    tx.topic_registration_idx = 0;
    tx.sending_reg_done_frame = nros_topic_count == 0;
    load_next_registration_frame();
    return true;
}

// static bool handle_subscription_data(uint8_t topic_id, void *data) {
//     topic_map[topic_id]->cb(data);
// }

static void tx_frame_complete() {
    if (!tx.is_registration) {
        tx.state = TX_IDLE;
        return;
    }

    if (tx.sending_reg_done_frame) {
        tx.state = TX_IDLE;
        tx.is_registration = false;
        set_link_state(LINK_CONNECTED);
        return;
    }

    // if (tx.is_registration) {
    //     if (!rx.awaiting_ack) {
    //         tx.topic_registration_idx++;
    //     }
    //     if (tx.topic_registration_idx >= nros_topic_count) {
    //         tx.sending_reg_done_frame = true;
    //     }
    //     rx.awaiting_ack = true;
    //     load_next_registration_frame();
    // }
    tx.state = TX_IDLE;
}

// static bool nros_dispatch(uint8_t topic_id, ucdrBuffer *buf) {
//     switch (rx.header.type) {
//     case NROS_PACKET_REGISTER:
//         break;
//     case NROS_PACKET_DATA:
//         topic_map[topic_id]->cb(buf);
//         break;
//     case NROS_PACKET_ACK:
//         printf("host ack: topic id[%d]", rx.header.topic_id);
//         rx.awaiting_ack = false;
//         break;
//     case NROS_PACKET_REGISTER_DONE:
//         break;
//     case NROS_PACKET_DISCOVER:
//         printf("starting registration\n");
//         register_topics_start();
//         break;
//     }
// }

static bool nros_dispatch(const nros_frame_header_t *header, const uint8_t payload[]) {
    switch (header->type) {
    case NROS_PACKET_REGISTER: {
        break;
    }
    case NROS_PACKET_DATA: {
        printf("got data for: %d\n", header->topic_id);
        for (int i = 0; i < header->length; i++) {
            printf("%hhX", payload[i]);
        }
        printf("\n");
        const nros_topic_t *topic = topic_map[header->topic_id];
        if (!topic || !topic->cb) {
            break; // not one of our subscriptions
        }
        ucdrBuffer buf;
        ucdr_init_buffer(&buf, (uint8_t *)payload, header->length);
        topic->cb(&buf);
        break;
    }
    case NROS_PACKET_ACK: {
        printf("host ack: topic id[%d]\n", header->topic_id);
        // older agents send an empty ACK, treat that as ok
        if (header->length >= sizeof(nros_ack_payload_t)) {
            const nros_ack_payload_t *ack = (const nros_ack_payload_t *)payload;
            if (ack->status != NROS_ACK_OK) {
                printf("host rejected topic id[%d]: status %d\n", ack->topic_id, ack->status);
            }
        }
        if (!rx.awaiting_ack) {
            break;
        }
        rx.awaiting_ack = false;
        if (!tx.is_registration) {
            break; // ack for not registration stuff
        }

        tx.topic_registration_idx++;
        if (tx.topic_registration_idx >= nros_topic_count) {
            tx.sending_reg_done_frame = true;
        }
        load_next_registration_frame();
        break;
    }
    case NROS_PACKET_REGISTER_DONE: {
        break;
    }
    case NROS_PACKET_DISCOVER: {
        printf("starting registration\n");
        // starts from link_poll once the transmitter is free, so a DISCOVER that arrives mid
        // publish isn't lost
        discover_pending = true;
        break;
    }
    case NROS_PACKET_PING: {
        break; // receiving it is the point
    }
    }
}

// void tx_poll() {
//     if (tx.state == TX_IDLE) {
//         return;
//     }

//     if (tx.state == TX_SENDING_HEADER) {
//         size_t n = transport->write((uint8_t *)&tx.header + tx.header_bytes_sent, sizeof(tx.header) - tx.header_bytes_sent, transport->user_data);
//         tx.header_bytes_sent += n;

//         if (tx.header_bytes_sent < sizeof(tx.header)) {
//             return;
//         }

//         if (tx.payload_len == 0) {
//             tx_frame_complete();
//             return;
//         }
//         tx.state = TX_SENDING_PAYLOAD;
//     }

//     if (tx.state == TX_SENDING_PAYLOAD) {
//         size_t n = transport->write(tx.buffer + tx.payload_bytes_sent, tx.payload_len - tx.payload_bytes_sent, transport->user_data);
//         tx.payload_bytes_sent += n;
//         if (tx.payload_bytes_sent < tx.payload_len) {
//             return;
//         }
//         tx_frame_complete();
//     }
// }

void tx_poll() {
    if (tx.state == TX_IDLE) {
        return;
    }

    if (tx.state == TX_SENDING_FRAME) {
        // debug
        //uint8_t *start = tx.buffer + tx.bytes_sent;
        size_t n = transport->write(tx.buffer + tx.bytes_sent, tx.frame_len - tx.bytes_sent, transport->user_data);
        // for (int i = 0; i < n; i++) {
        //     printf("%02hhX", start[i]);
        // }
        // printf("\n");
        // printf("wrote %d bytes\n", n);
        tx.bytes_sent += n;
        if (tx.bytes_sent < tx.frame_len) {
            return;
        }
        tx_frame_complete();
    }
}

// void rx_poll() {
//     if (rx.state == RX_WAIT_HEADER) {
//         size_t n = transport->read((uint8_t *)&rx.header + rx.bytes_have, sizeof(rx.header) - rx.bytes_have, transport->user_data);
//         rx.bytes_have += n;
//         if (rx.bytes_have < sizeof(rx.header)) {
//             return;
//         }

//         if (rx.header.magic != NROS_MAGIC || rx.header.length > NROS_BUFFER_SIZE) {
//             printf("rx broke\n");
//             rx.state = RX_WAIT_HEADER; // something went wrong, ignore and resync
//             rx.bytes_have = 0;
//             return;
//         }

//         rx.bytes_have = 0;
//         rx.state = RX_WAIT_PAYLOAD;
//     }

//     if (rx.state == RX_WAIT_PAYLOAD) {
//         size_t n = transport->read(rx.buffer + rx.bytes_have, rx.header.length - rx.bytes_have, transport->user_data);
//         rx.bytes_have += n;

//         if (rx.bytes_have < rx.header.length) {
//             return;
//         }

//         ucdrBuffer buf;
//         ucdr_init_buffer(&buf, rx.buffer, rx.header.length);
//         nros_dispatch(rx.header.topic_id, &buf);

//         rx.bytes_have = 0;
//         rx.state = RX_WAIT_HEADER;
//     } 
// }

void on_frame_cb(const nros_frame_header_t *header, const uint8_t payload[], void *user_data) {
    // ucdrBuffer buf;
    // ucdr_init_buffer(&buf, (uint8_t *)payload, header->length);
    last_rx_ms = now_ms();
    nros_dispatch(header, payload);
}

void rx_poll() {
    uint8_t chunk[64];
    size_t n = transport->read(chunk, sizeof(chunk), transport->user_data);
    if (n > 0) {
        nros_stream_parser_feed(&parser, chunk, n, on_frame_cb, NULL);
    }
}

static void send_hello() {
    tx.header = (nros_frame_header_t) { .type = NROS_PACKET_HELLO, .topic_id = 0, .length = 0 };
    tx.frame_len = nros_stream_encode_frame(&tx.header, NULL, 0, tx.buffer, NROS_BUFFER_SIZE);
    if (tx.frame_len == 0) {
        return;
    }
    tx.bytes_sent = 0;
    tx.is_registration = false;
    tx.state = TX_SENDING_FRAME;
}

// Moves the link along: starts registration on DISCOVER, announces while unregistered, and
// times out a stalled registration or a missing agent (the timeouts need nros_set_clock)
static void link_poll() {
    uint32_t now = now_ms();

    // timeouts first: a dead link can leave the transmitter stuck, so these can't wait for it
    if (clock_ms && link_state == LINK_REGISTERING && rx.awaiting_ack
        && now - ack_wait_start_ms >= NROS_ACK_TIMEOUT_MS) {
        printf("registration timed out\n");
        rx.awaiting_ack = false;
        tx.is_registration = false;
        set_link_state(LINK_DISCONNECTED);
    }
    if (clock_ms && link_state == LINK_CONNECTED && now - last_rx_ms >= NROS_AGENT_TIMEOUT_MS) {
        printf("lost the agent\n");
        tx.state = TX_IDLE; // drop whatever was stuck, the agent resyncs on the next frame
        set_link_state(LINK_DISCONNECTED);
    }

    if (tx.state != TX_IDLE) {
        return;
    }
    if (discover_pending) {
        discover_pending = false;
        set_link_state(LINK_REGISTERING);
        register_topics_start();
        return;
    }
    if (link_state == LINK_DISCONNECTED
        && (hello_pending || (clock_ms && now - last_hello_ms >= NROS_HELLO_PERIOD_MS))) {
        hello_pending = false;
        last_hello_ms = now;
        send_hello();
    }
}

void nros_spin() {
    rx_poll();
    tx_poll();
    link_poll();
}

void nros_set_clock(nros_millis_fn millis) {
    clock_ms = millis;
    last_rx_ms = last_hello_ms = ack_wait_start_ms = millis();
}

bool nros_is_connected(void) {
    return link_state == LINK_CONNECTED;
}

void nros_set_connection_cb(nros_connection_cb cb) {
    connection_cb = cb;
}

bool nros_init(nros_transport_t *t, bool use_nros_framing) {
    // do error checking
    transport = t;

    nros_stream_parser_init(&parser);

    bool init_ok = transport->init(t->user_data);
    bool open = transport->open();
    if (!open || !init_ok) {
        return false;
    }

    for (uint8_t i = 0; i < nros_topic_count; i++) {
        topic_map[nros_topics[i].topic_id] = &nros_topics[i];
    }

    return true;
}

bool nros_publish(uint8_t topic_id, nros_serialize_fn fn, const void* msg) {
    if (tx.state != TX_IDLE || link_state != LINK_CONNECTED) {
        return false;
    }

    ucdrBuffer buf;
    ucdr_init_buffer(&buf, tx.payload_buffer, NROS_BUFFER_SIZE);

    if (!ucdr_serialize_array_uint8_t(&buf, CDR_ENCAP_HEADER, sizeof(CDR_ENCAP_HEADER))) { // write in encapsulation header
        return false;
    }

    if (!fn(&buf, msg)) {
        return false;
    }

    size_t payload_len = ucdr_buffer_length(&buf);
    tx.header = (nros_frame_header_t) {
        .length = payload_len,
        .topic_id = topic_id,
        .type = NROS_PACKET_DATA,
    };

    tx.frame_len = nros_stream_encode_frame(&tx.header, tx.payload_buffer, payload_len, tx.buffer, NROS_BUFFER_SIZE);
    if (tx.frame_len == 0) {
        printf("publish frame too big for topic id[%d], not sending", topic_id);
        return false;
    }
    
    tx.bytes_sent = 0;
    tx.is_registration = false;
    tx.state = TX_SENDING_FRAME;

    // tx.payload_len = ucdr_buffer_length(&buf);
    // tx.header = (nros_frame_header_t){ .magic[0] = NROS_MAGIC0, .magic[1] = NROS_MAGIC1 .type = NROS_PACKET_DATA, .topic_id = topic_id, .length = tx.payload_len };
    // tx.header_bytes_sent = 0;
    // tx.payload_bytes_sent = 0;
    // tx.is_registration = false;
    // tx.state = TX_SENDING_HEADER;

    printf("serialized bytes: ");
    for (int i = 0; i < payload_len; i++) {
        printf("%02hhX", tx.payload_buffer[i]);
    }
    printf("\n");

    printf("successful pub\n");

    return true;
}
