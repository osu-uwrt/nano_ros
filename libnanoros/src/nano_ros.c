#include "nano_ros.h"
#include "protocol.h"
#include <stddef.h>
#include <string.h>
#include <assert.h>

#define NROS_BUFFER_SIZE 1024
#define NROS_TOPIC_ALLOC_SIZE 256
#define NROS_MAX_STR_SIZE 128

// typedef enum {
//     NROS_PACKET_REGISTER = 0x01, // register topics or reregister if coming from host
//     NROS_PACKET_DATA     = 0x02, // pub / sub data
//     NROS_PACKET_ACK      = 0x03, // with status
//     NROS_PACKET_REGISTER_DONE = 0x04, // marks registration as done
// } nros_packet_type_t;

typedef enum {
    RX_WAIT_HEADER,
    RX_WAIT_PAYLOAD,
} nros_rx_state_t;

typedef enum {
    TX_IDLE,
    TX_SENDING_HEADER,
    TX_SENDING_PAYLOAD,
} nros_tx_state_t;

// typedef struct {
//     nros_packet_type_t type;
//     uint8_t topic_id;
//     uint16_t length;
// } __attribute__((packed)) nros_frame_header_t;

// typedef struct {
//     uint8_t topic_id;
//     uint8_t qos;
//     uint8_t type_name_len;
//     uint8_t topic_name_len;
//     // type name and topic name are strings appended after this on the wire
//     // char topic_type[NROS_MAX_STR_SIZE];
//     // char topic_name[NROS_MAX_STR_SIZE];
// } __attribute__((packed)) nros_registration_payload_t;

// typedef struct {
//     uint8_t topic_id;    
//     uint8_t status;      // 0 = ok, nonzero = error code (name collision, unsupported qos, etc.)
// } __attribute__((packed)) nros_ack_payload_t;

typedef struct {
    uint8_t topic_idx;
    bool need_registration;

} nros_registration_ctx_t;

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
} nros_tx_ctx_t;

typedef struct {
    nros_rx_state_t state;
    nros_frame_header_t header;
    size_t bytes_have;
    uint8_t buffer[NROS_BUFFER_SIZE];
} nros_rx_ctx_t;

static nros_transport_t *transport;
static nros_topic_t const *topic_map[NROS_TOPIC_ALLOC_SIZE] = { 0 };

static nros_rx_ctx_t rx = { .state = RX_WAIT_HEADER };
static nros_tx_ctx_t tx = { .state = TX_IDLE };

static void nros_send_frame(nros_packet_type_t packet_type, uint8_t topic_id, uint8_t msg[], uint16_t msg_len) {
    switch (packet_type) {
    case NROS_PACKET_REGISTER:
        break;
    case NROS_PACKET_DATA:
        break;
    case NROS_PACKET_ACK:
        break;
    case NROS_PACKET_REGISTER_DONE:
        break;
    }
}


static void load_next_registration_frame() {
    if (tx.sending_reg_done_frame) {
        tx.header = (nros_frame_header_t){ .type = NROS_PACKET_REGISTER_DONE, .topic_id = 0, .length = 0 };
        tx.payload_len = 0;
    } else {
        const nros_topic_t *topic = &nros_topics[tx.topic_registration_idx];
        size_t type_name_len = strlen(topic->topic_type);
        size_t topic_name_len = strlen(topic->topic_name);
        assert(type_len <= UINT8_MAX && name_len <= UINT8_MAX);
        
        nros_registration_payload_t *reg = (nros_registration_payload_t *)tx.buffer;
        reg->topic_id = topic->topic_id;
        reg->qos = topic->qos;
        reg->type_name_len = type_name_len;
        reg->topic_name_len = topic_name_len;

        uint8_t *cursor = tx.buffer + sizeof(nros_registration_payload_t);
        memcpy(cursor, topic->topic_type, type_name_len);
        cursor += type_name_len;
        memcpy(cursor, topic->topic_name, topic_name_len);
        cursor += topic_name_len;

        tx.payload_len = cursor - tx.buffer;
        tx.header = (nros_frame_header_t){ .type = NROS_PACKET_REGISTER, .topic_id = topic->topic_id, .length = tx.payload_len };
    }
    tx.header_bytes_sent = 0;
    tx.payload_bytes_sent = 0;
    tx.state = TX_SENDING_HEADER;
}

static bool register_topics_start() {
    if (tx.state != TX_IDLE) {
        return false;
    }

    tx.is_registration = true;
    tx.topic_registration_idx = 0;
    tx.sending_reg_done_frame = false;
    load_next_registration_frame();
    return true;
}

static bool handle_subscription_data(uint8_t topic_id, void *data) {
    topic_map[topic_id]->cb(data);
}

static void tx_frame_complete() {
    if (!tx.is_registration) {
        tx.state = TX_IDLE;
        return;
    }

    if (tx.sending_reg_done_frame) {
        tx.state = TX_IDLE;
        return;
    }

    tx.topic_registration_idx++;
    if (tx.topic_registration_idx >= nros_topic_count) {
        tx.sending_reg_done_frame = true;
    }

    load_next_registration_frame();
}

bool nros_dispatch(uint8_t topic_id, ucdrBuffer *buf) {

}

void tx_poll() {
    if (tx.state == TX_IDLE) {
        return;
    }

    if (tx.state == TX_SENDING_HEADER) {
        size_t n = transport->write((uint8_t *)&tx.header + tx.header_bytes_sent, sizeof(tx.header) - tx.header_bytes_sent, transport->user_data);
        tx.header_bytes_sent += n;

        if (tx.header_bytes_sent < sizeof(tx.header)) {
            return;
        }

        if (tx.payload_len == 0) {
            tx_frame_complete();
            return;
        }
        tx.state = TX_SENDING_PAYLOAD;
    }

    if (tx.state == TX_SENDING_PAYLOAD) {
        size_t n = transport->write(tx.buffer + tx.payload_bytes_sent, tx.payload_len - tx.payload_bytes_sent, transport->user_data);
        tx.payload_bytes_sent += n;
        if (tx.payload_bytes_sent < tx.payload_len) {
            return;
        }
        tx_frame_complete();
    }
}

void rx_poll() {
    if (rx.state == RX_WAIT_HEADER) {
        size_t n = transport->read((uint8_t *)&rx.header + rx.bytes_have, sizeof(rx.header) - rx.bytes_have, transport->user_data);
        rx.bytes_have += n;
        if (rx.bytes_have < sizeof(rx.header)) {
            return;
        }

        if (rx.header.length > NROS_BUFFER_SIZE) {
            rx.state = RX_WAIT_HEADER; // something went wrong, ignore and resync
            rx.bytes_have = 0;
            return;
        }

        rx.bytes_have = 0;
        rx.state = RX_WAIT_PAYLOAD;
    }

    if (rx.state == RX_WAIT_PAYLOAD) {
        size_t n = transport->read(rx.buffer + rx.bytes_have, rx.header.length - rx.bytes_have, transport->user_data);
        rx.bytes_have += n;

        if (rx.bytes_have < rx.header.length) {
            return;
        }

        ucdrBuffer buf;
        ucdr_init_buffer(&buf, rx.buffer, rx.header.length);
        nros_dispatch(rx.header.topic_id, &buf);

        rx.bytes_have = 0;
        rx.state = RX_WAIT_HEADER;
    } 
}

void nros_spin() {
    rx_poll();
    tx_poll();
}

bool nros_init(nros_transport_t *t) {
    // do error checking
    transport = t;
    bool open = transport->open();
    if (!open) {
        return false;
    }

    for (uint8_t i = 0; i < nros_topic_count; i++) {
        topic_map[nros_topics[i].topic_id] = &nros_topics[i];
    }
    // tell host about the topics
    return true;
}

bool nros_publish(uint8_t topic_id, nros_serialize_fn fn, const void* msg) {
    if (tx.state != TX_IDLE) {
        return false;
    }

    ucdrBuffer buf;
    ucdr_init_buffer(&buf, tx.buffer, ucdr_buffer_length(&buf)); // tx.buffer + 4 then state endianness in those prev 4 bytes
    if (!fn(&buf, msg)) {
        return false;
    }

    tx.payload_len = ucdr_buffer_length(&buf);
    tx.header = (nros_frame_header_t){ .type = NROS_PACKET_DATA, .topic_id = topic_id, .length = tx.payload_len };
    tx.header_bytes_sent = 0;
    tx.payload_bytes_sent = 0;
    tx.is_registration = false;
    tx.state = TX_SENDING_HEADER;

    return true;
}
