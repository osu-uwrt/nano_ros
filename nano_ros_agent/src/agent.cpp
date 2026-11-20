#include "agent.hpp"
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <chrono>
#include <thread>

using namespace nros;

// how long run() sleeps with nothing to do before it checks running_ again
static constexpr int32_t IDLE_WAIT_MS = 100;

Agent::Agent(std::unique_ptr<Transport> transport) : transport_(std::move(transport)) {
    // subscriptions queue messages for the client from the executor thread, so wake run() up to send them
    to_client_msg_queue_.set_on_push([this] { transport_->wake(); });
}

nros_ack_status_t Agent::decode_registration_frame(WireFrame &frame, Topic &t) {
    if (frame.payload.size() < sizeof(nros_registration_payload_t)) {
        return NROS_ACK_MALFORMED;
    }

    nros_registration_payload_t payload;
    memcpy(&payload, frame.payload.data(), sizeof(payload));

    if (frame.payload.size() < sizeof(payload) + payload.type_name_len + payload.topic_name_len) {
        return NROS_ACK_MALFORMED;
    }

    const uint8_t *cursor = frame.payload.data() + sizeof(payload);
    t.msg_type.assign(cursor, cursor + payload.type_name_len);
    cursor += payload.type_name_len;
    t.topic_name.assign(cursor, cursor + payload.topic_name_len);

    t.id = frame.header.topic_id;
    t.qos = payload.qos;

    if (payload.qos >= NROS_QOS_COUNT) {
        return NROS_ACK_UNSUPPORTED_QOS;
    }

    return NROS_ACK_OK;
}

void Agent::send_ack(PeerID to, uint8_t topic_id, nros_ack_status_t status) {
    nros_ack_payload_t p;
    p.topic_id = topic_id;
    p.status = status;
 
    WireFrame ack;
    ack.header.type = NROS_PACKET_ACK;
    ack.header.topic_id = topic_id;
    ack.payload.assign(reinterpret_cast<uint8_t *>(&p), reinterpret_cast<uint8_t *>(&p) + sizeof(p));
    ack.header.length = ack.payload.size();
    transport_->send_frame(to, ack, 0);
    printf("(peer %u) acked topic %hhu with status %hhu\n", to, p.topic_id, p.status);
}

void Agent::send_discover() {
    WireFrame frame;
    frame.header.type = NROS_PACKET_DISCOVER;
    frame.header.topic_id = 0;
    frame.header.length = 0;
    transport_->send_frame(PEER_BROADCAST, frame, 0);
}

bool Agent::run() {
    if (!transport_->init()) {
        return false;
    }
    running_ = true;
    send_discover();

    // ctrl-c shuts rclcpp down from its signal thread, so wake the loop instead of waiting out IDLE_WAIT_MS
    auto context = rclcpp::contexts::get_global_default_context();
    auto on_shutdown = context->add_on_shutdown_callback([this] { stop(); });
 
    PeerID from;
    WireFrame frame;
    DeviceMessage msg;
    while (running_ && rclcpp::ok()) {
        // sleeps until a client sends something, a subscription queues a message, stop(), or the timeout
        Readiness r = transport_->wait_ready(IDLE_WAIT_MS);
        if (r.error) {
            // nothing reconnects the transport yet, so don't spin on a dead fd
            std::this_thread::sleep_for(std::chrono::milliseconds(IDLE_WAIT_MS));
        }

        while (running_ && transport_->recv_frame(from, frame, 0)) {
            frame_dispatch(from, frame);
        }
 
        // after reading, since reading can use up a wake()
        while (to_client_msg_queue_.try_pop(msg)) {
            transport_->send_frame(msg.peer, msg.frame, 0);
        }
    }
 
    context->remove_on_shutdown_callback(on_shutdown);
    running_ = false;
    sessions_.clear(); // stop the client nodes before the transport goes away
    transport_->fini();
    return true;
}

void Agent::frame_dispatch(PeerID from, WireFrame &frame) {
    switch (frame.header.type) {
 
    case NROS_PACKET_DATA: {
        auto it = sessions_.find(from);
        if (it == sessions_.end() || !it->second.node) {
            fprintf(stderr, "(peer %u) data on topic %hhu before registration finished, dropping\n",
                    from, frame.header.topic_id);
            break;
        }
        it->second.node->client_publish(frame);
        break;
    }
 
    case NROS_PACKET_REGISTER: {
        Session &s = sessions_[from]; // first contact creates the session
 
        // a REGISTER while a node exists means the board rebooted: start over
        if (s.node) {
            printf("(peer %u) re-registering, tearing down old node\n", from);
            s.node.reset();
            s.pending_topics.clear();
        }
 
        Topic t;
        nros_ack_status_t status = decode_registration_frame(frame, t);
        send_ack(from, frame.header.topic_id, status);
        if (status != NROS_ACK_OK) {
            fprintf(stderr, "(peer %u) rejecting registration for topic id %hhu: status %d (qos %hhu)\n",
                    from, frame.header.topic_id, status, t.qos);
            break;
        }
        printf("(peer %u) topic %hhu: %s [%s] qos %hhu\n",
               from, t.id, t.topic_name.c_str(), t.msg_type.c_str(), t.qos);
        s.pending_topics.push_back(std::move(t));
        break;
    }
 
    case NROS_PACKET_REGISTER_DONE: {
        auto it = sessions_.find(from);
        if (it == sessions_.end()) {
            fprintf(stderr, "(peer %u) REGISTER_DONE with no registrations, ignoring\n", from);
            break;
        }
        spawn_client(from, it->second);
        break;
    }
 
    default:
        fprintf(stderr, "(peer %u) unknown packet type %hhu\n", from, frame.header.type);
        break;
    }
}

void Agent::spawn_client(const PeerID peer, Session &s) {
    s.node.reset(); // destroy any old node first so the new one doesn't clash on the name
 
    std::string node_name = "peer_" + std::to_string(peer);
 
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    s.node = std::make_shared<ClientHandlerNode>(node_name, peer, options, to_client_msg_queue_);
    s.node->on_client_connect(s.pending_topics);
    s.pending_topics.clear();
    printf("(peer %u) spawned node nros_client_%s\n", peer, node_name.c_str());
}

void Agent::stop() {
    running_ = false;
    transport_->wake();
}

Agent::~Agent() {
    stop();
}