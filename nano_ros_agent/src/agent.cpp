#include "agent.hpp"
#include <poll.h>
#include <stdio.h>
#include <string.h>

using namespace nros;

Agent::Agent(std::unique_ptr<Transport> transport) : transport_(std::move(transport)) {}

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

// Asks a board (or every board, with PEER_BROADCAST) to register from scratch
void Agent::send_discover(PeerID to) {
    WireFrame frame;
    frame.header.type = NROS_PACKET_DISCOVER;
    frame.header.topic_id = 0;
    frame.header.length = 0;
    transport_->send_frame(to, frame, 0);
}

// Lets a registered board know the agent is still here; it reconnects if these stop
void Agent::send_ping(PeerID to) {
    WireFrame frame;
    frame.header.type = NROS_PACKET_PING;
    frame.header.topic_id = 0;
    frame.header.length = 0;
    transport_->send_frame(to, frame, 0);
}

static constexpr std::chrono::milliseconds PING_PERIOD{500};

bool Agent::run() {
    if (!transport_->init()) {
        return false;
    }
    running_ = true;
    send_discover(PEER_BROADCAST);
 
    PeerID from;
    WireFrame frame;
    while (running_ && rclcpp::ok()) {
        if (transport_->recv_frame(from, frame, 0)) {
            frame_dispatch(from, frame);
        }

        auto now = std::chrono::steady_clock::now();
        if (now - last_ping_ >= PING_PERIOD) {
            last_ping_ = now;
            for (auto &[peer, s] : sessions_) {
                if (s.node) { // only registered boards watch for pings
                    send_ping(peer);
                }
            }
        }
 
        // only this thread pops, so size() > 0 means wait_and_pop won't block
        while (to_client_msg_queue_.size() > 0) {
            DeviceMessage msg = to_client_msg_queue_.wait_and_pop();
            transport_->send_frame(msg.peer, msg.frame, 0);
        }
    }
 
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

    case NROS_PACKET_HELLO: {
        // the board isn't registered (it reset, or lost us): drop what we had for it and start over
        printf("(peer %u) says hello, asking it to register\n", from);
        sessions_.erase(from);
        send_discover(from);
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
}

Agent::~Agent() {
    stop();
}