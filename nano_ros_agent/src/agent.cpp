#include "agent.hpp"
#include <poll.h>
#include <stdio.h>
#include <string.h>

using namespace nros;

Agent::Agent(std::unique_ptr<Transport> transport) : transport_(std::move(transport)) {}

void decode_reg_frame(WireFrame &frame) {
    // for (size_t i = sizeof(nros_registration_payload_t); i < frame.payload.size(); i++) {
    //     fprintf(stdout, "%c", frame.payload[i]);
    // }
    // fprintf(stdout, "\n");

    uint8_t *bytes = frame.payload.data();
    nros_registration_payload_t *payload = reinterpret_cast<nros_registration_payload_t *>(bytes);
    uint8_t name_len = payload->topic_name_len;
    uint8_t type_len = payload->type_name_len;
    uint8_t *cursor = reinterpret_cast<uint8_t *>(payload);
    cursor += sizeof(nros_registration_payload_t);
    std::string name(cursor, cursor + type_len);
    cursor += type_len;
    std::string type(cursor, cursor + name_len);

    fprintf(stdout, "\t topic name: [%s]\n\ttopic msg type: [%s]", name.c_str(), type.c_str());
}

Topic Agent::decode_registration_frame(WireFrame &frame) {
    Topic t;

    uint8_t *bytes = frame.payload.data();
    nros_registration_payload_t *payload = reinterpret_cast<nros_registration_payload_t *>(bytes);
    uint8_t name_len = payload->topic_name_len;
    uint8_t type_len = payload->type_name_len;

    uint8_t *cursor = reinterpret_cast<uint8_t *>(payload);
    cursor += sizeof(nros_registration_payload_t);
    std::string type(cursor, cursor + type_len);
    cursor += type_len;
    std::string name(cursor, cursor + name_len);

    t.msg_type = type;
    t.topic_name = name;
    t.id = frame.header.topic_id;

    return t;
}

void Agent::ack_client() {
    WireFrame ack;
    ack.header.type = NROS_PACKET_ACK;
    ack.header.length = 0;
    ack.header.topic_id = ack_.topic_to_ack_id;
    transport_->send_frame(ack, 0);
    printf("acked\n");
}

void Agent::run() {
    transport_->init();
    running_ = true;
    bool need_registration = true;
    bool ack_topic_register = false;

    WireFrame frame;
    while (running_) {
        Readiness r = transport_->wait_ready(0);

        if (r.readable) {
            if (transport_->recv_frame(frame, 0)) {
                frame_dispatch(frame);
            }
        }

        if (r.writable) {
            if (need_registration) {
                need_registration = false;
                frame.header.type = NROS_PACKET_DISCOVER;
                frame.header.topic_id = 0;
                frame.header.length = 0;
                transport_->send_frame(frame, 0);
            }
            if (to_client_msg_queue_.size() > 0) {
                OutboundMsg msg = to_client_msg_queue_.wait_and_pop();
                printf("message header: type: %d, topic_id: %d\n", msg.frame.header.type, msg.frame.header.topic_id);
                transport_->send_frame(msg.frame, 0);
            }

            if (ack_.need_ack) {
                ack_client();
                ack_.need_ack = false;
            }
        }
    }
}

void Agent::frame_dispatch(WireFrame &frame) {
    switch (frame.header.type) {

    case NROS_PACKET_DATA: {
        client_->client_publish(frame);
        break;
    }

    case NROS_PACKET_REGISTER: {
        Topic t = decode_registration_frame(frame);
        ack_.need_ack = true;
        ack_.topic_to_ack_id = t.id;
        printf("topic name: %s\n", t.topic_name.c_str());
        printf("topic type: %s\n", t.msg_type.c_str());
        printf("topic id: %hhu\n", t.id);
        new_topics_.push_back(t);
        break;
    }

    case NROS_PACKET_REGISTER_DONE: {
        spawn_client(false); // determine if theres multiple clients for real
        break;
    }

    }
}

void Agent::spawn_client(bool expect_multiple_clients) {
    std::string node_name;
    if (expect_multiple_clients) {
        node_name = "nros_node_client_id=" + std::to_string(client_id_idx_);
    } else {
        node_name = "nros_client_node";
    }
    auto options = rclcpp::NodeOptions();
    // ClientHandlerNode node(node_name, options, to_client_msg_queue_)
    //clients_.push_back(std::move(std::make_shared<ClientHandlerNode>(node_name, options, to_client_msg_queue_)));
    options.use_intra_process_comms(true);
    client_ = std::make_shared<ClientHandlerNode>(node_name, options, to_client_msg_queue_);
    client_->on_client_connect(new_topics_);
}

void Agent::stop() {

}

Agent::~Agent() {

}