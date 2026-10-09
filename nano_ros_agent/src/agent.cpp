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
// using for testing at the moment, should not be built upon in this state
// void Agent::run() {
//     this->transport->init();
//     this->running = true;
//     bool has_discovered = false;
//     bool needs_ack = false;
//     uint8_t topic_to_ack = 0;
//     bool can_sub = false;
//     size_t sub_ticker = 0;

//     uint8_t ucdr_serialized_byte = 0x65; // 101

//     WireFrame frame;
//     while (this->running) {
//         Readiness r = transport->wait_ready(0);
//         if (r.error) {
//             fprintf(stderr, "error\n");
//             break;
//         }

//         if (r.writable) {
//             //fprintf(stdout, "writable\n");
//             if (!has_discovered) {
//                 frame.header.magic[0] = NROS_MAGIC0;
//                 frame.header.magic[1] = NROS_MAGIC1;
//                 frame.header.type = NROS_PACKET_DISCOVER;
//                 frame.header.topic_id = 0;
//                 frame.header.length = 0;
//                 bool sent = transport->send_frame(frame, 0);
//                 if (!sent) {
//                     fprintf(stderr, "problem sending discovery\n");
//                 } else { 
//                     fprintf(stderr, "sent discover\n");
//                     has_discovered = true;
//                 }
//             }

//             if (needs_ack) {
//                 needs_ack = false;
//                 frame.header.length = 0;
//                 frame.header.type = NROS_PACKET_ACK;
//                 frame.header.topic_id = topic_to_ack;
//                 frame.payload.clear();
//                 transport->send_frame(frame, 0);

//                 fprintf(stdout, "sent ack\n\n");
//             }

//             if (can_sub && sub_ticker > 5) {
//                 sub_ticker = 0;
//                 frame.header.type = NROS_PACKET_DATA;
//                 frame.header.length = 1;
//                 frame.header.topic_id = 0;
//                 frame.payload.clear();
//                 frame.payload.push_back(ucdr_serialized_byte);
//                 transport->send_frame(frame, 0);

//                 fprintf(stdout, "sent sub data\n");
//             }
//         }

//         if (r.readable) {
//             if (transport->recv_frame(frame, 0)) {
//                 uint8_t id = frame.header.topic_id;
//                 uint8_t type = frame.header.type;
//                 uint8_t length = frame.header.length;
//                 fprintf(stdout, "HEADER: id[%hhu], type[%hhu], length[%hhu]\n", id, type, length);
//                 switch (frame.header.type) {
//                 case NROS_PACKET_DATA:
//                     fprintf(stdout, "data from board: ");
//                     for (uint8_t byte : frame.payload) {
//                         fprintf(stdout, "%02hhX", byte);
//                     }
//                     fprintf(stdout, "\n");
//                     break;
//                 case NROS_PACKET_REGISTER:
//                     fprintf(stdout, "registration frame\n");
//                     needs_ack = true;
//                     topic_to_ack = frame.header.topic_id;
//                     decode_reg_frame(frame);
//                     break;
//                 case NROS_PACKET_REGISTER_DONE:
//                     fprintf(stdout, "board done registering\n");
//                     can_sub = true;
//                     break;
//                 }
//                 if (can_sub) {
//                     sub_ticker++;
//                 }
//                 fprintf(stdout, "\n");
//             }
//         }
//     }
// }

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
        printf("publishing\n");
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