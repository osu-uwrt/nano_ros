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

void Agent::ack_client() {
    nros_ack_payload_t p;
    p.topic_id = ack_.topic_to_ack_id;
    p.status = ack_.status;

    WireFrame ack;
    ack.header.type = NROS_PACKET_ACK;
    ack.header.topic_id = ack_.topic_to_ack_id;
    ack.payload.assign(reinterpret_cast<uint8_t *>(&p), reinterpret_cast<uint8_t *>(&p) + sizeof(p));
    ack.header.length = ack.payload.size();
    transport_->send_frame(ack, 0);
    printf("acked topic %hhu with status %hhu\n", p.topic_id, p.status);
}

// void Agent::run() {
//     transport_->init();
//     running_ = true;
//     bool need_registration = true;
//     bool ack_topic_register = false;

//     WireFrame frame;
//     while (running_) {
//         Readiness r = transport_->wait_ready(0);

//         if (r.readable) {
//             if (transport_->recv_frame(frame, 0)) {
//                 frame_dispatch(frame);
//             }
//         }

//         if (r.writable) {
//             if (need_registration) {
//                 need_registration = false;
//                 frame.header.type = NROS_PACKET_DISCOVER;
//                 frame.header.topic_id = 0;
//                 frame.header.length = 0;
//                 transport_->send_frame(frame, 0);
//             }
//             if (to_client_msg_queue_.size() > 0) {
//                 OutboundMsg msg = to_client_msg_queue_.wait_and_pop();
//                 printf("message header: type: %d, topic_id: %d\n", msg.frame.header.type, msg.frame.header.topic_id);
//                 transport_->send_frame(msg.frame, 0);
//             }

//             if (ack_.need_ack) {
//                 ack_client();
//                 ack_.need_ack = false;
//             }
//         }
//     }
// }

void Agent::run() {
    transport_->init();
    running_ = true;
    bool need_registration = true;
    bool ack_topic_register = false;

    WireFrame frame;
    while (running_) {
        //Readiness r = transport_->wait_ready(0);

        if (transport_->recv_frame(frame, 0)) {
            frame_dispatch(frame);
        }

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

void Agent::frame_dispatch(WireFrame &frame) {
    switch (frame.header.type) {

    case NROS_PACKET_DATA: {
        // the client node only exists after REGISTER_DONE
        if (!client_) {
            fprintf(stderr, "dropping data for topic id %hhu: client has not finished registering\n",
                    frame.header.topic_id);
            break;
        }
        client_->client_publish(frame);
        break;
    }

    case NROS_PACKET_REGISTER: {
        Topic t;
        nros_ack_status_t status = decode_registration_frame(frame, t);
        ack_.need_ack = true;
        ack_.topic_to_ack_id = frame.header.topic_id;
        ack_.status = status;
        if (status != NROS_ACK_OK) {
            fprintf(stderr, "rejecting registration for topic id %hhu: status %d (qos %hhu)\n",
                    frame.header.topic_id, status, t.qos);
            break;
        }
        printf("topic name: %s\n", t.topic_name.c_str());
        printf("topic type: %s\n", t.msg_type.c_str());
        printf("topic id: %hhu\n", t.id);
        printf("topic qos: %hhu\n", t.qos);
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