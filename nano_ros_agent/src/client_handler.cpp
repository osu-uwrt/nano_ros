#include "client_handler.hpp"
extern "C" {
#include "nros_stream_parser.h"
}

using namespace nros;

// make qos configurable firmware side and accept it here
void ClientHandlerNode::on_client_connect(const std::vector<Topic> &topics) {
    for (Topic t : topics) {
        topics_[t.id] = t;
        if (t.id > 127) {
            topics_[t.id].handle = create_generic_publisher(t.topic_name, t.msg_type, rclcpp::SystemDefaultsQoS());
        } else {
            topics_[t.id].handle = create_generic_subscription(t.topic_name, 
                t.msg_type,
                rclcpp::SystemDefaultsQoS(), // make configurable client side
                [this, client_id = client_id_, topic_id = topics_[t.id].id]
                (std::shared_ptr<rclcpp::SerializedMessage> msg) {
                    this->client_subscription_callback(client_id, topic_id, msg);
                }
            );
        }
        topics_[t.id].active = true;
    }
}

void ClientHandlerNode::client_subscription_callback(uint8_t client_id, uint8_t topic_id, std::shared_ptr<rclcpp::SerializedMessage> msg) {
    OutboundMsg out;
    out.client_id = client_id;
    out.frame.header.topic_id = topic_id;
    out.frame.header.type = NROS_PACKET_DATA;
    out.frame.header.length = msg->size();
    const auto &byte_buf = msg->get_rcl_serialized_message();
    std::vector<uint8_t> bytes(byte_buf.buffer, byte_buf.buffer + byte_buf.buffer_length);
    out.frame.payload = std::move(bytes);
    cb_data_queue_.push(out);
}

void ClientHandlerNode::client_publish(WireFrame &frame) {
    // c++ is not a good language
    if (auto *pub = std::get_if<rclcpp::GenericPublisher::SharedPtr>(&topics_[frame.header.topic_id].handle)) {
        rclcpp::SerializedMessage msg(frame.payload.size());
        auto &byte_buf = msg.get_rcl_serialized_message();
        memcpy(byte_buf.buffer, frame.payload.data(), frame.payload.size());
        (*pub)->publish(msg);
    }
}
