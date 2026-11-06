#include "client_handler.hpp"
extern "C" {
#include "nros_stream_parser.h"
}

using namespace nros;

// maps the wire QoS byte to an rclcpp profile. the agent rejects values >= NROS_QOS_COUNT at
// registration, so anything unknown here falls back to the default profile
static rclcpp::QoS qos_from_wire(uint8_t qos) {
    switch (static_cast<nros_qos_t>(qos)) {
    case NROS_QOS_SENSOR_DATA:
        return rclcpp::SensorDataQoS();
    case NROS_QOS_RELIABLE:
        return rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    case NROS_QOS_BEST_EFFORT:
        return rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    case NROS_QOS_TRANSIENT_LOCAL:
        return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    case NROS_QOS_SYSTEM_DEFAULT:
        return rclcpp::SystemDefaultsQoS();
    case NROS_QOS_DEFAULT:
    default:
        return rclcpp::QoS(rclcpp::KeepLast(10));
    }
}

void ClientHandlerNode::on_client_connect(const std::vector<Topic> &topics) {
    for (Topic t : topics) {
        topics_[t.id] = t;
        rclcpp::QoS qos = qos_from_wire(t.qos);
        if (t.id >= PUBLISHER_TOPIC_ID_START) {
            topics_[t.id].handle = create_generic_publisher(t.topic_name, t.msg_type, qos);
        } else {
            topics_[t.id].handle = create_generic_subscription(t.topic_name, 
                t.msg_type,
                qos,
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
    const auto &byte_buf = msg->get_rcl_serialized_message();

    constexpr size_t ENCAP_HEADER_SIZE = 4;
    if (byte_buf.buffer_length < ENCAP_HEADER_SIZE) {
        return;  // malformed, nothing to forward
    }

    std::vector<uint8_t> bytes(byte_buf.buffer + ENCAP_HEADER_SIZE, byte_buf.buffer + byte_buf.buffer_length);

    OutboundMsg out;
    out.client_id = client_id;
    out.frame.header.topic_id = topic_id;
    out.frame.header.type = NROS_PACKET_DATA;
    out.frame.header.length = bytes.size();   // length of the stripped payload
    out.frame.payload = std::move(bytes);
    cb_data_queue_.push(out);
}

void ClientHandlerNode::client_publish(WireFrame &frame) {
    // c++ is not a good language
    if (auto *pub = std::get_if<rclcpp::GenericPublisher::SharedPtr>(&topics_[frame.header.topic_id].handle)) {
        rclcpp::SerializedMessage msg(frame.payload.size());
        auto &byte_buf = msg.get_rcl_serialized_message();
        memcpy(byte_buf.buffer, frame.payload.data(), frame.payload.size());
        byte_buf.buffer_length = frame.payload.size();
        (*pub)->publish(msg);
    }
}
