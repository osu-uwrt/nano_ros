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

static const char *reset_reason_name(uint8_t reason) {
    switch (static_cast<nros_reset_reason_t>(reason)) {
    case NROS_RESET_POWER_ON: return "power on";
    case NROS_RESET_WATCHDOG: return "watchdog";
    case NROS_RESET_BROWNOUT: return "brownout";
    case NROS_RESET_SOFTWARE: return "software";
    case NROS_RESET_PIN: return "reset pin";
    case NROS_RESET_DEBUGGER: return "debugger";
    default: return "unknown";
    }
}

// Turns a board's status report into a /diagnostics entry, so it shows up in rqt_robot_monitor
void ClientHandlerNode::publish_status(const WireFrame &frame) {
    nros_status_payload_t p;
    if (frame.payload.size() < sizeof(p)) {
        return;
    }
    memcpy(&p, frame.payload.data(), sizeof(p));
    if (frame.payload.size() < sizeof(p) + p.board_name_len + p.version_len) {
        return;
    }
    const uint8_t *cursor = frame.payload.data() + sizeof(p);
    std::string board(cursor, cursor + p.board_name_len);
    cursor += p.board_name_len;
    std::string version(cursor, cursor + p.version_len);

    using diagnostic_msgs::msg::DiagnosticStatus;
    DiagnosticStatus status;
    status.name = "nano_ros: " + board;
    status.hardware_id = board;
    char faults_hex[11];
    snprintf(faults_hex, sizeof(faults_hex), "0x%08x", p.faults);

    if (p.faults != 0) {
        status.level = DiagnosticStatus::ERROR;
        status.message = std::string("faults ") + faults_hex;
    } else if (p.reset_reason == NROS_RESET_WATCHDOG || p.reset_reason == NROS_RESET_BROWNOUT) {
        status.level = DiagnosticStatus::WARN;
        status.message = std::string("last reset by ") + reset_reason_name(p.reset_reason);
    } else {
        status.level = DiagnosticStatus::OK;
        status.message = "ok";
    }

    auto kv = [&](const std::string &key, const std::string &value) {
        diagnostic_msgs::msg::KeyValue v;
        v.key = key;
        v.value = value;
        status.values.push_back(v);
    };
    kv("uptime_s", std::to_string(p.uptime_ms / 1000.0));
    kv("version", version.empty() ? "unknown" : version);
    kv("reset_reason", reset_reason_name(p.reset_reason));
    kv("faults", faults_hex);
    kv("dropped_publishes", std::to_string(p.dropped_publishes));

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    array.status.push_back(status);
    diagnostics_pub_->publish(array);

    std::lock_guard<std::mutex> lock(status_mutex_);
    last_status_ = status;
    last_status_time_ = std::chrono::steady_clock::now();
}

// Boards report about once a second, so three missed reports means it's gone quiet. Only boards
// that have reported at least once can go stale.
static constexpr std::chrono::seconds STATUS_STALE_AFTER{3};

void ClientHandlerNode::check_stale() {
    diagnostic_msgs::msg::DiagnosticStatus status;
    std::chrono::steady_clock::duration silent;
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        if (!last_status_) {
            return;
        }
        silent = std::chrono::steady_clock::now() - last_status_time_;
        if (silent < STATUS_STALE_AFTER) {
            return;
        }
        status = *last_status_;
    }

    auto secs = std::chrono::duration_cast<std::chrono::seconds>(silent).count();
    status.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    status.message = "no status for " + std::to_string(secs) + " s (last: " + status.message + ")";

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    array.status.push_back(status);
    diagnostics_pub_->publish(array);
}
