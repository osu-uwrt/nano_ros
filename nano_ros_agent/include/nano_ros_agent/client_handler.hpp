#pragma once
#include <string>
#include <variant>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/generic_publisher.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <functional>
#include <vector>
#include "transport.hpp"
#include "protocol.h"
#include "thread_safe_queue.hpp"

namespace nros {

constexpr uint8_t PUBLISHER_TOPIC_ID_START = UINT8_MAX / 2;
constexpr uint8_t PUBLISHER_TOPIC_ID_END = UINT8_MAX;
constexpr uint8_t SUBSCRIBER_TOPIC_ID_START = 0;
constexpr uint8_t SUBSCRIBER_TOPIC_ID_END = (UINT8_MAX / 2) - 1;

struct Topic {
    bool active = false;
    uint8_t id;
    uint8_t qos = NROS_QOS_DEFAULT; // nros_qos_t
    std::string topic_name;
    std::string msg_type;

    std::variant< 
        rclcpp::GenericPublisher::SharedPtr,
        rclcpp::GenericSubscription::SharedPtr
    > handle;

};

struct OutboundMsg {
    uint8_t client_id;
    WireFrame frame;
};


class ClientHandlerNode : public rclcpp::Node {
public:
    using OutboundCallback = std::function<void (std::shared_ptr<rclcpp::SerializedMessage>)>;

    ClientHandlerNode(const std::string &node_name, const rclcpp::NodeOptions &options, ThreadSafeQueue<OutboundMsg> &q)
    : rclcpp::Node("nros_client_" + node_name, options), cb_data_queue_{q} {
        diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
        stale_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { check_stale(); });
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(this->get_node_base_interface());
        spin_thread_ = std::thread([this](){ executor_->spin(); });
    }
    ~ClientHandlerNode() {
        executor_->cancel();
        if (spin_thread_.joinable()) {
            spin_thread_.join();
        }
    }
    void client_publish(WireFrame &frame);
    void client_subscription_callback(uint8_t client_id, uint8_t topic_id, std::shared_ptr<rclcpp::SerializedMessage> msg);
    void on_client_connect(const std::vector<Topic> &new_topics);
    void on_client_disconnect();
    void publish_status(const WireFrame &frame);
private:
    void setup_topic(Topic &t);
    void check_stale();

    bool expect_multiple_clients_ = false;
    uint32_t client_id_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    std::thread spin_thread_;
    OutboundCallback out_cb_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
    // the board's last status report, republished as STALE when reports stop
    rclcpp::TimerBase::SharedPtr stale_timer_;
    std::mutex status_mutex_;
    std::optional<diagnostic_msgs::msg::DiagnosticStatus> last_status_;
    std::chrono::steady_clock::time_point last_status_time_;
    ThreadSafeQueue<OutboundMsg> &cb_data_queue_;
    Topic topics_[UINT8_MAX];
};

} // namespace nros