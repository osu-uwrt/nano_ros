#pragma once
#include <string>
#include <variant>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/generic_publisher.hpp>
#include <rclcpp/generic_subscription.hpp>
#include "transport.hpp"
#include "thread_safe_queue.hpp"

namespace nros {

struct Topic {
    bool active = false;
    uint8_t id;
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
    void client_publish(const WireFrame &frame) const;
    void client_subscription_callback(uint8_t client_id, uint8_t topic_id, const std::shared_ptr<rclcpp::SerializedMessage>& msg) const;
    void on_client_connect(const std::vector<Topic> &new_topics);
    void on_client_disconnect();
private:
    void setup_topic(Topic &t);

    bool expect_multiple_clients_ = false;
    uint32_t client_id_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    std::thread spin_thread_;
    OutboundCallback out_cb_;
    ThreadSafeQueue<OutboundMsg> &cb_data_queue_;
    Topic topics_[UINT8_MAX];
};

} // namespace nros