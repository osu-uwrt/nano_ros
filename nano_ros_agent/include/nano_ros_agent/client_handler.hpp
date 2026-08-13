#pragma once

#include <memory>
#include <string>
#include <variant>
#include <rclcpp/rclcpp.hpp>
#include "transport.hpp"
#include "protocol.h"

namespace nros {

struct Topic {
    bool active = false;
    uint8_t id;
    std::string topic_name;
    std::string msg_type;

    std::variant< 
        std::shared_ptr<rclcpp::GenericPublisher>,
        std::shared_ptr<rclcpp::GenericSubscription>
    > handle;
};

class ClientHandler {
public:
    ClientHandler(std::unique_ptr<Transport> transport);
    ~ClientHandler();

    void run();
    void stop();

    bool has_pending_send;
private: 
    std::unique_ptr<Transport> transport;
    Topic topics[UINT8_MAX] = { 0 }; // indexed by topic id (uint8 can change) 
    bool running;
};

} // namespace nros