#pragma once
#include <atomic>
#include <memory>
#include <string>
#include <variant>
#include "transport.hpp"
#include "protocol.h"
#include "rclcpp/rclcpp.hpp"
#include "client_handler.hpp"
#include "thread_safe_queue.hpp"

namespace nros {

struct AckStatus {
    bool need_ack;
    uint8_t topic_to_ack_id;
    nros_ack_status_t status = NROS_ACK_OK;
};

class Agent {
public:
    Agent(std::unique_ptr<Transport> transport);
    ~Agent();

    // returns false if the transport couldn't be opened. Runs until stop() or rclcpp shuts down (SIGINT/SIGTERM)
    bool run();
    void stop();

    bool has_pending_send;
private:
    // each node gets the same callback for all subscriptions
    void frame_dispatch(WireFrame &frame);
    nros_ack_status_t decode_registration_frame(WireFrame &frame, Topic &t);
    void spawn_client(bool expect_multiple_clients);
    void ack_client();

    std::unique_ptr<Transport> transport_;
    ThreadSafeQueue<OutboundMsg> to_client_msg_queue_;
    //std::vector<std::shared_ptr<ClientHandlerNode>> clients_;
    std::shared_ptr<ClientHandlerNode> client_; // for testing
    std::vector<Topic> new_topics_;
    size_t client_id_idx_ = 0; // temp for dealing with client ids until its actually implemented
    AckStatus ack_ {};
    //Topic topics[UINT8_MAX] = { 0 }; // indexed by topic id (uint8 can change) 
    std::atomic<bool> running_ = false;
};

} // namespace nros