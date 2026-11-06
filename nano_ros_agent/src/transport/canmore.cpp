#include "canmore.hpp"
#include <net/if.h>
#include <string.h>
extern "C" {
#include "nros_stream_parser.h"
}

using namespace nros;

bool CanmoreTransport::init() {
    uint32_t idx = if_nametoindex(ifname_.c_str());
    if (!idx) {
        return false;
    }

    try {
        Canmore::AgentMsgHandler &handler = *this;
        can_agent_ = std::make_unique<Canmore::MsgAgent>(idx, handler);
        group_ = std::make_unique<Canmore::PollGroup>();
        group_->addFd(*can_agent_);
        wake_poll_fd_ = std::make_unique<WakePollFd>(wake_);
        group_->addFd(*wake_poll_fd_);
    } catch (const std::exception &e) {
        fprintf(stderr, "canmore init: %s\n", e.what());
        group_.reset();
        can_agent_.reset();
        return false;
    }
    completed_frames_.clear();
    peer_client_id_ = 0;
    connected = true;
    return true;
}

Readiness CanmoreTransport::wait_ready(int32_t timeout_ms) {
    Readiness r;
    if (!group_) {
        r.error = true;
        return r;
    }

    // returns after one event, which is a message (handleMessage), a wake(), or a signal
    try {
        if (completed_frames_.empty()) {
            group_->processEvent(timeout_ms);
        }
    } catch (std::exception &e) {
        fprintf(stderr, "canmore: processEvent failed: %s\n", e.what());
        connected = false;
        r.error = true;
        return r;
    }

    r.readable = !completed_frames_.empty();
    r.nothing_ready = !r.readable;
    return r;
}

// callback called by canmore
void CanmoreTransport::handleMessage(uint8_t clientId, uint8_t subtype, std::span<const uint8_t> data) {
    if (subtype != NROS_SUBTYPE || data.size() < sizeof(nros_frame_header_t)) {
        return;
    }

    WireFrame frame;
    memcpy(&frame.header, data.data(), sizeof(frame.header));
    frame.payload.assign(data.begin() + sizeof(frame.header), data.end());

    if (frame.payload.size() != frame.header.length) {
        return;
    }

    peer_client_id_ = clientId;
    completed_frames_.push_back(std::move(frame));
}

bool CanmoreTransport::send_frame(const nros::WireFrame &frame, int32_t timeout) {
    (void)timeout;

    if (!can_agent_) {
        return false;
    }

    if (frame.header.length != frame.payload.size()) {
        return false;
    }

    if (frame.payload.size() > MAX_PAYLOAD) {
        fprintf(stderr, "canmore: payload of %zu bytes exceeds max of %zu");
        return false;
    }

    //uint8_t buffer[NROS_BUFFER_SIZE];
    std::vector<uint8_t> buffer(sizeof(nros_frame_header_t) + frame.payload.size());
    memcpy(buffer.data(), &frame.header, sizeof(nros_frame_header_t));
    if (!frame.payload.empty()) {
        memcpy(buffer.data() + sizeof(nros_frame_header_t), frame.payload.data(), frame.payload.size());
    }

    uint8_t dest = (frame.header.type == NROS_PACKET_DISCOVER) ? 0 : peer_client_id_;

    try {
        can_agent_->transmitMessage(dest, NROS_SUBTYPE, buffer);
    } catch (std::exception &e) {
        fprintf(stderr, "canmore: send to client %u failed: %s\n", dest, e.what());
        return false;
    }

    return true;
}

bool CanmoreTransport::recv_frame(nros::WireFrame &frame, int32_t timeout) {
    if (!can_agent_) {
        return false;
    }

    using clock = std::chrono::steady_clock;
    const auto deadline = clock::now() + std::chrono::milliseconds(timeout > 0 ? timeout : 0);

    while (true) {
        if (!completed_frames_.empty()) {
            frame = std::move(completed_frames_.front());
            completed_frames_.pop_front();
            return true;
        }

        int wait_ms = -1;
        if (timeout >= 0) {
            auto left = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - clock::now()).count();
            wait_ms = left > 0 ? static_cast<int32_t>(left) : 0;
        }

        bool got_event;
        try {
            got_event = group_->processEvent(wait_ms);
        } catch (std::exception &e) {
            fprintf(stderr, "canmore: processEvent failed: %s\n", e.what());
            connected = false;
            return false;
        }

        if (!got_event) {
            return false;
        }

    }


}