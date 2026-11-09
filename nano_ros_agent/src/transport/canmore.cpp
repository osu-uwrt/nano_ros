#include "canmore.hpp"
#include <net/if.h>
#include <string.h>
extern "C" {
#include "nros_stream_parser.h"
}

using namespace nros;

// Stand-in for Canmore::MsgAgent::lookupDecodeError, which is declared but never implemented
const char *CanmoreTransport::decode_error_str(unsigned int errorCode) {
    switch (errorCode) {
    case CANMORE_MSG_DECODER_ERROR_BAD_SEQ_NUM:
        return "bad sequence number";
    case CANMORE_MSG_DECODER_ERROR_ZERO_LENGTH_PACKET:
        return "zero length packet";
    case CANMORE_MSG_DECODER_ERROR_UNEXPECTED_DATA:
        return "unexpected data";
    case CANMORE_MSG_DECODER_ERROR_CRC_FAIL:
        return "CRC failure";
    case CANMORE_MSG_DECODER_ERROR_SEQ_ZERO_NOT_EXTENDED:
        return "sequence zero not extended";
    case CANMORE_MSG_DECODER_ERROR_MSG_TOO_LARGE:
        return "message too large";
    case CANMORE_MSG_DECODER_ERROR_MSG_TOO_SMALL:
        return "message too small";
    case CANMORE_MSG_DECODER_ERROR_ROLLOVER_WITH_CANFD:
        return "sequence rollover with CAN FD";
    case CANMORE_MSG_DECODER_ERROR_INVALID_CLIENT_ID:
        return "invalid client id";
    default:
        return "unknown error";
    }
}

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
    // Readiness r;
    // try {
    //     if (msg_inbox_.empty()) {
    //         group_->processEvent(timeout_ms);
    //     }
    // } catch (std::exception &e) {
    //     connected = false;
    //     r.error = true;
    //     return r;
    // }

    // r.readable = !msg_inbox_.empty();
    // r.writable = connected;
    // r.nothing_ready = !r.readable && !r.writable;
    // return r;
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

void CanmoreTransport::handleDecodeError(uint8_t clientId, unsigned int errorCode) {
    fprintf(stderr, "canmore: decode error from client %u: %s (%u)\n", (unsigned) clientId, decode_error_str(errorCode), errorCode);
}

void CanmoreTransport::handleConflictingAgentError() {
    fprintf(stderr, "canmore: another agent is transmitting on %s! Is the old agent still running?\n", ifname_.c_str());
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

    if (frame.header.type == NROS_PACKET_DISCOVER) {
        // CANmore clients only accept frames addressed to their own ID; there is no real broadcast
        bool any_sent = false;
        for (uint8_t id = 1; id <= 31; id++) {
            try {
                can_agent_->transmitMessage(id, NROS_SUBTYPE, buffer);
                any_sent = true;
            } catch (const std::exception &e) {
                fprintf(stderr, "canmore: discover to client %u failed: %s\n", id, e.what());
                connected = false;
                return false;  // a send failing usually means the whole bus is down, no point continuing
            }
        }
        return any_sent;
    }


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

bool CanmoreTransport::fini() {
    // group_ holds weak refs into agent_, drop it first
    group_.reset();
    can_agent_.reset();
    completed_frames_.clear();
    connected = false;
    return true;
}

CanmoreTransport::~CanmoreTransport() {

}