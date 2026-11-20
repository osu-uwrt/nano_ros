#include "canmore.hpp"
#include "canmore/msg_encoding.h"
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

CanmoreTransport::~CanmoreTransport() {
    fini();
}

std::unique_ptr<CanmoreLink> CanmoreTransport::open_link() {
    using Backend = CanmoreOptions::Backend;
    CanmoreMsgHandler &handler = *this;

    switch (opts_.backend) {
    case Backend::SocketCan:
        return make_socketcan_link(opts_.ifname, handler);
    case Backend::GsUsb:
        return make_gs_usb_link(opts_.gs_usb, handler);
    case Backend::Auto:
        break;
    }

    // A named interface means SocketCAN, like the rest of titan's canmore tools. Otherwise prefer gs_usb, which
    // works without SocketCAN (macOS, WSL), but don't take an adapter the kernel already has: on Linux it's
    // probably a SocketCAN interface someone else is using.
    if (!opts_.ifname.empty() && socketcan_available()) {
        return make_socketcan_link(opts_.ifname, handler);
    }
    GsUsbOptions gs_usb = opts_.gs_usb;
    gs_usb.detach_kernel_driver = false;
    return make_gs_usb_link(gs_usb, handler);
}

bool CanmoreTransport::init() {
    fini();

    try {
        link_ = open_link();
        group_ = std::make_unique<Canmore::PollGroup>();
        group_->addFd(link_->pollFd());
    } catch (const std::exception &e) {
        fprintf(stderr, "canmore init: %s\n", e.what());
        group_.reset();
        link_.reset();
        return false;
    }
    printf("canmore: using %s\n", link_->description().c_str());
    completed_frames_.clear();
    connected = true;
    return true;
}

bool CanmoreTransport::fini() {
    // group_ holds weak refs into link_, drop it first
    group_.reset();
    link_.reset();
    completed_frames_.clear();
    connected = false;
    return true;
}

Readiness CanmoreTransport::wait_ready(int32_t timeout_ms) {
    Readiness r;
    if (!group_ || !connected) {
        r.error = true;
        return r;
    }

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
    r.writable = connected;
    r.nothing_ready = !r.readable && !r.writable;
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

    completed_frames_.push_back({clientId, std::move(frame)});
}

void CanmoreTransport::handleDecodeError(uint8_t clientId, unsigned int errorCode) {
    fprintf(stderr, "canmore: decode error from client %u: %s (%u)\n", (unsigned) clientId, decode_error_str(errorCode), errorCode);
}

void CanmoreTransport::handleConflictingAgentError() {
    // Fires for every frame the other agent sends, so only say it every few seconds
    auto now = std::chrono::steady_clock::now();
    if (now - last_conflict_report_ < std::chrono::seconds(5)) {
        return;
    }
    last_conflict_report_ = now;
    fprintf(stderr, "canmore: another agent is transmitting on %s! Is the old agent still running?\n",
            link_ ? link_->description().c_str() : "the bus");
}

bool CanmoreTransport::broadcast_message(const std::vector<uint8_t> bytes) {
    bool any_sent = false;
    for (uint8_t id = 1; id <= 31; id++) {
        try {
            link_->transmitMessage(id, NROS_SUBTYPE, bytes);
            any_sent = true;
        } catch (const std::exception &e) {
            fprintf(stderr, "canmore: discover to client %u failed: %s\n", id, e.what());
            connected = false;
            return false;  // a send failing usually means the whole bus is down, no point continuing
        }
    }
    return any_sent;   
}

bool CanmoreTransport::send_frame(const PeerID to, const nros::WireFrame &frame, int32_t timeout) {
    (void)timeout;

    if (!link_) {
        return false;
    }

    if (frame.header.length != frame.payload.size()) {
        return false;
    }

    if (frame.payload.size() > MAX_PAYLOAD) {
        fprintf(stderr, "canmore: payload of %zu bytes exceeds max of %zu\n", frame.payload.size(), MAX_PAYLOAD);
        return false;
    }

    //uint8_t buffer[NROS_BUFFER_SIZE];
    std::vector<uint8_t> buffer(sizeof(nros_frame_header_t) + frame.payload.size());
    memcpy(buffer.data(), &frame.header, sizeof(nros_frame_header_t));
    if (!frame.payload.empty()) {
        memcpy(buffer.data() + sizeof(nros_frame_header_t), frame.payload.data(), frame.payload.size());
    }

    if (to == PEER_BROADCAST) {
        return broadcast_message(buffer);
    }


    try {
        link_->transmitMessage(to, NROS_SUBTYPE, buffer);
    } catch (std::exception &e) {
        fprintf(stderr, "canmore: send to client %u failed: %s\n", to, e.what());
        return false;
    }

    return true;
}

bool CanmoreTransport::recv_frame(PeerID &from, nros::WireFrame &frame, int32_t timeout) {
    // After a bus error the link stays down until init() is called again
    if (!link_ || !connected) {
        return false;
    }

    using clock = std::chrono::steady_clock;
    const auto deadline = clock::now() + std::chrono::milliseconds(timeout > 0 ? timeout : 0);

    while (true) {
        if (!completed_frames_.empty()) {
            from = completed_frames_.front().peer;
            frame = std::move(completed_frames_.front().frame);
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
