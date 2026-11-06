#pragma once

#include "transport.hpp"
#include "canmore_link.hpp"
#include "canmore/protocol.h"

#include <chrono>
#include <deque>
#include <memory>
#include <string>

namespace nros {

struct CanmoreOptions {
    enum class Backend {
        Auto,       // SocketCAN if an interface is named (and this is Linux), otherwise gs_usb
        SocketCan,  // Linux only
        GsUsb,      // libusb, works anywhere libusb does (macOS, WSL with usbipd, Linux)
    };

    Backend backend = Backend::Auto;
    std::string ifname;    // SocketCAN interface, e.g. "can0". Bitrates come from `ip link`.
    GsUsbOptions gs_usb;   // adapter, channel and bitrates when using gs_usb
};

class CanmoreTransport : public Transport, private CanmoreMsgHandler {
public:
    explicit CanmoreTransport(CanmoreOptions opts) : opts_(std::move(opts)) {}
    ~CanmoreTransport();

    bool init() override;
    bool send_frame(const nros::WireFrame &frame, int32_t timeout) override;
    bool recv_frame(nros::WireFrame &frame, int32_t timeout) override;
    nros::Readiness wait_ready(int32_t timeout_ms) override;
    bool fini() override;

private:
    std::unique_ptr<CanmoreLink> open_link();

    // CanmoreMsgHandler overrides
    void handleMessage(uint8_t clientId, uint8_t subtype, std::span<const uint8_t> data) override;
    void handleDecodeError(uint8_t clientId, unsigned int errorCode) override;
    void handleConflictingAgentError() override;

    /**
     * same as #define CANMORE_MSG_SUBTYPE_XRCE_DDS 0 from canbus driver, should add subtype to reflect
     * nros agent. Problem exists however that in canbus_msg_write which transport code invokes, you
     * cannot choose a subtype, so that needs implemented as well and remains a TODO in the driver
     */
    static constexpr uint8_t NROS_SUBTYPE = 0;
    static constexpr size_t MAX_PAYLOAD = CANMORE_MAX_MSG_LENGTH - sizeof(nros_frame_header_t);
    CanmoreOptions opts_;
    std::unique_ptr<CanmoreLink> link_;
    std::unique_ptr<Canmore::PollGroup> group_;
    std::deque<WireFrame> completed_frames_;
    uint8_t peer_client_id_ = 0;
    std::chrono::steady_clock::time_point last_conflict_report_ {};
};

} // namespace nros
