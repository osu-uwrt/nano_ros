#pragma once

#include "canmore_cpp/PollFD.hpp"
#include "canmore_cpp/span_compat.hpp"
#include "gs_usb.hpp"

#include <cstdint>
#include <memory>
#include <string>

namespace nros {

/**
 * Same callbacks as Canmore::AgentMsgHandler. That one lives in MsgAgent.hpp, which needs SocketCAN headers, so it
 * can't be used on macOS.
 */
class CanmoreMsgHandler {
public:
    virtual ~CanmoreMsgHandler() = default;
    virtual void handleMessage(uint8_t clientId, uint8_t subtype, std::span<const uint8_t> data) = 0;
    virtual void handleDecodeError(uint8_t clientId, unsigned int errorCode) = 0;
    virtual void handleConflictingAgentError() {}
};

/**
 * A CANmore message agent on some CAN backend: Canmore::MsgAgent on a SocketCAN interface, or the same thing on a
 * gs_usb adapter through libusb.
 */
class CanmoreLink {
public:
    virtual ~CanmoreLink() = default;

    /** Same as Canmore::MsgAgent::transmitMessage: clientId 0 broadcasts. Throws on failure. */
    virtual void transmitMessage(uint8_t clientId, uint8_t subtype, std::span<const uint8_t> data) = 0;

    /** Add this to a Canmore::PollGroup; processing its events calls the handler */
    virtual Canmore::PollFD &pollFd() = 0;

    virtual std::string description() const = 0;
};

/** True if SocketCAN support was built in (Linux only) */
bool socketcan_available();

/** True if gs_usb support was built in (needs libusb) */
bool gs_usb_available();

/** Canmore::MsgAgent on a SocketCAN interface. Throws on failure, including when socketcan_available() is false. */
std::unique_ptr<CanmoreLink> make_socketcan_link(const std::string &ifname, CanmoreMsgHandler &handler,
                                                 std::span<const uint8_t> clientIdSelect = {});

/** The same message agent on a gs_usb adapter. Throws on failure, including when gs_usb_available() is false. */
std::unique_ptr<CanmoreLink> make_gs_usb_link(const GsUsbOptions &opts, CanmoreMsgHandler &handler,
                                              std::span<const uint8_t> clientIdSelect = {});

/** Describes a CANMORE_MSG_DECODER_ERROR_* code (Canmore::MsgAgent::lookupDecodeError is declared but not defined) */
std::string canmore_decode_error_str(unsigned int errorCode);

}  // namespace nros
