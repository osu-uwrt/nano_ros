#include "canmore_link.hpp"

#include "canmore/msg_encoding.h"

#include <stdexcept>

using namespace nros;

#ifndef NROS_HAVE_SOCKETCAN
bool nros::socketcan_available() {
    return false;
}

std::unique_ptr<CanmoreLink> nros::make_socketcan_link(const std::string &, CanmoreMsgHandler &,
                                                       std::span<const uint8_t>) {
    throw std::runtime_error("socketcan: not supported on this platform, use a gs_usb adapter");
}
#endif

#ifndef NROS_HAVE_GS_USB
bool nros::gs_usb_available() {
    return false;
}

std::unique_ptr<CanmoreLink> nros::make_gs_usb_link(const GsUsbOptions &, CanmoreMsgHandler &,
                                                    std::span<const uint8_t>) {
    throw std::runtime_error("gs_usb: the agent was built without libusb");
}
#endif

std::string nros::canmore_decode_error_str(unsigned int errorCode) {
    switch (errorCode) {
    case CANMORE_MSG_DECODER_ERROR_BAD_SEQ_NUM:
        return "bad sequence number";
    case CANMORE_MSG_DECODER_ERROR_ZERO_LENGTH_PACKET:
        return "zero length packet";
    case CANMORE_MSG_DECODER_ERROR_UNEXPECTED_DATA:
        return "unexpected data";
    case CANMORE_MSG_DECODER_ERROR_CRC_FAIL:
        return "CRC mismatch";
    case CANMORE_MSG_DECODER_ERROR_SEQ_ZERO_NOT_EXTENDED:
        return "first frame not extended";
    case CANMORE_MSG_DECODER_ERROR_MSG_TOO_LARGE:
        return "message too large";
    case CANMORE_MSG_DECODER_ERROR_MSG_TOO_SMALL:
        return "message too small";
    case CANMORE_MSG_DECODER_ERROR_ROLLOVER_WITH_CANFD:
        return "sequence number rollover in CAN FD mode";
    case CANMORE_MSG_DECODER_ERROR_INVALID_CLIENT_ID:
        return "invalid client ID";
    default:
        return "unknown error " + std::to_string(errorCode);
    }
}
