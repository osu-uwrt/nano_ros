#include "canmore_link.hpp"

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
