#include "gs_usb.hpp"

#include "canmore/msg_encoding.h"

#include <libusb.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <stdexcept>
#include <system_error>
#include <unistd.h>

using namespace nros;

static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "gs_usb structs are little endian and copied as-is");
static_assert(CANMORE_MAX_FD_FRAME_SIZE == can::FD_MAX_DLEN, "CANmore and gs_usb definitions do not match");
static_assert(CANMORE_MAX_FRAME_SIZE == can::MAX_DLEN, "CANmore and gs_usb definitions do not match");

namespace {

// gs_usb protocol, as implemented by the Linux kernel's drivers/net/can/usb/gs_usb.c and candleLight_fw
enum GsUsbRequest : uint8_t {
    GS_USB_BREQ_HOST_FORMAT = 0,
    GS_USB_BREQ_BITTIMING = 1,
    GS_USB_BREQ_MODE = 2,
    GS_USB_BREQ_BT_CONST = 4,
    GS_USB_BREQ_DEVICE_CONFIG = 5,
    GS_USB_BREQ_DATA_BITTIMING = 10,
    GS_USB_BREQ_BT_CONST_EXT = 11,
};

constexpr uint32_t GS_CAN_MODE_RESET = 0;
constexpr uint32_t GS_CAN_MODE_START = 1;
constexpr uint32_t GS_CAN_MODE_FD = 1u << 8;

constexpr uint32_t GS_CAN_FEATURE_FD = 1u << 8;
constexpr uint32_t GS_CAN_FEATURE_BT_CONST_EXT = 1u << 10;

constexpr uint8_t GS_CAN_FLAG_OVERFLOW = 1u << 0;
constexpr uint8_t GS_CAN_FLAG_FD = 1u << 1;
constexpr uint8_t GS_CAN_FLAG_BRS = 1u << 2;

// echo_id of frames received from the bus; anything else is the echo of a frame we transmitted
constexpr uint32_t GS_HOST_FRAME_ECHO_ID_RX = 0xFFFFFFFFu;

// SocketCAN's CAN_INV_FILTER: set in a filter's can_id to invert it
constexpr uint32_t CAN_INV_FILTER = 0x20000000U;

// See CANSocket.cpp: how long a transmit may wait for the bus before we decide nothing is ACKing our frames
constexpr auto MAX_BLOCKING_TIME = std::chrono::milliseconds(50);
constexpr unsigned CONTROL_TIMEOUT_MS = 1000;

struct __attribute__((packed)) GsHostFrameHeader {
    uint32_t echo_id;
    uint32_t can_id;
    uint8_t can_dlc;
    uint8_t channel;
    uint8_t flags;
    uint8_t reserved;
};
constexpr size_t GS_CLASSIC_FRAME_SIZE = sizeof(GsHostFrameHeader) + can::MAX_DLEN;
constexpr size_t GS_FD_FRAME_SIZE = sizeof(GsHostFrameHeader) + can::FD_MAX_DLEN;
constexpr size_t GS_TIMESTAMP_SIZE = 4;

struct GsHostConfig {
    uint32_t byte_order;
};

struct GsDeviceConfig {
    uint8_t reserved1, reserved2, reserved3;
    uint8_t icount;  // number of channels - 1
    uint32_t sw_version;
    uint32_t hw_version;
};

struct GsDeviceMode {
    uint32_t mode;
    uint32_t flags;
};

struct GsDeviceBittiming {
    uint32_t prop_seg, phase_seg1, phase_seg2, sjw, brp;
};

struct GsBittimingConst {
    uint32_t tseg1_min, tseg1_max, tseg2_min, tseg2_max, sjw_max, brp_min, brp_max, brp_inc;
};

struct GsDeviceBtConst {
    uint32_t feature;
    uint32_t fclk_can;
    GsBittimingConst btc;
};

struct GsDeviceBtConstExt {
    uint32_t feature;
    uint32_t fclk_can;
    GsBittimingConst btc;
    GsBittimingConst dbtc;
};

struct KnownAdapter {
    uint16_t vid, pid;
};

// Same list as the Linux gs_usb driver
constexpr KnownAdapter KNOWN_ADAPTERS[] = {
    { 0x1d50, 0x606f },  // candleLight / CANable (original gs_usb VID/PID)
    { 0x1209, 0x2323 },  // candleLight (pid.codes)
    { 0x1cd2, 0x606f },  // CES CANext FD
    { 0x16d0, 0x10b8 },  // ABE CANdebugger FD
    { 0x16d0, 0x0f30 },  // Xylanta SAINT3
};

bool isKnownAdapter(const libusb_device_descriptor &desc) {
    return std::any_of(std::begin(KNOWN_ADAPTERS), std::end(KNOWN_ADAPTERS),
                       [&](const KnownAdapter &a) { return a.vid == desc.idVendor && a.pid == desc.idProduct; });
}

std::string usbError(int rc) {
    return libusb_strerror(static_cast<libusb_error>(rc));
}

std::string hex4(uint16_t v) {
    char buf[5];
    snprintf(buf, sizeof(buf), "%04x", v);
    return buf;
}

/**
 * Pick bit timing like the MCP251XFD driver on the boards: sample point near 80%, using the smallest prescaler that
 * hits the bitrate exactly. That gives the most time quanta per bit, and the same quantum in the nominal and data
 * phases whenever the adapter allows it, which avoids quantization error at the bit rate switch.
 */
GsDeviceBittiming calcBittiming(const GsBittimingConst &c, uint32_t fclk, uint32_t bitrate, const char *phase) {
    uint32_t brp_inc = c.brp_inc ? c.brp_inc : 1;
    for (uint64_t brp = std::max<uint32_t>(c.brp_min, 1); brp <= c.brp_max; brp += brp_inc) {
        if (fclk % (brp * bitrate) != 0) {
            continue;
        }
        uint64_t tq = fclk / (brp * bitrate);  // time quanta per bit, including the 1 tq sync segment
        uint64_t tseg2 = std::clamp<uint64_t>((tq + 2) / 5, c.tseg2_min, c.tseg2_max);
        if (tq < 1 + tseg2) {
            continue;
        }
        uint64_t tseg1 = tq - 1 - tseg2;
        if (tseg1 < c.tseg1_min || tseg1 > c.tseg1_max) {
            continue;
        }

        GsDeviceBittiming bt;
        bt.prop_seg = tseg1 / 2;  // the adapter only uses prop_seg + phase_seg1; split it like Linux does
        bt.phase_seg1 = tseg1 - bt.prop_seg;
        bt.phase_seg2 = tseg2;
        bt.sjw = std::min<uint32_t>(tseg2, c.sjw_max);
        bt.brp = brp;
        return bt;
    }
    throw std::runtime_error("gs_usb: can't make a " + std::to_string(bitrate) + " bit/s " + phase +
                             " bitrate from the adapter's " + std::to_string(fclk) + " Hz clock");
}

}  // namespace

GsUsbDevice::GsUsbDevice(const GsUsbOptions &opts): channel_(opts.channel), filters_ { { 0, 0 } } {
    try {
        if (pipe(wake_pipe_) != 0) {
            throw std::system_error(errno, std::generic_category(), "gs_usb pipe");
        }
        for (int fd : wake_pipe_) {
            if (fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_NONBLOCK) != 0 || fcntl(fd, F_SETFD, FD_CLOEXEC) != 0) {
                throw std::system_error(errno, std::generic_category(), "gs_usb fcntl");
            }
        }
        pollDescriptor_ = Canmore::PollFDDescriptor::create(*this, wake_pipe_[0], POLLIN);

        open(opts);
        configure(opts);

        rx_transfers_.resize(NUM_RX_TRANSFERS);
        rx_buffers_.resize(NUM_RX_TRANSFERS);
        for (size_t i = 0; i < NUM_RX_TRANSFERS; i++) {
            rx_buffers_[i].resize(rx_buf_size_);
            rx_transfers_[i] = libusb_alloc_transfer(0);
            if (!rx_transfers_[i]) {
                throw std::runtime_error("gs_usb: libusb_alloc_transfer failed");
            }
            libusb_fill_bulk_transfer(rx_transfers_[i], handle_, ep_in_, rx_buffers_[i].data(), rx_buf_size_,
                                      &GsUsbDevice::rxCallback, this, 0);
            int rc = libusb_submit_transfer(rx_transfers_[i]);
            if (rc != 0) {
                throw std::runtime_error("gs_usb: submitting RX transfer failed: " + usbError(rc));
            }
            rx_transfers_active_++;
        }

        event_thread_ = std::thread(&GsUsbDevice::eventLoop, this);
    } catch (...) {
        close();
        throw;
    }
}

GsUsbDevice::~GsUsbDevice() {
    close();
}

void GsUsbDevice::open(const GsUsbOptions &opts) {
    int rc = libusb_init(&ctx_);
    if (rc != 0) {
        ctx_ = nullptr;
        throw std::runtime_error("gs_usb: libusb_init failed: " + usbError(rc));
    }

    libusb_device **list;
    ssize_t count = libusb_get_device_list(ctx_, &list);
    if (count < 0) {
        throw std::runtime_error("gs_usb: listing USB devices failed: " + usbError(count));
    }

    // Adapters that matched but couldn't be used, for the error message
    std::string skipped;
    for (ssize_t i = 0; i < count && !handle_; i++) {
        libusb_device_descriptor desc;
        if (libusb_get_device_descriptor(list[i], &desc) != 0 || !isKnownAdapter(desc)) {
            continue;
        }
        std::string id = hex4(desc.idVendor) + ":" + hex4(desc.idProduct);

        libusb_device_handle *h;
        rc = libusb_open(list[i], &h);
        if (rc != 0) {
            skipped += "\n  " + id + ": can't open (" + usbError(rc) + ")";
            if (rc == LIBUSB_ERROR_ACCESS) {
                skipped += ", you may need a udev rule or to run as root";
            }
            continue;
        }

        char serial[128] = "";
        if (desc.iSerialNumber) {
            libusb_get_string_descriptor_ascii(h, desc.iSerialNumber, reinterpret_cast<unsigned char *>(serial),
                                               sizeof(serial));
        }
        if (!opts.serial.empty() && opts.serial != serial) {
            skipped += "\n  " + id + " serial " + serial + ": not the requested serial";
            libusb_close(h);
            continue;
        }

        // Interface 0 is the CAN interface on every gs_usb adapter (it's what the Linux driver binds to)
        interface_ = 0;
        if (libusb_kernel_driver_active(h, interface_) == 1) {
            if (!opts.detach_kernel_driver) {
                skipped += "\n  " + id + " serial " + serial +
                           ": in use by the kernel's gs_usb driver (use its SocketCAN interface instead)";
                libusb_close(h);
                continue;
            }
            // Gives the adapter back to the kernel when we release it
            libusb_set_auto_detach_kernel_driver(h, 1);
        }

        rc = libusb_claim_interface(h, interface_);
        if (rc != 0) {
            skipped += "\n  " + id + " serial " + serial + ": can't claim interface (" + usbError(rc) + ")";
            libusb_close(h);
            continue;
        }

        handle_ = h;
        description_ = "gs_usb " + id + (serial[0] ? std::string(" serial ") + serial : "");
    }
    libusb_free_device_list(list, 1);

    if (!handle_) {
        std::string what = "gs_usb: no usable adapter found";
        if (!opts.serial.empty()) {
            what += " with serial " + opts.serial;
        }
        throw std::runtime_error(what + skipped);
    }

    // Find the bulk endpoints, falling back to the ones candleLight uses
    ep_in_ = 0x81;
    ep_out_ = 0x02;
    int max_packet = 64;
    libusb_config_descriptor *config;
    if (libusb_get_active_config_descriptor(libusb_get_device(handle_), &config) == 0) {
        if (config->bNumInterfaces > interface_ && config->interface[interface_].num_altsetting > 0) {
            const libusb_interface_descriptor &alt = config->interface[interface_].altsetting[0];
            for (int i = 0; i < alt.bNumEndpoints; i++) {
                const libusb_endpoint_descriptor &ep = alt.endpoint[i];
                if ((ep.bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) != LIBUSB_TRANSFER_TYPE_BULK) {
                    continue;
                }
                if (ep.bEndpointAddress & LIBUSB_ENDPOINT_IN) {
                    ep_in_ = ep.bEndpointAddress;
                    max_packet = std::max<int>(ep.wMaxPacketSize, 1);
                }
                else {
                    ep_out_ = ep.bEndpointAddress;
                }
            }
        }
        libusb_free_config_descriptor(config);
    }

    // Each RX transfer carries one host frame. Size the buffer for the largest one (FD + hardware timestamp),
    // rounded up to whole packets so libusb never reports an overflow.
    size_t largest = GS_FD_FRAME_SIZE + GS_TIMESTAMP_SIZE;
    rx_buf_size_ = (largest + max_packet - 1) / max_packet * max_packet;
}

void GsUsbDevice::configure(const GsUsbOptions &opts) {
    // Ask for little endian frames. Current firmware ignores this, but the Linux driver still sends it.
    GsHostConfig host = { 0x0000beef };
    try {
        controlOut(GS_USB_BREQ_HOST_FORMAT, 1, &host, sizeof(host));
    } catch (const std::exception &) {
    }

    GsDeviceConfig dconf;
    controlIn(GS_USB_BREQ_DEVICE_CONFIG, 1, &dconf, sizeof(dconf));
    if (channel_ > dconf.icount) {
        throw std::runtime_error("gs_usb: adapter has " + std::to_string(dconf.icount + 1) + " channel(s), channel " +
                                 std::to_string(channel_) + " requested");
    }

    GsDeviceBtConst bt_const;
    controlIn(GS_USB_BREQ_BT_CONST, channel_, &bt_const, sizeof(bt_const));
    bool adapter_fd = bt_const.feature & GS_CAN_FEATURE_FD;

    // CANSocket goes by whether the interface was brought up in FD mode. Here we configure the bus ourselves, so FD
    // is on when a data bitrate is set. Don't quietly fall back to classic CAN: a classic controller on an FD bus
    // error-frames every FD frame it sees, which takes down traffic for everyone.
    useCanFd = !opts.force_no_canfd && opts.data_bitrate != 0;
    if (useCanFd && !adapter_fd) {
        throw std::runtime_error(description_ +
                                 ": adapter doesn't support CAN FD. Set the data bitrate to 0 for classic CAN, but "
                                 "only on a classic CAN bus (classic controllers break FD traffic)");
    }

    GsDeviceBittiming bt = calcBittiming(bt_const.btc, bt_const.fclk_can, opts.bitrate, "nominal");
    GsDeviceBittiming dbt {};
    if (useCanFd) {
        GsBittimingConst dbtc = bt_const.btc;
        if (bt_const.feature & GS_CAN_FEATURE_BT_CONST_EXT) {
            GsDeviceBtConstExt ext;
            controlIn(GS_USB_BREQ_BT_CONST_EXT, channel_, &ext, sizeof(ext));
            dbtc = ext.dbtc;
        }
        dbt = calcBittiming(dbtc, bt_const.fclk_can, opts.data_bitrate, "data");
    }

    // Reset in case a previous run left the channel started, then configure and start it
    GsDeviceMode mode = { GS_CAN_MODE_RESET, 0 };
    controlOut(GS_USB_BREQ_MODE, channel_, &mode, sizeof(mode));
    controlOut(GS_USB_BREQ_BITTIMING, channel_, &bt, sizeof(bt));
    if (useCanFd) {
        controlOut(GS_USB_BREQ_DATA_BITTIMING, channel_, &dbt, sizeof(dbt));
    }
    mode = { GS_CAN_MODE_START, useCanFd ? GS_CAN_MODE_FD : 0 };
    controlOut(GS_USB_BREQ_MODE, channel_, &mode, sizeof(mode));

    // Like the Linux driver, FD capable adapters always get FD sized host frames
    tx_frame_size_ = adapter_fd ? GS_FD_FRAME_SIZE : GS_CLASSIC_FRAME_SIZE;

    description_ += " ch" + std::to_string(channel_) + ", " + std::to_string(opts.bitrate) +
                    (useCanFd ? "/" + std::to_string(opts.data_bitrate) + " bit/s CAN FD" : " bit/s classic CAN");
}

void GsUsbDevice::close() noexcept {
    // The event loop cancels the RX transfers and runs until they've all called back
    stopping_ = true;
    if (event_thread_.joinable()) {
        event_thread_.join();
    }
    else if (ctx_) {
        // The constructor failed before starting the thread, but may have submitted transfers
        eventLoop();
    }

    bool transfers_done;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        transfers_done = rx_transfers_active_ == 0 && tx_transfers_active_ == 0;
    }

    if (handle_) {
        try {
            GsDeviceMode mode = { GS_CAN_MODE_RESET, 0 };
            controlOut(GS_USB_BREQ_MODE, channel_, &mode, sizeof(mode));
        } catch (const std::exception &) {
        }
        libusb_release_interface(handle_, interface_);
        libusb_close(handle_);
        handle_ = nullptr;
    }

    // If the event loop gave up waiting, libusb may still own these, so leak them instead
    if (transfers_done) {
        for (libusb_transfer *t : rx_transfers_) {
            libusb_free_transfer(t);
        }
    }
    rx_transfers_.clear();

    if (ctx_ && transfers_done) {
        libusb_exit(ctx_);
    }
    ctx_ = nullptr;

    for (int &fd : wake_pipe_) {
        if (fd >= 0) {
            ::close(fd);
            fd = -1;
        }
    }
}

void GsUsbDevice::controlOut(uint8_t request, uint16_t value, const void *data, uint16_t len) {
    std::vector<uint8_t> buf(static_cast<const uint8_t *>(data), static_cast<const uint8_t *>(data) + len);
    int rc = libusb_control_transfer(handle_, LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
                                     request, value, interface_, buf.data(), len, CONTROL_TIMEOUT_MS);
    if (rc != len) {
        throw std::runtime_error("gs_usb: control request " + std::to_string(request) +
                                 " failed: " + (rc < 0 ? usbError(rc) : "short write"));
    }
}

void GsUsbDevice::controlIn(uint8_t request, uint16_t value, void *data, uint16_t len) {
    int rc = libusb_control_transfer(handle_, LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
                                     request, value, interface_, static_cast<unsigned char *>(data), len,
                                     CONTROL_TIMEOUT_MS);
    if (rc != len) {
        throw std::runtime_error("gs_usb: control request " + std::to_string(request) +
                                 " failed: " + (rc < 0 ? usbError(rc) : "short read"));
    }
}

void GsUsbDevice::eventLoop() {
    std::chrono::steady_clock::time_point give_up {};
    while (true) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (stopping_ && rx_transfers_active_ == 0 && tx_transfers_active_ == 0) {
                return;
            }
        }
        if (stopping_) {
            // Cancel every pass, in case a callback resubmitted its transfer just as we started stopping.
            // Cancelling a transfer that isn't in flight is harmless.
            for (libusb_transfer *t : rx_transfers_) {
                if (t) {
                    libusb_cancel_transfer(t);
                }
            }

            // Cancelled transfers should call back almost immediately; don't hang shutdown if one never does
            auto now = std::chrono::steady_clock::now();
            if (give_up == std::chrono::steady_clock::time_point {}) {
                give_up = now + std::chrono::seconds(1);
            }
            else if (now > give_up) {
                return;
            }
        }

        timeval tv = { 0, 100 * 1000 };
        int rc = libusb_handle_events_timeout_completed(ctx_, &tv, nullptr);
        if (rc != 0 && rc != LIBUSB_ERROR_INTERRUPTED) {
            fail("libusb event handling failed: " + usbError(rc));
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

void GsUsbDevice::wake() noexcept {
    uint8_t b = 1;
    // Fails with EAGAIN when the pipe is already full, which still means poll() will wake up
    (void) !write(wake_pipe_[1], &b, 1);
}

void GsUsbDevice::fail(const std::string &why) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (error_.empty()) {
            error_ = description_ + ": " + why;
        }
    }
    tx_cv_.notify_all();
    wake();
}

void GsUsbDevice::rxCallback(libusb_transfer *transfer) {
    static_cast<GsUsbDevice *>(transfer->user_data)->onRxTransfer(transfer);
}

void GsUsbDevice::onRxTransfer(libusb_transfer *transfer) {
    switch (transfer->status) {
    case LIBUSB_TRANSFER_COMPLETED:
        break;
    case LIBUSB_TRANSFER_CANCELLED:
        break;
    case LIBUSB_TRANSFER_NO_DEVICE:
        fail("adapter disconnected");
        break;
    case LIBUSB_TRANSFER_TIMED_OUT:
    case LIBUSB_TRANSFER_OVERFLOW:
        // Recoverable: drop this transfer's data and resubmit
        break;
    default:
        fail("RX transfer failed (libusb status " + std::to_string(transfer->status) + ")");
        break;
    }

    if (transfer->status == LIBUSB_TRANSFER_COMPLETED &&
        static_cast<size_t>(transfer->actual_length) >= sizeof(GsHostFrameHeader)) {
        GsHostFrameHeader hdr;
        memcpy(&hdr, transfer->buffer, sizeof(hdr));
        const uint8_t *data = transfer->buffer + sizeof(hdr);
        size_t data_avail = transfer->actual_length - sizeof(hdr);

        if (hdr.echo_id != GS_HOST_FRAME_ECHO_ID_RX) {
            // Our own frame made it onto the bus. Like a CAN_RAW socket, don't deliver it to ourselves.
            releaseTxSlot(hdr.echo_id);
        }
        else if (hdr.channel == channel_ && !(hdr.can_id & can::ERR_FLAG)) {
            if ((hdr.flags & GS_CAN_FLAG_OVERFLOW) && !overflow_reported_) {
                overflow_reported_ = true;
                fprintf(stderr, "%s: adapter RX overflow, frames were dropped\n", description_.c_str());
            }

            size_t len = (hdr.flags & GS_CAN_FLAG_FD) ? canmore_fd_dlc2len(hdr.can_dlc & 0xF)
                                                       : std::min<size_t>(hdr.can_dlc, can::MAX_DLEN);
            if (hdr.can_id & can::RTR_FLAG) {
                len = 0;
            }
            if (len <= data_avail) {
                RxFrame frame;
                frame.can_id = hdr.can_id;
                frame.len = len;
                memcpy(frame.data.data(), data, len);
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    rx_queue_.push_back(frame);
                }
                wake();
            }
        }
    }

    bool resubmit = !stopping_ && (transfer->status == LIBUSB_TRANSFER_COMPLETED ||
                                   transfer->status == LIBUSB_TRANSFER_TIMED_OUT ||
                                   transfer->status == LIBUSB_TRANSFER_OVERFLOW);
    if (resubmit) {
        int rc = libusb_submit_transfer(transfer);
        if (rc == 0) {
            return;
        }
        fail(rc == LIBUSB_ERROR_NO_DEVICE ? "adapter disconnected" : "resubmitting RX transfer failed: " + usbError(rc));
    }

    std::lock_guard<std::mutex> lock(mutex_);
    rx_transfers_active_--;
}

void GsUsbDevice::txCallback(libusb_transfer *transfer) {
    auto *self = static_cast<GsUsbDevice *>(transfer->user_data);
    GsHostFrameHeader hdr;
    memcpy(&hdr, transfer->buffer, sizeof(hdr));

    if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
        // The adapter never got the frame, so no echo is coming to free its slot
        self->releaseTxSlot(hdr.echo_id);
        if (transfer->status == LIBUSB_TRANSFER_NO_DEVICE) {
            self->fail("adapter disconnected");
        }
        else if (transfer->status != LIBUSB_TRANSFER_CANCELLED) {
            fprintf(stderr, "%s: TX transfer failed (libusb status %d)\n", self->description_.c_str(),
                    transfer->status);
        }
    }

    {
        std::lock_guard<std::mutex> lock(self->mutex_);
        self->tx_transfers_active_--;
    }
    // LIBUSB_TRANSFER_FREE_BUFFER | LIBUSB_TRANSFER_FREE_TRANSFER: libusb frees both after this returns
}

void GsUsbDevice::releaseTxSlot(uint32_t echo_id) {
    if (echo_id >= NUM_TX_SLOTS) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        tx_busy_[echo_id] = false;
    }
    tx_cv_.notify_all();
}

void GsUsbDevice::setRxFilters(const std::span<const can::Filter> &rxFilters) {
    filters_.assign(rxFilters.begin(), rxFilters.end());
}

void GsUsbDevice::populateFds(std::vector<std::weak_ptr<Canmore::PollFDDescriptor>> &descriptors) {
    descriptors.push_back(pollDescriptor_);
}

void GsUsbDevice::transmitFrame(uint32_t can_id, const std::span<const uint8_t> &data) {
    if (!useCanFd) {
        if (data.size() > can::MAX_DLEN) {
            throw std::logic_error("Attempting to transmit packet greater than maximum CAN data length");
        }
    }
    else {
        if (data.size() > can::FD_MAX_DLEN) {
            throw std::logic_error("Attempting to transmit packet greater than maximum CAN data length");
        }
        if (canmore_fd_dlc2len(canmore_fd_len2dlc(data.size())) != data.size()) {
            throw std::logic_error("Invalid CAN FD frame length: " + std::to_string(data.size()));
        }
    }

    // Wait for the adapter to have room, the same way a SocketCAN write blocks for up to SO_SNDTIMEO
    uint32_t slot = NUM_TX_SLOTS;
    {
        std::unique_lock<std::mutex> lock(mutex_);
        auto ready = [&] {
            if (!error_.empty()) {
                return true;
            }
            for (uint32_t i = 0; i < NUM_TX_SLOTS; i++) {
                if (!tx_busy_[i]) {
                    slot = i;
                    return true;
                }
            }
            return false;
        };
        if (!tx_cv_.wait_for(lock, MAX_BLOCKING_TIME, ready)) {
            throw std::system_error(std::make_error_code(std::errc::timed_out),
                                    "gs_usb write: adapter TX queue full (is anything ACKing on the bus?)");
        }
        if (!error_.empty()) {
            throw std::runtime_error(error_);
        }
        tx_busy_[slot] = true;
        tx_transfers_active_++;
    }

    auto *buf = static_cast<uint8_t *>(calloc(1, tx_frame_size_));
    libusb_transfer *transfer = buf ? libusb_alloc_transfer(0) : nullptr;
    if (!transfer) {
        free(buf);
        std::lock_guard<std::mutex> lock(mutex_);
        tx_busy_[slot] = false;
        tx_transfers_active_--;
        throw std::bad_alloc();
    }

    // Like CANSocket, every frame goes out as an FD frame with bit rate switching once FD is on
    GsHostFrameHeader hdr = {};
    hdr.echo_id = slot;
    hdr.can_id = can_id;
    hdr.can_dlc = useCanFd ? canmore_fd_len2dlc(data.size()) : data.size();
    hdr.channel = channel_;
    hdr.flags = useCanFd ? (GS_CAN_FLAG_FD | GS_CAN_FLAG_BRS) : 0;
    memcpy(buf, &hdr, sizeof(hdr));
    std::copy_n(data.data(), data.size(), buf + sizeof(hdr));

    libusb_fill_bulk_transfer(transfer, handle_, ep_out_, buf, tx_frame_size_, &GsUsbDevice::txCallback, this,
                              CONTROL_TIMEOUT_MS);
    transfer->flags = LIBUSB_TRANSFER_FREE_BUFFER | LIBUSB_TRANSFER_FREE_TRANSFER;

    int rc = libusb_submit_transfer(transfer);
    if (rc != 0) {
        libusb_free_transfer(transfer);  // frees buf too
        {
            std::lock_guard<std::mutex> lock(mutex_);
            tx_busy_[slot] = false;
            tx_transfers_active_--;
        }
        if (rc == LIBUSB_ERROR_NO_DEVICE) {
            fail("adapter disconnected");
        }
        throw std::runtime_error("gs_usb write: " + usbError(rc));
    }
}

void GsUsbDevice::clearRxBuffer() {
    uint8_t drain[64];
    while (read(wake_pipe_[0], drain, sizeof(drain)) > 0) {
    }
    std::lock_guard<std::mutex> lock(mutex_);
    rx_queue_.clear();
}

void GsUsbDevice::handleEvent(const pollfd &fd) {
    if (fd.revents & POLLIN) {
        uint8_t drain[64];
        while (read(wake_pipe_[0], drain, sizeof(drain)) > 0) {
        }

        std::deque<RxFrame> frames;
        std::string error;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            frames.swap(rx_queue_);
            error = error_;
        }

        for (const RxFrame &frame : frames) {
            bool match = std::any_of(filters_.begin(), filters_.end(), [&](const can::Filter &f) {
                bool equal = (frame.can_id & f.can_mask) == (f.can_id & ~CAN_INV_FILTER & f.can_mask);
                return (f.can_id & CAN_INV_FILTER) ? !equal : equal;
            });
            if (match) {
                handleFrame(frame.can_id, std::span<const uint8_t>(frame.data.data(), frame.len));
            }
        }

        if (!error.empty()) {
            throw std::runtime_error(error);
        }
    }
    if (fd.revents & (POLLERR | POLLHUP)) {
        throw std::runtime_error("gs_usb wake pipe unexpectedly closed");
    }
}
