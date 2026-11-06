#pragma once

#include "canmore_cpp/PollFD.hpp"
#include "canmore_cpp/span_compat.hpp"

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

struct libusb_context;
struct libusb_device_handle;
struct libusb_transfer;

namespace nros {

/**
 * CAN ID flags and masks. Same values as SocketCAN's <linux/can.h> (and the gs_usb wire format), redefined
 * here because that header doesn't exist on macOS.
 */
namespace can {
constexpr uint32_t EFF_FLAG = 0x80000000U;
constexpr uint32_t RTR_FLAG = 0x40000000U;
constexpr uint32_t ERR_FLAG = 0x20000000U;
constexpr uint32_t SFF_MASK = 0x000007FFU;
constexpr uint32_t EFF_MASK = 0x1FFFFFFFU;
constexpr size_t MAX_DLEN = 8;
constexpr size_t FD_MAX_DLEN = 64;

/** Same matching rule as SocketCAN's struct can_filter: (rx_id & can_mask) == (can_id & can_mask) */
struct Filter {
    uint32_t can_id;
    uint32_t can_mask;
};
}  // namespace can

struct GsUsbOptions {
    std::string serial;               // USB serial number to open; empty opens the first gs_usb adapter found
    uint8_t channel = 0;              // channel on multi-channel adapters
    uint32_t bitrate = 1000000;       // nominal (arbitration) bitrate
    uint32_t data_bitrate = 5000000;  // CAN FD data bitrate; 0 runs the adapter in classic CAN mode
    bool force_no_canfd = false;      // like CANSocket's forceNoCanFd
    bool detach_kernel_driver = true; // Linux only: take the adapter from the kernel's gs_usb driver if it has it
};

/**
 * @brief Userspace (libusb) driver for gs_usb / candleLight CAN adapters, for hosts without SocketCAN (macOS, WSL).
 *
 * Mirrors Canmore::CANSocket so the CANmore message layer on top looks the same as on SocketCAN:
 *  - CAN FD is used when the adapter supports it and a data bitrate is configured (unless forced off)
 *  - FD frames are sent with bit rate switching
 *  - transmitFrame blocks for at most 50 ms waiting for the adapter to accept the frame, then throws
 *  - RX filters use SocketCAN's matching rules (applied in software)
 *  - the adapter's own TX echoes and error frames are not delivered, like a default CAN_RAW socket
 *  - a lost device makes handleEvent throw, like a SocketCAN POLLERR
 *
 * libusb runs on a background thread. Received frames are queued and a self-pipe wakes up poll(), so this is a
 * Canmore::PollFDHandler and handleFrame is always called from the thread running the PollGroup.
 */
class GsUsbDevice : public Canmore::PollFDHandler {
public:
    explicit GsUsbDevice(const GsUsbOptions &opts);
    virtual ~GsUsbDevice();

    GsUsbDevice(GsUsbDevice const &) = delete;
    GsUsbDevice &operator=(GsUsbDevice const &) = delete;

    size_t getMaxFrameSize() const { return useCanFd ? can::FD_MAX_DLEN : can::MAX_DLEN; }

    /** Human-readable adapter description, e.g. "gs_usb 1d50:606f serial 0042 ch0, 1000000/5000000 bit/s CAN FD" */
    const std::string &description() const { return description_; }

protected:
    void transmitFrame(uint32_t can_id, const std::span<const uint8_t> &data);
    void clearRxBuffer();
    void setRxFilters(const std::span<const can::Filter> &rxFilters);
    virtual void handleFrame(uint32_t can_id, const std::span<const uint8_t> &data) = 0;
    bool usingCanFd() const { return useCanFd; }

public:
    void populateFds(std::vector<std::weak_ptr<Canmore::PollFDDescriptor>> &descriptors) override;

protected:
    void handleEvent(const pollfd &fd) override;

private:
    struct RxFrame {
        uint32_t can_id;
        uint8_t len;
        std::array<uint8_t, can::FD_MAX_DLEN> data;
    };

    static constexpr size_t NUM_RX_TRANSFERS = 8;
    static constexpr uint32_t NUM_TX_SLOTS = 10;  // same as the Linux gs_usb driver's GS_MAX_TX_URBS

    void open(const GsUsbOptions &opts);
    void configure(const GsUsbOptions &opts);
    void close() noexcept;
    void controlOut(uint8_t request, uint16_t value, const void *data, uint16_t len);
    void controlIn(uint8_t request, uint16_t value, void *data, uint16_t len);
    void eventLoop();
    void wake() noexcept;
    void fail(const std::string &why);
    static void rxCallback(libusb_transfer *transfer);
    static void txCallback(libusb_transfer *transfer);
    void onRxTransfer(libusb_transfer *transfer);
    void releaseTxSlot(uint32_t echo_id);

    libusb_context *ctx_ = nullptr;
    libusb_device_handle *handle_ = nullptr;
    int interface_ = -1;
    uint8_t ep_in_ = 0;
    uint8_t ep_out_ = 0;
    size_t rx_buf_size_ = 0;
    size_t tx_frame_size_ = 0;
    uint8_t channel_ = 0;
    bool useCanFd = false;
    std::string description_;

    std::vector<can::Filter> filters_;

    std::thread event_thread_;
    std::atomic<bool> stopping_ { false };
    std::vector<libusb_transfer *> rx_transfers_;
    std::vector<std::vector<uint8_t>> rx_buffers_;

    std::mutex mutex_;  // guards everything below
    std::condition_variable tx_cv_;
    std::deque<RxFrame> rx_queue_;
    std::array<bool, NUM_TX_SLOTS> tx_busy_ {};
    unsigned rx_transfers_active_ = 0;
    unsigned tx_transfers_active_ = 0;
    std::string error_;
    bool overflow_reported_ = false;

    int wake_pipe_[2] = { -1, -1 };
    std::shared_ptr<Canmore::PollFDDescriptor> pollDescriptor_;
};

}  // namespace nros
