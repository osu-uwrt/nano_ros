#pragma once
#include <vector>
#include <optional>
#include "protocol.h"
#include "wake_event.hpp"

namespace nros {

struct WireFrame {
    nros_frame_header_t header;
    std::vector<uint8_t> payload;
};

struct Readiness {
    bool readable = false;
    bool writable = false;
    bool nothing_ready = false;
    bool error = false;
};

class Transport {
public: 
    virtual ~Transport() = default;

    virtual bool init() = 0;

    virtual bool recv_frame(WireFrame &frame, int32_t timeout) = 0;

    virtual bool send_frame(const WireFrame &frame, int32_t timeout) = 0;

    // blocks until there may be a frame to recv_frame(), wake() is called, or timeout_ms passes (-1 waits forever)
    virtual Readiness wait_ready(int32_t timeout_ms) = 0;

    // makes a blocked wait_ready() return, or the next one if nothing is waiting. Safe to call from any thread
    void wake() {
        wake_.signal();
    }

    virtual bool fini() = 0;

    virtual bool is_connected() {
        return connected;
    }

protected:
    bool connected = false;
    // subclasses add wake_.fd() to what wait_ready() polls, and drain it when it fires
    WakeEvent wake_;
    
};

} // namespace nros