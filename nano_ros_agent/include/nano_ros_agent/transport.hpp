#pragma once
#include <vector>
#include <optional>
#include "protocol.h"

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

    virtual Readiness wait_ready(int32_t timeout_ms) = 0;

    virtual bool fini() = 0;

    virtual bool is_connected() {
        return connected;
    }

protected:
    bool connected = false;
    
};

} // namespace nros