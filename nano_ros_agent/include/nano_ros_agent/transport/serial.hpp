#include "transport.hpp"
#include <filesystem>
#include <poll.h>
#include <deque>

extern "C" {
#include "nros_stream_parser.h"
}

namespace nros {

class SerialTransport : public Transport {
public:
    SerialTransport(const char *dev, const char *baud);
    ~SerialTransport();

    bool init() override;
    bool send_frame(const nros::WireFrame &frame, int32_t timeout) override;
    bool recv_frame(nros::WireFrame &frame, int32_t timeout) override;
    nros::Readiness wait_ready(int32_t timeout_ms) override;
    bool fini() override;

private:
    static void on_frame_trampoline(const nros_frame_header_t *hdr, const uint8_t payload[], void *user_data);
    void on_frame(const nros_frame_header_t *hdr, const uint8_t payload[]);

    nros_stream_parser_t parser;
    std::deque<WireFrame> completed_frames;
    struct pollfd poll_fd;
    const char *dev_path;
    const char *baud_rate;
};

} // namespace nros