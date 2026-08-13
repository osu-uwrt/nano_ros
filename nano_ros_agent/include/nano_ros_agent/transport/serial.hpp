#include "transport.hpp"
#include <filesystem>
#include <poll.h>

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
    struct pollfd poll_fd;
    const char *dev_path;
    const char *baud_rate;
};

} // namespace nros