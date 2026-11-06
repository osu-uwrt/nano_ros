#include "transport.hpp"
#include <filesystem>
#include <deque>
#include "canmore_cpp/MsgAgent.hpp"

namespace nros {

class CanmoreTransport : public Transport, private Canmore::AgentMsgHandler {
public:
    CanmoreTransport(std::string ifname) : ifname_(std::move(ifname)) {}
    ~CanmoreTransport();

    bool init() override;
    bool send_frame(const nros::WireFrame &frame, int32_t timeout) override;
    bool recv_frame(nros::WireFrame &frame, int32_t timeout) override;
    nros::Readiness wait_ready(int32_t timeout_ms) override;
    bool fini() override;

private:
    // Canmore::AgentMsgHandler overrides
    void handleMessage(uint8_t clientId, uint8_t subtype, std::span<const uint8_t> data) override;
    void handleDecodeError(uint8_t clientId, unsigned int errorCode) override;
    void handleConflictingAgentError() override;

    // puts the transport's wake fd in the poll group, so wake() interrupts processEvent
    class WakePollFd : public Canmore::PollFDHandler {
    public:
        explicit WakePollFd(WakeEvent &wake)
            : wake_(wake), descriptor_(Canmore::PollFDDescriptor::create(*this, wake.fd(), POLLIN)) {}

        void populateFds(std::vector<std::weak_ptr<Canmore::PollFDDescriptor>> &descriptors) override {
            descriptors.push_back(descriptor_);
        }

    protected:
        void handleEvent(const pollfd &fd) override {
            (void)fd;
            wake_.drain();
        }

    private:
        WakeEvent &wake_;
        std::shared_ptr<Canmore::PollFDDescriptor> descriptor_;
    };

    /**
     * same as #define CANMORE_MSG_SUBTYPE_XRCE_DDS 0 from canbus driver, should add subtype to reflect
     * nros agent. Problem exists however that in canbus_msg_write which transport code invokes, you 
     * cannot choose a subtype, so that needs implemented as well and remains a TODO in the driver
     */
    static constexpr uint8_t NROS_SUBTYPE = 0;
    static constexpr size_t MAX_PAYLOAD = CANMORE_MAX_MSG_LENGTH - sizeof(nros_frame_header_t);
    std::string ifname_;
    std::unique_ptr<Canmore::MsgAgent> can_agent_;
    std::unique_ptr<WakePollFd> wake_poll_fd_;
    std::unique_ptr<Canmore::PollGroup> group_;
    std::deque<WireFrame> completed_frames_;
    uint8_t peer_client_id_;
    
};

} // namespace nros