#include "canmore_link.hpp"

#include "canmore_cpp/MsgAgent.hpp"

#include <net/if.h>
#include <system_error>

using namespace nros;

namespace {

/** titan_canmore's own Canmore::MsgAgent, unchanged, so SocketCAN behaves exactly like the rest of the team's tools */
class SocketCanLink : public CanmoreLink, private Canmore::AgentMsgHandler {
public:
    SocketCanLink(const std::string &ifname, unsigned ifIndex, CanmoreMsgHandler &handler,
                  std::span<const uint8_t> clientIdSelect):
        handler_(handler), agent_(ifIndex, *this, clientIdSelect),
        description_("socketcan " + ifname + (agent_.getMaxFrameSize() > CAN_MAX_DLEN ? " (CAN FD)" : " (classic CAN)")) {}

    void transmitMessage(uint8_t clientId, uint8_t subtype, std::span<const uint8_t> data) override {
        agent_.transmitMessage(clientId, subtype, data);
    }

    Canmore::PollFD &pollFd() override { return agent_; }

    std::string description() const override { return description_; }

private:
    void handleMessage(uint8_t clientId, uint8_t subtype, std::span<const uint8_t> data) override {
        handler_.handleMessage(clientId, subtype, data);
    }
    void handleDecodeError(uint8_t clientId, unsigned int errorCode) override {
        handler_.handleDecodeError(clientId, errorCode);
    }
    void handleConflictingAgentError() override { handler_.handleConflictingAgentError(); }

    CanmoreMsgHandler &handler_;
    Canmore::MsgAgent agent_;
    std::string description_;
};

}  // namespace

bool nros::socketcan_available() {
    return true;
}

std::unique_ptr<CanmoreLink> nros::make_socketcan_link(const std::string &ifname, CanmoreMsgHandler &handler,
                                                       std::span<const uint8_t> clientIdSelect) {
    unsigned idx = if_nametoindex(ifname.c_str());
    if (!idx) {
        throw std::system_error(errno, std::generic_category(), "socketcan: no interface named " + ifname);
    }
    return std::make_unique<SocketCanLink>(ifname, idx, handler, clientIdSelect);
}
