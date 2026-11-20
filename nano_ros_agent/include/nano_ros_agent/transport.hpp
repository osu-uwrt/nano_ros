#pragma once
#include <vector>
#include <optional>
#include "protocol.h"
#include "wake_event.hpp"

namespace nros {

/**
 * PeerID is to be implemented and determined by the code that inherits Transport.
 * 
 * For example, with canmore transport, the PeerID is just the client id from 
 * the wire.
 * 
 * The code/class inheriting this could also just maintain a mapping of PeerID to 
 * its own way of addressing clients in the case a uint32_t is not sufficient.
 */
using PeerID = uint32_t;

/**
 * a nano_ros message the way it appears on the wire: [header | payload], ommitting any 
 * upper level header or CRC
 */
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

/**
 * A message either coming from a client or going out to a client
 */
struct DeviceMessage {
    PeerID peer; // the id of the device that sent this frame or who to send this message to
    WireFrame frame;
};

/**
 * Reserved value so transport code can decide if a message is meant to be a broadcast
 * i.e. a broadcast of DISCOVER onto a bus so all clients give up their topics
 */
constexpr PeerID PEER_BROADCAST = UINT32_MAX;


/**
 * @brief abstract class that transport implementations inherit so the agents control loop stays 
 *        the same regardless of underlying implementation
 */
class Transport {
public: 
    virtual ~Transport() = default;

    virtual bool init() = 0;

    virtual bool recv_frame(PeerID &from, WireFrame &frame, int32_t timeout) = 0;

    virtual bool send_frame(const PeerID to, const WireFrame &frame, int32_t timeout) = 0;

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