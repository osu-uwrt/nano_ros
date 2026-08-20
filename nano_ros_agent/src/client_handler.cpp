#include "client_handler.hpp"
#include <poll.h>
#include <stdio.h>

using namespace nros;

ClientHandler::ClientHandler(std::unique_ptr<Transport> transport) : transport(std::move(transport)) {}

// using for testing at the moment, should not be built upon in this state
void ClientHandler::run() {
    this->transport->init();
    this->running = true;
    bool has_discovered = false;
    bool needs_ack = false;
    uint8_t topic_to_ack = 0;
    bool can_sub = false;
    size_t sub_ticker = 0;

    uint8_t ucdr_serialized_byte = 0x65; // 101

    WireFrame frame;
    while (this->running) {
        Readiness r = transport->wait_ready(0);
        if (r.error) {
            fprintf(stderr, "error\n");
            break;
        }

        if (r.writable) {
            //fprintf(stdout, "writable\n");
            if (!has_discovered) {
                frame.header.magic[0] = NROS_MAGIC0;
                frame.header.magic[1] = NROS_MAGIC1;
                frame.header.type = NROS_PACKET_DISCOVER;
                frame.header.topic_id = 0;
                frame.header.length = 0;
                bool sent = transport->send_frame(frame, 0);
                if (!sent) {
                    fprintf(stderr, "problem sending discovery\n");
                } else {
                    fprintf(stderr, "sent discover\n");
                    has_discovered = true;
                }
            }

            if (needs_ack) {
                needs_ack = false;
                frame.header.length = 0;
                frame.header.type = NROS_PACKET_ACK;
                frame.header.topic_id = topic_to_ack;
                frame.payload.clear();
                transport->send_frame(frame, 0);

                fprintf(stdout, "sent ack\n\n");
            }
        }

        if (r.readable) {
            if (transport->recv_frame(frame, 0)) {
                uint8_t id = frame.header.topic_id;
                uint8_t type = frame.header.type;
                uint8_t length = frame.header.length;
                fprintf(stdout, "HEADER: id[%hhu], type[%hhu], length[%hhu]\n", id, type, length);
                switch (frame.header.type) {
                case NROS_PACKET_DATA:
                    fprintf(stdout, "data from board: ");
                    for (uint8_t byte : frame.payload) {
                        fprintf(stdout, "%02hhX", byte);
                    }
                    fprintf(stdout, "\n");
                    break;
                case NROS_PACKET_REGISTER:
                    needs_ack = true;
                    topic_to_ack = frame.header.topic_id;
                    fprintf(stdout, "registration frame\n");
                    break;
                case NROS_PACKET_REGISTER_DONE:
                    fprintf(stdout, "board done registering\n");
                    can_sub = true;
                    break;
                }
                // if (can_sub) {
                //     sub_ticker++;
                // }
                fprintf(stdout, "\n");
            }
        }
    }
}

void ClientHandler::stop() {

}

ClientHandler::~ClientHandler() {

}