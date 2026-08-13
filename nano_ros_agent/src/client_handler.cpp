#include "client_handler.hpp"
#include <poll.h>
#include <stdio.h>

using namespace nros;

ClientHandler::ClientHandler(std::unique_ptr<Transport> transport) : transport(std::move(transport)) {}

void ClientHandler::run() {
    this->transport->init();
    this->running = true;
    WireFrame frame;
    while (this->running) {
        Readiness r = transport->wait_ready(0);
        if (r.error) {
            fprintf(stderr, "error\n");
            break;
        }
        
        if (r.writable) {
            //fprintf(stdout, "writable\n");
        }

        if (r.readable) {
            transport->recv_frame(frame, 0);
            fprintf(stdout, "got a frame\n");
            uint8_t id = frame.header.topic_id;
            uint8_t type = frame.header.type;
            uint8_t length = frame.header.length;
            fprintf(stdout, "HEADER: id[%hhu], type[%hhu], length[%hhu1]\n", id, type, length);
        }
    }
}

void ClientHandler::stop() {

}

ClientHandler::~ClientHandler() {

}