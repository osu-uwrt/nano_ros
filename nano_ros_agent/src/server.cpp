#include <iostream>
#include <memory>
#include "serial.hpp"
#include "transport.hpp"
#include "client_handler.hpp"

int main(int argc, char *argv[]) {
    const char *port = "/dev/ttyACM0";
    const char *baud = "115200";
    // std::unique_ptr<nros::Transport> t = std::make_unique<nros::SerialTransport>(port.c_str(), "11520");
    nros::ClientHandler handler(std::make_unique<nros::SerialTransport>(port, baud));
    handler.run();
    return 0;
}