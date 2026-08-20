#include <iostream>
#include <memory>
#include "serial.hpp"
#include "transport.hpp"
#include "client_handler.hpp"

int main(int argc, char *argv[]) {
    const char *port = "/dev/ttyACM1";
    const char *baud = "115200";
    nros::ClientHandler handler(std::make_unique<nros::SerialTransport>(port, baud));
    handler.run();
    return 0;
}