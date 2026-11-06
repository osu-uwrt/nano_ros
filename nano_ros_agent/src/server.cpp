#include <iostream>
#include <memory>
#include "serial.hpp"
#include "transport.hpp"
#include "agent.hpp"

void setup(std::unique_ptr<nros::Transport> &transport, int argc, char *argv[]) {
    for (size_t i = 1; i < argc; i++) {
        if (strcmp(argv[i], "serial") == 0) {
            const char *dev_path = argv[i + 1];
            const char *baud = "115200";
            transport = std::make_unique<nros::SerialTransport>(dev_path, baud);
        } else if (strcmp(argv[i], "can") == 0) {
        }
    }
}

int main(int argc, char *argv[]) {
    std::unique_ptr<nros::Transport> transport;
    rclcpp::init(argc, argv);
    // /dev/nros-ros comes from udev/70-nano-ros.rules
    const char *port = argc > 1 ? argv[1] : "/dev/nros-ros";
    const char *baud = "115200";
    nros::Agent handler(std::make_unique<nros::SerialTransport>(port, baud));
    handler.run();
    return 0;
}