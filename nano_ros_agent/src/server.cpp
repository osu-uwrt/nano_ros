#include <iostream>
#include <memory>
#include "serial.hpp"
#include "transport.hpp"
#include "agent.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    const char *port = "/dev/ttyACM1";
    const char *baud = "115200";
    nros::Agent handler(std::make_unique<nros::SerialTransport>(port, baud));
    handler.run();
    return 0;
}