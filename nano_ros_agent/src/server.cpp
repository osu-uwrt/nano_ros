#include <cstdio>
#include <memory>
#include <string>
#include <vector>
#include "serial.hpp"
#include "transport.hpp"
#include "agent.hpp"

static void usage(const char *prog) {
    fprintf(stderr,
            "usage: %s serial <device> [baud]\n"
            "\n"
            "serial: baud defaults to 115200\n",
            prog);
}

// Builds the transport from the non-ROS arguments, or prints why it can't and returns null
static std::unique_ptr<nros::Transport> parse_transport(const std::vector<std::string> &args) {
    const char *prog = args.empty() ? "nano_ros_agent" : args[0].c_str();
    if (args.size() < 2) {
        usage(prog);
        return nullptr;
    }

    const std::string &kind = args[1];
    if (kind == "-h" || kind == "--help") {
        usage(prog);
        return nullptr;
    }

    if (kind == "serial") {
        if (args.size() < 3 || args.size() > 4) {
            usage(prog);
            return nullptr;
        }
        // SerialTransport keeps these pointers; args lives in main for as long as the transport does
        const char *baud = args.size() == 4 ? args[3].c_str() : "115200";
        return std::make_unique<nros::SerialTransport>(args[2].c_str(), baud);
    }

    if (kind == "can") {
        fprintf(stderr, "the can transport isn't supported yet\n");
        return nullptr;
    }

    fprintf(stderr, "unknown transport: %s\n\n", kind.c_str());
    usage(prog);
    return nullptr;
}

int main(int argc, char *argv[]) {
    // Strips --ros-args and friends, leaving our own arguments
    std::vector<std::string> args = rclcpp::init_and_remove_ros_arguments(argc, argv);

    std::unique_ptr<nros::Transport> transport = parse_transport(args);
    if (!transport) {
        rclcpp::shutdown();
        return 2;
    }

    nros::Agent handler(std::move(transport));
    if (!handler.run()) {
        fprintf(stderr, "couldn't open the transport\n");
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
