#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>
#include "serial.hpp"
#include "canmore.hpp"
#include "transport.hpp"
#include "agent.hpp"

static void usage(const char *prog) {
    fprintf(stderr,
            "usage: %s serial <device> [baud]\n"
            "       %s can [<ifname>] [options]\n"
            "\n"
            "serial: baud defaults to 115200\n"
            "\n"
            "can: with an interface name (e.g. can0) uses SocketCAN, configured with `ip link`. Without one, uses\n"
            "the first gs_usb (candleLight) adapter the kernel isn't using.\n"
            "  --gs-usb            use gs_usb even on Linux, taking the adapter from the kernel driver if needed\n"
            "  --serial <number>   gs_usb adapter serial number (default: first adapter found)\n"
            "  --channel <n>       gs_usb adapter channel (default: 0)\n"
            "  --bitrate <bit/s>   gs_usb nominal bitrate (default: 1000000)\n"
            "  --dbitrate <bit/s>  gs_usb CAN FD data bitrate, 0 for classic CAN (default: 5000000)\n",
            prog, prog);
}

static bool parse_uint(const char *s, unsigned long max, unsigned long &out) {
    if (!s || !*s || *s == '-') {
        return false;
    }
    char *end;
    errno = 0;
    out = strtoul(s, &end, 10);
    return errno == 0 && *end == '\0' && out <= max;
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
        nros::CanmoreOptions opts;
        for (size_t i = 2; i < args.size(); i++) {
            const std::string &arg = args[i];
            bool has_value = i + 1 < args.size();
            unsigned long n;

            if (arg == "--gs-usb") {
                opts.backend = nros::CanmoreOptions::Backend::GsUsb;
            } else if (arg == "--serial" && has_value) {
                opts.gs_usb.serial = args[++i];
            } else if (arg == "--channel" && has_value && parse_uint(args[i + 1].c_str(), 255, n)) {
                opts.gs_usb.channel = n;
                i++;
            } else if (arg == "--bitrate" && has_value && parse_uint(args[i + 1].c_str(), 1000000, n) && n > 0) {
                opts.gs_usb.bitrate = n;
                i++;
            } else if (arg == "--dbitrate" && has_value && parse_uint(args[i + 1].c_str(), UINT32_MAX, n)) {
                opts.gs_usb.data_bitrate = n;
                i++;
            } else if (arg[0] != '-' && opts.ifname.empty()) {
                opts.ifname = arg;
            } else {
                fprintf(stderr, "bad or incomplete argument: %s\n\n", arg.c_str());
                usage(prog);
                return nullptr;
            }
        }

        if (opts.backend == nros::CanmoreOptions::Backend::GsUsb && !opts.ifname.empty()) {
            fprintf(stderr, "--gs-usb doesn't take an interface name (use --serial to pick an adapter)\n");
            return nullptr;
        }
        return std::make_unique<nros::CanmoreTransport>(opts);
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
