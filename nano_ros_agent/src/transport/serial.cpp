#include "serial.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>

using namespace nros;

static speed_t get_baud(const char *baud_str) {
    speed_t rv;
    if (0 == strcmp(baud_str, "0")) {
        rv = B0;
    }
    else if (0 == strcmp(baud_str, "50")) {
        rv = B50;
    }
    else if (0 == strcmp(baud_str, "75")) {
        rv = B75;
    }
    else if (0 == strcmp(baud_str, "110")) {
        rv = B110;
    }
    else if (0 == strcmp(baud_str, "134")) {
        rv = B134;
    }
    else if (0 == strcmp(baud_str, "150")) {
        rv = B150;
    }
    else if (0 == strcmp(baud_str, "200")) {
        rv = B200;
    }
    else if (0 == strcmp(baud_str, "300")) {
        rv = B300;
    }
    else if (0 == strcmp(baud_str, "600")) {
        rv = B600;
    }
    else if (0 == strcmp(baud_str, "1200")) {
        rv = B1200;
    }
    else if (0 == strcmp(baud_str, "1800")) {
        rv = B1800;
    }
    else if (0 == strcmp(baud_str, "2400")) {
        rv = B2400;
    }
    else if (0 == strcmp(baud_str, "4800")) {
        rv = B4800;
    }
    else if (0 == strcmp(baud_str, "9600")) {
        rv = B9600;
    }
    else if (0 == strcmp(baud_str, "19200")) {
        rv = B19200;
    }
    else if (0 == strcmp(baud_str, "38400")) {
        rv = B38400;
    }
    else if (0 == strcmp(baud_str, "57600")) {
        rv = B57600;
    }
    else if (0 == strcmp(baud_str, "115200")) {
        rv = B115200;
    }
    else if (0 == strcmp(baud_str, "230400")) {
        rv = B230400;
    } else {
        fprintf(stderr, "WARNING: unrecognized baud rate, defaulting to 115200\n");
        rv = B115200;
    }
    return rv;
}

SerialTransport::SerialTransport(const char *dev, const char *baud) : dev_path{dev}, baud_rate{baud} {}

bool SerialTransport::send_frame(const WireFrame &frame, int32_t timeout) {
    int32_t status = poll(&this->poll_fd, 1, timeout);
    if (status < 0) {
        return false; // poll failed
    } else if (status == 0) {
        return false; // nothing ready
    } else if (this->poll_fd.revents & (POLLHUP | POLLERR | POLLNVAL)) {
        return false; // disconnected or fd invalid
    } else if (!(this->poll_fd.revents & POLLOUT)) {
        return false; // something went wrong
    }

    ssize_t header_bytes_written = 0;
    while (header_bytes_written < sizeof(nros_frame_header_t)) {
        ssize_t n = write(poll_fd.fd, reinterpret_cast<const uint8_t *>(&frame.header), sizeof(nros_frame_header_t));
        if (n > 0) {
            header_bytes_written += n;
        } else if (n == 0) {
            return false;
        } else if (errno == EAGAIN || errno == EWOULDBLOCK) {
            struct pollfd pfd = this->poll_fd;
            if (poll(&pfd, 1, timeout) <= 0) {
                return false;
            }
        } else {
            fprintf(stderr, "error writing header: %s\n", strerror(errno));
            return false; 
        }
    }

    ssize_t payload_bytes_written = 0;
    while (payload_bytes_written < frame.header.length) {
        ssize_t n = write(poll_fd.fd, frame.payload.data(), frame.header.length);
        if (n > 0) {
            payload_bytes_written += n;
        } else if (n == 0) {
            return false;
        } else if (errno == EAGAIN || errno == EWOULDBLOCK) {
            struct pollfd pfd = this->poll_fd;
            if (poll(&pfd, 1, timeout) <= 0) {
                return false;
            }
        } else {
            fprintf(stderr, "error writing payload: %s\n", strerror(errno));
            return false;
        }
    }
}

bool SerialTransport::recv_frame(WireFrame &frame, int32_t timeout) {
    int32_t status = poll(&this->poll_fd, 1, timeout);
    if (status < 0) {
        return false; // poll failed
    } else if (status == 0) {
        return false; // nothing ready
    } else if (this->poll_fd.revents & (POLLHUP | POLLERR | POLLNVAL)) {
        return false; // disconnected or fd invalid
    } else if (!(this->poll_fd.revents & POLLIN)) {
        return false; // something went wrong
    }

    // read header
    uint8_t *header_buf = reinterpret_cast<uint8_t *>(&frame.header);
    ssize_t header_bytes_have = 0;
    while (header_bytes_have < sizeof(nros_frame_header_t)) {
        ssize_t n = read(this->poll_fd.fd, header_buf + header_bytes_have, sizeof(nros_frame_header_t) - header_bytes_have);
        if (n > 0) {
            header_bytes_have += n;
        } else if (n == 0) {
            return false;
        } else if (errno == EAGAIN || errno == EWOULDBLOCK) {
            struct pollfd pfd = this->poll_fd;
            if (poll(&pfd, 1, timeout) <= 0) {
                return false;
            }
        } else {
            fprintf(stderr, "error reading header: %s\n", strerror(errno));
            return false;
        }
    }

    // read payload
    frame.payload.resize(frame.header.length);
    ssize_t payload_bytes_have = 0;
    while (payload_bytes_have < frame.header.length) {
        ssize_t n = read(this->poll_fd.fd, frame.payload.data() + payload_bytes_have, frame.header.length - payload_bytes_have);
        if (n > 0) {
            payload_bytes_have += n;
        } else if (n == 0) {
            return false;
        } else if (errno == EAGAIN || errno == EWOULDBLOCK) {
            struct pollfd pfd = this->poll_fd;
            if (poll(&pfd, 1, timeout) <= 0) {
                return false;
            } 
        } else {
            fprintf(stderr, "error reading payload: %s\n", strerror(errno));
            return false;
        }
    }

    return true;
}

Readiness SerialTransport::wait_ready(int32_t timeout_ms) {
    Readiness r = { 0 };

    int32_t status = poll(&this->poll_fd, 1, timeout_ms);
    if (status == 0) {
        r.nothing_ready = true;
        return r;
    }

    if (status < 0) {
        r.error = true;
        return r;
    }

    if (this->poll_fd.revents & (POLLHUP | POLLERR | POLLNVAL)) {
        r.error = true;
    }

    if (this->poll_fd.revents & POLLIN) {
        //fprintf(stdout, "poll in\n");
        r.readable = true;
    }

    if (this->poll_fd.revents & POLLOUT) {
        //fprintf(stdout, "poll out\n");
        r.writable = true;
    }

    return r;
}

bool SerialTransport::init() {
    int fd = open(this->dev_path, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        fprintf(stderr, "couldn't open serial device at path [%s]\n", this->dev_path);
        return false;
    }
    fprintf(stdout, "opened fd\n");

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        fprintf(stderr, "counldn't get tty attributes for device\n");
        close(fd);
        return false;
    }
    fprintf(stdout, "got attrs\n");

    cfmakeraw(&tty);

    speed_t baud = get_baud(this->baud_rate);
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        fprintf(stderr, "couldn't set tty attributes for device\n");
        close(fd);
        return false;
    }
    fprintf(stdout, "set attrs\n");

    this->poll_fd.fd = fd;
    this->poll_fd.events = POLLIN | POLLOUT;

    fprintf(stdout, "success init\n");

    return true;
}

bool SerialTransport::fini() {
    if (close(this->poll_fd.fd) != 0) {
        fprintf(stderr, "couldn't close serial device (fd %d)\n", this->poll_fd.fd);
        return false;
    }
    this->poll_fd.fd = -1;
    return true;
}

SerialTransport::~SerialTransport() {
    // close(fd);
}