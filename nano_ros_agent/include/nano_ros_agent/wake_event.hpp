#pragma once
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <unistd.h>
#include <system_error>

namespace nros {

/**
 * Lets another thread wake a thread that's blocked in poll(). Poll fd() for POLLIN: signal() makes it readable until
 * drain() is called. A self-pipe instead of an eventfd so it also works on macOS
 */
class WakeEvent {
public:
    WakeEvent() {
        if (pipe(fds_) != 0) {
            throw std::system_error(errno, std::generic_category(), "pipe");
        }
        for (int fd : fds_) {
            fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_NONBLOCK);
            fcntl(fd, F_SETFD, FD_CLOEXEC);
        }
    }

    ~WakeEvent() {
        close(fds_[0]);
        close(fds_[1]);
    }

    WakeEvent(const WakeEvent &) = delete;
    WakeEvent &operator=(const WakeEvent &) = delete;

    int fd() const { return fds_[0]; }

    // safe to call from any thread
    void signal() {
        uint8_t b = 1;
        // EAGAIN means the pipe is full, so it's already signaled
        while (write(fds_[1], &b, 1) < 0 && errno == EINTR) {}
    }

    void drain() {
        uint8_t buf[64];
        while (true) {
            ssize_t n = read(fds_[0], buf, sizeof(buf));
            if (n > 0 || (n < 0 && errno == EINTR)) {
                continue;
            }
            return;
        }
    }

private:
    int fds_[2];
};

} // namespace nros
