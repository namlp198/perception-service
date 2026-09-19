#pragma once

// Implementation detail of src/transport/ only; it is deliberately not published under include/
// because no other subsystem may depend on socket types (see AGENTS.md dependency boundaries).
//
// Every call here is bounded by an explicit timeout. This boundary runs beside live video capture,
// so no socket operation may block indefinitely and none of them may raise SIGPIPE.

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>

#if defined(__unix__) || defined(__APPLE__)
#define PERCEPTION_HAS_POSIX_SOCKETS 1
#endif

#if defined(PERCEPTION_HAS_POSIX_SOCKETS)

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace perception::transport::sockets {

#if defined(MSG_NOSIGNAL)
constexpr int kSendFlags = MSG_NOSIGNAL;
#else
constexpr int kSendFlags = 0;
#endif

// Owns a file descriptor so that every early return closes it exactly once.
class Descriptor {
  public:
    Descriptor() = default;
    explicit Descriptor(int descriptor) noexcept : descriptor_(descriptor) {}
    ~Descriptor() { reset(); }

    Descriptor(const Descriptor&) = delete;
    auto operator=(const Descriptor&) -> Descriptor& = delete;
    Descriptor(Descriptor&& other) noexcept : descriptor_(other.release()) {}
    auto operator=(Descriptor&& other) noexcept -> Descriptor& {
        if (this != &other) {
            reset(other.release());
        }
        return *this;
    }

    [[nodiscard]] auto get() const noexcept -> int { return descriptor_; }
    [[nodiscard]] auto valid() const noexcept -> bool { return descriptor_ >= 0; }

    auto release() noexcept -> int {
        const int held = descriptor_;
        descriptor_ = -1;
        return held;
    }

    void reset(int descriptor = -1) noexcept {
        if (descriptor_ >= 0) {
            ::close(descriptor_);
        }
        descriptor_ = descriptor;
    }

  private:
    int descriptor_{-1};
};

[[nodiscard]] inline auto last_error() -> std::string { return std::strerror(errno); }

inline auto set_non_blocking(int descriptor, bool enabled) -> bool {
    const int flags = ::fcntl(descriptor, F_GETFL, 0);
    if (flags < 0) {
        return false;
    }
    const int updated = enabled ? (flags | O_NONBLOCK) : (flags & ~O_NONBLOCK);
    return ::fcntl(descriptor, F_SETFL, updated) == 0;
}

inline auto set_io_timeouts(int descriptor, std::uint32_t timeout_ms) -> bool {
    timeval timeout{};
    timeout.tv_sec = static_cast<time_t>(timeout_ms / 1'000U);
    timeout.tv_usec = static_cast<suseconds_t>((timeout_ms % 1'000U) * 1'000U);
    const bool receive_set =
        ::setsockopt(descriptor, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) == 0;
    const bool send_set =
        ::setsockopt(descriptor, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout)) == 0;
    return receive_set && send_set;
}

// Connects with a bounded deadline instead of the kernel default, which can exceed a minute.
[[nodiscard]] inline auto connect_with_timeout(std::string_view host, std::uint16_t port,
                                               std::uint32_t timeout_ms, std::string& error)
    -> Descriptor {
    Descriptor socket_descriptor(::socket(AF_INET, SOCK_STREAM, 0));
    if (!socket_descriptor.valid()) {
        error = "socket(): " + last_error();
        return {};
    }

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = ::htons(port);
    const std::string host_text(host);
    if (::inet_pton(AF_INET, host_text.c_str(), &address.sin_addr) != 1) {
        error = "not an IPv4 address: " + host_text;
        return {};
    }

    if (!set_non_blocking(socket_descriptor.get(), true)) {
        error = "fcntl(): " + last_error();
        return {};
    }

    const int connect_result = ::connect(socket_descriptor.get(),
                                         reinterpret_cast<const sockaddr*>(&address),
                                         static_cast<socklen_t>(sizeof(address)));
    if (connect_result != 0) {
        if (errno != EINPROGRESS) {
            error = "connect(): " + last_error();
            return {};
        }
        pollfd waiter{};
        waiter.fd = socket_descriptor.get();
        waiter.events = POLLOUT;
        const int ready = ::poll(&waiter, 1, static_cast<int>(timeout_ms));
        if (ready == 0) {
            error = "connect timed out";
            return {};
        }
        if (ready < 0) {
            error = "poll(): " + last_error();
            return {};
        }
        int pending_error = 0;
        auto length = static_cast<socklen_t>(sizeof(pending_error));
        if (::getsockopt(socket_descriptor.get(), SOL_SOCKET, SO_ERROR, &pending_error, &length) !=
            0) {
            error = "getsockopt(): " + last_error();
            return {};
        }
        if (pending_error != 0) {
            error = "connect(): " + std::string(std::strerror(pending_error));
            return {};
        }
    }

    if (!set_non_blocking(socket_descriptor.get(), false) ||
        !set_io_timeouts(socket_descriptor.get(), timeout_ms)) {
        error = "socket option: " + last_error();
        return {};
    }
    return socket_descriptor;
}

inline auto send_all(int descriptor, std::string_view payload, std::string& error) -> bool {
    std::size_t sent = 0;
    while (sent < payload.size()) {
        const ssize_t written =
            ::send(descriptor, payload.data() + sent, payload.size() - sent, kSendFlags);
        if (written > 0) {
            sent += static_cast<std::size_t>(written);
            continue;
        }
        if (written < 0 && errno == EINTR) {
            continue;
        }
        error = "send(): " + last_error();
        return false;
    }
    return true;
}

// Reads until the peer closes, `max_bytes` is reached, or the socket timeout expires. When
// `stop_at_newline` is set a complete newline-terminated reply also ends the read, so a peer that
// keeps its write side open is still served.
inline auto receive_until_eof(int descriptor, std::size_t max_bytes, bool stop_at_newline,
                              std::string& out, std::string& error) -> bool {
    out.clear();
    char buffer[4'096];
    while (out.size() <= max_bytes) {
        const ssize_t received = ::recv(descriptor, buffer, sizeof(buffer), 0);
        if (received == 0) {
            return true;
        }
        if (received < 0) {
            if (errno == EINTR) {
                continue;
            }
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                error = "receive timed out";
                return !out.empty();
            }
            error = "recv(): " + last_error();
            return false;
        }
        out.append(buffer, static_cast<std::size_t>(received));
        if (stop_at_newline && out.find('\n') != std::string::npos) {
            return true;
        }
    }
    error = "response exceeded its size limit";
    return false;
}

} // namespace perception::transport::sockets

#endif // PERCEPTION_HAS_POSIX_SOCKETS
