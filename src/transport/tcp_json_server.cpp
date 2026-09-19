#include "perception/transport/tcp_json_server.hpp"

#include "socket_support.hpp"

#include <atomic>
#include <mutex>
#include <thread>
#include <utility>

#if defined(PERCEPTION_HAS_SPDLOG)
#include <spdlog/spdlog.h>
#endif

namespace perception::transport {

class TcpJsonServer::Impl {
  public:
    Impl(core::TransportConfig value, StateProvider& provider)
        : config(std::move(value)), router(provider) {}

    core::TransportConfig config;
    RequestRouter router;
    std::atomic_bool running{false};

    mutable std::mutex metrics_mutex;
    ServerMetrics metrics;

    void record_error(std::string message) {
        const std::lock_guard<std::mutex> guard(metrics_mutex);
        ++metrics.connection_errors;
        metrics.last_error = std::move(message);
    }

#if defined(PERCEPTION_HAS_POSIX_SOCKETS)
    sockets::Descriptor listener;
    // Written to from stop() so a blocked poll() returns immediately instead of waiting for the
    // next client to arrive.
    sockets::Descriptor wake_read;
    sockets::Descriptor wake_write;
    std::thread worker;

    auto bind_listener() -> bool {
        sockets::Descriptor descriptor(::socket(AF_INET, SOCK_STREAM, 0));
        if (!descriptor.valid()) {
            record_error("socket(): " + sockets::last_error());
            return false;
        }
        int reuse = 1;
        if (::setsockopt(descriptor.get(), SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) != 0) {
            record_error("setsockopt(SO_REUSEADDR): " + sockets::last_error());
            return false;
        }

        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_port = ::htons(config.port);
        if (config.bind_address.empty() || config.bind_address == "0.0.0.0") {
            address.sin_addr.s_addr = ::htonl(INADDR_ANY);
        } else if (::inet_pton(AF_INET, config.bind_address.c_str(), &address.sin_addr) != 1) {
            record_error("transport.bind_address is not an IPv4 address: " + config.bind_address);
            return false;
        }

        if (::bind(descriptor.get(), reinterpret_cast<const sockaddr*>(&address),
                   static_cast<socklen_t>(sizeof(address))) != 0) {
            record_error("bind(): " + sockets::last_error());
            return false;
        }
        if (::listen(descriptor.get(), 8) != 0) {
            record_error("listen(): " + sockets::last_error());
            return false;
        }
        listener = std::move(descriptor);
        return true;
    }

    auto create_wakeup() -> bool {
        int pipe_fds[2] = {-1, -1};
        if (::pipe(pipe_fds) != 0) {
            record_error("pipe(): " + sockets::last_error());
            return false;
        }
        wake_read.reset(pipe_fds[0]);
        wake_write.reset(pipe_fds[1]);
        return sockets::set_non_blocking(wake_read.get(), true);
    }

    void run() {
        while (running.load()) {
            pollfd waiters[2]{};
            waiters[0].fd = listener.get();
            waiters[0].events = POLLIN;
            waiters[1].fd = wake_read.get();
            waiters[1].events = POLLIN;

            const int ready = ::poll(waiters, 2, -1);
            if (ready < 0) {
                if (errno == EINTR) {
                    continue;
                }
                record_error("poll(): " + sockets::last_error());
                break;
            }
            if ((waiters[1].revents & POLLIN) != 0) {
                break;
            }
            if ((waiters[0].revents & POLLIN) == 0) {
                continue;
            }

            sockets::Descriptor client(::accept(listener.get(), nullptr, nullptr));
            if (!client.valid()) {
                if (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK) {
                    record_error("accept(): " + sockets::last_error());
                }
                continue;
            }
            serve(client.get());
        }
    }

    void serve(int client) {
        if (!sockets::set_io_timeouts(client, config.request_timeout_ms)) {
            record_error("socket timeout: " + sockets::last_error());
            return;
        }

        std::string request;
        std::string error;
        // Accept both framings: a caller that half-closes (payload-service's client style) and one
        // that terminates its request with a newline.
        if (!sockets::receive_until_eof(client, config.max_request_bytes, true, request, error)) {
            const std::string reply =
                make_error_response("", error.empty() ? "request could not be read" : error)
                    .dump() +
                "\n";
            std::string send_error;
            (void)sockets::send_all(client, reply, send_error);
            const std::lock_guard<std::mutex> guard(metrics_mutex);
            ++metrics.requests_rejected;
            metrics.last_error = error;
            return;
        }

        const std::string reply = router.handle(request) + "\n";
        if (!sockets::send_all(client, reply, error)) {
            record_error(error);
            return;
        }
        ::shutdown(client, SHUT_WR);
        const std::lock_guard<std::mutex> guard(metrics_mutex);
        ++metrics.requests_served;
    }
#endif
};

TcpJsonServer::TcpJsonServer(core::TransportConfig config, StateProvider& provider)
    : impl_(std::make_unique<Impl>(std::move(config), provider)) {}

TcpJsonServer::~TcpJsonServer() { stop(); }

auto TcpJsonServer::start() -> bool {
#if defined(PERCEPTION_HAS_POSIX_SOCKETS)
    if (impl_->running.load()) {
        return true;
    }
    if (!impl_->bind_listener() || !impl_->create_wakeup()) {
#if defined(PERCEPTION_HAS_SPDLOG)
        spdlog::error("transport server failed to start on port {}: {}", impl_->config.port,
                      metrics().last_error);
#endif
        return false;
    }
    impl_->running.store(true);
    impl_->worker = std::thread([impl = impl_.get()]() { impl->run(); });
#if defined(PERCEPTION_HAS_SPDLOG)
    spdlog::info("transport server listening on {}:{} protocol_version={}",
                 impl_->config.bind_address, impl_->config.port, kProtocolVersion);
#endif
    return true;
#else
#if defined(PERCEPTION_HAS_SPDLOG)
    spdlog::warn("transport server is unavailable: this platform has no POSIX sockets");
#endif
    return false;
#endif
}

void TcpJsonServer::stop() noexcept {
#if defined(PERCEPTION_HAS_POSIX_SOCKETS)
    if (!impl_->running.exchange(false)) {
        return;
    }
    if (impl_->wake_write.valid()) {
        const char token = 'x';
        // A failed wake still ends the loop: the descriptors close below and poll() returns. The
        // result is bound to a variable because glibc marks write() warn_unused_result, which a
        // plain (void) cast does not satisfy.
        const ssize_t woken = ::write(impl_->wake_write.get(), &token, 1);
        static_cast<void>(woken);
    }
    if (impl_->worker.joinable()) {
        impl_->worker.join();
    }
    impl_->listener.reset();
    impl_->wake_read.reset();
    impl_->wake_write.reset();
#if defined(PERCEPTION_HAS_SPDLOG)
    spdlog::info("transport server stopped");
#endif
#endif
}

auto TcpJsonServer::running() const noexcept -> bool { return impl_->running.load(); }

auto TcpJsonServer::metrics() const -> ServerMetrics {
    const std::lock_guard<std::mutex> guard(impl_->metrics_mutex);
    return impl_->metrics;
}

} // namespace perception::transport
