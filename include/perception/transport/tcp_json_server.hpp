#pragma once

#include "perception/core/config.hpp"
#include "perception/transport/protocol.hpp"

#include <cstdint>
#include <memory>
#include <string>

namespace perception::transport {

struct ServerMetrics {
    std::uint64_t requests_served{};
    std::uint64_t requests_rejected{};
    std::uint64_t connection_errors{};
    std::string last_error;
};

// Serves the perception API on its own thread, one request per connection, with payload-service's
// framing. It never touches the camera or the streaming branch: a stalled or hostile client can
// only delay other clients of this port, never capture or RTSP.
class TcpJsonServer final {
  public:
    TcpJsonServer(core::TransportConfig config, StateProvider& provider);
    ~TcpJsonServer();

    TcpJsonServer(const TcpJsonServer&) = delete;
    auto operator=(const TcpJsonServer&) -> TcpJsonServer& = delete;
    TcpJsonServer(TcpJsonServer&&) = delete;
    auto operator=(TcpJsonServer&&) -> TcpJsonServer& = delete;

    // Returns false when the port cannot be bound or the platform has no POSIX sockets. A failed
    // start is reported to the caller and must not be treated as a reason to stop streaming.
    [[nodiscard]] auto start() -> bool;
    void stop() noexcept;
    [[nodiscard]] auto running() const noexcept -> bool;
    [[nodiscard]] auto metrics() const -> ServerMetrics;

  private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace perception::transport
