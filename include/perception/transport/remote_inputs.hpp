#pragma once

#include "perception/core/config.hpp"
#include "perception/transport/protocol.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

namespace perception::transport {

struct PeerMetrics {
    bool enabled{};
    std::uint64_t requests{};
    std::uint64_t failures{};
    // Samples accepted; a peer answering with no fix yet counts as a request, not a failure.
    std::uint64_t samples{};
    std::uint64_t last_sample_age_ms{};
    bool fresh{};
    std::string last_error;
};

struct RemoteInputMetrics {
    PeerMetrics gnss;
    PeerMetrics vendor_motion;
};

// Polls the two mission-side peers on their own threads and keeps only the latest sample of each.
//
// Both peers are optional: one that stops answering makes its measurement unavailable, which
// degrades localization quality. It never blocks capture or streaming, exactly as a silent IMU
// never stops RGB/depth. There is no queue here by design - a stale measurement has no value to a
// state estimator, so only the newest one is kept.
class RemoteInputs final {
  public:
    explicit RemoteInputs(core::TransportConfig config);
    ~RemoteInputs();

    RemoteInputs(const RemoteInputs&) = delete;
    auto operator=(const RemoteInputs&) -> RemoteInputs& = delete;
    RemoteInputs(RemoteInputs&&) = delete;
    auto operator=(RemoteInputs&&) -> RemoteInputs& = delete;

    // Starts one thread per enabled peer. Returns false only when a peer is enabled and the
    // platform cannot provide sockets at all.
    [[nodiscard]] auto start() -> bool;
    void stop() noexcept;

    // Latest sample, or nullopt when the peer is disabled, has never answered, or the sample is
    // older than its configured staleness timeout. Both the local arrival age and the peer's own
    // reported age are checked.
    [[nodiscard]] auto gnss() const -> std::optional<GnssSample>;
    [[nodiscard]] auto vendor_motion() const -> std::optional<VendorMotionSample>;

    [[nodiscard]] auto metrics() const -> RemoteInputMetrics;

  private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace perception::transport
