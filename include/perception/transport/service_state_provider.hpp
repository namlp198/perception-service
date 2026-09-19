#pragma once

#include "perception/core/config.hpp"
#include "perception/health/camera_health.hpp"
#include "perception/transport/protocol.hpp"

#include <chrono>
#include <mutex>
#include <optional>
#include <utility>

namespace perception::transport {

// Bridges the running service to the transport contract. It reads the V0.1 camera health snapshot
// and, once the V0.5 branch exists, the EKF estimate. Keeping it out of RequestRouter is what lets
// the whole contract be tested without a camera.
class ServiceStateProvider final : public StateProvider {
  public:
    ServiceStateProvider(const core::ServiceConfig& config, const health::CameraHealth& camera);

    [[nodiscard]] auto health() const -> ServiceHealthState override;
    [[nodiscard]] auto localization() const -> LocalizationState override;
    auto set_map_origin(double latitude_deg, double longitude_deg, std::string& error)
        -> bool override;

    // Sample rates are computed once per metrics interval by the service loop; recomputing them
    // here would duplicate that window and could disagree with the journal.
    void update_sample_rates(double accelerometer_hz, double gyroscope_hz);

    [[nodiscard]] auto map_origin() const -> std::optional<std::pair<double, double>>;

  private:
    const core::ServiceConfig& config_;
    const health::CameraHealth& camera_;
    // Service start, so `uptime_s` reports something rather than a constant zero.
    std::chrono::steady_clock::time_point started_at_{std::chrono::steady_clock::now()};
    mutable std::mutex mutex_;
    double accelerometer_hz_{0.0};
    double gyroscope_hz_{0.0};
    std::optional<std::pair<double, double>> map_origin_;
};

} // namespace perception::transport
