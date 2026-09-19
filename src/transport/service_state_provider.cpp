#include "perception/transport/service_state_provider.hpp"

#include <chrono>

namespace perception::transport {
namespace {

auto steady_now_ns() -> std::uint64_t {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}

auto status_name(health::CameraStatus status) -> const char* {
    switch (status) {
    case health::CameraStatus::Initializing:
        return "initializing";
    case health::CameraStatus::Healthy:
        return "healthy";
    case health::CameraStatus::Degraded:
        return "degraded";
    case health::CameraStatus::Disconnected:
        return "disconnected";
    case health::CameraStatus::Stopped:
        break;
    }
    return "stopped";
}

} // namespace

ServiceStateProvider::ServiceStateProvider(const core::ServiceConfig& config,
                                           const health::CameraHealth& camera)
    : config_(config), camera_(camera) {}

auto ServiceStateProvider::health() const -> ServiceHealthState {
    const auto snapshot = camera_.snapshot(steady_now_ns());
    const auto imu_timeout = std::chrono::milliseconds(config_.camera.imu.liveness_timeout_ms);
    // Same readiness rule the service journal uses: both motion streams must be present and fresh.
    const bool imu_ready = config_.camera.imu.enabled && snapshot.accelerometer_samples > 0 &&
                           snapshot.gyroscope_samples > 0 &&
                           snapshot.accelerometer_age <= imu_timeout &&
                           snapshot.gyroscope_age <= imu_timeout;

    ServiceHealthState state;
    state.status = status_name(snapshot.status);
    state.frames_received = snapshot.frames_received;
    state.frames_dropped = snapshot.frames_dropped;
    state.frame_age_ms = static_cast<std::uint64_t>(snapshot.frame_age.count());
    state.accelerometer_age_ms = static_cast<std::uint64_t>(snapshot.accelerometer_age.count());
    state.gyroscope_age_ms = static_cast<std::uint64_t>(snapshot.gyroscope_age.count());
    state.imu_ready = imu_ready;
    // The EKF branch does not exist yet, so readiness cannot be claimed beyond its inputs.
    state.ekf_ready = imu_ready;
    state.rtsp_enabled = config_.streaming.enabled;
    const auto uptime = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::steady_clock::now() - started_at_);
    state.uptime_s = uptime.count() > 0 ? static_cast<std::uint64_t>(uptime.count()) : 0U;
    {
        const std::lock_guard<std::mutex> guard(mutex_);
        state.accelerometer_hz = accelerometer_hz_;
        state.gyroscope_hz = gyroscope_hz_;
    }
    return state;
}

auto ServiceStateProvider::localization() const -> LocalizationState {
    // Deliberately unavailable until V0.5 fills it: a zero pose presented as an estimate is worse
    // for the mission side than an explicit absence.
    LocalizationState state;
    state.available = false;
    state.quality = LocalizationQuality::Lost;
    return state;
}

auto ServiceStateProvider::set_map_origin(double latitude_deg, double longitude_deg,
                                          std::string& error) -> bool {
    if (!config_.transport.enabled) {
        error = "transport is disabled";
        return false;
    }
    const std::lock_guard<std::mutex> guard(mutex_);
    map_origin_ = std::make_pair(latitude_deg, longitude_deg);
    return true;
}

void ServiceStateProvider::update_sample_rates(double accelerometer_hz, double gyroscope_hz) {
    const std::lock_guard<std::mutex> guard(mutex_);
    accelerometer_hz_ = accelerometer_hz;
    gyroscope_hz_ = gyroscope_hz;
}

auto ServiceStateProvider::map_origin() const -> std::optional<std::pair<double, double>> {
    const std::lock_guard<std::mutex> guard(mutex_);
    return map_origin_;
}

} // namespace perception::transport
