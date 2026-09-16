#pragma once

#include "perception/camera/camera_types.hpp"

#include <chrono>
#include <cstdint>

namespace perception::health {

enum class CameraStatus { Initializing, Healthy, Degraded, Disconnected, Stopped };

struct CameraHealthSnapshot {
    CameraStatus status{CameraStatus::Stopped};
    std::uint64_t frames_received{};
    std::uint64_t frames_dropped{};
    std::chrono::milliseconds frame_age{};
    std::uint64_t accelerometer_samples{};
    std::uint64_t gyroscope_samples{};
    std::uint64_t accelerometer_dropped{};
    std::uint64_t gyroscope_dropped{};
    std::chrono::milliseconds accelerometer_age{};
    std::chrono::milliseconds gyroscope_age{};
};

class CameraHealth {
  public:
    void set_status(CameraStatus status) noexcept;
    void record_frame(std::uint64_t capture_timestamp_ns) noexcept;
    void record_drop() noexcept;
    void record_imu(const camera::ImuBatch& imu) noexcept;
    [[nodiscard]] auto snapshot(std::uint64_t now_ns) const noexcept -> CameraHealthSnapshot;

  private:
    CameraStatus status_{CameraStatus::Stopped};
    std::uint64_t frames_received_{0};
    std::uint64_t frames_dropped_{0};
    std::uint64_t last_capture_timestamp_ns_{0};
    std::uint64_t accelerometer_samples_{0};
    std::uint64_t gyroscope_samples_{0};
    std::uint64_t accelerometer_dropped_{0};
    std::uint64_t gyroscope_dropped_{0};
    std::uint64_t last_accelerometer_capture_timestamp_ns_{0};
    std::uint64_t last_gyroscope_capture_timestamp_ns_{0};
};

} // namespace perception::health
