#include "perception/health/camera_health.hpp"

namespace perception::health {

void CameraHealth::set_status(CameraStatus status) noexcept {
    status_ = status;
}

void CameraHealth::record_frame(std::uint64_t capture_timestamp_ns) noexcept {
    ++frames_received_;
    last_capture_timestamp_ns_ = capture_timestamp_ns;
    status_ = CameraStatus::Healthy;
}

void CameraHealth::record_drop() noexcept {
    ++frames_dropped_;
}

void CameraHealth::record_imu(const camera::ImuBatch& imu) noexcept {
    accelerometer_samples_ += static_cast<std::uint64_t>(imu.accelerometer.size());
    gyroscope_samples_ += static_cast<std::uint64_t>(imu.gyroscope.size());
    accelerometer_dropped_ = imu.accelerometer_dropped;
    gyroscope_dropped_ = imu.gyroscope_dropped;
    if (!imu.accelerometer.empty()) {
        last_accelerometer_capture_timestamp_ns_ = imu.accelerometer.back().capture_timestamp_ns;
    }
    if (!imu.gyroscope.empty()) {
        last_gyroscope_capture_timestamp_ns_ = imu.gyroscope.back().capture_timestamp_ns;
    }
}

auto CameraHealth::snapshot(std::uint64_t now_ns) const noexcept -> CameraHealthSnapshot {
    std::chrono::milliseconds age{0};
    if (last_capture_timestamp_ns_ != 0 && now_ns >= last_capture_timestamp_ns_) {
        age = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::nanoseconds(now_ns - last_capture_timestamp_ns_));
    }
    const auto sample_age = [now_ns](std::uint64_t timestamp_ns) {
        if (timestamp_ns == 0 || now_ns < timestamp_ns) {
            return std::chrono::milliseconds{0};
        }
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::nanoseconds(now_ns - timestamp_ns));
    };
    return {status_,
            frames_received_,
            frames_dropped_,
            age,
            accelerometer_samples_,
            gyroscope_samples_,
            accelerometer_dropped_,
            gyroscope_dropped_,
            sample_age(last_accelerometer_capture_timestamp_ns_),
            sample_age(last_gyroscope_capture_timestamp_ns_)};
}

} // namespace perception::health
