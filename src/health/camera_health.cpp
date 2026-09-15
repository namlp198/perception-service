#include "perception/health/camera_health.hpp"

namespace perception::health {

void CameraHealth::set_status(CameraStatus status) noexcept { status_ = status; }

void CameraHealth::record_frame(std::uint64_t capture_timestamp_ns) noexcept {
    ++frames_received_;
    last_capture_timestamp_ns_ = capture_timestamp_ns;
    status_ = CameraStatus::Healthy;
}

void CameraHealth::record_drop() noexcept { ++frames_dropped_; }

auto CameraHealth::snapshot(std::uint64_t now_ns) const noexcept -> CameraHealthSnapshot {
    std::chrono::milliseconds age{0};
    if (last_capture_timestamp_ns_ != 0 && now_ns >= last_capture_timestamp_ns_) {
        age = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::nanoseconds(now_ns - last_capture_timestamp_ns_));
    }
    return {status_, frames_received_, frames_dropped_, age};
}

}  // namespace perception::health
