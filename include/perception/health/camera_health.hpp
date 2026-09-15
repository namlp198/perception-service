#pragma once

#include <chrono>
#include <cstdint>

namespace perception::health {

enum class CameraStatus { Initializing, Healthy, Degraded, Disconnected, Stopped };

struct CameraHealthSnapshot {
    CameraStatus status{CameraStatus::Stopped};
    std::uint64_t frames_received{};
    std::uint64_t frames_dropped{};
    std::chrono::milliseconds frame_age{};
};

class CameraHealth {
public:
    void set_status(CameraStatus status) noexcept;
    void record_frame(std::uint64_t capture_timestamp_ns) noexcept;
    void record_drop() noexcept;
    [[nodiscard]] auto snapshot(std::uint64_t now_ns) const noexcept -> CameraHealthSnapshot;

private:
    CameraStatus status_{CameraStatus::Stopped};
    std::uint64_t frames_received_{0};
    std::uint64_t frames_dropped_{0};
    std::uint64_t last_capture_timestamp_ns_{0};
};

}  // namespace perception::health
