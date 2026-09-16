#pragma once

#include "perception/camera/camera.hpp"
#include "perception/camera/camera_info.hpp"
#include "perception/core/config.hpp"

#include <memory>
#include <optional>

namespace perception::camera {

class RealSenseCamera final : public ICamera {
  public:
    explicit RealSenseCamera(core::CameraConfig config);
    ~RealSenseCamera() override;

    RealSenseCamera(const RealSenseCamera&) = delete;
    auto operator=(const RealSenseCamera&) -> RealSenseCamera& = delete;
    RealSenseCamera(RealSenseCamera&&) noexcept;
    auto operator=(RealSenseCamera&&) noexcept -> RealSenseCamera&;

    [[nodiscard]] bool initialize() override;
    [[nodiscard]] bool start() override;
    void stop() noexcept override;
    [[nodiscard]] bool capture(CameraFrameSet& frame_set) override;
    [[nodiscard]] auto device_info() const -> std::optional<CameraDeviceInfo>;
    // Operator-triggered USB re-enumeration of the selected device (equivalent to a replug).
    // Never invoked automatically: it interrupts RGB/depth for several seconds.
    [[nodiscard]] bool hardware_reset();

  private:
    class Impl;
    std::unique_ptr<Impl> impl_;

    // Motion Module session management; independent of the video pipeline lifecycle.
    bool start_motion() noexcept;
    void stop_motion() noexcept;
    void update_motion_state() noexcept;
};

} // namespace perception::camera
