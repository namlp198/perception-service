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
    void disable_imu() noexcept;
    [[nodiscard]] bool imu_enabled() const noexcept;
    [[nodiscard]] bool capture(CameraFrameSet& frame_set) override;
    [[nodiscard]] auto device_info() const -> std::optional<CameraDeviceInfo>;

  private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace perception::camera
