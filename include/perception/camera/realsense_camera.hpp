#pragma once

#include "perception/camera/camera.hpp"
#include "perception/core/config.hpp"

#include <memory>

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

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace perception::camera
