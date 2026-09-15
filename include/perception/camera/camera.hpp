#pragma once

#include "perception/camera/camera_types.hpp"

namespace perception::camera {

class ICamera {
public:
    virtual ~ICamera() = default;
    [[nodiscard]] virtual bool initialize() = 0;
    [[nodiscard]] virtual bool start() = 0;
    virtual void stop() noexcept = 0;
    [[nodiscard]] virtual bool capture(CameraFrameSet& frame_set) = 0;
};

}  // namespace perception::camera
