#pragma once

#include "perception/camera/camera_types.hpp"
#include "perception/streaming/stream_publisher.hpp"

#include <cstddef>

namespace perception::streaming {

class StreamManager {
public:
    explicit StreamManager(IStreamPublisher& publisher);
    [[nodiscard]] bool publish(const camera::CameraFrameSet& frame_set);
    [[nodiscard]] auto frames_published() const noexcept -> std::size_t;
    [[nodiscard]] auto frames_dropped() const noexcept -> std::size_t;

private:
    IStreamPublisher& publisher_;
    std::size_t frames_published_{0};
    std::size_t frames_dropped_{0};
};

}  // namespace perception::streaming
