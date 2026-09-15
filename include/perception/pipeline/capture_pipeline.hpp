#pragma once

#include "perception/camera/camera.hpp"
#include "perception/health/camera_health.hpp"
#include "perception/streaming/stream_manager.hpp"

#include <atomic>

namespace perception::pipeline {

class CapturePipeline {
public:
    CapturePipeline(camera::ICamera& camera, streaming::StreamManager& streaming,
                    health::CameraHealth& health);
    [[nodiscard]] bool run_once();
    void request_stop() noexcept;
    [[nodiscard]] bool stop_requested() const noexcept;

private:
    camera::ICamera& camera_;
    streaming::StreamManager& streaming_;
    health::CameraHealth& health_;
    std::atomic_bool stop_requested_{false};
};

}  // namespace perception::pipeline
