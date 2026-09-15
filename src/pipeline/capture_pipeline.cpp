#include "perception/pipeline/capture_pipeline.hpp"

namespace perception::pipeline {

CapturePipeline::CapturePipeline(camera::ICamera& camera, streaming::StreamManager& streaming,
                                 health::CameraHealth& health)
    : camera_(camera), streaming_(streaming), health_(health) {}

bool CapturePipeline::run_once() {
    if (stop_requested_) {
        return false;
    }
    camera::CameraFrameSet frame_set;
    if (!camera_.capture(frame_set)) {
        health_.record_drop();
        health_.set_status(health::CameraStatus::Degraded);
        return false;
    }
    health_.record_frame(frame_set.capture_timestamp_ns);
    return streaming_.publish(frame_set);
}

void CapturePipeline::request_stop() noexcept { stop_requested_ = true; }
bool CapturePipeline::stop_requested() const noexcept { return stop_requested_; }

}  // namespace perception::pipeline
