#include "perception/streaming/stream_manager.hpp"

#include "perception/streaming/depth_visualizer.hpp"

#include <utility>

namespace perception::streaming {

StreamManager::StreamManager(IStreamPublisher& publisher,
                             core::RtspStreamConfig depth_visual_config)
    : publisher_(publisher), depth_visual_config_(std::move(depth_visual_config)) {}

bool StreamManager::publish(const camera::CameraFrameSet& frame_set) {
    const bool rgb = publisher_.publish(StreamId::Rgb, frame_set.rgb);
    const bool left = publisher_.publish(StreamId::InfraredLeft, frame_set.ir_left);
    const bool right = publisher_.publish(StreamId::InfraredRight, frame_set.ir_right);
    bool depth = true;
    if (depth_visual_config_.enabled) {
        const camera::ImageFrame visual =
            colorize_depth(frame_set.depth, depth_visual_config_.min_distance_m,
                           depth_visual_config_.max_distance_m);
        depth = publisher_.publish(StreamId::DepthVisual, visual);
    }
    if (rgb && left && right && depth) {
        ++frames_published_;
        return true;
    }
    ++frames_dropped_;
    return false;
}

auto StreamManager::frames_published() const noexcept -> std::size_t {
    return frames_published_;
}
auto StreamManager::frames_dropped() const noexcept -> std::size_t {
    return frames_dropped_;
}

} // namespace perception::streaming
