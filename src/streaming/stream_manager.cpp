#include "perception/streaming/stream_manager.hpp"

namespace perception::streaming {

StreamManager::StreamManager(IStreamPublisher& publisher) : publisher_(publisher) {}

bool StreamManager::publish(const camera::CameraFrameSet& frame_set) {
    const bool rgb = publisher_.publish(StreamId::Rgb, frame_set.rgb);
    const bool left = publisher_.publish(StreamId::InfraredLeft, frame_set.ir_left);
    const bool right = publisher_.publish(StreamId::InfraredRight, frame_set.ir_right);
    if (rgb && left && right) {
        ++frames_published_;
        return true;
    }
    ++frames_dropped_;
    return false;
}

auto StreamManager::frames_published() const noexcept -> std::size_t { return frames_published_; }
auto StreamManager::frames_dropped() const noexcept -> std::size_t { return frames_dropped_; }

}  // namespace perception::streaming
