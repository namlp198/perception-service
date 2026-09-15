#pragma once

#include "perception/camera/camera_types.hpp"
#include "perception/streaming/stream_types.hpp"

namespace perception::streaming {

class IStreamPublisher {
public:
    virtual ~IStreamPublisher() = default;
    [[nodiscard]] virtual bool start() = 0;
    virtual void stop() noexcept = 0;
    [[nodiscard]] virtual bool publish(StreamId stream, const camera::ImageFrame& frame) = 0;
};

}  // namespace perception::streaming
