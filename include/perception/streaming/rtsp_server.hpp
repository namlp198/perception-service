#pragma once

#include "perception/core/config.hpp"
#include "perception/streaming/stream_publisher.hpp"

#include <memory>

namespace perception::streaming {

class RtspServer final : public IStreamPublisher {
public:
    explicit RtspServer(core::StreamingConfig config);
    ~RtspServer() override;

    RtspServer(const RtspServer&) = delete;
    auto operator=(const RtspServer&) -> RtspServer& = delete;
    RtspServer(RtspServer&&) noexcept;
    auto operator=(RtspServer&&) noexcept -> RtspServer&;

    [[nodiscard]] bool start() override;
    void stop() noexcept override;
    [[nodiscard]] bool publish(StreamId stream, const camera::ImageFrame& frame) override;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace perception::streaming
