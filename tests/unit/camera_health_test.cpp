#include "test_support.hpp"

#include "perception/health/camera_health.hpp"

#include <chrono>

auto camera_health_test() -> bool {
    perception::health::CameraHealth health;
    health.set_status(perception::health::CameraStatus::Initializing);
    health.record_frame(1'000'000'000);
    health.record_drop();
    const auto snapshot = health.snapshot(1'025'000'000);
    CHECK_TRUE(snapshot.status == perception::health::CameraStatus::Healthy);
    CHECK_TRUE(snapshot.frames_received == 1);
    CHECK_TRUE(snapshot.frames_dropped == 1);
    CHECK_TRUE(snapshot.frame_age == std::chrono::milliseconds(25));
    return true;
}
