#include "test_support.hpp"

#include "perception/health/camera_health.hpp"

#include <chrono>

auto camera_health_test() -> bool {
    perception::health::CameraHealth health;
    health.set_status(perception::health::CameraStatus::Initializing);
    health.record_frame(1'000'000'000);
    health.record_drop();
    perception::camera::ImuBatch imu;
    imu.accelerometer.push_back({1'010'000'000, 1'011'000'000, {0.0F, 9.8F, 0.0F}});
    imu.gyroscope.push_back({1'012'000'000, 1'013'000'000, {0.1F, 0.2F, 0.3F}});
    imu.accelerometer_dropped = 2;
    imu.gyroscope_dropped = 3;
    health.record_imu(imu);
    const auto snapshot = health.snapshot(1'025'000'000);
    CHECK_TRUE(snapshot.status == perception::health::CameraStatus::Healthy);
    CHECK_TRUE(snapshot.frames_received == 1);
    CHECK_TRUE(snapshot.frames_dropped == 1);
    CHECK_TRUE(snapshot.frame_age == std::chrono::milliseconds(25));
    CHECK_TRUE(snapshot.accelerometer_samples == 1);
    CHECK_TRUE(snapshot.gyroscope_samples == 1);
    CHECK_TRUE(snapshot.accelerometer_dropped == 2);
    CHECK_TRUE(snapshot.gyroscope_dropped == 3);
    CHECK_TRUE(snapshot.accelerometer_age == std::chrono::milliseconds(14));
    CHECK_TRUE(snapshot.gyroscope_age == std::chrono::milliseconds(12));
    return true;
}
