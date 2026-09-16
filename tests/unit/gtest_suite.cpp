#include "perception/core/bounded_queue.hpp"
#include "perception/core/config.hpp"
#include "perception/health/camera_health.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <stdexcept>

TEST(BoundedQueue, DropsOldestItemWhenFull) {
    perception::core::BoundedQueue<int> queue(2);
    queue.push(1);
    queue.push(2);
    queue.push(3);
    EXPECT_EQ(queue.size(), 2);
    EXPECT_EQ(queue.dropped_count(), 1);
    EXPECT_EQ(queue.try_pop(), 2);
    EXPECT_EQ(queue.try_pop(), 3);
}

TEST(Config, RejectsQueueOutsideFreshDataRange) {
    perception::core::ServiceConfig config;
    EXPECT_NO_THROW(perception::core::validate_config(config));
    config.streaming.queue_capacity = 0;
    EXPECT_THROW(perception::core::validate_config(config), std::invalid_argument);
}

TEST(CameraHealth, ReportsFrameAgeAndCounters) {
    perception::health::CameraHealth health;
    health.record_frame(1'000'000'000);
    health.record_drop();
    perception::camera::ImuBatch imu;
    imu.accelerometer.push_back({1'010'000'000, 1'011'000'000, {0.0F, 9.8F, 0.0F}});
    imu.gyroscope.push_back({1'012'000'000, 1'013'000'000, {0.1F, 0.2F, 0.3F}});
    health.record_imu(imu);
    const auto snapshot = health.snapshot(1'025'000'000);
    EXPECT_EQ(snapshot.status, perception::health::CameraStatus::Healthy);
    EXPECT_EQ(snapshot.frames_received, 1);
    EXPECT_EQ(snapshot.frames_dropped, 1);
    EXPECT_EQ(snapshot.frame_age, std::chrono::milliseconds(25));
    EXPECT_EQ(snapshot.accelerometer_samples, 1);
    EXPECT_EQ(snapshot.gyroscope_samples, 1);
    EXPECT_EQ(snapshot.accelerometer_age, std::chrono::milliseconds(14));
    EXPECT_EQ(snapshot.gyroscope_age, std::chrono::milliseconds(12));
}

TEST(Config, RejectsInvalidImuRatesAndCapacity) {
    perception::core::ServiceConfig config;
    EXPECT_EQ(config.camera.imu.accelerometer_fps, 100);
    EXPECT_EQ(config.camera.imu.gyroscope_fps, 200);
    config.camera.imu.enabled = true;
    config.camera.imu.accelerometer_fps = 0;
    EXPECT_THROW(perception::core::validate_config(config), std::invalid_argument);

    config.camera.imu.accelerometer_fps = 100;
    config.camera.imu.queue_capacity = 31;
    EXPECT_THROW(perception::core::validate_config(config), std::invalid_argument);
}
