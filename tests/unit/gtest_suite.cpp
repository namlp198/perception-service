#include "perception/core/bounded_queue.hpp"
#include "perception/core/config.hpp"
#include "perception/health/camera_health.hpp"
#include "perception/streaming/depth_visualizer.hpp"

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
    EXPECT_FALSE(config.camera.infrared_left.enabled);
    EXPECT_FALSE(config.camera.infrared_right.enabled);
    EXPECT_FALSE(config.streaming.infrared_left.enabled);
    EXPECT_FALSE(config.streaming.infrared_right.enabled);
    EXPECT_FALSE(config.streaming.hardware_encoder);
    EXPECT_TRUE(config.streaming.allow_software_fallback);
    EXPECT_TRUE(config.camera.imu.enabled);
    EXPECT_EQ(config.camera.imu.startup_timeout_ms, 2'000U);
    EXPECT_EQ(config.camera.imu.liveness_timeout_ms, 1'000U);
    EXPECT_EQ(config.camera.imu.restart_interval_ms, 30'000U);
    EXPECT_EQ(config.streaming.session_timeout_s, 20U);
    EXPECT_NO_THROW(perception::core::validate_config(config));
    // Sessions must expire fast enough to release a dead client's transport, not faster than
    // a healthy client's keep-alive cadence.
    config.streaming.session_timeout_s = 4;
    EXPECT_THROW(perception::core::validate_config(config), std::invalid_argument);
    config.streaming.session_timeout_s = 20;
    // A Motion Module retry faster than the startup proof window is rejected; 0 disables retries.
    config.camera.imu.restart_interval_ms = 1'000;
    EXPECT_THROW(perception::core::validate_config(config), std::invalid_argument);
    config.camera.imu.restart_interval_ms = 0;
    EXPECT_NO_THROW(perception::core::validate_config(config));
    config.camera.imu.restart_interval_ms = 30'000;
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

TEST(DepthVisualizer, PreservesTimestampsAndMarksInvalidDepthBlack) {
    perception::camera::DepthFrame depth;
    depth.sensor_timestamp_ns = 10;
    depth.capture_timestamp_ns = 20;
    depth.frame_number = 30;
    depth.width = 4;
    depth.height = 1;
    depth.depth_scale_m = 0.001F;
    depth.data = {0U, 500U, 1'250U, 2'000U};

    const auto visual = perception::streaming::colorize_depth(depth, 0.5F, 2.0F);
    EXPECT_EQ(visual.sensor_timestamp_ns, depth.sensor_timestamp_ns);
    EXPECT_EQ(visual.capture_timestamp_ns, depth.capture_timestamp_ns);
    EXPECT_EQ(visual.frame_number, depth.frame_number);
    ASSERT_EQ(visual.data.size(), 12U);
    EXPECT_EQ(visual.data[0], 0U);
    EXPECT_EQ(visual.data[1], 0U);
    EXPECT_EQ(visual.data[2], 0U);
    EXPECT_GT(visual.data[3], visual.data[5]);
    EXPECT_TRUE(visual.data[6] > 0U || visual.data[7] > 0U || visual.data[8] > 0U);
    EXPECT_EQ(visual.data[9], 0U);
    EXPECT_EQ(visual.data[10], 0U);
    EXPECT_GT(visual.data[11], 0U);
}

TEST(Config, RejectsInvalidDepthVisualRange) {
    perception::core::ServiceConfig config;
    config.streaming.depth_visual.min_distance_m = 2.0F;
    config.streaming.depth_visual.max_distance_m = 1.0F;
    EXPECT_THROW(perception::core::validate_config(config), std::invalid_argument);
}
