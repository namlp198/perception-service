#include "test_support.hpp"

#include "perception/core/config.hpp"

#include <stdexcept>

auto config_test() -> bool {
    perception::core::ServiceConfig config;
    perception::core::validate_config(config);
    CHECK_TRUE(config.camera.imu.accelerometer_fps == 100);
    CHECK_TRUE(config.camera.imu.gyroscope_fps == 200);
    CHECK_TRUE(config.camera.imu.enabled);
    CHECK_TRUE(!config.camera.infrared_left.enabled);
    CHECK_TRUE(!config.camera.infrared_right.enabled);
    CHECK_TRUE(!config.streaming.infrared_left.enabled);
    CHECK_TRUE(!config.streaming.infrared_right.enabled);
    CHECK_TRUE(!config.streaming.hardware_encoder);
    CHECK_TRUE(config.streaming.allow_software_fallback);
    CHECK_TRUE(config.camera.imu.startup_timeout_ms == 2'000);
    CHECK_TRUE(config.camera.imu.liveness_timeout_ms == 1'000);
    CHECK_TRUE(config.camera.imu.restart_interval_ms == 30'000);
    CHECK_TRUE(config.streaming.session_timeout_s == 20);

    // Sessions must expire quickly enough to release a dead client's transport, but never so
    // fast that a healthy client's keep-alive cadence is treated as death.
    config.streaming.session_timeout_s = 4;
    bool session_rejected = false;
    try {
        perception::core::validate_config(config);
    } catch (const std::invalid_argument&) {
        session_rejected = true;
    }
    CHECK_TRUE(session_rejected);
    config.streaming.session_timeout_s = 20;

    // A retry cadence faster than the startup proof window would restart the Motion Module
    // before it can ever be judged, so it is rejected; 0 disables retries.
    config.camera.imu.restart_interval_ms = 1'000;
    bool restart_rejected = false;
    try {
        perception::core::validate_config(config);
    } catch (const std::invalid_argument&) {
        restart_rejected = true;
    }
    CHECK_TRUE(restart_rejected);
    config.camera.imu.restart_interval_ms = 0;
    perception::core::validate_config(config);
    config.camera.imu.restart_interval_ms = 30'000;

    config.streaming.queue_capacity = 0;
    bool rejected = false;
    try {
        perception::core::validate_config(config);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    CHECK_TRUE(rejected);

    config.streaming.queue_capacity = 3;
    config.camera.imu.enabled = true;
    config.camera.imu.gyroscope_fps = 0;
    rejected = false;
    try {
        perception::core::validate_config(config);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    CHECK_TRUE(rejected);

    config.camera.imu.gyroscope_fps = 200;
    config.camera.imu.queue_capacity = 31;
    rejected = false;
    try {
        perception::core::validate_config(config);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    CHECK_TRUE(rejected);

    config.camera.imu.queue_capacity = 512;
    config.streaming.depth_visual.min_distance_m = 2.0F;
    config.streaming.depth_visual.max_distance_m = 1.0F;
    rejected = false;
    try {
        perception::core::validate_config(config);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    CHECK_TRUE(rejected);
    return true;
}
