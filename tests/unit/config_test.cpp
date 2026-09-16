#include "test_support.hpp"

#include "perception/core/config.hpp"

#include <stdexcept>

auto config_test() -> bool {
    perception::core::ServiceConfig config;
    perception::core::validate_config(config);
    CHECK_TRUE(config.camera.imu.accelerometer_fps == 100);
    CHECK_TRUE(config.camera.imu.gyroscope_fps == 200);

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
    return true;
}
