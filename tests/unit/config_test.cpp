#include "test_support.hpp"

#include "perception/core/config.hpp"

#include <stdexcept>

auto config_test() -> bool {
    perception::core::ServiceConfig config;
    perception::core::validate_config(config);
    CHECK_TRUE(config.camera.imu.accelerometer_fps == 100);
    CHECK_TRUE(config.camera.imu.gyroscope_fps == 200);
    CHECK_TRUE(!config.camera.infrared_left.enabled);
    CHECK_TRUE(!config.camera.infrared_right.enabled);
    CHECK_TRUE(!config.streaming.infrared_left.enabled);
    CHECK_TRUE(!config.streaming.infrared_right.enabled);

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
