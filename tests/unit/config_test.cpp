#include "test_support.hpp"

#include "perception/core/config.hpp"

#include <stdexcept>

auto config_test() -> bool {
    perception::core::ServiceConfig config;
    perception::core::validate_config(config);

    config.streaming.queue_capacity = 0;
    bool rejected = false;
    try {
        perception::core::validate_config(config);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    CHECK_TRUE(rejected);
    return true;
}
