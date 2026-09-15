#pragma once

#include "perception/core/config.hpp"

#include <atomic>

namespace perception::core {

class Application {
public:
    explicit Application(ServiceConfig config);
    [[nodiscard]] int run();
    void request_stop() noexcept;

private:
    ServiceConfig config_;
    std::atomic_bool stop_requested_{false};
};

}  // namespace perception::core
