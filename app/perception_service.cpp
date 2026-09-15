#include "perception/core/application.hpp"
#include "perception/core/config.hpp"

#include <exception>
#include <atomic>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <thread>

namespace {
volatile std::sig_atomic_t stop_requested = 0;

extern "C" void handle_signal(int) { stop_requested = 1; }
}  // namespace

int main(int argc, char* argv[]) {
    const std::filesystem::path config_path =
        argc > 1 ? std::filesystem::path(argv[1]) : std::filesystem::path("config/default.yaml");
    try {
        perception::core::Application application(perception::core::load_config(config_path));
        std::signal(SIGINT, handle_signal);
        std::signal(SIGTERM, handle_signal);

        std::atomic_bool finished{false};
        int exit_code = 1;
        std::thread service_thread([&application, &finished, &exit_code] {
            exit_code = application.run();
            finished = true;
        });
        while (!finished && stop_requested == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        if (stop_requested != 0) {
            application.request_stop();
        }
        service_thread.join();
        return exit_code;
    } catch (const std::exception& error) {
        std::cerr << "perception-service: " << error.what() << '\n';
        return 1;
    }
}
