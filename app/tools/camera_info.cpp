#include "perception/camera/realsense_camera.hpp"
#include "perception/core/config.hpp"

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>

int main(int argc, char* argv[]) {
    const std::filesystem::path config_path =
        argc > 1 ? std::filesystem::path(argv[1]) : std::filesystem::path("config/default.yaml");
    const std::string action = argc > 2 ? argv[2] : "info";
    if (action != "info" && action != "--hardware-reset") {
        std::cerr << "Usage: camera-info [config.yaml] [--hardware-reset]\n";
        return 2;
    }
    const auto config = perception::core::load_config(config_path);
    perception::camera::RealSenseCamera camera(config.camera);
    if (!camera.initialize()) {
        std::cerr << "No RealSense camera found, or the backend was disabled at build time.\n";
        return 1;
    }
    const auto info = camera.device_info();
    if (!info.has_value()) {
        std::cerr << "RealSense camera information is unavailable.\n";
        return 2;
    }

    std::cout << "model: " << info->model << '\n'
              << "serial: " << info->serial << '\n'
              << "firmware: " << info->firmware << '\n'
              << "usb_mode: " << info->usb_mode << '\n'
              << "depth_scale_m: " << std::fixed << std::setprecision(6) << info->depth_scale_m
              << '\n'
              << "stream_profiles:\n";
    for (const auto& profile : info->stream_profiles) {
        std::cout << "  - stream=" << profile.stream << " index=" << profile.index
                  << " format=" << profile.format << " fps=" << profile.fps;
        if (profile.width > 0 && profile.height > 0) {
            std::cout << " size=" << profile.width << 'x' << profile.height;
        }
        if (profile.intrinsics.has_value()) {
            const auto& intrinsics = *profile.intrinsics;
            std::cout << " fx=" << intrinsics.focal_x << " fy=" << intrinsics.focal_y
                      << " ppx=" << intrinsics.principal_x << " ppy=" << intrinsics.principal_y
                      << " distortion=" << intrinsics.distortion_model;
        } else if (profile.width > 0 && profile.height > 0) {
            std::cout << " intrinsics=unavailable";
        }
        std::cout << '\n';
    }
    if (action == "--hardware-reset") {
        // Field recovery for a Motion Module that stays silent (motion_callbacks=0) until the
        // camera re-enumerates; equivalent to a replug and only ever run by an operator.
        if (!camera.hardware_reset()) {
            std::cerr << "Hardware reset failed.\n";
            return 3;
        }
        std::cout << "hardware_reset: issued; wait a few seconds for USB re-enumeration\n";
    }
    return 0;
}
