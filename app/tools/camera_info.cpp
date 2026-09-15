#include "perception/camera/realsense_camera.hpp"
#include "perception/core/config.hpp"

#include <iostream>

int main() {
    perception::camera::RealSenseCamera camera(perception::core::CameraConfig{});
    if (!camera.initialize()) {
        std::cerr << "No RealSense camera found, or the backend was disabled at build time.\n";
        return 1;
    }
    std::cout << "RealSense camera discovered. Detailed profile reporting is Milestone 1 work.\n";
    return 0;
}
