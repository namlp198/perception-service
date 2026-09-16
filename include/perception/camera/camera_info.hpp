#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace perception::camera {

struct CameraIntrinsics {
    int width{};
    int height{};
    float principal_x{};
    float principal_y{};
    float focal_x{};
    float focal_y{};
    std::string distortion_model;
    std::array<float, 5> coefficients{};
};

struct CameraStreamProfileInfo {
    std::string stream;
    int index{};
    std::string format;
    int fps{};
    int width{};
    int height{};
    std::optional<CameraIntrinsics> intrinsics;
};

struct CameraDeviceInfo {
    std::string model;
    std::string serial;
    std::string firmware;
    std::string usb_mode;
    float depth_scale_m{};
    std::vector<CameraStreamProfileInfo> stream_profiles;
};

} // namespace perception::camera
