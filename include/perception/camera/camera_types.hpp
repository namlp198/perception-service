#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <vector>

namespace perception::camera {

enum class PixelFormat { Bgr8, Gray8, DepthZ16 };

struct ImageFrame {
    std::uint64_t sensor_timestamp_ns{};
    std::uint64_t capture_timestamp_ns{};
    std::uint64_t frame_number{};
    int width{};
    int height{};
    int stride_bytes{};
    PixelFormat format{PixelFormat::Bgr8};
    std::vector<std::uint8_t> data;
};

struct DepthFrame {
    std::uint64_t sensor_timestamp_ns{};
    std::uint64_t capture_timestamp_ns{};
    std::uint64_t frame_number{};
    int width{};
    int height{};
    float depth_scale_m{};
    std::vector<std::uint16_t> data;
};

struct ImuSample {
    std::uint64_t sensor_timestamp_ns{};
    std::uint64_t capture_timestamp_ns{};
    std::array<float, 3> acceleration_mps2{};
    std::array<float, 3> angular_velocity_rps{};
};

struct CameraFrameSet {
    std::uint64_t capture_timestamp_ns{};
    ImageFrame rgb;
    ImageFrame ir_left;
    ImageFrame ir_right;
    DepthFrame depth;
    std::optional<ImuSample> imu;
};

}  // namespace perception::camera
