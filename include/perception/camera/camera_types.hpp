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

struct AccelerometerSample {
    std::uint64_t sensor_timestamp_ns{};
    std::uint64_t capture_timestamp_ns{};
    std::array<float, 3> acceleration_mps2{};
};

struct GyroscopeSample {
    std::uint64_t sensor_timestamp_ns{};
    std::uint64_t capture_timestamp_ns{};
    std::array<float, 3> angular_velocity_rps{};
};

struct ImuBatch {
    std::vector<AccelerometerSample> accelerometer;
    std::vector<GyroscopeSample> gyroscope;
    std::uint64_t accelerometer_dropped{};
    std::uint64_t gyroscope_dropped{};
};

struct CameraFrameSet {
    std::uint64_t capture_timestamp_ns{};
    ImageFrame rgb;
    ImageFrame ir_left;
    ImageFrame ir_right;
    DepthFrame depth;
    std::optional<ImuBatch> imu;
};

} // namespace perception::camera
