#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>

namespace perception::core {

struct ImageStreamConfig {
    bool enabled{true};
    int width{640};
    int height{480};
    int fps{30};
};

struct ImuConfig {
    bool enabled{true};
    int accelerometer_fps{100};
    int gyroscope_fps{200};
    std::size_t queue_capacity{512};
    std::uint32_t startup_timeout_ms{2'000};
    std::uint32_t liveness_timeout_ms{1'000};
    // Bounded Motion Module restart cadence while IMU data is missing or stale; 0 disables it.
    // Kept slow so retries never stress the shared USB device that also carries RGB/depth.
    std::uint32_t restart_interval_ms{30'000};
};

struct CameraConfig {
    std::string type{"realsense"};
    std::string serial;
    ImageStreamConfig rgb;
    ImageStreamConfig depth;
    ImageStreamConfig infrared_left{false, 640, 480, 30};
    ImageStreamConfig infrared_right{false, 640, 480, 30};
    ImuConfig imu;
    std::uint32_t capture_timeout_ms{1'000};
    std::uint32_t reconnect_interval_ms{2'000};
};

struct RtspStreamConfig {
    bool enabled{true};
    std::string path;
    float min_distance_m{0.2F};
    float max_distance_m{5.0F};
};

struct StreamingConfig {
    bool enabled{true};
    std::string bind_address{"0.0.0.0"};
    std::uint16_t port{8554};
    bool hardware_encoder{false};
    bool allow_software_fallback{true};
    std::uint32_t bitrate_kbps{4'000};
    std::uint32_t keyframe_interval{30};
    std::size_t queue_capacity{3};
    // A client that vanishes without TEARDOWN keeps its transport on the shared media until its
    // session expires; a dead TCP transport back-pressures every other viewer of that mount.
    std::uint32_t session_timeout_s{20};
    RtspStreamConfig rgb{true, "/camera/rgb", 0.2F, 5.0F};
    RtspStreamConfig infrared_left{false, "/camera/ir_left", 0.2F, 5.0F};
    RtspStreamConfig infrared_right{false, "/camera/ir_right", 0.2F, 5.0F};
    RtspStreamConfig depth_visual{true, "/camera/depth_visual", 0.2F, 5.0F};
};

struct RobotAgentConfig {
    bool enabled{false};
    std::string host{"192.168.1.206"};
    std::uint16_t port{0};
};

struct ServiceConfig {
    CameraConfig camera;
    StreamingConfig streaming;
    RobotAgentConfig robot_agent;
};

[[nodiscard]] ServiceConfig load_config(const std::filesystem::path& path);
void validate_config(const ServiceConfig& config);

} // namespace perception::core
