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

// A peer perception-service polls for one of the EKF's inbound measurements. Both peers are
// optional at runtime: a peer that stops answering degrades localization quality and must never
// stall capture or streaming.
struct PeerPollConfig {
    bool enabled{false};
    std::string host;
    std::uint16_t port{0};
    std::uint32_t poll_interval_ms{200};
    std::uint32_t request_timeout_ms{1'000};
    // A measurement older than this is reported as unavailable instead of being fused.
    std::uint32_t staleness_timeout_ms{2'000};
};

struct TransportConfig {
    // Inbound API that robot-agent calls; same JSON/TCP framing as payload-service.
    bool enabled{false};
    std::string bind_address{"0.0.0.0"};
    std::uint16_t port{50053};
    std::uint32_t request_timeout_ms{5'000};
    std::size_t max_request_bytes{65'536};
    // GNSS/RTK is read straight from payload-service: routing it through robot-agent would add a
    // second hop to the one measurement whose timestamp and age the EKF depends on.
    PeerPollConfig payload_service{false, "192.168.1.206", 50052, 200, 1'000, 2'000};
    // Vendor odometry/heading still comes from robot-agent, which owns the quadruped link.
    PeerPollConfig robot_agent_status{false, "192.168.1.206", 5'080, 200, 1'000, 2'000};
};

// Where the camera body sits on the robot, and how depth is turned into robot-frame points.
// These are survey values, not tuning knobs: a wrong mount silently rotates or shifts every point
// the perception and localization branches consume.
struct GeometryConfig {
    double camera_x_m{0.0};
    double camera_y_m{0.0};
    double camera_z_m{0.0};
    double camera_roll_deg{0.0};
    double camera_pitch_deg{0.0};
    double camera_yaw_deg{0.0};
    double min_range_m{0.2};
    double max_range_m{5.0};
    int row_stride{1};
    int column_stride{1};
};

struct ServiceConfig {
    CameraConfig camera;
    StreamingConfig streaming;
    RobotAgentConfig robot_agent;
    TransportConfig transport;
    GeometryConfig geometry;
};

[[nodiscard]] ServiceConfig load_config(const std::filesystem::path& path);
void validate_config(const ServiceConfig& config);

} // namespace perception::core
