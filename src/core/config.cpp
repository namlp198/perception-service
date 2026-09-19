#include "perception/core/config.hpp"

#include <stdexcept>
#include <string>

#if defined(PERCEPTION_HAS_YAML_CPP)
#include <yaml-cpp/yaml.h>
#endif

namespace perception::core {
namespace {

void validate_stream(const ImageStreamConfig& stream, const std::string& name) {
    if (!stream.enabled) {
        return;
    }
    if (stream.width <= 0 || stream.height <= 0 || stream.fps <= 0) {
        throw std::invalid_argument(name + " dimensions and FPS must be greater than zero");
    }
}

#if defined(PERCEPTION_HAS_YAML_CPP)
void read_image_stream(const YAML::Node& node, ImageStreamConfig& target) {
    if (!node) {
        return;
    }
    target.enabled = node["enabled"].as<bool>(target.enabled);
    target.width = node["width"].as<int>(target.width);
    target.height = node["height"].as<int>(target.height);
    target.fps = node["fps"].as<int>(target.fps);
}

void read_rtsp_stream(const YAML::Node& node, RtspStreamConfig& target) {
    if (!node) {
        return;
    }
    target.enabled = node["enabled"].as<bool>(target.enabled);
    target.path = node["path"].as<std::string>(target.path);
    target.min_distance_m = node["min_distance_m"].as<float>(target.min_distance_m);
    target.max_distance_m = node["max_distance_m"].as<float>(target.max_distance_m);
}

void read_peer_poll(const YAML::Node& node, PeerPollConfig& target) {
    if (!node) {
        return;
    }
    target.enabled = node["enabled"].as<bool>(target.enabled);
    target.host = node["host"].as<std::string>(target.host);
    target.port = node["port"].as<std::uint16_t>(target.port);
    target.poll_interval_ms = node["poll_interval_ms"].as<std::uint32_t>(target.poll_interval_ms);
    target.request_timeout_ms =
        node["request_timeout_ms"].as<std::uint32_t>(target.request_timeout_ms);
    target.staleness_timeout_ms =
        node["staleness_timeout_ms"].as<std::uint32_t>(target.staleness_timeout_ms);
}
#endif

void validate_peer_poll(const PeerPollConfig& peer, const std::string& name) {
    if (!peer.enabled) {
        return;
    }
    if (peer.host.empty() || peer.port == 0) {
        throw std::invalid_argument(name + " requires a host and a non-zero port");
    }
    if (peer.poll_interval_ms == 0 || peer.request_timeout_ms == 0) {
        throw std::invalid_argument(name + " poll interval and request timeout must be positive");
    }
    if (peer.poll_interval_ms > 5'000) {
        throw std::invalid_argument(name + " poll_interval_ms must not exceed 5000");
    }
    // A request allowed to outlast its own poll period lets slow peers queue up behind each other.
    if (peer.request_timeout_ms > peer.poll_interval_ms * 5) {
        throw std::invalid_argument(name +
                                    " request_timeout_ms must not exceed 5x poll_interval_ms");
    }
    if (peer.staleness_timeout_ms < peer.poll_interval_ms) {
        throw std::invalid_argument(name +
                                    " staleness_timeout_ms must be at least poll_interval_ms");
    }
}

} // namespace

ServiceConfig load_config(const std::filesystem::path& path) {
    ServiceConfig config;
#if defined(PERCEPTION_HAS_YAML_CPP)
    const YAML::Node root = YAML::LoadFile(path.string());
    const YAML::Node camera = root["camera"];
    if (camera) {
        config.camera.type = camera["type"].as<std::string>(config.camera.type);
        config.camera.serial = camera["serial"].as<std::string>(config.camera.serial);
        const YAML::Node imu = camera["imu"];
        if (imu) {
            config.camera.imu.enabled = imu["enabled"].as<bool>(config.camera.imu.enabled);
            config.camera.imu.accelerometer_fps =
                imu["accelerometer_fps"].as<int>(config.camera.imu.accelerometer_fps);
            config.camera.imu.gyroscope_fps =
                imu["gyroscope_fps"].as<int>(config.camera.imu.gyroscope_fps);
            config.camera.imu.queue_capacity =
                imu["queue_capacity"].as<std::size_t>(config.camera.imu.queue_capacity);
            config.camera.imu.startup_timeout_ms =
                imu["startup_timeout_ms"].as<std::uint32_t>(config.camera.imu.startup_timeout_ms);
            config.camera.imu.liveness_timeout_ms =
                imu["liveness_timeout_ms"].as<std::uint32_t>(config.camera.imu.liveness_timeout_ms);
            config.camera.imu.restart_interval_ms =
                imu["restart_interval_ms"].as<std::uint32_t>(config.camera.imu.restart_interval_ms);
        }
        config.camera.capture_timeout_ms =
            camera["capture_timeout_ms"].as<std::uint32_t>(config.camera.capture_timeout_ms);
        config.camera.reconnect_interval_ms =
            camera["reconnect_interval_ms"].as<std::uint32_t>(config.camera.reconnect_interval_ms);
        read_image_stream(camera["rgb"], config.camera.rgb);
        read_image_stream(camera["depth"], config.camera.depth);
        read_image_stream(camera["infrared_left"], config.camera.infrared_left);
        read_image_stream(camera["infrared_right"], config.camera.infrared_right);
    }

    const YAML::Node rtsp = root["streaming"]["rtsp"];
    if (rtsp) {
        config.streaming.enabled = rtsp["enabled"].as<bool>(config.streaming.enabled);
        config.streaming.bind_address =
            rtsp["bind_address"].as<std::string>(config.streaming.bind_address);
        config.streaming.port = rtsp["port"].as<std::uint16_t>(config.streaming.port);
        config.streaming.hardware_encoder =
            rtsp["hardware_encoder"].as<bool>(config.streaming.hardware_encoder);
        config.streaming.allow_software_fallback =
            rtsp["allow_software_fallback"].as<bool>(config.streaming.allow_software_fallback);
        config.streaming.bitrate_kbps =
            rtsp["bitrate_kbps"].as<std::uint32_t>(config.streaming.bitrate_kbps);
        config.streaming.keyframe_interval =
            rtsp["keyframe_interval"].as<std::uint32_t>(config.streaming.keyframe_interval);
        config.streaming.queue_capacity =
            rtsp["queue_capacity"].as<std::size_t>(config.streaming.queue_capacity);
        config.streaming.session_timeout_s =
            rtsp["session_timeout_s"].as<std::uint32_t>(config.streaming.session_timeout_s);
        read_rtsp_stream(rtsp["rgb"], config.streaming.rgb);
        read_rtsp_stream(rtsp["infrared_left"], config.streaming.infrared_left);
        read_rtsp_stream(rtsp["infrared_right"], config.streaming.infrared_right);
        read_rtsp_stream(rtsp["depth_visual"], config.streaming.depth_visual);
    }

    const YAML::Node robot_agent = root["robot_agent"];
    if (robot_agent) {
        config.robot_agent.enabled = robot_agent["enabled"].as<bool>(config.robot_agent.enabled);
        config.robot_agent.host = robot_agent["host"].as<std::string>(config.robot_agent.host);
        config.robot_agent.port = robot_agent["port"].as<std::uint16_t>(config.robot_agent.port);
    }

    const YAML::Node transport = root["transport"];
    if (transport) {
        config.transport.enabled = transport["enabled"].as<bool>(config.transport.enabled);
        config.transport.bind_address =
            transport["bind_address"].as<std::string>(config.transport.bind_address);
        config.transport.port = transport["port"].as<std::uint16_t>(config.transport.port);
        config.transport.request_timeout_ms =
            transport["request_timeout_ms"].as<std::uint32_t>(config.transport.request_timeout_ms);
        config.transport.max_request_bytes =
            transport["max_request_bytes"].as<std::size_t>(config.transport.max_request_bytes);
        read_peer_poll(transport["payload_service"], config.transport.payload_service);
        read_peer_poll(transport["robot_agent_status"], config.transport.robot_agent_status);
    }

    const YAML::Node geometry = root["geometry"];
    if (geometry) {
        const YAML::Node mount = geometry["camera_mount"];
        if (mount) {
            config.geometry.camera_x_m = mount["x_m"].as<double>(config.geometry.camera_x_m);
            config.geometry.camera_y_m = mount["y_m"].as<double>(config.geometry.camera_y_m);
            config.geometry.camera_z_m = mount["z_m"].as<double>(config.geometry.camera_z_m);
            config.geometry.camera_roll_deg =
                mount["roll_deg"].as<double>(config.geometry.camera_roll_deg);
            config.geometry.camera_pitch_deg =
                mount["pitch_deg"].as<double>(config.geometry.camera_pitch_deg);
            config.geometry.camera_yaw_deg =
                mount["yaw_deg"].as<double>(config.geometry.camera_yaw_deg);
        }
        const YAML::Node cloud = geometry["point_cloud"];
        if (cloud) {
            config.geometry.min_range_m = cloud["min_range_m"].as<double>(config.geometry.min_range_m);
            config.geometry.max_range_m = cloud["max_range_m"].as<double>(config.geometry.max_range_m);
            config.geometry.row_stride = cloud["row_stride"].as<int>(config.geometry.row_stride);
            config.geometry.column_stride =
                cloud["column_stride"].as<int>(config.geometry.column_stride);
        }
    }
#else
    (void)path;
    throw std::runtime_error("yaml-cpp support was not available when the service was built");
#endif
    validate_config(config);
    return config;
}

void validate_config(const ServiceConfig& config) {
    if (config.camera.type != "realsense") {
        throw std::invalid_argument("camera.type must be 'realsense'");
    }
    validate_stream(config.camera.rgb, "camera.rgb");
    validate_stream(config.camera.depth, "camera.depth");
    validate_stream(config.camera.infrared_left, "camera.infrared_left");
    validate_stream(config.camera.infrared_right, "camera.infrared_right");
    if (!config.camera.imu.enabled) {
        throw std::invalid_argument("camera.imu must be enabled for perception-service readiness");
    }
    if (config.camera.imu.accelerometer_fps <= 0 || config.camera.imu.gyroscope_fps <= 0) {
        throw std::invalid_argument("camera.imu sample rates must be greater than zero");
    }
    if (config.camera.imu.queue_capacity < 32 || config.camera.imu.queue_capacity > 4'096) {
        throw std::invalid_argument("camera.imu queue_capacity must be in the range 32..4096");
    }
    if (config.camera.imu.startup_timeout_ms == 0 || config.camera.imu.liveness_timeout_ms == 0) {
        throw std::invalid_argument("camera.imu timeout values must be greater than zero");
    }
    if (config.camera.imu.restart_interval_ms != 0 &&
        config.camera.imu.restart_interval_ms < config.camera.imu.startup_timeout_ms) {
        throw std::invalid_argument(
            "camera.imu restart_interval_ms must be 0 or at least startup_timeout_ms");
    }
    if (config.camera.capture_timeout_ms == 0 || config.camera.reconnect_interval_ms == 0) {
        throw std::invalid_argument("camera timeout values must be greater than zero");
    }
    if (config.streaming.enabled && config.streaming.port == 0) {
        throw std::invalid_argument("streaming RTSP port must be greater than zero");
    }
    if (config.streaming.queue_capacity < 2 || config.streaming.queue_capacity > 3) {
        throw std::invalid_argument("streaming queue_capacity must be in the range 2..3");
    }
    if (config.streaming.session_timeout_s < 5 || config.streaming.session_timeout_s > 300) {
        throw std::invalid_argument("streaming session_timeout_s must be in the range 5..300");
    }
    if (config.streaming.bitrate_kbps == 0 || config.streaming.keyframe_interval == 0) {
        throw std::invalid_argument(
            "streaming bitrate and keyframe interval must be greater than zero");
    }
    if (config.streaming.depth_visual.enabled && !config.camera.depth.enabled) {
        throw std::invalid_argument("camera.depth must be enabled for the depth visual stream");
    }
    if (config.streaming.depth_visual.min_distance_m < 0.0F ||
        config.streaming.depth_visual.max_distance_m <=
            config.streaming.depth_visual.min_distance_m) {
        throw std::invalid_argument(
            "depth visual max_distance_m must be greater than min_distance_m >= 0");
    }
    if (config.robot_agent.enabled && config.robot_agent.port == 0) {
        throw std::invalid_argument("robot_agent.port is intentionally unset; disable integration");
    }
    if (config.transport.enabled) {
        if (config.transport.port == 0) {
            throw std::invalid_argument("transport.port must be greater than zero");
        }
        if (config.transport.port == config.streaming.port) {
            throw std::invalid_argument("transport.port must differ from the RTSP port");
        }
        if (config.transport.request_timeout_ms == 0) {
            throw std::invalid_argument("transport.request_timeout_ms must be greater than zero");
        }
        if (config.transport.max_request_bytes < 256 ||
            config.transport.max_request_bytes > 1'048'576) {
            throw std::invalid_argument(
                "transport.max_request_bytes must be in the range 256..1048576");
        }
    }
    validate_peer_poll(config.transport.payload_service, "transport.payload_service");
    validate_peer_poll(config.transport.robot_agent_status, "transport.robot_agent_status");

    if (config.geometry.min_range_m < 0.0 ||
        config.geometry.max_range_m <= config.geometry.min_range_m) {
        throw std::invalid_argument(
            "geometry.point_cloud max_range_m must be greater than min_range_m >= 0");
    }
    if (config.geometry.row_stride < 1 || config.geometry.column_stride < 1) {
        throw std::invalid_argument("geometry.point_cloud strides must be at least 1");
    }
    // A mount angle outside one turn is a data-entry error, not an exotic installation.
    const double angles[] = {config.geometry.camera_roll_deg, config.geometry.camera_pitch_deg,
                             config.geometry.camera_yaw_deg};
    for (const double angle : angles) {
        if (angle < -360.0 || angle > 360.0) {
            throw std::invalid_argument("geometry.camera_mount angles must be within +/-360 deg");
        }
    }
}

} // namespace perception::core
