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
#endif

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
}

} // namespace perception::core
