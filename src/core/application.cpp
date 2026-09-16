#include "perception/core/application.hpp"

#include "perception/camera/realsense_camera.hpp"
#include "perception/health/camera_health.hpp"
#include "perception/pipeline/capture_pipeline.hpp"
#include "perception/streaming/rtsp_server.hpp"
#include "perception/streaming/stream_manager.hpp"

#include <chrono>
#include <thread>
#include <utility>

#if defined(PERCEPTION_HAS_SPDLOG)
#include <spdlog/spdlog.h>
#endif

namespace perception::core {

Application::Application(ServiceConfig config) : config_(std::move(config)) {}

int Application::run() {
    validate_config(config_);
    camera::RealSenseCamera camera(config_.camera);
    health::CameraHealth camera_health;
    camera_health.set_status(health::CameraStatus::Initializing);
    const auto connect_camera = [&camera]() {
        if (!camera.initialize()) {
            return false;
        }
        if (camera.start()) {
            return true;
        }
        if (!camera.imu_enabled()) {
            return false;
        }
#if defined(PERCEPTION_HAS_SPDLOG)
        spdlog::error("RealSense video+IMU profile could not start; retrying in degraded "
                      "video-only mode so RTSP remains available");
#endif
        camera.disable_imu();
        return camera.initialize() && camera.start();
    };
    while (!stop_requested_ && !connect_camera()) {
#if defined(PERCEPTION_HAS_SPDLOG)
        spdlog::warn("RealSense unavailable; retrying in {} ms",
                     config_.camera.reconnect_interval_ms);
#endif
        camera_health.set_status(health::CameraStatus::Disconnected);
        std::this_thread::sleep_for(
            std::chrono::milliseconds(config_.camera.reconnect_interval_ms));
    }
    if (stop_requested_) {
        return 0;
    }

#if defined(PERCEPTION_HAS_SPDLOG)
    if (const auto info = camera.device_info(); info.has_value()) {
        spdlog::info("D435i ready model={} serial={} firmware={} usb_mode={} depth_scale_m={}",
                     info->model, info->serial, info->firmware, info->usb_mode,
                     info->depth_scale_m);
    }
#endif

    streaming::RtspServer rtsp(config_.streaming);
    if (config_.streaming.enabled && !rtsp.start()) {
#if defined(PERCEPTION_HAS_SPDLOG)
        spdlog::error("RTSP server failed to start");
#endif
        camera.stop();
        return 3;
    }

    streaming::StreamManager streams(rtsp);
    pipeline::CapturePipeline capture(camera, streams, camera_health);
    std::size_t consecutive_capture_failures = 0;
    auto next_metrics_at = std::chrono::steady_clock::now() + std::chrono::seconds(5);
#if defined(PERCEPTION_HAS_SPDLOG)
    auto previous_metrics_at = std::chrono::steady_clock::now();
    std::uint64_t previous_accelerometer_samples = 0;
    std::uint64_t previous_gyroscope_samples = 0;
#endif
#if defined(PERCEPTION_HAS_SPDLOG)
    spdlog::info("perception-service running; RTSP port={}", config_.streaming.port);
#endif
    while (!stop_requested_) {
        if (!capture.run_once()) {
            ++consecutive_capture_failures;
            if (consecutive_capture_failures >= 3) {
                camera_health.set_status(health::CameraStatus::Disconnected);
                camera.stop();
#if defined(PERCEPTION_HAS_SPDLOG)
                spdlog::warn("camera capture failed; entering reconnect loop");
#endif
                while (!stop_requested_ && !connect_camera()) {
                    std::this_thread::sleep_for(
                        std::chrono::milliseconds(config_.camera.reconnect_interval_ms));
                }
                consecutive_capture_failures = 0;
            }
        } else {
            consecutive_capture_failures = 0;
        }

        const auto now = std::chrono::steady_clock::now();
        if (now >= next_metrics_at) {
#if defined(PERCEPTION_HAS_SPDLOG)
            const auto now_ns = static_cast<std::uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch())
                    .count());
            const auto health = camera_health.snapshot(now_ns);
            const double interval_seconds =
                std::chrono::duration<double>(now - previous_metrics_at).count();
            const double accelerometer_hz =
                interval_seconds > 0.0 ? static_cast<double>(health.accelerometer_samples -
                                                             previous_accelerometer_samples) /
                                             interval_seconds
                                       : 0.0;
            const double gyroscope_hz =
                interval_seconds > 0.0
                    ? static_cast<double>(health.gyroscope_samples - previous_gyroscope_samples) /
                          interval_seconds
                    : 0.0;
            spdlog::info("camera.frames_received={} camera.frames_dropped={} "
                         "camera.frame_age_ms={} stream.frames_published={} "
                         "stream.frames_dropped={} imu.accel_samples={} "
                         "imu.accel_hz={:.1f} imu.accel_age_ms={} imu.accel_dropped={} "
                         "imu.gyro_samples={} imu.gyro_hz={:.1f} imu.gyro_age_ms={} "
                         "imu.gyro_dropped={}",
                         health.frames_received, health.frames_dropped, health.frame_age.count(),
                         streams.frames_published(), streams.frames_dropped(),
                         health.accelerometer_samples, accelerometer_hz,
                         health.accelerometer_age.count(), health.accelerometer_dropped,
                         health.gyroscope_samples, gyroscope_hz, health.gyroscope_age.count(),
                         health.gyroscope_dropped);
            previous_metrics_at = now;
            previous_accelerometer_samples = health.accelerometer_samples;
            previous_gyroscope_samples = health.gyroscope_samples;
#endif
            next_metrics_at = now + std::chrono::seconds(5);
        }
    }

    capture.request_stop();
    rtsp.stop();
    camera.stop();
#if defined(PERCEPTION_HAS_SPDLOG)
    spdlog::info("perception-service stopped cleanly");
#endif
    return 0;
}

void Application::request_stop() noexcept {
    stop_requested_ = true;
}

} // namespace perception::core
