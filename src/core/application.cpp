#include "perception/core/application.hpp"

#include "perception/camera/realsense_camera.hpp"
#include "perception/health/camera_health.hpp"
#include "perception/pipeline/capture_pipeline.hpp"
#include "perception/streaming/rtsp_server.hpp"
#include "perception/streaming/stream_manager.hpp"
#include "perception/transport/remote_inputs.hpp"
#include "perception/transport/service_state_provider.hpp"
#include "perception/transport/tcp_json_server.hpp"

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
    // start() reports only the video pipeline. The Motion Module is started, monitored and
    // restarted inside the camera, so an IMU fault never gates RGB/depth.
    const auto connect_camera = [&camera]() { return camera.initialize() && camera.start(); };
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

    streaming::StreamManager streams(rtsp, config_.streaming.depth_visual);

    // The mission boundary is started after streaming and is never allowed to fail the service:
    // operator video must survive a mission-side peer or a busy port, exactly as it survives a
    // silent IMU.
    transport::ServiceStateProvider state_provider(config_, camera_health);
    transport::TcpJsonServer transport_server(config_.transport, state_provider);
    if (config_.transport.enabled && !transport_server.start()) {
#if defined(PERCEPTION_HAS_SPDLOG)
        spdlog::error("transport server unavailable; RGB/depth streaming continues");
#endif
    }
    transport::RemoteInputs remote_inputs(config_.transport);
    if (!remote_inputs.start()) {
#if defined(PERCEPTION_HAS_SPDLOG)
        spdlog::error("remote input polling unavailable; localization inputs are missing");
#endif
    }

    pipeline::CapturePipeline capture(camera, streams, camera_health);
    std::size_t consecutive_capture_failures = 0;
    auto next_metrics_at = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    // Sample rates are measured here and handed to the transport provider, so the journal and the
    // `perception.get_health` reply can never disagree about them.
    auto previous_metrics_at = std::chrono::steady_clock::now();
    std::uint64_t previous_accelerometer_samples = 0;
    std::uint64_t previous_gyroscope_samples = 0;
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
            state_provider.update_sample_rates(accelerometer_hz, gyroscope_hz);
#if defined(PERCEPTION_HAS_SPDLOG)
            const auto imu_timeout =
                std::chrono::milliseconds(config_.camera.imu.liveness_timeout_ms);
            const bool imu_ready = config_.camera.imu.enabled && health.accelerometer_samples > 0 &&
                                   health.gyroscope_samples > 0 &&
                                   health.accelerometer_age <= imu_timeout &&
                                   health.gyroscope_age <= imu_timeout;
            spdlog::info("camera.frames_received={} camera.frames_dropped={} "
                         "camera.frame_age_ms={} stream.frames_published={} "
                         "stream.frames_dropped={} imu.accel_samples={} "
                         "imu.accel_hz={:.1f} imu.accel_age_ms={} imu.accel_dropped={} "
                         "imu.gyro_samples={} imu.gyro_hz={:.1f} imu.gyro_age_ms={} "
                         "imu.gyro_dropped={} imu.ready={} ekf.ready={}",
                         health.frames_received, health.frames_dropped, health.frame_age.count(),
                         streams.frames_published(), streams.frames_dropped(),
                         health.accelerometer_samples, accelerometer_hz,
                         health.accelerometer_age.count(), health.accelerometer_dropped,
                         health.gyroscope_samples, gyroscope_hz, health.gyroscope_age.count(),
                         health.gyroscope_dropped, imu_ready, imu_ready);
            if (config_.transport.enabled || config_.transport.payload_service.enabled ||
                config_.transport.robot_agent_status.enabled) {
                const auto server_metrics = transport_server.metrics();
                const auto input_metrics = remote_inputs.metrics();
                spdlog::info("transport.listening={} transport.requests={} "
                             "transport.rejected={} transport.errors={} "
                             "gnss.enabled={} gnss.fresh={} gnss.samples={} "
                             "gnss.failures={} gnss.age_ms={} "
                             "vendor.enabled={} vendor.fresh={} vendor.samples={} "
                             "vendor.failures={} vendor.age_ms={}",
                             transport_server.running(), server_metrics.requests_served,
                             server_metrics.requests_rejected, server_metrics.connection_errors,
                             input_metrics.gnss.enabled, input_metrics.gnss.fresh,
                             input_metrics.gnss.samples, input_metrics.gnss.failures,
                             input_metrics.gnss.last_sample_age_ms,
                             input_metrics.vendor_motion.enabled, input_metrics.vendor_motion.fresh,
                             input_metrics.vendor_motion.samples,
                             input_metrics.vendor_motion.failures,
                             input_metrics.vendor_motion.last_sample_age_ms);
            }
#endif
            previous_metrics_at = now;
            previous_accelerometer_samples = health.accelerometer_samples;
            previous_gyroscope_samples = health.gyroscope_samples;
            next_metrics_at = now + std::chrono::seconds(5);
        }
    }

    capture.request_stop();
    remote_inputs.stop();
    transport_server.stop();
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
