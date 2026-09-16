#include "perception/camera/realsense_camera.hpp"

#include "perception/core/bounded_queue.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#if defined(PERCEPTION_HAS_REALSENSE)
#include <librealsense2/rs.hpp>
#endif

#if defined(PERCEPTION_HAS_SPDLOG)
#include <spdlog/spdlog.h>
#endif

namespace perception::camera {

// The D435i is one USB device but two independent producers: the video sensors (depth + color)
// and the Motion Module. A single rs2::pipeline that enables all of them does not release any
// frameset until every enabled stream has produced at least one frame, so a silent IMU also
// silences RGB/depth. The video streams therefore run in a video-only pipeline and the Motion
// Module is opened through its own rs2::sensor session that can fail, stall or restart without
// touching the video path.
class RealSenseCamera::Impl {
  public:
    explicit Impl(core::CameraConfig value) : config(std::move(value)) {}

    core::CameraConfig config;
    bool initialized{false};
    bool running{false};
    std::optional<CameraDeviceInfo> device_info;
#if defined(PERCEPTION_HAS_REALSENSE)
    rs2::context context;
    std::optional<rs2::device> device;
    rs2::pipeline pipeline{context};
    rs2::config pipeline_config;
    bool pipeline_started{false};
    float depth_scale_m{0.001F};

    std::optional<rs2::sensor> motion_sensor;
    bool motion_started{false};
    std::unique_ptr<core::BoundedQueue<AccelerometerSample>> accelerometer_queue;
    std::unique_ptr<core::BoundedQueue<GyroscopeSample>> gyroscope_queue;
    std::atomic<std::uint64_t> last_accelerometer_capture_ns{0};
    std::atomic<std::uint64_t> last_gyroscope_capture_ns{0};
    std::uint64_t motion_attempt_ns{0};
    // Diagnostics: every sensor callback invocation, whatever the frame type, so a silent
    // Motion Module can be told apart from frames that arrive with an unexpected type.
    std::atomic<std::uint64_t> motion_callbacks{0};
    std::atomic<bool> first_accelerometer_logged{false};
    std::atomic<bool> first_gyroscope_logged{false};
    bool motion_ready_logged{false};
    bool motion_fault_logged{false};
#endif
};

#if defined(PERCEPTION_HAS_REALSENSE)
namespace {

auto read_device_info(const rs2::device& device) -> CameraDeviceInfo {
    CameraDeviceInfo info;
    const auto read_info = [&device](rs2_camera_info key) -> std::string {
        return device.supports(key) ? device.get_info(key) : "unavailable";
    };
    info.model = read_info(RS2_CAMERA_INFO_NAME);
    info.serial = read_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    info.firmware = read_info(RS2_CAMERA_INFO_FIRMWARE_VERSION);
    info.usb_mode = read_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR);

    for (const rs2::sensor& sensor : device.query_sensors()) {
        if (const rs2::depth_sensor depth = sensor.as<rs2::depth_sensor>()) {
            info.depth_scale_m = depth.get_depth_scale();
        }
        for (const rs2::stream_profile& profile : sensor.get_stream_profiles()) {
            CameraStreamProfileInfo profile_info;
            profile_info.stream = rs2_stream_to_string(profile.stream_type());
            profile_info.index = profile.stream_index();
            profile_info.format = rs2_format_to_string(profile.format());
            profile_info.fps = profile.fps();
            if (const rs2::video_stream_profile video = profile.as<rs2::video_stream_profile>()) {
                profile_info.width = video.width();
                profile_info.height = video.height();
                try {
                    const rs2_intrinsics source = video.get_intrinsics();
                    CameraIntrinsics intrinsics;
                    intrinsics.width = source.width;
                    intrinsics.height = source.height;
                    intrinsics.principal_x = source.ppx;
                    intrinsics.principal_y = source.ppy;
                    intrinsics.focal_x = source.fx;
                    intrinsics.focal_y = source.fy;
                    intrinsics.distortion_model = rs2_distortion_to_string(source.model);
                    for (std::size_t index = 0; index < intrinsics.coefficients.size(); ++index) {
                        intrinsics.coefficients[index] = source.coeffs[index];
                    }
                    profile_info.intrinsics = intrinsics;
                } catch (const rs2::error&) {
                    // Some firmware-exposed video profiles are valid for enumeration but do not
                    // publish calibration. Keep the profile and report intrinsics as unavailable.
                }
            }
            info.stream_profiles.push_back(std::move(profile_info));
        }
    }
    return info;
}

auto steady_now_ns() -> std::uint64_t {
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                          std::chrono::steady_clock::now().time_since_epoch())
                                          .count());
}

[[maybe_unused]] auto describe_profile(const rs2::stream_profile& profile) -> std::string {
    if (!profile) {
        return "<null profile>";
    }
    return std::string(rs2_stream_to_string(profile.stream_type())) + "#" +
           std::to_string(profile.stream_index()) + " " + rs2_format_to_string(profile.format()) +
           " " + std::to_string(profile.fps()) + "Hz uid=" + std::to_string(profile.unique_id());
}

struct MotionProfiles {
    rs2::sensor sensor;
    rs2::stream_profile accelerometer;
    rs2::stream_profile gyroscope;
};

// Select the sensor that exposes both requested motion profiles with exact rates; librealsense
// rejects a request it cannot resolve, and a silently substituted rate would invalidate EKF timing.
auto find_motion_profiles(const rs2::device& device, const core::ImuConfig& imu)
    -> std::optional<MotionProfiles> {
    for (const rs2::sensor& sensor : device.query_sensors()) {
        std::optional<rs2::stream_profile> accelerometer;
        std::optional<rs2::stream_profile> gyroscope;
        for (const rs2::stream_profile& profile : sensor.get_stream_profiles()) {
            if (profile.format() != RS2_FORMAT_MOTION_XYZ32F) {
                continue;
            }
            if (profile.stream_type() == RS2_STREAM_ACCEL &&
                profile.fps() == imu.accelerometer_fps) {
                accelerometer = profile;
            } else if (profile.stream_type() == RS2_STREAM_GYRO &&
                       profile.fps() == imu.gyroscope_fps) {
                gyroscope = profile;
            }
        }
        if (accelerometer.has_value() && gyroscope.has_value()) {
            return MotionProfiles{sensor, *accelerometer, *gyroscope};
        }
    }
    return std::nullopt;
}

} // namespace
#endif

RealSenseCamera::RealSenseCamera(core::CameraConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

RealSenseCamera::~RealSenseCamera() {
    stop();
}
RealSenseCamera::RealSenseCamera(RealSenseCamera&&) noexcept = default;
auto RealSenseCamera::operator=(RealSenseCamera&&) noexcept -> RealSenseCamera& = default;

bool RealSenseCamera::initialize() {
#if defined(PERCEPTION_HAS_REALSENSE)
    // librealsense reports USB control-transfer and power-state failures only through its own
    // logger; without this they never reach the journal and a silent Motion Module looks healthy.
    static std::once_flag librealsense_logging_once;
    std::call_once(librealsense_logging_once, []() { rs2::log_to_console(RS2_LOG_SEVERITY_WARN); });
    try {
        const rs2::device_list devices = impl_->context.query_devices();
        if (devices.size() == 0) {
            return false;
        }
        std::optional<rs2::device> selected;
        if (!impl_->config.serial.empty()) {
            for (const rs2::device& device : devices) {
                if (device.supports(RS2_CAMERA_INFO_SERIAL_NUMBER) &&
                    impl_->config.serial == device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)) {
                    selected = device;
                    break;
                }
            }
            if (!selected.has_value()) {
                return false;
            }
        } else {
            selected = devices[0];
        }
        impl_->pipeline_config = rs2::config{};
        impl_->pipeline_config.enable_device(selected->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
        impl_->device = selected;
        impl_->device_info = read_device_info(*selected);
        impl_->initialized = true;
        return true;
    } catch (const rs2::error& error) {
#if defined(PERCEPTION_HAS_SPDLOG)
        spdlog::error("RealSense discovery failed: {}", error.what());
#endif
        return false;
    }
#else
    return false;
#endif
}

void RealSenseCamera::stop_motion() noexcept {
#if defined(PERCEPTION_HAS_REALSENSE)
    if (impl_->motion_sensor.has_value()) {
        try {
            if (impl_->motion_started) {
                impl_->motion_sensor->stop();
            }
            impl_->motion_sensor->close();
        } catch (...) {
            // Shutdown and restart paths must not throw; a failed close is retried by reopen.
        }
    }
    // sensor.stop() above joins the callback thread, so the queues can be released safely.
    impl_->motion_sensor.reset();
    impl_->motion_started = false;
    if (impl_->accelerometer_queue) {
        impl_->accelerometer_queue->stop();
    }
    if (impl_->gyroscope_queue) {
        impl_->gyroscope_queue->stop();
    }
    impl_->accelerometer_queue.reset();
    impl_->gyroscope_queue.reset();
    impl_->last_accelerometer_capture_ns = 0;
    impl_->last_gyroscope_capture_ns = 0;
    impl_->motion_ready_logged = false;
#endif
}

bool RealSenseCamera::start_motion() noexcept {
#if defined(PERCEPTION_HAS_REALSENSE)
    stop_motion();
    impl_->motion_attempt_ns = steady_now_ns();
    impl_->motion_fault_logged = false;
    if (!impl_->device.has_value()) {
        return false;
    }
    const auto& imu = impl_->config.imu;
    try {
        std::optional<MotionProfiles> motion = find_motion_profiles(*impl_->device, imu);
        if (!motion.has_value()) {
#if defined(PERCEPTION_HAS_SPDLOG)
            spdlog::error("D435i Motion Module does not expose accel={} Hz gyro={} Hz "
                          "MOTION_XYZ32F profiles; imu.ready=false ekf.ready=false; "
                          "RGB/depth remain available",
                          imu.accelerometer_fps, imu.gyroscope_fps);
#endif
            return false;
        }
        impl_->accelerometer_queue =
            std::make_unique<core::BoundedQueue<AccelerometerSample>>(imu.queue_capacity);
        impl_->gyroscope_queue =
            std::make_unique<core::BoundedQueue<GyroscopeSample>>(imu.queue_capacity);
        impl_->motion_sensor = motion->sensor;
        impl_->motion_sensor->open(
            std::vector<rs2::stream_profile>{motion->accelerometer, motion->gyroscope});
        impl_->motion_callbacks = 0;
        impl_->first_accelerometer_logged = false;
        impl_->first_gyroscope_logged = false;
        impl_->motion_sensor->start([this](const rs2::frame& frame) {
            [[maybe_unused]] const std::uint64_t callback_index = ++impl_->motion_callbacks;
            const rs2::motion_frame motion_frame = frame.as<rs2::motion_frame>();
            if (!motion_frame) {
#if defined(PERCEPTION_HAS_SPDLOG)
                if (callback_index == 1U) {
                    spdlog::warn("D435i Motion Module delivered a non-motion frame first: {}",
                                 describe_profile(frame.get_profile()));
                }
#endif
                return;
            }
            const std::uint64_t capture_ns = steady_now_ns();
            const auto sensor_timestamp_ns =
                static_cast<std::uint64_t>(motion_frame.get_timestamp() * 1'000'000.0);
            const rs2_vector data = motion_frame.get_motion_data();
            const rs2_stream stream = motion_frame.get_profile().stream_type();
            if (stream == RS2_STREAM_ACCEL && impl_->accelerometer_queue) {
                impl_->accelerometer_queue->push(
                    {sensor_timestamp_ns, capture_ns, {data.x, data.y, data.z}});
                impl_->last_accelerometer_capture_ns = capture_ns;
#if defined(PERCEPTION_HAS_SPDLOG)
                if (!impl_->first_accelerometer_logged.exchange(true)) {
                    spdlog::info("D435i first accelerometer sample {} ({:.3f},{:.3f},{:.3f})",
                                 describe_profile(motion_frame.get_profile()), data.x, data.y,
                                 data.z);
                }
#endif
            } else if (stream == RS2_STREAM_GYRO && impl_->gyroscope_queue) {
                impl_->gyroscope_queue->push(
                    {sensor_timestamp_ns, capture_ns, {data.x, data.y, data.z}});
                impl_->last_gyroscope_capture_ns = capture_ns;
#if defined(PERCEPTION_HAS_SPDLOG)
                if (!impl_->first_gyroscope_logged.exchange(true)) {
                    spdlog::info("D435i first gyroscope sample {} ({:.3f},{:.3f},{:.3f})",
                                 describe_profile(motion_frame.get_profile()), data.x, data.y,
                                 data.z);
                }
#endif
            }
        });
        impl_->motion_started = true;
#if defined(PERCEPTION_HAS_SPDLOG)
        spdlog::info("D435i Motion Module started sensor='{}' accel=[{}] gyro=[{}] "
                     "independently of video; awaiting samples",
                     motion->sensor.get_info(RS2_CAMERA_INFO_NAME),
                     describe_profile(motion->accelerometer), describe_profile(motion->gyroscope));
#endif
        return true;
    } catch (const rs2::error& error) {
#if defined(PERCEPTION_HAS_SPDLOG)
        spdlog::error("D435i Motion Module start failed: {}; imu.ready=false ekf.ready=false; "
                      "RGB/depth remain available",
                      error.what());
#endif
        stop_motion();
        return false;
    }
#else
    return false;
#endif
}

// Tracks IMU liveness for logging and schedules bounded Motion Module restarts. It never
// influences the video path: RGB/depth capture continues whatever this decides.
void RealSenseCamera::update_motion_state() noexcept {
#if defined(PERCEPTION_HAS_REALSENSE)
    const auto& imu = impl_->config.imu;
    const std::uint64_t now_ns = steady_now_ns();
    const std::uint64_t last_accel = impl_->last_accelerometer_capture_ns.load();
    const std::uint64_t last_gyro = impl_->last_gyroscope_capture_ns.load();
    const auto to_ns = [](std::uint32_t milliseconds) {
        return static_cast<std::uint64_t>(milliseconds) * 1'000'000ULL;
    };
    const bool received_both = last_accel != 0U && last_gyro != 0U;
    const bool fresh = received_both && now_ns >= last_accel && now_ns >= last_gyro &&
                       now_ns - last_accel <= to_ns(imu.liveness_timeout_ms) &&
                       now_ns - last_gyro <= to_ns(imu.liveness_timeout_ms);
    const std::uint64_t since_attempt_ns =
        now_ns >= impl_->motion_attempt_ns ? now_ns - impl_->motion_attempt_ns : 0U;
    const bool startup_expired =
        impl_->motion_started && !received_both && since_attempt_ns > to_ns(imu.startup_timeout_ms);
    const bool stale = received_both && !fresh;
    const bool faulted = !impl_->motion_started || startup_expired || stale;

    if (fresh) {
        if (!impl_->motion_ready_logged) {
#if defined(PERCEPTION_HAS_SPDLOG)
            spdlog::info("D435i Motion Module ready accel={} Hz gyro={} Hz imu.ready=true "
                         "ekf.ready=true",
                         imu.accelerometer_fps, imu.gyroscope_fps);
#endif
            impl_->motion_ready_logged = true;
        }
        impl_->motion_fault_logged = false;
        return;
    }
    if ((startup_expired || stale) && !impl_->motion_fault_logged) {
#if defined(PERCEPTION_HAS_SPDLOG)
        spdlog::error("imu.ready=false ekf.ready=false reason={} timeout_ms={} "
                      "motion_callbacks={}; RGB/depth remain available",
                      startup_expired ? "startup_timeout" : "samples_stale",
                      startup_expired ? imu.startup_timeout_ms : imu.liveness_timeout_ms,
                      impl_->motion_callbacks.load());
#endif
        impl_->motion_ready_logged = false;
        impl_->motion_fault_logged = true;
    }
    if (faulted && imu.restart_interval_ms > 0 &&
        since_attempt_ns >= to_ns(imu.restart_interval_ms)) {
#if defined(PERCEPTION_HAS_SPDLOG)
        const char* reason = !impl_->motion_started ? "not_started"
                             : startup_expired      ? "startup_timeout"
                                                    : "samples_stale";
        spdlog::warn("Restarting D435i Motion Module (reason={}) without stopping video", reason);
#endif
        (void)start_motion();
    }
#endif
}

bool RealSenseCamera::start() {
#if defined(PERCEPTION_HAS_REALSENSE)
    if (!impl_->initialized) {
        return false;
    }
    try {
        const auto& camera = impl_->config;
        if (camera.rgb.enabled) {
            impl_->pipeline_config.enable_stream(RS2_STREAM_COLOR, camera.rgb.width,
                                                 camera.rgb.height, RS2_FORMAT_BGR8,
                                                 camera.rgb.fps);
        }
        if (camera.depth.enabled) {
            impl_->pipeline_config.enable_stream(RS2_STREAM_DEPTH, camera.depth.width,
                                                 camera.depth.height, RS2_FORMAT_Z16,
                                                 camera.depth.fps);
        }
        if (camera.infrared_left.enabled) {
            impl_->pipeline_config.enable_stream(RS2_STREAM_INFRARED, 1, camera.infrared_left.width,
                                                 camera.infrared_left.height, RS2_FORMAT_Y8,
                                                 camera.infrared_left.fps);
        }
        if (camera.infrared_right.enabled) {
            impl_->pipeline_config.enable_stream(
                RS2_STREAM_INFRARED, 2, camera.infrared_right.width, camera.infrared_right.height,
                RS2_FORMAT_Y8, camera.infrared_right.fps);
        }
        const bool any_video = camera.rgb.enabled || camera.depth.enabled ||
                               camera.infrared_left.enabled || camera.infrared_right.enabled;
        if (any_video) {
            const rs2::pipeline_profile profile = impl_->pipeline.start(impl_->pipeline_config);
            impl_->pipeline_started = true;
            // The Motion Module must be opened on the pipeline's own device instance. A second
            // rs2::device for the same camera runs its own global-time keeper, which tries to claim
            // the depth interface the pipeline already holds and fails every 100 ms
            // ("failed to claim usb interface 0, is busy").
            impl_->device = profile.get_device();
            const rs2::depth_sensor sensor = impl_->device->first<rs2::depth_sensor>();
            impl_->depth_scale_m = sensor.get_depth_scale();
#if defined(PERCEPTION_HAS_SPDLOG)
            spdlog::info("D435i video pipeline started rgb={} depth={} ir_left={} ir_right={}",
                         camera.rgb.enabled, camera.depth.enabled, camera.infrared_left.enabled,
                         camera.infrared_right.enabled);
#endif
        } else {
            // Diagnostic IMU-only session: an empty rs2::config would otherwise start every
            // default stream, so the video pipeline is skipped entirely.
#if defined(PERCEPTION_HAS_SPDLOG)
            spdlog::info("D435i video pipeline skipped: no video stream enabled");
#endif
        }
        impl_->running = true;
    } catch (const rs2::error& error) {
#if defined(PERCEPTION_HAS_SPDLOG)
        spdlog::error("RealSense video start failed: {}", error.what());
#endif
        stop();
        return false;
    }
    // A Motion Module failure is reported through imu.ready/ekf.ready only; video is already up.
    if (impl_->config.imu.enabled) {
        (void)start_motion();
    }
    return true;
#else
    return false;
#endif
}

void RealSenseCamera::stop() noexcept {
    if (!impl_) {
        return;
    }
#if defined(PERCEPTION_HAS_REALSENSE)
    stop_motion();
    if (impl_->pipeline_started) {
        try {
            impl_->pipeline.stop();
        } catch (...) {
            // Destructors and shutdown paths must not throw.
        }
    }
    impl_->pipeline_started = false;
    impl_->motion_attempt_ns = 0;
    impl_->motion_fault_logged = false;
#endif
    impl_->running = false;
}

bool RealSenseCamera::capture(CameraFrameSet& frame_set) {
#if defined(PERCEPTION_HAS_REALSENSE)
    if (!impl_->running) {
        return false;
    }
    try {
        if (impl_->config.imu.enabled) {
            update_motion_state();
        }
        rs2::frameset frames;
        if (impl_->pipeline_started) {
            if (!impl_->pipeline.try_wait_for_frames(
                    &frames, static_cast<unsigned int>(impl_->config.capture_timeout_ms))) {
                return false;
            }
        } else {
            // IMU-only session: pace the caller like a 30 fps video capture would.
            std::this_thread::sleep_for(std::chrono::milliseconds(33));
        }

        const auto now = std::chrono::steady_clock::now().time_since_epoch();
        const auto capture_ns = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
        frame_set = {};
        frame_set.capture_timestamp_ns = capture_ns;

        const auto copy_image = [capture_ns](const rs2::video_frame& source, PixelFormat format,
                                             ImageFrame& target) {
            if (!source) {
                return;
            }
            target.sensor_timestamp_ns =
                static_cast<std::uint64_t>(source.get_timestamp() * 1'000'000.0);
            target.capture_timestamp_ns = capture_ns;
            target.frame_number = static_cast<std::uint64_t>(source.get_frame_number());
            target.width = source.get_width();
            target.height = source.get_height();
            target.stride_bytes = source.get_stride_in_bytes();
            target.format = format;
            const auto byte_count = static_cast<std::size_t>(target.stride_bytes) *
                                    static_cast<std::size_t>(target.height);
            target.data.resize(byte_count);
            std::memcpy(target.data.data(), source.get_data(), byte_count);
        };

        copy_image(frames.get_color_frame(), PixelFormat::Bgr8, frame_set.rgb);
        copy_image(frames.get_infrared_frame(1), PixelFormat::Gray8, frame_set.ir_left);
        copy_image(frames.get_infrared_frame(2), PixelFormat::Gray8, frame_set.ir_right);

        const rs2::depth_frame depth = frames.get_depth_frame();
        if (depth) {
            frame_set.depth.sensor_timestamp_ns =
                static_cast<std::uint64_t>(depth.get_timestamp() * 1'000'000.0);
            frame_set.depth.capture_timestamp_ns = capture_ns;
            frame_set.depth.frame_number = static_cast<std::uint64_t>(depth.get_frame_number());
            frame_set.depth.width = depth.get_width();
            frame_set.depth.height = depth.get_height();
            frame_set.depth.depth_scale_m = impl_->depth_scale_m;
            const auto element_count = static_cast<std::size_t>(depth.get_width()) *
                                       static_cast<std::size_t>(depth.get_height());
            frame_set.depth.data.resize(element_count);
            std::memcpy(frame_set.depth.data.data(), depth.get_data(),
                        element_count * sizeof(std::uint16_t));
        }
        if (impl_->accelerometer_queue && impl_->gyroscope_queue) {
            ImuBatch imu;
            while (const auto sample = impl_->accelerometer_queue->try_pop()) {
                imu.accelerometer.push_back(*sample);
            }
            imu.accelerometer_dropped = impl_->accelerometer_queue->dropped_count();
            while (const auto sample = impl_->gyroscope_queue->try_pop()) {
                imu.gyroscope.push_back(*sample);
            }
            imu.gyroscope_dropped = impl_->gyroscope_queue->dropped_count();
            if (!imu.accelerometer.empty() || !imu.gyroscope.empty()) {
                frame_set.imu = std::move(imu);
            }
        }
        return true;
    } catch (const rs2::error& error) {
#if defined(PERCEPTION_HAS_SPDLOG)
        spdlog::error("RealSense capture failed: {}", error.what());
#endif
        return false;
    }
#else
    (void)frame_set;
    return false;
#endif
}

auto RealSenseCamera::device_info() const -> std::optional<CameraDeviceInfo> {
    return impl_->device_info;
}

bool RealSenseCamera::hardware_reset() {
#if defined(PERCEPTION_HAS_REALSENSE)
    if (!impl_->device.has_value()) {
        return false;
    }
    stop();
    try {
        impl_->device->hardware_reset();
    } catch (const rs2::error& error) {
#if defined(PERCEPTION_HAS_SPDLOG)
        spdlog::error("RealSense hardware reset failed: {}", error.what());
#endif
        return false;
    }
    // The device re-enumerates; the cached handle and discovery are stale until initialize().
    impl_->device.reset();
    impl_->initialized = false;
#if defined(PERCEPTION_HAS_SPDLOG)
    spdlog::warn("RealSense hardware reset issued; the D435i re-enumerates over USB");
#endif
    return true;
#else
    return false;
#endif
}

} // namespace perception::camera
