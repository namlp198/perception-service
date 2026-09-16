#include "perception/camera/realsense_camera.hpp"

#include "perception/core/bounded_queue.hpp"

#include <chrono>
#include <cstdint>
#include <cstring>
#include <optional>
#include <string>
#include <utility>

#if defined(PERCEPTION_HAS_REALSENSE)
#include <librealsense2/rs.hpp>
#endif

#if defined(PERCEPTION_HAS_SPDLOG)
#include <spdlog/spdlog.h>
#endif

namespace perception::camera {

class RealSenseCamera::Impl {
  public:
    explicit Impl(core::CameraConfig value) : config(std::move(value)) {}

    core::CameraConfig config;
    bool initialized{false};
    bool running{false};
    std::optional<CameraDeviceInfo> device_info;
#if defined(PERCEPTION_HAS_REALSENSE)
    rs2::pipeline pipeline;
    rs2::config pipeline_config;
    std::unique_ptr<rs2::frame_queue> video_queue;
    std::unique_ptr<core::BoundedQueue<AccelerometerSample>> accelerometer_queue;
    std::unique_ptr<core::BoundedQueue<GyroscopeSample>> gyroscope_queue;
    float depth_scale_m{0.001F};
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
    try {
        const rs2::context context;
        const rs2::device_list devices = context.query_devices();
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
        impl_->pipeline_config.enable_device(selected->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
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
        if (camera.imu.enabled) {
            impl_->pipeline_config.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F,
                                                 camera.imu.accelerometer_fps);
            impl_->pipeline_config.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F,
                                                 camera.imu.gyroscope_fps);
            impl_->accelerometer_queue = std::make_unique<core::BoundedQueue<AccelerometerSample>>(
                camera.imu.queue_capacity);
            impl_->gyroscope_queue =
                std::make_unique<core::BoundedQueue<GyroscopeSample>>(camera.imu.queue_capacity);
        }
        impl_->video_queue = std::make_unique<rs2::frame_queue>(2);
        const rs2::pipeline_profile profile =
            impl_->pipeline.start(impl_->pipeline_config, [this](const rs2::frame& frame) {
                const auto now = std::chrono::steady_clock::now().time_since_epoch();
                const auto capture_ns = static_cast<std::uint64_t>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
                if (const rs2::motion_frame motion = frame.as<rs2::motion_frame>()) {
                    const rs2_vector data = motion.get_motion_data();
                    const auto sensor_timestamp_ns =
                        static_cast<std::uint64_t>(motion.get_timestamp() * 1'000'000.0);
                    if (motion.get_profile().stream_type() == RS2_STREAM_ACCEL &&
                        impl_->accelerometer_queue) {
                        impl_->accelerometer_queue->push(
                            {sensor_timestamp_ns, capture_ns, {data.x, data.y, data.z}});
                    } else if (motion.get_profile().stream_type() == RS2_STREAM_GYRO &&
                               impl_->gyroscope_queue) {
                        impl_->gyroscope_queue->push(
                            {sensor_timestamp_ns, capture_ns, {data.x, data.y, data.z}});
                    }
                    return;
                }
                if (frame.is<rs2::frameset>() && impl_->video_queue) {
                    impl_->video_queue->enqueue(frame);
                }
            });
        const rs2::depth_sensor sensor = profile.get_device().first<rs2::depth_sensor>();
        impl_->depth_scale_m = sensor.get_depth_scale();
        impl_->running = true;
        return true;
    } catch (const rs2::error& error) {
#if defined(PERCEPTION_HAS_SPDLOG)
        spdlog::error("RealSense start failed: {}", error.what());
#endif
        impl_->running = false;
        impl_->video_queue.reset();
        impl_->accelerometer_queue.reset();
        impl_->gyroscope_queue.reset();
        return false;
    }
#else
    return false;
#endif
}

void RealSenseCamera::stop() noexcept {
#if defined(PERCEPTION_HAS_REALSENSE)
    if (impl_ && impl_->running) {
        try {
            impl_->pipeline.stop();
        } catch (...) {
            // Destructors and shutdown paths must not throw.
        }
    }
#endif
    if (impl_) {
        impl_->running = false;
#if defined(PERCEPTION_HAS_REALSENSE)
        impl_->video_queue.reset();
        if (impl_->accelerometer_queue) {
            impl_->accelerometer_queue->stop();
        }
        if (impl_->gyroscope_queue) {
            impl_->gyroscope_queue->stop();
        }
        impl_->accelerometer_queue.reset();
        impl_->gyroscope_queue.reset();
#endif
    }
}

void RealSenseCamera::disable_imu() noexcept {
    stop();
    impl_->config.imu.enabled = false;
    impl_->initialized = false;
#if defined(PERCEPTION_HAS_REALSENSE)
    impl_->pipeline_config = rs2::config{};
#endif
}

bool RealSenseCamera::imu_enabled() const noexcept {
    return impl_->config.imu.enabled;
}

bool RealSenseCamera::capture(CameraFrameSet& frame_set) {
#if defined(PERCEPTION_HAS_REALSENSE)
    if (!impl_->running) {
        return false;
    }
    try {
        if (!impl_->video_queue) {
            return false;
        }
        rs2::frame queued_frame;
        if (!impl_->video_queue->try_wait_for_frame(
                &queued_frame, static_cast<unsigned int>(impl_->config.capture_timeout_ms))) {
            return false;
        }
        const rs2::frameset frames = queued_frame.as<rs2::frameset>();
        if (!frames) {
            return false;
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
        if (impl_->accelerometer_queue || impl_->gyroscope_queue) {
            ImuBatch imu;
            if (impl_->accelerometer_queue) {
                while (const auto sample = impl_->accelerometer_queue->try_pop()) {
                    imu.accelerometer.push_back(*sample);
                }
                imu.accelerometer_dropped = impl_->accelerometer_queue->dropped_count();
            }
            if (impl_->gyroscope_queue) {
                while (const auto sample = impl_->gyroscope_queue->try_pop()) {
                    imu.gyroscope.push_back(*sample);
                }
                imu.gyroscope_dropped = impl_->gyroscope_queue->dropped_count();
            }
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

} // namespace perception::camera
