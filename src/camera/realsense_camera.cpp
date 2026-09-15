#include "perception/camera/realsense_camera.hpp"

#include <chrono>
#include <cstdint>
#include <cstring>
#include <mutex>
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
#if defined(PERCEPTION_HAS_REALSENSE)
    rs2::pipeline pipeline;
    rs2::config pipeline_config;
    std::unique_ptr<rs2::frame_queue> video_queue;
    std::mutex imu_mutex;
    ImuSample latest_imu;
    bool acceleration_received{false};
    bool gyroscope_received{false};
    float depth_scale_m{0.001F};
#endif
};

RealSenseCamera::RealSenseCamera(core::CameraConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

RealSenseCamera::~RealSenseCamera() { stop(); }
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
        if (!impl_->config.serial.empty()) {
            impl_->pipeline_config.enable_device(impl_->config.serial);
        }
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
                                                 camera.rgb.height, RS2_FORMAT_BGR8, camera.rgb.fps);
        }
        if (camera.depth.enabled) {
            impl_->pipeline_config.enable_stream(RS2_STREAM_DEPTH, camera.depth.width,
                                                 camera.depth.height, RS2_FORMAT_Z16, camera.depth.fps);
        }
        if (camera.infrared_left.enabled) {
            impl_->pipeline_config.enable_stream(RS2_STREAM_INFRARED, 1,
                                                 camera.infrared_left.width,
                                                 camera.infrared_left.height, RS2_FORMAT_Y8,
                                                 camera.infrared_left.fps);
        }
        if (camera.infrared_right.enabled) {
            impl_->pipeline_config.enable_stream(RS2_STREAM_INFRARED, 2,
                                                 camera.infrared_right.width,
                                                 camera.infrared_right.height, RS2_FORMAT_Y8,
                                                 camera.infrared_right.fps);
        }
        if (camera.imu_enabled) {
            impl_->pipeline_config.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
            impl_->pipeline_config.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
        }
        impl_->video_queue = std::make_unique<rs2::frame_queue>(2);
        impl_->acceleration_received = false;
        impl_->gyroscope_received = false;
        const rs2::pipeline_profile profile =
            impl_->pipeline.start(impl_->pipeline_config, [this](const rs2::frame& frame) {
                const auto now = std::chrono::steady_clock::now().time_since_epoch();
                const auto capture_ns = static_cast<std::uint64_t>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
                if (const rs2::motion_frame motion = frame.as<rs2::motion_frame>()) {
                    const rs2_vector data = motion.get_motion_data();
                    std::scoped_lock lock(impl_->imu_mutex);
                    impl_->latest_imu.sensor_timestamp_ns =
                        static_cast<std::uint64_t>(motion.get_timestamp() * 1'000'000.0);
                    impl_->latest_imu.capture_timestamp_ns = capture_ns;
                    if (motion.get_profile().stream_type() == RS2_STREAM_ACCEL) {
                        impl_->latest_imu.acceleration_mps2 = {data.x, data.y, data.z};
                        impl_->acceleration_received = true;
                    } else if (motion.get_profile().stream_type() == RS2_STREAM_GYRO) {
                        impl_->latest_imu.angular_velocity_rps = {data.x, data.y, data.z};
                        impl_->gyroscope_received = true;
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
#endif
    }
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
        {
            std::scoped_lock lock(impl_->imu_mutex);
            if (impl_->acceleration_received && impl_->gyroscope_received) {
                frame_set.imu = impl_->latest_imu;
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

}  // namespace perception::camera
