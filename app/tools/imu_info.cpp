#include "perception/camera/realsense_camera.hpp"
#include "perception/core/config.hpp"

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>

int main(int argc, char* argv[]) {
    const std::filesystem::path config_path =
        argc > 1 ? std::filesystem::path(argv[1]) : std::filesystem::path("config/default.yaml");
    const int duration_seconds = argc > 2 ? std::atoi(argv[2]) : 10;
    if (duration_seconds <= 0 || duration_seconds > 300) {
        std::cerr << "Duration must be in the range 1..300 seconds.\n";
        return 2;
    }

    auto config = perception::core::load_config(config_path).camera;
    config.imu.enabled = true;
    // Optional rate overrides let a field diagnostic try other exposed profile pairs (for
    // example 200/200) without editing the production configuration.
    if (argc > 3) {
        config.imu.accelerometer_fps = std::atoi(argv[3]);
    }
    if (argc > 4) {
        config.imu.gyroscope_fps = std::atoi(argv[4]);
    }
    const std::string video_mode = argc > 5 ? argv[5] : "video";
    if (config.imu.accelerometer_fps <= 0 || config.imu.gyroscope_fps <= 0 ||
        (video_mode != "video" && video_mode != "no-video")) {
        std::cerr << "Usage: imu-info [config.yaml] [seconds] [accel_fps] [gyro_fps] "
                     "[video|no-video]\n";
        return 2;
    }
    // "no-video" opens only the Motion Module, isolating the IMU from any pipeline interaction.
    if (video_mode == "no-video") {
        config.rgb.enabled = false;
        config.depth.enabled = false;
        config.infrared_left.enabled = false;
        config.infrared_right.enabled = false;
    }
    // A bounded diagnostic must observe one Motion Module session, not a restart cycle.
    config.imu.restart_interval_ms = 0;
    std::cout << "requested accel_fps=" << config.imu.accelerometer_fps
              << " gyro_fps=" << config.imu.gyroscope_fps << " mode=" << video_mode << '\n';
    perception::camera::RealSenseCamera camera(config);
    if (!camera.initialize() || !camera.start()) {
        std::cerr
            << "Unable to start D435i video and IMU streams. Stop perception-service first.\n";
        return 1;
    }

    const auto started_at = std::chrono::steady_clock::now();
    const auto deadline = started_at + std::chrono::seconds(duration_seconds);
    std::uint64_t accelerometer_count = 0;
    std::uint64_t gyroscope_count = 0;
    perception::camera::AccelerometerSample latest_accelerometer;
    perception::camera::GyroscopeSample latest_gyroscope;
    bool has_accelerometer = false;
    bool has_gyroscope = false;

    while (std::chrono::steady_clock::now() < deadline) {
        perception::camera::CameraFrameSet frames;
        if (!camera.capture(frames) || !frames.imu.has_value()) {
            continue;
        }
        accelerometer_count += static_cast<std::uint64_t>(frames.imu->accelerometer.size());
        gyroscope_count += static_cast<std::uint64_t>(frames.imu->gyroscope.size());
        if (!frames.imu->accelerometer.empty()) {
            latest_accelerometer = frames.imu->accelerometer.back();
            has_accelerometer = true;
        }
        if (!frames.imu->gyroscope.empty()) {
            latest_gyroscope = frames.imu->gyroscope.back();
            has_gyroscope = true;
        }
    }
    camera.stop();

    const double elapsed_seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - started_at).count();
    std::cout << std::fixed << std::setprecision(3) << "elapsed_s=" << elapsed_seconds << '\n'
              << "accelerometer_samples=" << accelerometer_count
              << " effective_hz=" << static_cast<double>(accelerometer_count) / elapsed_seconds
              << '\n'
              << "gyroscope_samples=" << gyroscope_count
              << " effective_hz=" << static_cast<double>(gyroscope_count) / elapsed_seconds << '\n';
    if (has_accelerometer) {
        std::cout << "latest_accel_mps2=" << latest_accelerometer.acceleration_mps2[0] << ','
                  << latest_accelerometer.acceleration_mps2[1] << ','
                  << latest_accelerometer.acceleration_mps2[2]
                  << " sensor_timestamp_ns=" << latest_accelerometer.sensor_timestamp_ns << '\n';
    }
    if (has_gyroscope) {
        std::cout << "latest_gyro_rps=" << latest_gyroscope.angular_velocity_rps[0] << ','
                  << latest_gyroscope.angular_velocity_rps[1] << ','
                  << latest_gyroscope.angular_velocity_rps[2]
                  << " sensor_timestamp_ns=" << latest_gyroscope.sensor_timestamp_ns << '\n';
    }
    return has_accelerometer && has_gyroscope ? 0 : 3;
}
