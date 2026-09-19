#include "perception/camera/realsense_camera.hpp"
#include "perception/core/config.hpp"
#include "perception/dataset/dataset.hpp"

#include <chrono>
#include <csignal>
#include <ctime>
#include <filesystem>
#include <iostream>
#include <string>

namespace {

volatile std::sig_atomic_t g_stop_requested = 0;

void handle_signal(int /*signal*/) { g_stop_requested = 1; }

auto utc_now_iso8601() -> std::string {
    const auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm parts{};
#if defined(_WIN32)
    gmtime_s(&parts, &now);
#else
    gmtime_r(&now, &parts);
#endif
    char buffer[32];
    if (std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", &parts) == 0) {
        return {};
    }
    return buffer;
}

} // namespace

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "Usage: dataset-record <config.yaml> <output-directory> [seconds]\n"
                  << "Records D435i frame sets for offline replay. Stop early with Ctrl+C.\n"
                  << "The perception-service must be stopped first: one process owns the D435i.\n";
        return 2;
    }
    const std::filesystem::path config_path(argv[1]);
    const std::filesystem::path output_directory(argv[2]);
    const double seconds = argc > 3 ? std::stod(argv[3]) : 10.0;
    if (seconds <= 0.0) {
        std::cerr << "Recording duration must be greater than zero.\n";
        return 2;
    }

    const auto config = perception::core::load_config(config_path);
    perception::camera::RealSenseCamera camera(config.camera);
    if (!camera.initialize()) {
        // The most common cause by far is a running service still holding the device, which
        // librealsense reports as "failed to claim usb interface, interface 0, is busy" followed
        // by a power-state failure. Saying "no camera found" there sends the operator hunting for
        // a cabling fault that does not exist.
        std::cerr << "Could not open the D435i.\n"
                  << "If the log above shows 'is busy' or 'failed to set power state', the camera "
                     "is held by another process:\n"
                     "  systemctl --user stop perception-service-user.service\n"
                     "  ./scripts/jetson/record-dataset.sh --confirm-service-interruption "
                     "<output-directory> [seconds]\n"
                  << "The helper script stops the service, records and restarts it. Otherwise the "
                     "camera is absent or the backend was disabled at build time.\n";
        return 1;
    }

    perception::dataset::DatasetManifest manifest;
    manifest.created_utc = utc_now_iso8601();
    bool rgb_exact = false;
    bool depth_exact = false;
    if (const auto info = camera.device_info(); info.has_value()) {
        manifest.device = *info;
        manifest.rgb_intrinsics = perception::dataset::select_stream_intrinsics(
            *info, "color", config.camera.rgb.width, config.camera.rgb.height, &rgb_exact);
        manifest.depth_intrinsics = perception::dataset::select_stream_intrinsics(
            *info, "depth", config.camera.depth.width, config.camera.depth.height, &depth_exact);
    }
    manifest.mount.x_m = config.geometry.camera_x_m;
    manifest.mount.y_m = config.geometry.camera_y_m;
    manifest.mount.z_m = config.geometry.camera_z_m;
    manifest.mount.roll_deg = config.geometry.camera_roll_deg;
    manifest.mount.pitch_deg = config.geometry.camera_pitch_deg;
    manifest.mount.yaw_deg = config.geometry.camera_yaw_deg;
    manifest.notes = "recorded by dataset-record";

    if (!manifest.depth_intrinsics.has_value()) {
        // Without intrinsics a replay cannot deproject, so say so rather than leave it to be
        // discovered offline.
        std::cerr << "Warning: no depth intrinsics were reported; the dataset will not be usable "
                     "for deprojection.\n";
    } else if (!depth_exact) {
        // Intrinsics from another resolution have the wrong focal length and principal point;
        // deprojecting with them is quietly wrong, which is worse than having none.
        std::cerr << "Warning: no depth profile matched " << config.camera.depth.width << 'x'
                  << config.camera.depth.height
                  << "; the stored intrinsics belong to a different resolution.\n";
    }
    if (manifest.rgb_intrinsics.has_value() && !rgb_exact) {
        std::cerr << "Warning: the stored colour intrinsics belong to a different resolution than "
                  << config.camera.rgb.width << 'x' << config.camera.rgb.height << ".\n";
    }

    perception::dataset::DatasetWriter writer;
    std::string error;
    if (!writer.open(output_directory, manifest, error)) {
        std::cerr << "Could not start recording: " << error << '\n';
        return 1;
    }

    if (!camera.start()) {
        std::cerr << "Camera failed to start.\n";
        return 1;
    }

    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(static_cast<long long>(seconds * 1000.0));
    std::uint64_t capture_failures = 0;
    auto next_report = std::chrono::steady_clock::now() + std::chrono::seconds(1);

    while (g_stop_requested == 0 && std::chrono::steady_clock::now() < deadline) {
        perception::camera::CameraFrameSet frame_set;
        if (!camera.capture(frame_set)) {
            ++capture_failures;
            if (capture_failures >= 30) {
                std::cerr << "Capture failed 30 times in a row; stopping.\n";
                break;
            }
            continue;
        }
        capture_failures = 0;
        if (!writer.append(frame_set, error)) {
            std::cerr << "Recording failed: " << error << '\n';
            break;
        }
        if (const auto now = std::chrono::steady_clock::now(); now >= next_report) {
            std::cout << "frames=" << writer.frames_written()
                      << " payload_mb=" << (writer.bytes_written() / (1024 * 1024)) << '\n'
                      << std::flush;
            next_report = now + std::chrono::seconds(1);
        }
    }

    camera.stop();
    if (!writer.close(error)) {
        std::cerr << "Failed to finish the dataset: " << error << '\n';
        return 1;
    }

    std::cout << "Recorded " << writer.frames_written() << " frame sets ("
              << (writer.bytes_written() / (1024 * 1024)) << " MB of payload) to "
              << output_directory.string() << '\n';
    return writer.frames_written() > 0 ? 0 : 1;
}
