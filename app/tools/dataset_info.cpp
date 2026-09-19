#include "perception/dataset/dataset.hpp"
#include "perception/geometry/point_cloud.hpp"
#include "perception/geometry/transform.hpp"

#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>

namespace {

void print_intrinsics(const char* label, const perception::camera::CameraIntrinsics& intrinsics) {
    std::cout << "  " << label << ": " << intrinsics.width << 'x' << intrinsics.height
              << " fx=" << intrinsics.focal_x << " fy=" << intrinsics.focal_y
              << " cx=" << intrinsics.principal_x << " cy=" << intrinsics.principal_y
              << " model=" << intrinsics.distortion_model << '\n';
}

} // namespace

// Verifies a recording offline: it replays through the same ICamera path the live service uses,
// then deprojects one frame with the manifest's own intrinsics. A dataset that passes here can be
// consumed by the depth and localization branches with no camera present.
int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: dataset-info <dataset-directory> [frame-index]\n";
        return 2;
    }
    const std::filesystem::path directory(argv[1]);
    const std::size_t frame_index = argc > 2 ? static_cast<std::size_t>(std::atoll(argv[2])) : 0;

    perception::dataset::DatasetReader reader;
    std::string error;
    if (!reader.open(directory, error)) {
        std::cerr << "Could not open dataset: " << error << '\n';
        return 1;
    }

    const auto& manifest = reader.manifest();
    std::cout << "format_version: " << manifest.format_version << '\n'
              << "created_utc: " << manifest.created_utc << '\n'
              << "device: " << manifest.device.model << " serial=" << manifest.device.serial
              << " firmware=" << manifest.device.firmware
              << " depth_scale_m=" << manifest.device.depth_scale_m << '\n'
              << "frames: " << reader.frame_count() << '\n'
              << "intrinsics:\n";
    if (manifest.rgb_intrinsics.has_value()) {
        print_intrinsics("rgb", *manifest.rgb_intrinsics);
    } else {
        std::cout << "  rgb: MISSING\n";
    }
    if (manifest.depth_intrinsics.has_value()) {
        print_intrinsics("depth", *manifest.depth_intrinsics);
    } else {
        std::cout << "  depth: MISSING (this dataset cannot be deprojected)\n";
    }
    std::cout << "mount: x=" << manifest.mount.x_m << " y=" << manifest.mount.y_m
              << " z=" << manifest.mount.z_m << " roll=" << manifest.mount.roll_deg
              << " pitch=" << manifest.mount.pitch_deg << " yaw=" << manifest.mount.yaw_deg << '\n';

    if (reader.frame_count() == 0) {
        std::cerr << "Dataset contains no frames.\n";
        return 1;
    }

    // Timing comes from the recorded capture timestamps, so a dropped-frame gap is visible here.
    perception::camera::CameraFrameSet first;
    perception::camera::CameraFrameSet last;
    if (!reader.read(0, first, error) || !reader.read(reader.frame_count() - 1, last, error)) {
        std::cerr << "Could not read the dataset bounds: " << error << '\n';
        return 1;
    }
    if (last.capture_timestamp_ns > first.capture_timestamp_ns) {
        const double span_s =
            static_cast<double>(last.capture_timestamp_ns - first.capture_timestamp_ns) / 1e9;
        std::cout << "duration_s: " << std::fixed << std::setprecision(2) << span_s
                  << " mean_fps: "
                  << (span_s > 0.0 ? static_cast<double>(reader.frame_count() - 1) / span_s : 0.0)
                  << std::defaultfloat << '\n';
    }

    if (frame_index >= reader.frame_count()) {
        std::cerr << "Frame index " << frame_index << " is out of range.\n";
        return 2;
    }
    perception::camera::CameraFrameSet frame_set;
    if (!reader.read(frame_index, frame_set, error)) {
        std::cerr << "Could not read frame " << frame_index << ": " << error << '\n';
        return 1;
    }

    std::cout << "frame " << frame_index << ":\n"
              << "  rgb: " << frame_set.rgb.width << 'x' << frame_set.rgb.height << ' '
              << frame_set.rgb.data.size() << " bytes\n"
              << "  depth: " << frame_set.depth.width << 'x' << frame_set.depth.height << ' '
              << frame_set.depth.data.size() << " samples scale=" << frame_set.depth.depth_scale_m
              << '\n';
    if (frame_set.imu.has_value()) {
        std::cout << "  imu: accel=" << frame_set.imu->accelerometer.size()
                  << " gyro=" << frame_set.imu->gyroscope.size()
                  << " accel_dropped=" << frame_set.imu->accelerometer_dropped
                  << " gyro_dropped=" << frame_set.imu->gyroscope_dropped << '\n';
    } else {
        std::cout << "  imu: none recorded\n";
    }

    if (!manifest.depth_intrinsics.has_value()) {
        std::cerr << "No depth intrinsics: skipping deprojection. The dataset is not usable for "
                     "the depth or localization branches.\n";
        return 1;
    }

    perception::geometry::DeprojectOptions options;
    auto cloud =
        perception::geometry::deproject_depth(frame_set.depth, *manifest.depth_intrinsics, options);
    std::cout << "  point_cloud: " << cloud.points.size() << " points frame=" << cloud.frame_id
              << " rejected_invalid=" << cloud.rejected_invalid
              << " rejected_out_of_range=" << cloud.rejected_out_of_range << '\n';
    if (cloud.points.empty()) {
        std::cerr << "Deprojection produced no points; check the configured range against the "
                     "scene.\n";
        return 1;
    }

    perception::geometry::transform_in_place(
        cloud, perception::geometry::camera_optical_to_base_link(manifest.mount),
        perception::geometry::frames::kBaseLink);
    const auto& sample = cloud.points.front();
    std::cout << "  first point in " << cloud.frame_id << ": x=" << sample.x << " y=" << sample.y
              << " z=" << sample.z << '\n'
              << "Dataset is replayable and deprojectable.\n";
    return 0;
}
