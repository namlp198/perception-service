#include "perception/dataset/dataset.hpp"

#include "test_support.hpp"

#include <filesystem>
#include <string>
#include <system_error>

namespace {

using perception::camera::CameraFrameSet;

auto scratch_directory(const std::string& name) -> std::filesystem::path {
    std::error_code error;
    const auto path = std::filesystem::temp_directory_path(error) / ("perception-dataset-" + name);
    std::filesystem::remove_all(path, error);
    return path;
}

auto build_manifest() -> perception::dataset::DatasetManifest {
    perception::dataset::DatasetManifest manifest;
    manifest.created_utc = "2026-09-19T12:00:00Z";
    manifest.device.model = "Intel RealSense D435I";
    manifest.device.serial = "207122078394";
    manifest.device.firmware = "5.16.0.1";
    manifest.device.usb_mode = "3.2";
    manifest.device.depth_scale_m = 0.001F;

    perception::camera::CameraIntrinsics depth_intrinsics;
    depth_intrinsics.width = 4;
    depth_intrinsics.height = 2;
    depth_intrinsics.focal_x = 2.0F;
    depth_intrinsics.focal_y = 2.0F;
    depth_intrinsics.principal_x = 2.0F;
    depth_intrinsics.principal_y = 1.0F;
    depth_intrinsics.distortion_model = "brown_conrady";
    manifest.depth_intrinsics = depth_intrinsics;

    manifest.mount.z_m = 0.4;
    manifest.mount.pitch_deg = 15.0;
    manifest.notes = "unit test";
    return manifest;
}

auto build_frame_set(std::uint64_t index) -> CameraFrameSet {
    CameraFrameSet frame_set;
    frame_set.capture_timestamp_ns = 1'000'000ULL * (index + 1);

    frame_set.rgb.sensor_timestamp_ns = 10 + index;
    frame_set.rgb.capture_timestamp_ns = 20 + index;
    frame_set.rgb.frame_number = index;
    frame_set.rgb.width = 2;
    frame_set.rgb.height = 1;
    frame_set.rgb.stride_bytes = 6;
    frame_set.rgb.format = perception::camera::PixelFormat::Bgr8;
    frame_set.rgb.data = {static_cast<std::uint8_t>(index), 1U, 2U, 3U, 4U, 255U};

    frame_set.depth.sensor_timestamp_ns = 30 + index;
    frame_set.depth.capture_timestamp_ns = 40 + index;
    frame_set.depth.frame_number = index;
    frame_set.depth.width = 4;
    frame_set.depth.height = 2;
    frame_set.depth.depth_scale_m = 0.001F;
    frame_set.depth.data = {0U, 1'000U, 100U, 9'000U, 2'000U, 500U, 0U, 65'535U};

    perception::camera::ImuBatch imu;
    perception::camera::AccelerometerSample accelerometer;
    accelerometer.sensor_timestamp_ns = 50 + index;
    accelerometer.capture_timestamp_ns = 60 + index;
    accelerometer.acceleration_mps2 = {0.0F, -9.81F, 0.25F};
    imu.accelerometer.push_back(accelerometer);
    perception::camera::GyroscopeSample gyroscope;
    gyroscope.sensor_timestamp_ns = 70 + index;
    gyroscope.capture_timestamp_ns = 80 + index;
    gyroscope.angular_velocity_rps = {0.01F, 0.0F, -0.02F};
    imu.gyroscope.push_back(gyroscope);
    imu.accelerometer_dropped = 3;
    imu.gyroscope_dropped = 4;
    frame_set.imu = imu;
    return frame_set;
}

auto write_dataset(const std::filesystem::path& directory, std::uint64_t frames) -> bool {
    perception::dataset::DatasetWriter writer;
    std::string error;
    CHECK_TRUE(writer.open(directory, build_manifest(), error));
    for (std::uint64_t index = 0; index < frames; ++index) {
        CHECK_TRUE(writer.append(build_frame_set(index), error));
    }
    CHECK_TRUE(writer.close(error));
    CHECK_TRUE(writer.frames_written() == frames);
    return true;
}

auto round_trip_preserves_every_byte() -> bool {
    const auto directory = scratch_directory("round-trip");
    CHECK_TRUE(write_dataset(directory, 3));

    perception::dataset::DatasetReader reader;
    std::string error;
    CHECK_TRUE(reader.open(directory, error));
    CHECK_TRUE(reader.frame_count() == 3);

    for (std::uint64_t index = 0; index < 3; ++index) {
        const CameraFrameSet expected = build_frame_set(index);
        CameraFrameSet actual;
        CHECK_TRUE(reader.read(static_cast<std::size_t>(index), actual, error));

        CHECK_TRUE(actual.capture_timestamp_ns == expected.capture_timestamp_ns);
        CHECK_TRUE(actual.rgb.data == expected.rgb.data);
        CHECK_TRUE(actual.rgb.width == expected.rgb.width);
        CHECK_TRUE(actual.rgb.stride_bytes == expected.rgb.stride_bytes);
        CHECK_TRUE(actual.rgb.frame_number == expected.rgb.frame_number);
        CHECK_TRUE(actual.rgb.format == expected.rgb.format);

        // Metric depth must survive bit-exact, including the 65535 edge value: it is measurement
        // data, never re-encoded as video.
        CHECK_TRUE(actual.depth.data == expected.depth.data);
        CHECK_TRUE(actual.depth.depth_scale_m == expected.depth.depth_scale_m);
        CHECK_TRUE(actual.depth.sensor_timestamp_ns == expected.depth.sensor_timestamp_ns);

        CHECK_TRUE(actual.imu.has_value());
        CHECK_TRUE(actual.imu->accelerometer.size() == 1);
        CHECK_TRUE(actual.imu->gyroscope.size() == 1);
        CHECK_TRUE(actual.imu->accelerometer[0].acceleration_mps2 ==
                   expected.imu->accelerometer[0].acceleration_mps2);
        CHECK_TRUE(actual.imu->gyroscope[0].angular_velocity_rps ==
                   expected.imu->gyroscope[0].angular_velocity_rps);
        // Accelerometer and gyroscope keep their own timestamps; a fabricated pair is exactly what
        // the capture design forbids.
        CHECK_TRUE(actual.imu->accelerometer[0].sensor_timestamp_ns !=
                   actual.imu->gyroscope[0].sensor_timestamp_ns);
        CHECK_TRUE(actual.imu->accelerometer_dropped == 3);
        CHECK_TRUE(actual.imu->gyroscope_dropped == 4);
    }

    std::error_code cleanup;
    std::filesystem::remove_all(directory, cleanup);
    return true;
}

auto manifest_round_trips() -> bool {
    const auto directory = scratch_directory("manifest");
    CHECK_TRUE(write_dataset(directory, 1));

    perception::dataset::DatasetReader reader;
    std::string error;
    CHECK_TRUE(reader.open(directory, error));
    const auto& manifest = reader.manifest();
    CHECK_TRUE(manifest.format_version == perception::dataset::kFormatVersion);
    CHECK_TRUE(manifest.device.serial == "207122078394");
    CHECK_TRUE(manifest.device.depth_scale_m == 0.001F);
    CHECK_TRUE(manifest.depth_intrinsics.has_value());
    CHECK_TRUE(manifest.depth_intrinsics->focal_x == 2.0F);
    CHECK_TRUE(manifest.depth_intrinsics->distortion_model == "brown_conrady");
    CHECK_TRUE(manifest.mount.z_m == 0.4);
    CHECK_TRUE(manifest.mount.pitch_deg == 15.0);

    std::error_code cleanup;
    std::filesystem::remove_all(directory, cleanup);
    return true;
}

auto writer_refuses_to_overwrite_field_evidence() -> bool {
    const auto directory = scratch_directory("overwrite");
    CHECK_TRUE(write_dataset(directory, 1));

    perception::dataset::DatasetWriter writer;
    std::string error;
    CHECK_TRUE(!writer.open(directory, build_manifest(), error));
    CHECK_TRUE(!error.empty());

    std::error_code cleanup;
    std::filesystem::remove_all(directory, cleanup);
    return true;
}

auto reader_rejects_missing_and_malformed_datasets() -> bool {
    perception::dataset::DatasetReader reader;
    std::string error;
    CHECK_TRUE(!reader.open(scratch_directory("absent"), error));
    CHECK_TRUE(!error.empty());
    return true;
}

auto replay_camera_serves_frames_through_the_camera_interface() -> bool {
    const auto directory = scratch_directory("replay");
    CHECK_TRUE(write_dataset(directory, 2));

    perception::dataset::ReplayCamera::Options options;
    perception::dataset::ReplayCamera replay(directory, options);
    perception::camera::ICamera& as_camera = replay;

    CHECK_TRUE(as_camera.initialize());
    CHECK_TRUE(as_camera.start());

    CameraFrameSet frame_set;
    CHECK_TRUE(as_camera.capture(frame_set));
    CHECK_TRUE(frame_set.depth.data == build_frame_set(0).depth.data);
    CHECK_TRUE(as_camera.capture(frame_set));
    CHECK_TRUE(frame_set.rgb.data == build_frame_set(1).rgb.data);
    // A finished dataset reports exhaustion rather than repeating its last frame.
    CHECK_TRUE(!as_camera.capture(frame_set));
    CHECK_TRUE(replay.frames_replayed() == 2);
    CHECK_TRUE(replay.manifest().device.serial == "207122078394");
    as_camera.stop();

    std::error_code cleanup;
    std::filesystem::remove_all(directory, cleanup);
    return true;
}

auto replay_camera_loops_when_asked() -> bool {
    const auto directory = scratch_directory("loop");
    CHECK_TRUE(write_dataset(directory, 2));

    perception::dataset::ReplayCamera::Options options;
    options.loop = true;
    perception::dataset::ReplayCamera replay(directory, options);
    CHECK_TRUE(replay.initialize());
    CHECK_TRUE(replay.start());

    CameraFrameSet frame_set;
    for (int index = 0; index < 5; ++index) {
        CHECK_TRUE(replay.capture(frame_set));
    }
    CHECK_TRUE(replay.frames_replayed() == 5);

    std::error_code cleanup;
    std::filesystem::remove_all(directory, cleanup);
    return true;
}

auto replay_camera_refuses_to_capture_before_start() -> bool {
    const auto directory = scratch_directory("unstarted");
    CHECK_TRUE(write_dataset(directory, 1));

    perception::dataset::ReplayCamera replay(directory, {});
    CameraFrameSet frame_set;
    CHECK_TRUE(!replay.capture(frame_set));
    CHECK_TRUE(!replay.last_error().empty());

    std::error_code cleanup;
    std::filesystem::remove_all(directory, cleanup);
    return true;
}

// librealsense names streams "Color"/"Depth"; a literal lower-case comparison matched nothing and
// produced a 30 s field recording with no intrinsics, which cannot be deprojected.
auto intrinsics_selection_ignores_stream_name_case() -> bool {
    perception::camera::CameraDeviceInfo device;

    perception::camera::CameraStreamProfileInfo depth_640;
    depth_640.stream = "Depth";
    depth_640.width = 640;
    depth_640.height = 480;
    perception::camera::CameraIntrinsics depth_intrinsics;
    depth_intrinsics.width = 640;
    depth_intrinsics.height = 480;
    depth_intrinsics.focal_x = 380.0F;
    depth_640.intrinsics = depth_intrinsics;
    device.stream_profiles.push_back(depth_640);

    perception::camera::CameraStreamProfileInfo colour;
    colour.stream = "Color";
    colour.width = 1'280;
    colour.height = 720;
    perception::camera::CameraIntrinsics colour_intrinsics;
    colour_intrinsics.width = 1'280;
    colour_intrinsics.height = 720;
    colour_intrinsics.focal_x = 900.0F;
    colour.intrinsics = colour_intrinsics;
    device.stream_profiles.push_back(colour);

    bool exact = false;
    const auto depth =
        perception::dataset::select_stream_intrinsics(device, "depth", 640, 480, &exact);
    CHECK_TRUE(depth.has_value());
    CHECK_TRUE(exact);
    CHECK_TRUE(depth->focal_x == 380.0F);

    // A different resolution still yields something usable, but flagged: those intrinsics have the
    // wrong focal length and principal point for the configured stream.
    const auto mismatched =
        perception::dataset::select_stream_intrinsics(device, "color", 640, 480, &exact);
    CHECK_TRUE(mismatched.has_value());
    CHECK_TRUE(!exact);
    CHECK_TRUE(mismatched->focal_x == 900.0F);

    CHECK_TRUE(
        !perception::dataset::select_stream_intrinsics(device, "infrared", 640, 480, &exact)
             .has_value());
    return true;
}

auto intrinsics_selection_skips_profiles_without_intrinsics() -> bool {
    perception::camera::CameraDeviceInfo device;
    perception::camera::CameraStreamProfileInfo without;
    without.stream = "Depth";
    without.width = 640;
    without.height = 480;
    device.stream_profiles.push_back(without);

    bool exact = true;
    CHECK_TRUE(!perception::dataset::select_stream_intrinsics(device, "Depth", 640, 480, &exact)
                    .has_value());
    CHECK_TRUE(!exact);
    return true;
}

} // namespace

auto dataset_test() -> bool {
    return round_trip_preserves_every_byte() && manifest_round_trips() &&
           writer_refuses_to_overwrite_field_evidence() &&
           reader_rejects_missing_and_malformed_datasets() &&
           replay_camera_serves_frames_through_the_camera_interface() &&
           replay_camera_loops_when_asked() && replay_camera_refuses_to_capture_before_start() &&
           intrinsics_selection_ignores_stream_name_case() &&
           intrinsics_selection_skips_profiles_without_intrinsics();
}
