#pragma once

#include "perception/camera/camera_info.hpp"
#include "perception/camera/camera_types.hpp"
#include "perception/geometry/transform.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace perception::geometry {

struct Point3f {
    float x{};
    float y{};
    float z{};
};

struct PointCloud {
    std::uint64_t sensor_timestamp_ns{};
    std::uint64_t capture_timestamp_ns{};
    std::uint64_t frame_number{};
    // The frame the coordinates are expressed in; it travels with the data so a consumer can never
    // silently mix optical-frame and robot-frame points.
    std::string frame_id{std::string(frames::kCameraOptical)};
    std::vector<Point3f> points;
    // Pixels skipped because their depth was invalid or outside the configured range. Reported so
    // an empty cloud can be told apart from a cloud that was never populated.
    std::uint64_t rejected_invalid{};
    std::uint64_t rejected_out_of_range{};
};

struct DeprojectOptions {
    // Invalid depth is exactly zero in Z16 and is always dropped; these bound the useful range.
    double min_range_m{0.2};
    double max_range_m{5.0};
    // Subsampling. 1 keeps every pixel; larger values keep every Nth row/column.
    int row_stride{1};
    int column_stride{1};
};

// Pinhole deprojection of one metric depth frame into the camera optical frame.
//
// Distortion coefficients are not applied: the D435i reports depth with an unmodified Brown-Conrady
// model whose coefficients are zero, and applying a model the sensor did not use would introduce
// error rather than remove it. Callers that must know can ask `has_distortion`.
[[nodiscard]] auto deproject_depth(const camera::DepthFrame& depth,
                                   const camera::CameraIntrinsics& intrinsics,
                                   const DeprojectOptions& options) -> PointCloud;

// Moves a cloud into another frame and records the new frame id.
void transform_in_place(PointCloud& cloud, const Transform3& transform, std::string_view frame_id);

[[nodiscard]] auto has_distortion(const camera::CameraIntrinsics& intrinsics) noexcept -> bool;

// Single-pixel deprojection, exposed for callers that need one point rather than a cloud.
// Returns nullopt for invalid (zero) depth.
[[nodiscard]] auto deproject_pixel(const camera::CameraIntrinsics& intrinsics, double pixel_x,
                                   double pixel_y, double depth_m) -> std::optional<Vec3>;

// Inverse of `deproject_pixel`. Returns nullopt for points at or behind the image plane.
[[nodiscard]] auto project_point(const camera::CameraIntrinsics& intrinsics, const Vec3& point)
    -> std::optional<std::pair<double, double>>;

} // namespace perception::geometry
