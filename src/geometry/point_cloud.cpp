#include "perception/geometry/point_cloud.hpp"

#include <cmath>

namespace perception::geometry {

auto has_distortion(const camera::CameraIntrinsics& intrinsics) noexcept -> bool {
    for (const float coefficient : intrinsics.coefficients) {
        if (coefficient != 0.0F) {
            return true;
        }
    }
    return false;
}

auto deproject_pixel(const camera::CameraIntrinsics& intrinsics, double pixel_x, double pixel_y,
                     double depth_m) -> std::optional<Vec3> {
    if (depth_m <= 0.0 || intrinsics.focal_x == 0.0F || intrinsics.focal_y == 0.0F) {
        return std::nullopt;
    }
    const double x = (pixel_x - static_cast<double>(intrinsics.principal_x)) /
                     static_cast<double>(intrinsics.focal_x) * depth_m;
    const double y = (pixel_y - static_cast<double>(intrinsics.principal_y)) /
                     static_cast<double>(intrinsics.focal_y) * depth_m;
    return Vec3{x, y, depth_m};
}

auto project_point(const camera::CameraIntrinsics& intrinsics, const Vec3& point)
    -> std::optional<std::pair<double, double>> {
    if (point.z <= 0.0) {
        return std::nullopt;
    }
    const double pixel_x = (point.x / point.z * static_cast<double>(intrinsics.focal_x)) +
                           static_cast<double>(intrinsics.principal_x);
    const double pixel_y = (point.y / point.z * static_cast<double>(intrinsics.focal_y)) +
                           static_cast<double>(intrinsics.principal_y);
    return std::make_pair(pixel_x, pixel_y);
}

auto deproject_depth(const camera::DepthFrame& depth, const camera::CameraIntrinsics& intrinsics,
                     const DeprojectOptions& options) -> PointCloud {
    PointCloud cloud;
    cloud.sensor_timestamp_ns = depth.sensor_timestamp_ns;
    cloud.capture_timestamp_ns = depth.capture_timestamp_ns;
    cloud.frame_number = depth.frame_number;

    const int row_stride = options.row_stride > 0 ? options.row_stride : 1;
    const int column_stride = options.column_stride > 0 ? options.column_stride : 1;
    if (depth.width <= 0 || depth.height <= 0 || depth.depth_scale_m <= 0.0F) {
        return cloud;
    }
    const auto expected =
        static_cast<std::size_t>(depth.width) * static_cast<std::size_t>(depth.height);
    if (depth.data.size() < expected) {
        return cloud;
    }

    const auto reserve_rows = static_cast<std::size_t>((depth.height + row_stride - 1) / row_stride);
    const auto reserve_columns =
        static_cast<std::size_t>((depth.width + column_stride - 1) / column_stride);
    cloud.points.reserve(reserve_rows * reserve_columns);

    const double scale = static_cast<double>(depth.depth_scale_m);
    for (int row = 0; row < depth.height; row += row_stride) {
        const auto row_offset = static_cast<std::size_t>(row) * static_cast<std::size_t>(depth.width);
        for (int column = 0; column < depth.width; column += column_stride) {
            const std::uint16_t raw = depth.data[row_offset + static_cast<std::size_t>(column)];
            if (raw == 0U) {
                // Zero is librealsense's "no measurement", not a point at the sensor origin.
                ++cloud.rejected_invalid;
                continue;
            }
            const double depth_m = static_cast<double>(raw) * scale;
            if (depth_m < options.min_range_m || depth_m > options.max_range_m) {
                ++cloud.rejected_out_of_range;
                continue;
            }
            const auto point = deproject_pixel(intrinsics, static_cast<double>(column),
                                               static_cast<double>(row), depth_m);
            if (!point.has_value()) {
                ++cloud.rejected_invalid;
                continue;
            }
            cloud.points.push_back(Point3f{static_cast<float>(point->x),
                                           static_cast<float>(point->y),
                                           static_cast<float>(point->z)});
        }
    }
    return cloud;
}

void transform_in_place(PointCloud& cloud, const Transform3& transform, std::string_view frame_id) {
    for (Point3f& point : cloud.points) {
        const Vec3 moved = transform.apply(
            Vec3{static_cast<double>(point.x), static_cast<double>(point.y),
                 static_cast<double>(point.z)});
        point = Point3f{static_cast<float>(moved.x), static_cast<float>(moved.y),
                        static_cast<float>(moved.z)};
    }
    cloud.frame_id = std::string(frame_id);
}

} // namespace perception::geometry
