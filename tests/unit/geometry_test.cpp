#include "perception/geometry/point_cloud.hpp"
#include "perception/geometry/transform.hpp"

#include "test_support.hpp"

#include <cmath>

namespace {

using perception::geometry::Transform3;
using perception::geometry::Vec3;

auto close(double actual, double expected, double tolerance = 1e-9) -> bool {
    return std::fabs(actual - expected) <= tolerance;
}

auto close_point(const Vec3& point, double x, double y, double z) -> bool {
    return close(point.x, x, 1e-9) && close(point.y, y, 1e-9) && close(point.z, z, 1e-9);
}

auto identity_leaves_points_untouched() -> bool {
    const Transform3 identity;
    const Vec3 moved = identity.apply(Vec3{1.0, -2.0, 3.5});
    CHECK_TRUE(close_point(moved, 1.0, -2.0, 3.5));
    return true;
}

auto yaw_rotation_turns_forward_into_left() -> bool {
    const Transform3 yaw = Transform3::from_euler_translation(
        0.0, 0.0, perception::geometry::degrees_to_radians(90.0), Vec3{});
    // +x (forward) becomes +y (left) under a +90 degree yaw.
    const Vec3 rotated = yaw.apply(Vec3{1.0, 0.0, 0.0});
    CHECK_TRUE(close_point(rotated, 0.0, 1.0, 0.0));
    return true;
}

auto translation_applies_after_rotation() -> bool {
    const Transform3 transform = Transform3::from_euler_translation(
        0.0, 0.0, perception::geometry::degrees_to_radians(90.0), Vec3{1.0, 2.0, 3.0});
    const Vec3 moved = transform.apply(Vec3{1.0, 0.0, 0.0});
    CHECK_TRUE(close_point(moved, 1.0, 3.0, 3.0));

    // Directions must not pick up the translation.
    const Vec3 direction = transform.rotate(Vec3{1.0, 0.0, 0.0});
    CHECK_TRUE(close_point(direction, 0.0, 1.0, 0.0));
    return true;
}

auto inverse_round_trips() -> bool {
    const Transform3 transform = Transform3::from_euler_translation(
        perception::geometry::degrees_to_radians(10.0),
        perception::geometry::degrees_to_radians(-20.0),
        perception::geometry::degrees_to_radians(35.0), Vec3{0.3, -0.15, 0.42});
    const Vec3 original{1.25, -0.75, 2.5};
    const Vec3 back = transform.inverse().apply(transform.apply(original));
    CHECK_TRUE(close(back.x, original.x, 1e-9));
    CHECK_TRUE(close(back.y, original.y, 1e-9));
    CHECK_TRUE(close(back.z, original.z, 1e-9));
    return true;
}

auto composition_applies_right_operand_first() -> bool {
    const Transform3 translate = Transform3::from_euler_translation(0.0, 0.0, 0.0, Vec3{1.0, 0.0, 0.0});
    const Transform3 yaw = Transform3::from_euler_translation(
        0.0, 0.0, perception::geometry::degrees_to_radians(90.0), Vec3{});

    // Rotate first, then translate.
    const Vec3 rotate_then_move = translate.compose(yaw).apply(Vec3{1.0, 0.0, 0.0});
    CHECK_TRUE(close_point(rotate_then_move, 1.0, 1.0, 0.0));

    // Translate first, then rotate: a different result, which is the point of ordering.
    const Vec3 move_then_rotate = yaw.compose(translate).apply(Vec3{1.0, 0.0, 0.0});
    CHECK_TRUE(close_point(move_then_rotate, 0.0, 2.0, 0.0));
    return true;
}

auto optical_axes_map_onto_robot_axes() -> bool {
    const Transform3 optical_to_body = perception::geometry::camera_optical_to_camera_body();
    // Optical +z is forward -> body +x.
    CHECK_TRUE(close_point(optical_to_body.apply(Vec3{0.0, 0.0, 1.0}), 1.0, 0.0, 0.0));
    // Optical +x is right -> body -y.
    CHECK_TRUE(close_point(optical_to_body.apply(Vec3{1.0, 0.0, 0.0}), 0.0, -1.0, 0.0));
    // Optical +y is down -> body -z.
    CHECK_TRUE(close_point(optical_to_body.apply(Vec3{0.0, 1.0, 0.0}), 0.0, 0.0, -1.0));
    return true;
}

auto mounted_camera_places_points_in_base_link() -> bool {
    perception::geometry::MountExtrinsics mount;
    mount.x_m = 0.25;
    mount.z_m = 0.40;
    const Transform3 to_base = perception::geometry::camera_optical_to_base_link(mount);

    // A point 2 m straight ahead of the lens sits 2 m ahead of the mount, at mount height.
    const Vec3 ahead = to_base.apply(Vec3{0.0, 0.0, 2.0});
    CHECK_TRUE(close_point(ahead, 2.25, 0.0, 0.40));

    // A pitched-down camera lowers what it sees ahead of the robot.
    mount.pitch_deg = 30.0;
    const Vec3 pitched = perception::geometry::camera_optical_to_base_link(mount).apply(
        Vec3{0.0, 0.0, 2.0});
    CHECK_TRUE(pitched.z < 0.40);
    CHECK_TRUE(pitched.x > 0.0);
    return true;
}

auto build_depth_frame() -> perception::camera::DepthFrame {
    perception::camera::DepthFrame depth;
    depth.width = 4;
    depth.height = 2;
    depth.depth_scale_m = 0.001F;
    depth.sensor_timestamp_ns = 11;
    depth.capture_timestamp_ns = 22;
    depth.frame_number = 33;
    //            invalid, 1.0 m,  0.1 m (too near), 9.0 m (too far)
    depth.data = {0U, 1'000U, 100U, 9'000U,
                  //  2.0 m,  0.5 m, invalid, 3.0 m
                  2'000U, 500U, 0U, 3'000U};
    return depth;
}

auto build_intrinsics() -> perception::camera::CameraIntrinsics {
    perception::camera::CameraIntrinsics intrinsics;
    intrinsics.width = 4;
    intrinsics.height = 2;
    intrinsics.focal_x = 2.0F;
    intrinsics.focal_y = 2.0F;
    intrinsics.principal_x = 2.0F;
    intrinsics.principal_y = 1.0F;
    intrinsics.distortion_model = "brown_conrady";
    return intrinsics;
}

auto deprojection_drops_invalid_and_out_of_range_depth() -> bool {
    const auto depth = build_depth_frame();
    const auto intrinsics = build_intrinsics();
    perception::geometry::DeprojectOptions options;
    options.min_range_m = 0.2;
    options.max_range_m = 5.0;

    const auto cloud = perception::geometry::deproject_depth(depth, intrinsics, options);
    // Kept: 1.0, 2.0, 0.5, 3.0 m. Dropped: two zeroes as invalid, 0.1 and 9.0 m as out of range.
    CHECK_TRUE(cloud.points.size() == 4);
    CHECK_TRUE(cloud.rejected_invalid == 2);
    CHECK_TRUE(cloud.rejected_out_of_range == 2);
    CHECK_TRUE(cloud.frame_id == "camera_optical");
    CHECK_TRUE(cloud.sensor_timestamp_ns == 11);
    CHECK_TRUE(cloud.capture_timestamp_ns == 22);
    CHECK_TRUE(cloud.frame_number == 33);
    return true;
}

auto deprojection_matches_the_pinhole_model() -> bool {
    const auto intrinsics = build_intrinsics();
    // Pixel (3, 0) at 1 m: x = (3-2)/2 * 1, y = (0-1)/2 * 1.
    const auto point = perception::geometry::deproject_pixel(intrinsics, 3.0, 0.0, 1.0);
    CHECK_TRUE(point.has_value());
    CHECK_TRUE(close(point->x, 0.5, 1e-9));
    CHECK_TRUE(close(point->y, -0.5, 1e-9));
    CHECK_TRUE(close(point->z, 1.0, 1e-9));

    // Projecting it back must land on the original pixel.
    const auto pixel = perception::geometry::project_point(intrinsics, *point);
    CHECK_TRUE(pixel.has_value());
    CHECK_TRUE(close(pixel->first, 3.0, 1e-9));
    CHECK_TRUE(close(pixel->second, 0.0, 1e-9));

    // Zero depth is librealsense's "no measurement", never a point at the origin.
    CHECK_TRUE(!perception::geometry::deproject_pixel(intrinsics, 1.0, 1.0, 0.0).has_value());
    // Points at or behind the image plane cannot be projected.
    CHECK_TRUE(!perception::geometry::project_point(intrinsics, Vec3{1.0, 1.0, 0.0}).has_value());
    return true;
}

auto deprojection_honours_subsampling() -> bool {
    const auto depth = build_depth_frame();
    const auto intrinsics = build_intrinsics();
    perception::geometry::DeprojectOptions options;
    options.column_stride = 2;
    options.row_stride = 2;
    const auto cloud = perception::geometry::deproject_depth(depth, intrinsics, options);
    // Only row 0, columns 0 and 2 are visited: an invalid zero and a 0.1 m reading.
    CHECK_TRUE(cloud.points.empty());
    CHECK_TRUE(cloud.rejected_invalid == 1);
    CHECK_TRUE(cloud.rejected_out_of_range == 1);
    return true;
}

auto cloud_transform_records_its_new_frame() -> bool {
    const auto depth = build_depth_frame();
    const auto intrinsics = build_intrinsics();
    auto cloud = perception::geometry::deproject_depth(depth, intrinsics, {});

    perception::geometry::MountExtrinsics mount;
    mount.z_m = 0.5;
    perception::geometry::transform_in_place(
        cloud, perception::geometry::camera_optical_to_base_link(mount),
        perception::geometry::frames::kBaseLink);
    CHECK_TRUE(cloud.frame_id == "base_link");
    CHECK_TRUE(!cloud.points.empty());
    // Every point rose by the mount height, so none can sit at the old zero plane by accident.
    for (const auto& point : cloud.points) {
        CHECK_TRUE(point.z > 0.0F);
    }
    return true;
}

auto distortion_is_reported_rather_than_silently_applied() -> bool {
    auto intrinsics = build_intrinsics();
    CHECK_TRUE(!perception::geometry::has_distortion(intrinsics));
    intrinsics.coefficients[2] = 0.001F;
    CHECK_TRUE(perception::geometry::has_distortion(intrinsics));
    return true;
}

auto malformed_depth_frames_yield_an_empty_cloud() -> bool {
    perception::camera::DepthFrame truncated;
    truncated.width = 4;
    truncated.height = 2;
    truncated.depth_scale_m = 0.001F;
    truncated.data = {1'000U, 1'000U};
    const auto cloud = perception::geometry::deproject_depth(truncated, build_intrinsics(), {});
    CHECK_TRUE(cloud.points.empty());

    perception::camera::DepthFrame unscaled = build_depth_frame();
    unscaled.depth_scale_m = 0.0F;
    CHECK_TRUE(perception::geometry::deproject_depth(unscaled, build_intrinsics(), {}).points.empty());
    return true;
}

} // namespace

auto geometry_test() -> bool {
    return identity_leaves_points_untouched() && yaw_rotation_turns_forward_into_left() &&
           translation_applies_after_rotation() && inverse_round_trips() &&
           composition_applies_right_operand_first() && optical_axes_map_onto_robot_axes() &&
           mounted_camera_places_points_in_base_link() &&
           deprojection_drops_invalid_and_out_of_range_depth() &&
           deprojection_matches_the_pinhole_model() && deprojection_honours_subsampling() &&
           cloud_transform_records_its_new_frame() &&
           distortion_is_reported_rather_than_silently_applied() &&
           malformed_depth_frames_yield_an_empty_cloud();
}
