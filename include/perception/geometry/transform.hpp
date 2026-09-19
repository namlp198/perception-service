#pragma once

#include <array>
#include <string>
#include <string_view>

// Rigid-body transforms between the frames this service reasons in.
//
// The math is project-owned rather than Eigen-based: it is a 3x3 rotation and a translation, the
// existing frame types already carry plain arrays, and keeping it dependency-free means the host
// test build needs nothing extra. Eigen enters when the EKF needs its solvers, where it earns the
// dependency.
namespace perception::geometry {

// Frame names used across the service. They match the fundamental document's model: `base_link` is
// the robot body frame, and the camera contributes two frames because the optical convention is
// not the robot convention.
namespace frames {
inline constexpr std::string_view kCameraOptical = "camera_optical";
inline constexpr std::string_view kCameraBody = "camera_body";
inline constexpr std::string_view kBaseLink = "base_link";
} // namespace frames

struct Vec3 {
    double x{};
    double y{};
    double z{};
};

[[nodiscard]] auto operator==(const Vec3& lhs, const Vec3& rhs) noexcept -> bool;

class Transform3 {
  public:
    // Identity: same origin, same axes.
    Transform3() = default;

    // Rotation is applied as R * v + t. `rotation` is row-major.
    Transform3(std::array<double, 9> rotation, Vec3 translation) noexcept;

    // Intrinsic Z-Y-X composition (yaw, then pitch, then roll), the convention used by the mount
    // configuration and by the vendor's Roll/Pitch/Yaw report.
    [[nodiscard]] static auto from_euler_translation(double roll_rad, double pitch_rad,
                                                     double yaw_rad, Vec3 translation)
        -> Transform3;

    // Rotates and translates a point.
    [[nodiscard]] auto apply(const Vec3& point) const noexcept -> Vec3;
    // Rotates only. Velocities and directions must use this: translating them is meaningless.
    [[nodiscard]] auto rotate(const Vec3& direction) const noexcept -> Vec3;

    [[nodiscard]] auto inverse() const noexcept -> Transform3;
    // Returns this * other, i.e. apply `other` first and then this one.
    [[nodiscard]] auto compose(const Transform3& other) const noexcept -> Transform3;

    [[nodiscard]] auto rotation() const noexcept -> const std::array<double, 9>& {
        return rotation_;
    }
    [[nodiscard]] auto translation() const noexcept -> const Vec3& { return translation_; }

  private:
    // Row-major identity.
    std::array<double, 9> rotation_{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    Vec3 translation_{};
};

// Optical convention (x right, y down, z forward) to robot body convention (x forward, y left,
// z up). This fixed axis swap is why a camera contributes two frames; forgetting it silently
// rotates every point cloud by 90 degrees.
[[nodiscard]] auto camera_optical_to_camera_body() -> Transform3;

// Where the camera body sits on the robot, from configuration.
struct MountExtrinsics {
    double x_m{};
    double y_m{};
    double z_m{};
    double roll_deg{};
    double pitch_deg{};
    double yaw_deg{};
};

[[nodiscard]] auto mount_to_transform(const MountExtrinsics& mount) -> Transform3;

// Full camera_optical -> base_link chain for the configured mount.
[[nodiscard]] auto camera_optical_to_base_link(const MountExtrinsics& mount) -> Transform3;

[[nodiscard]] auto degrees_to_radians(double degrees) noexcept -> double;

} // namespace perception::geometry
