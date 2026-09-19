#include "perception/geometry/transform.hpp"

#include <cmath>

namespace perception::geometry {
namespace {

constexpr double kPi = 3.14159265358979323846;

auto multiply(const std::array<double, 9>& lhs, const std::array<double, 9>& rhs)
    -> std::array<double, 9> {
    std::array<double, 9> result{};
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t column = 0; column < 3; ++column) {
            double sum = 0.0;
            for (std::size_t inner = 0; inner < 3; ++inner) {
                sum += lhs[(row * 3) + inner] * rhs[(inner * 3) + column];
            }
            result[(row * 3) + column] = sum;
        }
    }
    return result;
}

} // namespace

auto operator==(const Vec3& lhs, const Vec3& rhs) noexcept -> bool {
    return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
}

Transform3::Transform3(std::array<double, 9> rotation, Vec3 translation) noexcept
    : rotation_(rotation), translation_(translation) {}

auto Transform3::from_euler_translation(double roll_rad, double pitch_rad, double yaw_rad,
                                        Vec3 translation) -> Transform3 {
    const double sin_roll = std::sin(roll_rad);
    const double cos_roll = std::cos(roll_rad);
    const double sin_pitch = std::sin(pitch_rad);
    const double cos_pitch = std::cos(pitch_rad);
    const double sin_yaw = std::sin(yaw_rad);
    const double cos_yaw = std::cos(yaw_rad);

    // R = Rz(yaw) * Ry(pitch) * Rx(roll), expanded.
    const std::array<double, 9> rotation{
        cos_yaw * cos_pitch,
        (cos_yaw * sin_pitch * sin_roll) - (sin_yaw * cos_roll),
        (cos_yaw * sin_pitch * cos_roll) + (sin_yaw * sin_roll),
        sin_yaw * cos_pitch,
        (sin_yaw * sin_pitch * sin_roll) + (cos_yaw * cos_roll),
        (sin_yaw * sin_pitch * cos_roll) - (cos_yaw * sin_roll),
        -sin_pitch,
        cos_pitch * sin_roll,
        cos_pitch * cos_roll,
    };
    return Transform3(rotation, translation);
}

auto Transform3::apply(const Vec3& point) const noexcept -> Vec3 {
    const Vec3 rotated = rotate(point);
    return Vec3{rotated.x + translation_.x, rotated.y + translation_.y, rotated.z + translation_.z};
}

auto Transform3::rotate(const Vec3& direction) const noexcept -> Vec3 {
    return Vec3{
        (rotation_[0] * direction.x) + (rotation_[1] * direction.y) + (rotation_[2] * direction.z),
        (rotation_[3] * direction.x) + (rotation_[4] * direction.y) + (rotation_[5] * direction.z),
        (rotation_[6] * direction.x) + (rotation_[7] * direction.y) + (rotation_[8] * direction.z)};
}

auto Transform3::inverse() const noexcept -> Transform3 {
    // A rotation matrix's inverse is its transpose, so no general matrix inversion is needed.
    const std::array<double, 9> transposed{rotation_[0], rotation_[3], rotation_[6],
                                           rotation_[1], rotation_[4], rotation_[7],
                                           rotation_[2], rotation_[5], rotation_[8]};
    const Transform3 rotation_only(transposed, Vec3{});
    const Vec3 moved = rotation_only.rotate(translation_);
    return Transform3(transposed, Vec3{-moved.x, -moved.y, -moved.z});
}

auto Transform3::compose(const Transform3& other) const noexcept -> Transform3 {
    return Transform3(multiply(rotation_, other.rotation_), apply(other.translation_));
}

auto camera_optical_to_camera_body() -> Transform3 {
    // Optical x (right) -> body -y, optical y (down) -> body -z, optical z (forward) -> body x.
    const std::array<double, 9> rotation{0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0};
    return Transform3(rotation, Vec3{});
}

auto mount_to_transform(const MountExtrinsics& mount) -> Transform3 {
    return Transform3::from_euler_translation(
        degrees_to_radians(mount.roll_deg), degrees_to_radians(mount.pitch_deg),
        degrees_to_radians(mount.yaw_deg), Vec3{mount.x_m, mount.y_m, mount.z_m});
}

auto camera_optical_to_base_link(const MountExtrinsics& mount) -> Transform3 {
    return mount_to_transform(mount).compose(camera_optical_to_camera_body());
}

auto degrees_to_radians(double degrees) noexcept -> double { return degrees * kPi / 180.0; }

} // namespace perception::geometry
