#pragma once

#include "perception/transport/json.hpp"

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <string_view>

// Typed, versioned contract between perception-service and the two mission-side peers.
//
// Framing matches payload-service so robot-agent can reuse the client it already has: one
// request per TCP connection, the caller half-closes after writing, the reply is one JSON
// object followed by '\n'. Every reply carries `protocol_version`; every measurement carries
// its own source timestamp and age, because the EKF cannot use a value whose age it cannot see.
namespace perception::transport {

inline constexpr std::int64_t kProtocolVersion = 1;

// ---------------------------------------------------------------------------------------------
// Inbound measurements (perception polls these peers; it is the client)
// ---------------------------------------------------------------------------------------------

enum class GnssFixMode : std::int32_t {
    Unknown = 0,
    None = 1,
    Autonomous = 2,
    Differential = 3,
    RtkFixed = 4,
    RtkFloat = 5,
    Estimated = 6,
};

[[nodiscard]] auto gnss_fix_mode_name(GnssFixMode mode) -> std::string_view;

// Mirrors one `gnss.get_status` sample from payload-service. Flags are kept separate from values
// exactly as the peer sends them: a zero latitude is only meaningful with `has_position`.
struct GnssSample {
    std::uint64_t timestamp_unix_ms{};
    std::uint64_t age_ms{};
    bool has_position{};
    double latitude_deg{};
    double longitude_deg{};
    bool has_course{};
    double course_deg{};
    bool has_heading{};
    double heading_deg{};
    GnssFixMode fix_mode{GnssFixMode::Unknown};
    std::string fix_mode_name;
    bool has_satellites_used{};
    std::int32_t satellites_used{};
    bool has_hdop{};
    double hdop{};
    bool has_speed{};
    double speed_over_ground_mps{};
    // Per-field freshness from the native RTK bridge (`rtk_gnss_get_latest_v2`). Absent on older
    // payload-service builds, in which case only the sample-level timestamp/age are usable.
    bool has_freshness{};
    bool position_valid{};
    bool fix_mode_valid{};
    bool heading_valid{};
    std::uint64_t position_age_ms{};
    std::uint64_t fix_mode_age_ms{};
    std::uint64_t heading_age_ms{};
};

// Decodes the payload-service `{"status":"ok","fix":{...}}` envelope. Returns nullopt with an
// error when the call failed or carried no fix; an absent fix is normal, not a transport fault.
[[nodiscard]] auto decode_gnss_status(const json::Value& response, std::string* error)
    -> std::optional<GnssSample>;

// The vendor quadruped's own attitude and body-frame motion, read from robot-agent's
// `/api/v1/status` snapshot. Angles and rates are RADIANS on the wire; a bare pass-through puts a
// ~57x error into heading. Field names match a live capture from robot 192.168.1.206 on
// 2026-09-19: Roll, Pitch, Yaw, OmegaZ, LinearX, LinearY, Height.
//
// Freshness comes from `received_at_utc` only. The sibling `source_time` is robot-local wall time
// (observed 8 h ahead of UTC) and must never be used as an age.
struct VendorMotionSample {
    std::string received_at_utc;
    bool has_attitude{};
    double roll_rad{};
    double pitch_rad{};
    double yaw_rad{};
    bool has_yaw{};
    // Body-frame velocity and yaw rate: the vendor odometry input the EKF fuses.
    bool has_velocity{};
    double linear_x_mps{};
    double linear_y_mps{};
    bool has_yaw_rate{};
    double yaw_rate_rps{};
    bool has_height{};
    double height_m{};
};

[[nodiscard]] auto decode_vendor_motion(const json::Value& status_snapshot, std::string* error)
    -> std::optional<VendorMotionSample>;

// ---------------------------------------------------------------------------------------------
// Outbound state (perception serves these; it is the server)
// ---------------------------------------------------------------------------------------------

enum class LocalizationQuality : std::int32_t {
    // No estimate at all: nothing downstream may navigate on this.
    Lost = 0,
    // Global fix drives the estimate.
    GlobalFix = 1,
    // Global fix degraded; the estimate is carried by dead reckoning within its budget.
    DeadReckoning = 2,
};

[[nodiscard]] auto localization_quality_name(LocalizationQuality quality) -> std::string_view;

struct Pose2d {
    double x_m{};
    double y_m{};
    double yaw_rad{};
};

struct LocalizationState {
    // False until the EKF exists and has converged; the whole V0.5 branch is reported as
    // unavailable rather than as a fabricated zero pose.
    bool available{};
    std::uint64_t timestamp_unix_ms{};
    Pose2d map_pose;
    Pose2d odom_pose;
    double velocity_x_mps{};
    double velocity_y_mps{};
    double yaw_rate_rps{};
    double position_covariance_m2{};
    double yaw_covariance_rad2{};
    LocalizationQuality quality{LocalizationQuality::Lost};
    std::uint64_t dead_reckoning_elapsed_ms{};
    std::uint64_t dead_reckoning_budget_ms{};
};

// Sensor/service health, sourced from the V0.1 camera health snapshot.
struct ServiceHealthState {
    std::string status;
    std::uint64_t uptime_s{};
    std::uint64_t frames_received{};
    std::uint64_t frames_dropped{};
    std::uint64_t frame_age_ms{};
    double accelerometer_hz{};
    double gyroscope_hz{};
    std::uint64_t accelerometer_age_ms{};
    std::uint64_t gyroscope_age_ms{};
    bool imu_ready{};
    bool ekf_ready{};
    bool rtsp_enabled{};
};

// Everything the router needs from the running service. Implemented by the application and by
// tests; it keeps the router free of camera, streaming and socket types.
class StateProvider {
  public:
    virtual ~StateProvider() = default;
    [[nodiscard]] virtual auto health() const -> ServiceHealthState = 0;
    [[nodiscard]] virtual auto localization() const -> LocalizationState = 0;
    // Sets the map origin from the mission's first waypoint. Returns false with a reason when the
    // service cannot accept it yet.
    virtual auto set_map_origin(double latitude_deg, double longitude_deg, std::string& error)
        -> bool = 0;
};

// Pure request -> response mapping. It never throws and never touches a socket, so the whole
// contract is testable on a host with no camera, no Jetson and no peer.
class RequestRouter {
  public:
    explicit RequestRouter(StateProvider& provider) noexcept : provider_(provider) {}

    // `request_json` is the raw client payload; the returned string is the reply body without
    // its trailing newline.
    [[nodiscard]] auto handle(std::string_view request_json) -> std::string;

  private:
    [[nodiscard]] auto dispatch(std::string_view action, const json::Value& payload) -> json::Value;

    std::reference_wrapper<StateProvider> provider_;
};

[[nodiscard]] auto make_error_response(std::string_view action, std::string_view message)
    -> json::Value;

} // namespace perception::transport
