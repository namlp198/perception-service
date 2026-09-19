#include "perception/transport/protocol.hpp"

#include <utility>

namespace perception::transport {
namespace {

auto read_bool(const json::Value& node, std::string_view key, bool fallback) -> bool {
    const json::Value* member = node.find(key);
    if (member == nullptr) {
        return fallback;
    }
    return member->as_bool().value_or(fallback);
}

auto read_double(const json::Value& node, std::string_view key, double fallback) -> double {
    const json::Value* member = node.find(key);
    if (member == nullptr) {
        return fallback;
    }
    return member->as_number().value_or(fallback);
}

auto read_unsigned(const json::Value& node, std::string_view key) -> std::uint64_t {
    const json::Value* member = node.find(key);
    if (member == nullptr) {
        return 0;
    }
    const auto number = member->as_number();
    if (!number || *number < 0.0) {
        return 0;
    }
    return static_cast<std::uint64_t>(*number);
}

auto read_optional_number(const json::Value& node, std::string_view key) -> std::optional<double> {
    const json::Value* member = node.find(key);
    if (member == nullptr) {
        return std::nullopt;
    }
    return member->as_number();
}

auto read_string(const json::Value& node, std::string_view key) -> std::string {
    const json::Value* member = node.find(key);
    if (member == nullptr) {
        return {};
    }
    const std::string* text = member->as_string();
    return text == nullptr ? std::string{} : *text;
}

void fail(std::string* error, std::string message) {
    if (error != nullptr) {
        *error = std::move(message);
    }
}

// The vendor status service accepts both the nested `Items.MotionStatus` shape and a flat `Items`
// carrying the motion fields directly; robot-agent and RoboStation both accept both, so this
// boundary must too.
auto find_motion_items(const json::Value& stream_payload) -> const json::Value* {
    const json::Value* device = stream_payload.find("PatrolDevice");
    const json::Value* source = device != nullptr ? device : &stream_payload;
    const json::Value* items = source->find("Items");
    if (items == nullptr) {
        return nullptr;
    }
    const json::Value* nested = items->find("MotionStatus");
    return nested != nullptr ? nested : items;
}

auto pose_to_json(const Pose2d& pose) -> json::Value {
    json::Value node;
    node.set("x_m", json::Value::real(pose.x_m));
    node.set("y_m", json::Value::real(pose.y_m));
    node.set("yaw_rad", json::Value::real(pose.yaw_rad));
    return node;
}

auto make_ok_response(std::string_view action) -> json::Value {
    json::Value response;
    response.set("status", json::Value::string("ok"));
    response.set("protocol_version", json::Value::integer(kProtocolVersion));
    response.set("action", json::Value::string(std::string(action)));
    return response;
}

auto health_to_json(const ServiceHealthState& health) -> json::Value {
    json::Value node;
    node.set("status", json::Value::string(health.status));
    node.set("uptime_s", json::Value::integer(static_cast<std::int64_t>(health.uptime_s)));
    node.set("frames_received",
             json::Value::integer(static_cast<std::int64_t>(health.frames_received)));
    node.set("frames_dropped",
             json::Value::integer(static_cast<std::int64_t>(health.frames_dropped)));
    node.set("frame_age_ms", json::Value::integer(static_cast<std::int64_t>(health.frame_age_ms)));
    node.set("accelerometer_hz", json::Value::real(health.accelerometer_hz));
    node.set("gyroscope_hz", json::Value::real(health.gyroscope_hz));
    node.set("accelerometer_age_ms",
             json::Value::integer(static_cast<std::int64_t>(health.accelerometer_age_ms)));
    node.set("gyroscope_age_ms",
             json::Value::integer(static_cast<std::int64_t>(health.gyroscope_age_ms)));
    node.set("imu_ready", json::Value::boolean(health.imu_ready));
    node.set("ekf_ready", json::Value::boolean(health.ekf_ready));
    node.set("rtsp_enabled", json::Value::boolean(health.rtsp_enabled));
    return node;
}

auto localization_to_json(const LocalizationState& state) -> json::Value {
    json::Value node;
    node.set("available", json::Value::boolean(state.available));
    node.set("timestamp_unix_ms",
             json::Value::integer(static_cast<std::int64_t>(state.timestamp_unix_ms)));
    node.set("quality", json::Value::string(std::string(localization_quality_name(state.quality))));
    node.set("map_pose", pose_to_json(state.map_pose));
    node.set("odom_pose", pose_to_json(state.odom_pose));
    node.set("velocity_x_mps", json::Value::real(state.velocity_x_mps));
    node.set("velocity_y_mps", json::Value::real(state.velocity_y_mps));
    node.set("yaw_rate_rps", json::Value::real(state.yaw_rate_rps));
    node.set("position_covariance_m2", json::Value::real(state.position_covariance_m2));
    node.set("yaw_covariance_rad2", json::Value::real(state.yaw_covariance_rad2));
    node.set("dead_reckoning_elapsed_ms",
             json::Value::integer(static_cast<std::int64_t>(state.dead_reckoning_elapsed_ms)));
    node.set("dead_reckoning_budget_ms",
             json::Value::integer(static_cast<std::int64_t>(state.dead_reckoning_budget_ms)));
    return node;
}

} // namespace

auto gnss_fix_mode_name(GnssFixMode mode) -> std::string_view {
    switch (mode) {
    case GnssFixMode::None:
        return "none";
    case GnssFixMode::Autonomous:
        return "autonomous";
    case GnssFixMode::Differential:
        return "differential";
    case GnssFixMode::RtkFixed:
        return "rtk_fixed";
    case GnssFixMode::RtkFloat:
        return "rtk_float";
    case GnssFixMode::Estimated:
        return "estimated";
    case GnssFixMode::Unknown:
        break;
    }
    return "unknown";
}

auto localization_quality_name(LocalizationQuality quality) -> std::string_view {
    switch (quality) {
    case LocalizationQuality::GlobalFix:
        return "global_fix";
    case LocalizationQuality::DeadReckoning:
        return "dead_reckoning";
    case LocalizationQuality::Lost:
        break;
    }
    return "lost";
}

auto decode_gnss_status(const json::Value& response, std::string* error)
    -> std::optional<GnssSample> {
    if (error != nullptr) {
        error->clear();
    }
    const json::Value* status = response.find("status");
    const std::string* status_text = status == nullptr ? nullptr : status->as_string();
    if (status_text == nullptr || *status_text != "ok") {
        const json::Value* reason = response.find("error");
        const std::string* reason_text = reason == nullptr ? nullptr : reason->as_string();
        fail(error, reason_text != nullptr ? *reason_text : "payload-service rejected the request");
        return std::nullopt;
    }

    const json::Value* fix = response.find("fix");
    if (fix == nullptr || fix->is_null()) {
        fail(error, "payload-service has no GNSS fix yet");
        return std::nullopt;
    }

    GnssSample sample;
    sample.timestamp_unix_ms = read_unsigned(*fix, "timestamp_unix_ms");
    sample.age_ms = read_unsigned(*fix, "age_ms");
    sample.has_position = read_bool(*fix, "has_position", false);
    sample.latitude_deg = read_double(*fix, "latitude_deg", 0.0);
    sample.longitude_deg = read_double(*fix, "longitude_deg", 0.0);
    sample.has_course = read_bool(*fix, "has_course", false);
    sample.course_deg = read_double(*fix, "course_deg", 0.0);
    sample.has_heading = read_bool(*fix, "has_heading", false);
    sample.heading_deg = read_double(*fix, "heading_deg", 0.0);

    const auto fix_mode = static_cast<std::int32_t>(read_double(*fix, "fix_mode", 0.0));
    sample.fix_mode = fix_mode >= static_cast<std::int32_t>(GnssFixMode::Unknown) &&
                              fix_mode <= static_cast<std::int32_t>(GnssFixMode::Estimated)
                          ? static_cast<GnssFixMode>(fix_mode)
                          : GnssFixMode::Unknown;
    sample.fix_mode_name = read_string(*fix, "fix_mode_name");
    if (sample.fix_mode_name.empty()) {
        sample.fix_mode_name = std::string(gnss_fix_mode_name(sample.fix_mode));
    }

    sample.has_satellites_used = read_bool(*fix, "has_satellites_used", false);
    sample.satellites_used = static_cast<std::int32_t>(read_double(*fix, "satellites_used", 0.0));
    sample.has_hdop = read_bool(*fix, "has_hdop", false);
    sample.hdop = read_double(*fix, "hdop", 0.0);
    sample.has_speed = read_bool(*fix, "has_speed", false);
    sample.speed_over_ground_mps = read_double(*fix, "speed_over_ground_mps", 0.0);

    sample.has_freshness = fix->find("freshness_version") != nullptr;
    if (sample.has_freshness) {
        sample.position_valid = read_bool(*fix, "position_valid", false);
        sample.fix_mode_valid = read_bool(*fix, "fix_mode_valid", false);
        sample.heading_valid = read_bool(*fix, "heading_valid", false);
        sample.position_age_ms = read_unsigned(*fix, "position_age_ms");
        sample.fix_mode_age_ms = read_unsigned(*fix, "fix_mode_age_ms");
        sample.heading_age_ms = read_unsigned(*fix, "heading_age_ms");
    }
    return sample;
}

auto decode_vendor_motion(const json::Value& status_snapshot, std::string* error)
    -> std::optional<VendorMotionSample> {
    if (error != nullptr) {
        error->clear();
    }
    const json::Value* streams = status_snapshot.find("streams");
    if (streams == nullptr) {
        fail(error, "robot-agent status carries no streams");
        return std::nullopt;
    }
    const json::Value* motion = streams->find("motion_status");
    if (motion == nullptr || motion->is_null()) {
        fail(error, "robot-agent status carries no motion_status stream");
        return std::nullopt;
    }

    VendorMotionSample sample;
    sample.received_at_utc = read_string(*motion, "received_at_utc");
    if (sample.received_at_utc.empty()) {
        // Without the receipt timestamp the sample's age is unknown, and an aged yaw is exactly
        // what must never reach a state estimator.
        fail(error, "motion_status has no received_at_utc");
        return std::nullopt;
    }

    const json::Value* payload = motion->find("payload");
    if (payload == nullptr) {
        fail(error, "motion_status has no payload");
        return std::nullopt;
    }
    const json::Value* items = find_motion_items(*payload);
    if (items == nullptr) {
        fail(error, "motion_status payload has no Items");
        return std::nullopt;
    }
    const json::Value* yaw = items->find("Yaw");
    const auto yaw_value = yaw == nullptr ? std::nullopt : yaw->as_number();
    if (!yaw_value) {
        fail(error, "motion_status carries no numeric Yaw");
        return std::nullopt;
    }
    sample.yaw_rad = *yaw_value;
    sample.has_yaw = true;

    // Everything below is optional: the vendor firmware may omit fields, and a missing one must
    // degrade that single input rather than discard an otherwise valid heading.
    const auto roll = read_optional_number(*items, "Roll");
    const auto pitch = read_optional_number(*items, "Pitch");
    if (roll.has_value() && pitch.has_value()) {
        sample.roll_rad = *roll;
        sample.pitch_rad = *pitch;
        sample.has_attitude = true;
    }

    const auto linear_x = read_optional_number(*items, "LinearX");
    const auto linear_y = read_optional_number(*items, "LinearY");
    if (linear_x.has_value() && linear_y.has_value()) {
        sample.linear_x_mps = *linear_x;
        sample.linear_y_mps = *linear_y;
        sample.has_velocity = true;
    }

    if (const auto omega_z = read_optional_number(*items, "OmegaZ"); omega_z.has_value()) {
        sample.yaw_rate_rps = *omega_z;
        sample.has_yaw_rate = true;
    }

    if (const auto height = read_optional_number(*items, "Height"); height.has_value()) {
        sample.height_m = *height;
        sample.has_height = true;
    }
    return sample;
}

auto make_error_response(std::string_view action, std::string_view message) -> json::Value {
    json::Value response;
    response.set("status", json::Value::string("error"));
    response.set("protocol_version", json::Value::integer(kProtocolVersion));
    response.set("action", json::Value::string(std::string(action)));
    response.set("error", json::Value::string(std::string(message)));
    return response;
}

auto RequestRouter::handle(std::string_view request_json) -> std::string {
    std::string parse_error;
    const auto request = json::parse(request_json, &parse_error);
    if (!request) {
        return make_error_response("", parse_error).dump();
    }
    if (request->kind() != json::Value::Kind::Object) {
        return make_error_response("", "request must be a JSON object").dump();
    }

    const json::Value* action_node = request->find("action");
    const std::string* action_text = action_node == nullptr ? nullptr : action_node->as_string();
    if (action_text == nullptr || action_text->empty()) {
        return make_error_response("", "missing required field: action").dump();
    }

    // A peer speaking a different contract version must be refused loudly, not silently coerced.
    if (const json::Value* version = request->find("protocol_version"); version != nullptr) {
        const auto requested = version->as_integer();
        if (!requested || *requested != kProtocolVersion) {
            return make_error_response(*action_text, "unsupported protocol_version").dump();
        }
    }

    const json::Value* payload = request->find("payload");
    const json::Value empty_payload = json::Value::object({});
    return dispatch(*action_text, payload != nullptr ? *payload : empty_payload).dump();
}

auto RequestRouter::dispatch(std::string_view action, const json::Value& payload)
    -> json::Value {
    StateProvider& provider = provider_.get();

    if (action == "service.health") {
        const ServiceHealthState health = provider.health();
        json::Value response = make_ok_response(action);
        response.set("service", json::Value::string("perception-service"));
        response.set("health", health_to_json(health));
        return response;
    }

    if (action == "perception.get_health") {
        json::Value response = make_ok_response(action);
        response.set("health", health_to_json(provider.health()));
        return response;
    }

    if (action == "perception.get_localization") {
        json::Value response = make_ok_response(action);
        response.set("localization", localization_to_json(provider.localization()));
        return response;
    }

    if (action == "perception.get_environment") {
        // V0.3 output. Declared now so robot-agent can code against a stable shape and simply see
        // `available=false` until the environment branch exists.
        json::Value environment;
        environment.set("available", json::Value::boolean(false));
        environment.set("reason", json::Value::string("environment perception is not implemented"));
        json::Value response = make_ok_response(action);
        response.set("environment", std::move(environment));
        return response;
    }

    if (action == "perception.set_map_origin") {
        const json::Value* latitude = payload.find("latitude_deg");
        const json::Value* longitude = payload.find("longitude_deg");
        const auto latitude_value = latitude == nullptr ? std::nullopt : latitude->as_number();
        const auto longitude_value = longitude == nullptr ? std::nullopt : longitude->as_number();
        if (!latitude_value || !longitude_value) {
            return make_error_response(action, "latitude_deg and longitude_deg are required");
        }
        if (*latitude_value < -90.0 || *latitude_value > 90.0 || *longitude_value < -180.0 ||
            *longitude_value > 180.0) {
            return make_error_response(action, "latitude_deg or longitude_deg is out of range");
        }
        std::string error;
        if (!provider.set_map_origin(*latitude_value, *longitude_value, error)) {
            return make_error_response(action, error.empty() ? "map origin rejected" : error);
        }
        json::Value response = make_ok_response(action);
        response.set("latitude_deg", json::Value::real(*latitude_value));
        response.set("longitude_deg", json::Value::real(*longitude_value));
        return response;
    }

    return make_error_response(action, "unsupported action");
}

} // namespace perception::transport
