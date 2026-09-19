#include "perception/transport/json.hpp"
#include "perception/transport/protocol.hpp"

#include "test_support.hpp"

#include <string>

namespace {

using perception::transport::GnssFixMode;
using perception::transport::LocalizationQuality;
using perception::transport::LocalizationState;
using perception::transport::RequestRouter;
using perception::transport::ServiceHealthState;
using perception::transport::StateProvider;
namespace json = perception::transport::json;

class FakeStateProvider final : public StateProvider {
  public:
    [[nodiscard]] auto health() const -> ServiceHealthState override { return health_; }
    [[nodiscard]] auto localization() const -> LocalizationState override { return localization_; }

    auto set_map_origin(double latitude_deg, double longitude_deg, std::string& error)
        -> bool override {
        if (reject_origin) {
            error = "map origin rejected by test";
            return false;
        }
        origin_latitude_deg = latitude_deg;
        origin_longitude_deg = longitude_deg;
        return true;
    }

    ServiceHealthState health_{};
    LocalizationState localization_{};
    bool reject_origin{false};
    double origin_latitude_deg{0.0};
    double origin_longitude_deg{0.0};
};

auto parse_or_fail(const std::string& text) -> json::Value {
    std::string error;
    auto value = json::parse(text, &error);
    return value ? *value : json::Value{};
}

auto json_real_keeps_decimal_point() -> bool {
    // The 2026-07-23 Yaw failure came from integer zeroes replacing float zeroes on the wire, so
    // a real value must never serialize as a bare integer here.
    CHECK_TRUE(json::Value::real(0.0).dump() == "0.0");
    CHECK_TRUE(json::Value::real(-0.0).dump() == "-0.0");
    CHECK_TRUE(json::Value::real(5.0).dump() == "5.0");
    CHECK_TRUE(json::Value::integer(0).dump() == "0");
    CHECK_TRUE(json::Value::real(0.5).dump() == "0.5");
    return true;
}

auto json_round_trips_precise_coordinates() -> bool {
    const double latitude = 21.028511123456789;
    const std::string text = json::Value::real(latitude).dump();
    const json::Value parsed = parse_or_fail(text);
    CHECK_TRUE(parsed.as_number().has_value());
    CHECK_TRUE(*parsed.as_number() == latitude);
    return true;
}

auto json_preserves_object_field_order() -> bool {
    json::Value node;
    node.set("b", json::Value::integer(1));
    node.set("a", json::Value::integer(2));
    node.set("b", json::Value::integer(3));
    CHECK_TRUE(node.dump() == R"({"b":3,"a":2})");
    return true;
}

auto json_rejects_malformed_input() -> bool {
    std::string error;
    CHECK_TRUE(!json::parse("{\"a\":1", &error).has_value());
    CHECK_TRUE(!error.empty());
    CHECK_TRUE(!json::parse("{} trailing", &error).has_value());
    CHECK_TRUE(!json::parse("", &error).has_value());
    CHECK_TRUE(json::parse("  {\"a\": 1}  \n", &error).has_value());
    return true;
}

auto json_parses_escapes_and_nesting() -> bool {
    const json::Value parsed = parse_or_fail(R"({"text":"a\"b\\cA\n"})");
    const json::Value* text = parsed.find("text");
    CHECK_TRUE(text != nullptr);
    CHECK_TRUE(*text->as_string() == "a\"b\\cA\n");
    return true;
}

auto gnss_decode_reads_payload_service_envelope() -> bool {
    const json::Value response = parse_or_fail(R"({
        "status": "ok",
        "action": "gnss.get_status",
        "connected": true,
        "sample_count": 412,
        "fix": {
            "timestamp_unix_ms": 1758067200000,
            "age_ms": 120,
            "has_position": true,
            "latitude_deg": 21.0285112,
            "longitude_deg": 105.8048001,
            "has_course": true,
            "course_deg": 87.5,
            "has_heading": true,
            "heading_deg": 91.25,
            "fix_mode": 4,
            "fix_mode_name": "rtk_fixed",
            "has_satellites_used": true,
            "satellites_used": 24,
            "has_hdop": true,
            "hdop": 0.7,
            "has_speed": true,
            "speed_over_ground_mps": 0.42,
            "freshness_version": 1,
            "position_valid": true,
            "fix_mode_valid": true,
            "heading_valid": false,
            "position_timestamp_unix_ms": 1758067200000,
            "position_age_ms": 120,
            "fix_mode_timestamp_unix_ms": 1758067199000,
            "fix_mode_age_ms": 1120,
            "heading_timestamp_unix_ms": 0,
            "heading_age_ms": 99999
        }
    })");

    std::string error;
    const auto sample = perception::transport::decode_gnss_status(response, &error);
    CHECK_TRUE(sample.has_value());
    CHECK_TRUE(error.empty());
    CHECK_TRUE(sample->timestamp_unix_ms == 1758067200000ULL);
    CHECK_TRUE(sample->age_ms == 120ULL);
    CHECK_TRUE(sample->has_position);
    CHECK_TRUE(sample->fix_mode == GnssFixMode::RtkFixed);
    CHECK_TRUE(sample->fix_mode_name == "rtk_fixed");
    CHECK_TRUE(sample->satellites_used == 24);
    CHECK_TRUE(sample->has_freshness);
    CHECK_TRUE(sample->position_valid);
    CHECK_TRUE(!sample->heading_valid);
    CHECK_TRUE(sample->fix_mode_age_ms == 1120ULL);
    return true;
}

auto gnss_decode_reports_absent_and_failed_fixes() -> bool {
    std::string error;
    const json::Value no_fix = parse_or_fail(R"({"status":"ok","fix":null})");
    CHECK_TRUE(!perception::transport::decode_gnss_status(no_fix, &error).has_value());
    CHECK_TRUE(!error.empty());

    const json::Value failure = parse_or_fail(R"({"status":"error","error":"adapter not connected"})");
    CHECK_TRUE(!perception::transport::decode_gnss_status(failure, &error).has_value());
    CHECK_TRUE(error == "adapter not connected");

    // An older payload-service without the v2 freshness metadata still yields a usable sample.
    const json::Value legacy = parse_or_fail(
        R"({"status":"ok","fix":{"timestamp_unix_ms":10,"has_position":true,"fix_mode":5}})");
    const auto sample = perception::transport::decode_gnss_status(legacy, &error);
    CHECK_TRUE(sample.has_value());
    CHECK_TRUE(!sample->has_freshness);
    CHECK_TRUE(sample->fix_mode == GnssFixMode::RtkFloat);
    CHECK_TRUE(sample->fix_mode_name == "rtk_float");
    return true;
}

auto vendor_motion_decode_accepts_both_shapes() -> bool {
    std::string error;
    const json::Value nested = parse_or_fail(R"({
        "streams": {
            "motion_status": {
                "received_at_utc": "2026-09-19T03:04:05.123+00:00",
                "payload": {"PatrolDevice": {"Type": 1002, "Items": {"MotionStatus": {"Yaw": 1.5707963}}}}
            }
        }
    })");
    const auto nested_sample = perception::transport::decode_vendor_motion(nested, &error);
    CHECK_TRUE(nested_sample.has_value());
    CHECK_TRUE(nested_sample->has_yaw);
    CHECK_TRUE(nested_sample->yaw_rad > 1.57 && nested_sample->yaw_rad < 1.58);

    const json::Value flat = parse_or_fail(R"({
        "streams": {
            "motion_status": {
                "received_at_utc": "2026-09-19T03:04:05.123+00:00",
                "payload": {"PatrolDevice": {"Items": {"Yaw": 0.25}}}
            }
        }
    })");
    const auto flat_sample = perception::transport::decode_vendor_motion(flat, &error);
    CHECK_TRUE(flat_sample.has_value());
    CHECK_TRUE(flat_sample->yaw_rad == 0.25);
    return true;
}

// Field names and magnitudes copied from a live capture of robot-agent 192.168.1.206
// (GET /api/v1/status, 2026-09-19) so the decoder is pinned to the real vendor shape.
auto vendor_motion_decode_reads_the_live_vendor_shape() -> bool {
    const json::Value snapshot = parse_or_fail(R"({
        "streams": {
            "motion_status": {
                "updated_at": "2026-09-19T11:15:40.575+00:00",
                "received_at_utc": "2026-09-19T11:15:40.575+00:00",
                "source_time": "2026-09-19 19:15:40.573",
                "payload": {
                    "PatrolDevice": {
                        "Type": 1002,
                        "Command": 4,
                        "Time": "2026-09-19 19:15:40.573",
                        "Items": {
                            "MotionStatus": {
                                "Roll": 0.005853095557540655,
                                "Pitch": 0.003533279523253441,
                                "Yaw": -1.4777541160583496,
                                "OmegaZ": -0.0031251865439116955,
                                "LinearX": 0.0,
                                "LinearY": 0.0,
                                "Height": 0.10076384246349335,
                                "Payload": 0.0,
                                "RemainMile": 12.994800567626953
                            },
                            "MotorStatus": {"LeftFrontHipX": 0.6266317367553711}
                        }
                    }
                }
            }
        }
    })");

    std::string error;
    const auto sample = perception::transport::decode_vendor_motion(snapshot, &error);
    CHECK_TRUE(sample.has_value());
    CHECK_TRUE(sample->received_at_utc == "2026-09-19T11:15:40.575+00:00");
    CHECK_TRUE(sample->has_yaw);
    CHECK_TRUE(sample->yaw_rad == -1.4777541160583496);
    CHECK_TRUE(sample->has_attitude);
    CHECK_TRUE(sample->roll_rad == 0.005853095557540655);
    CHECK_TRUE(sample->has_yaw_rate);
    CHECK_TRUE(sample->yaw_rate_rps == -0.0031251865439116955);
    // A stationary robot reports exact zero velocity; that is a measurement, not a missing field.
    CHECK_TRUE(sample->has_velocity);
    CHECK_TRUE(sample->linear_x_mps == 0.0);
    CHECK_TRUE(sample->has_height);
    return true;
}

auto vendor_motion_decode_degrades_field_by_field() -> bool {
    std::string error;
    // Yaw alone still yields a usable heading; the absent inputs are simply flagged missing.
    const json::Value yaw_only = parse_or_fail(R"({
        "streams": {"motion_status": {"received_at_utc": "2026-09-19T11:15:40.575+00:00",
            "payload": {"PatrolDevice": {"Items": {"MotionStatus": {"Yaw": 0.5}}}}}}
    })");
    const auto sample = perception::transport::decode_vendor_motion(yaw_only, &error);
    CHECK_TRUE(sample.has_value());
    CHECK_TRUE(sample->has_yaw);
    CHECK_TRUE(!sample->has_velocity);
    CHECK_TRUE(!sample->has_attitude);
    CHECK_TRUE(!sample->has_yaw_rate);

    // A half-present velocity pair is not a velocity.
    const json::Value partial = parse_or_fail(R"({
        "streams": {"motion_status": {"received_at_utc": "2026-09-19T11:15:40.575+00:00",
            "payload": {"PatrolDevice": {"Items": {"MotionStatus": {"Yaw": 0.5, "LinearX": 0.3}}}}}}
    })");
    const auto half = perception::transport::decode_vendor_motion(partial, &error);
    CHECK_TRUE(half.has_value());
    CHECK_TRUE(!half->has_velocity);
    return true;
}

auto vendor_motion_decode_rejects_untimestamped_and_malformed() -> bool {
    std::string error;
    const json::Value no_receipt = parse_or_fail(
        R"({"streams":{"motion_status":{"payload":{"PatrolDevice":{"Items":{"Yaw":0.25}}}}}})");
    CHECK_TRUE(!perception::transport::decode_vendor_motion(no_receipt, &error).has_value());
    CHECK_TRUE(!error.empty());

    const json::Value non_numeric = parse_or_fail(
        R"({"streams":{"motion_status":{"received_at_utc":"2026-09-19T03:04:05.123+00:00",
            "payload":{"PatrolDevice":{"Items":{"Yaw":"north"}}}}}})");
    CHECK_TRUE(!perception::transport::decode_vendor_motion(non_numeric, &error).has_value());

    const json::Value no_stream = parse_or_fail(R"({"streams":{}})");
    CHECK_TRUE(!perception::transport::decode_vendor_motion(no_stream, &error).has_value());
    return true;
}

auto router_reports_health_and_version() -> bool {
    FakeStateProvider provider;
    provider.health_.status = "healthy";
    provider.health_.frames_received = 380'000;
    provider.health_.accelerometer_hz = 100.7;
    provider.health_.imu_ready = true;
    RequestRouter router(provider);

    const json::Value response = parse_or_fail(router.handle(R"({"action":"service.health"})"));
    CHECK_TRUE(*response.find("status")->as_string() == "ok");
    CHECK_TRUE(*response.find("protocol_version")->as_integer() ==
               perception::transport::kProtocolVersion);
    const json::Value* health = response.find("health");
    CHECK_TRUE(health != nullptr);
    CHECK_TRUE(*health->find("imu_ready")->as_bool());
    CHECK_TRUE(*health->find("frames_received")->as_integer() == 380'000);
    return true;
}

auto router_reports_localization_as_unavailable_before_the_ekf_exists() -> bool {
    FakeStateProvider provider;
    RequestRouter router(provider);

    const json::Value response =
        parse_or_fail(router.handle(R"({"action":"perception.get_localization"})"));
    const json::Value* localization = response.find("localization");
    CHECK_TRUE(localization != nullptr);
    // No EKF yet: the branch must report unavailable rather than a fabricated zero pose.
    CHECK_TRUE(!*localization->find("available")->as_bool());
    CHECK_TRUE(*localization->find("quality")->as_string() == "lost");
    return true;
}

auto router_serializes_pose_components_as_reals() -> bool {
    FakeStateProvider provider;
    provider.localization_.available = true;
    provider.localization_.quality = LocalizationQuality::DeadReckoning;
    provider.localization_.map_pose = {0.0, 0.0, 0.0};
    RequestRouter router(provider);

    const std::string body = router.handle(R"({"action":"perception.get_localization"})");
    CHECK_TRUE(body.find(R"("x_m":0.0)") != std::string::npos);
    CHECK_TRUE(body.find(R"("x_m":0,)") == std::string::npos);
    CHECK_TRUE(body.find(R"("quality":"dead_reckoning")") != std::string::npos);
    return true;
}

auto router_rejects_bad_requests() -> bool {
    FakeStateProvider provider;
    RequestRouter router(provider);

    const json::Value no_action = parse_or_fail(router.handle(R"({"payload":{}})"));
    CHECK_TRUE(*no_action.find("status")->as_string() == "error");

    const json::Value malformed = parse_or_fail(router.handle("{not json"));
    CHECK_TRUE(*malformed.find("status")->as_string() == "error");

    const json::Value unknown = parse_or_fail(router.handle(R"({"action":"camera.take_photo"})"));
    CHECK_TRUE(*unknown.find("error")->as_string() == "unsupported action");

    // A peer on a different contract version is refused, never silently coerced.
    const json::Value wrong_version = parse_or_fail(
        router.handle(R"({"action":"service.health","protocol_version":99})"));
    CHECK_TRUE(*wrong_version.find("error")->as_string() == "unsupported protocol_version");

    const json::Value right_version = parse_or_fail(
        router.handle(R"({"action":"service.health","protocol_version":1})"));
    CHECK_TRUE(*right_version.find("status")->as_string() == "ok");
    return true;
}

auto router_validates_map_origin() -> bool {
    FakeStateProvider provider;
    RequestRouter router(provider);

    const json::Value accepted = parse_or_fail(router.handle(
        R"({"action":"perception.set_map_origin","payload":{"latitude_deg":21.0285,"longitude_deg":105.8048}})"));
    CHECK_TRUE(*accepted.find("status")->as_string() == "ok");
    CHECK_TRUE(provider.origin_latitude_deg == 21.0285);

    const json::Value missing = parse_or_fail(
        router.handle(R"({"action":"perception.set_map_origin","payload":{"latitude_deg":21.0}})"));
    CHECK_TRUE(*missing.find("status")->as_string() == "error");

    const json::Value out_of_range = parse_or_fail(router.handle(
        R"({"action":"perception.set_map_origin","payload":{"latitude_deg":120.0,"longitude_deg":0.0}})"));
    CHECK_TRUE(*out_of_range.find("status")->as_string() == "error");

    provider.reject_origin = true;
    const json::Value rejected = parse_or_fail(router.handle(
        R"({"action":"perception.set_map_origin","payload":{"latitude_deg":21.0,"longitude_deg":105.0}})"));
    CHECK_TRUE(*rejected.find("error")->as_string() == "map origin rejected by test");
    return true;
}

auto router_declares_environment_branch_as_unavailable() -> bool {
    FakeStateProvider provider;
    RequestRouter router(provider);
    const json::Value response =
        parse_or_fail(router.handle(R"({"action":"perception.get_environment"})"));
    CHECK_TRUE(*response.find("status")->as_string() == "ok");
    CHECK_TRUE(!*response.find("environment")->find("available")->as_bool());
    return true;
}

} // namespace

auto transport_test() -> bool {
    return json_real_keeps_decimal_point() && json_round_trips_precise_coordinates() &&
           json_preserves_object_field_order() && json_rejects_malformed_input() &&
           json_parses_escapes_and_nesting() && gnss_decode_reads_payload_service_envelope() &&
           gnss_decode_reports_absent_and_failed_fixes() &&
           vendor_motion_decode_accepts_both_shapes() &&
           vendor_motion_decode_reads_the_live_vendor_shape() &&
           vendor_motion_decode_degrades_field_by_field() &&
           vendor_motion_decode_rejects_untimestamped_and_malformed() &&
           router_reports_health_and_version() &&
           router_reports_localization_as_unavailable_before_the_ekf_exists() &&
           router_serializes_pose_components_as_reals() && router_rejects_bad_requests() &&
           router_validates_map_origin() && router_declares_environment_branch_as_unavailable();
}
