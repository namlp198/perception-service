#include "perception/core/bounded_queue.hpp"
#include "perception/core/config.hpp"
#include "perception/health/camera_health.hpp"
#include "perception/streaming/depth_visualizer.hpp"
#include "perception/transport/http.hpp"
#include "perception/transport/json.hpp"
#include "perception/transport/protocol.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <stdexcept>
#include <string>

TEST(BoundedQueue, DropsOldestItemWhenFull) {
    perception::core::BoundedQueue<int> queue(2);
    queue.push(1);
    queue.push(2);
    queue.push(3);
    EXPECT_EQ(queue.size(), 2);
    EXPECT_EQ(queue.dropped_count(), 1);
    EXPECT_EQ(queue.try_pop(), 2);
    EXPECT_EQ(queue.try_pop(), 3);
}

TEST(Config, RejectsQueueOutsideFreshDataRange) {
    perception::core::ServiceConfig config;
    EXPECT_FALSE(config.camera.infrared_left.enabled);
    EXPECT_FALSE(config.camera.infrared_right.enabled);
    EXPECT_FALSE(config.streaming.infrared_left.enabled);
    EXPECT_FALSE(config.streaming.infrared_right.enabled);
    EXPECT_FALSE(config.streaming.hardware_encoder);
    EXPECT_TRUE(config.streaming.allow_software_fallback);
    EXPECT_TRUE(config.camera.imu.enabled);
    EXPECT_EQ(config.camera.imu.startup_timeout_ms, 2'000U);
    EXPECT_EQ(config.camera.imu.liveness_timeout_ms, 1'000U);
    EXPECT_EQ(config.camera.imu.restart_interval_ms, 30'000U);
    EXPECT_EQ(config.streaming.session_timeout_s, 20U);
    EXPECT_NO_THROW(perception::core::validate_config(config));
    // Sessions must expire fast enough to release a dead client's transport, not faster than
    // a healthy client's keep-alive cadence.
    config.streaming.session_timeout_s = 4;
    EXPECT_THROW(perception::core::validate_config(config), std::invalid_argument);
    config.streaming.session_timeout_s = 20;
    // A Motion Module retry faster than the startup proof window is rejected; 0 disables retries.
    config.camera.imu.restart_interval_ms = 1'000;
    EXPECT_THROW(perception::core::validate_config(config), std::invalid_argument);
    config.camera.imu.restart_interval_ms = 0;
    EXPECT_NO_THROW(perception::core::validate_config(config));
    config.camera.imu.restart_interval_ms = 30'000;
    config.streaming.queue_capacity = 0;
    EXPECT_THROW(perception::core::validate_config(config), std::invalid_argument);
}

TEST(CameraHealth, ReportsFrameAgeAndCounters) {
    perception::health::CameraHealth health;
    health.record_frame(1'000'000'000);
    health.record_drop();
    perception::camera::ImuBatch imu;
    imu.accelerometer.push_back({1'010'000'000, 1'011'000'000, {0.0F, 9.8F, 0.0F}});
    imu.gyroscope.push_back({1'012'000'000, 1'013'000'000, {0.1F, 0.2F, 0.3F}});
    health.record_imu(imu);
    const auto snapshot = health.snapshot(1'025'000'000);
    EXPECT_EQ(snapshot.status, perception::health::CameraStatus::Healthy);
    EXPECT_EQ(snapshot.frames_received, 1);
    EXPECT_EQ(snapshot.frames_dropped, 1);
    EXPECT_EQ(snapshot.frame_age, std::chrono::milliseconds(25));
    EXPECT_EQ(snapshot.accelerometer_samples, 1);
    EXPECT_EQ(snapshot.gyroscope_samples, 1);
    EXPECT_EQ(snapshot.accelerometer_age, std::chrono::milliseconds(14));
    EXPECT_EQ(snapshot.gyroscope_age, std::chrono::milliseconds(12));
}

TEST(Config, RejectsInvalidImuRatesAndCapacity) {
    perception::core::ServiceConfig config;
    EXPECT_EQ(config.camera.imu.accelerometer_fps, 100);
    EXPECT_EQ(config.camera.imu.gyroscope_fps, 200);
    config.camera.imu.enabled = true;
    config.camera.imu.accelerometer_fps = 0;
    EXPECT_THROW(perception::core::validate_config(config), std::invalid_argument);

    config.camera.imu.accelerometer_fps = 100;
    config.camera.imu.queue_capacity = 31;
    EXPECT_THROW(perception::core::validate_config(config), std::invalid_argument);
}

TEST(DepthVisualizer, PreservesTimestampsAndMarksInvalidDepthBlack) {
    perception::camera::DepthFrame depth;
    depth.sensor_timestamp_ns = 10;
    depth.capture_timestamp_ns = 20;
    depth.frame_number = 30;
    depth.width = 4;
    depth.height = 1;
    depth.depth_scale_m = 0.001F;
    depth.data = {0U, 500U, 1'250U, 2'000U};

    const auto visual = perception::streaming::colorize_depth(depth, 0.5F, 2.0F);
    EXPECT_EQ(visual.sensor_timestamp_ns, depth.sensor_timestamp_ns);
    EXPECT_EQ(visual.capture_timestamp_ns, depth.capture_timestamp_ns);
    EXPECT_EQ(visual.frame_number, depth.frame_number);
    ASSERT_EQ(visual.data.size(), 12U);
    EXPECT_EQ(visual.data[0], 0U);
    EXPECT_EQ(visual.data[1], 0U);
    EXPECT_EQ(visual.data[2], 0U);
    EXPECT_GT(visual.data[3], visual.data[5]);
    EXPECT_TRUE(visual.data[6] > 0U || visual.data[7] > 0U || visual.data[8] > 0U);
    EXPECT_EQ(visual.data[9], 0U);
    EXPECT_EQ(visual.data[10], 0U);
    EXPECT_GT(visual.data[11], 0U);
}

TEST(Config, RejectsInvalidDepthVisualRange) {
    perception::core::ServiceConfig config;
    config.streaming.depth_visual.min_distance_m = 2.0F;
    config.streaming.depth_visual.max_distance_m = 1.0F;
    EXPECT_THROW(perception::core::validate_config(config), std::invalid_argument);
}

namespace {

using perception::transport::LocalizationState;
using perception::transport::RequestRouter;
using perception::transport::ServiceHealthState;
using perception::transport::StateProvider;
namespace pjson = perception::transport::json;

class StubStateProvider final : public StateProvider {
  public:
    [[nodiscard]] auto health() const -> ServiceHealthState override { return health_; }
    [[nodiscard]] auto localization() const -> LocalizationState override { return localization_; }
    auto set_map_origin(double latitude_deg, double longitude_deg, std::string& error)
        -> bool override {
        (void)error;
        latitude_ = latitude_deg;
        longitude_ = longitude_deg;
        return true;
    }

    ServiceHealthState health_{};
    LocalizationState localization_{};
    double latitude_{0.0};
    double longitude_{0.0};
};

} // namespace

TEST(TransportJson, RealValuesKeepTheirDecimalPoint) {
    EXPECT_EQ(pjson::Value::real(0.0).dump(), "0.0");
    EXPECT_EQ(pjson::Value::real(5.0).dump(), "5.0");
    EXPECT_EQ(pjson::Value::integer(0).dump(), "0");
}

TEST(TransportJson, PreservesFieldOrderAndRejectsTrailingData) {
    pjson::Value node;
    node.set("b", pjson::Value::integer(1));
    node.set("a", pjson::Value::integer(2));
    EXPECT_EQ(node.dump(), R"({"b":1,"a":2})");

    std::string error;
    EXPECT_FALSE(pjson::parse("{} trailing", &error).has_value());
    EXPECT_FALSE(error.empty());
}

TEST(TransportProtocol, DecodesPayloadServiceGnssEnvelope) {
    std::string error;
    const auto response = pjson::parse(
        R"({"status":"ok","fix":{"timestamp_unix_ms":17,"age_ms":120,"has_position":true,
            "latitude_deg":21.0285112,"fix_mode":4,"fix_mode_name":"rtk_fixed",
            "freshness_version":1,"position_valid":true,"fix_mode_age_ms":1120}})",
        &error);
    ASSERT_TRUE(response.has_value());

    const auto sample = perception::transport::decode_gnss_status(*response, &error);
    ASSERT_TRUE(sample.has_value());
    EXPECT_EQ(sample->fix_mode, perception::transport::GnssFixMode::RtkFixed);
    EXPECT_TRUE(sample->has_freshness);
    EXPECT_EQ(sample->fix_mode_age_ms, 1120U);
}

TEST(TransportProtocol, ReportsAbsentGnssFixAsUnavailable) {
    std::string error;
    const auto response = pjson::parse(R"({"status":"ok","fix":null})", &error);
    ASSERT_TRUE(response.has_value());
    EXPECT_FALSE(perception::transport::decode_gnss_status(*response, &error).has_value());
    EXPECT_FALSE(error.empty());
}

TEST(TransportProtocol, RejectsVendorMotionWithoutReceiptTimestamp) {
    std::string error;
    const auto response = pjson::parse(
        R"({"streams":{"motion_status":{"payload":{"PatrolDevice":{"Items":{"Yaw":0.25}}}}}})",
        &error);
    ASSERT_TRUE(response.has_value());
    EXPECT_FALSE(perception::transport::decode_vendor_motion(*response, &error).has_value());
}

TEST(TransportProtocol, AcceptsFlatAndNestedVendorMotionItems) {
    std::string error;
    const auto response = pjson::parse(
        R"({"streams":{"motion_status":{"received_at_utc":"2026-09-19T03:04:05.123+00:00",
            "payload":{"PatrolDevice":{"Items":{"MotionStatus":{"Yaw":0.5}}}}}}})",
        &error);
    ASSERT_TRUE(response.has_value());
    const auto sample = perception::transport::decode_vendor_motion(*response, &error);
    ASSERT_TRUE(sample.has_value());
    EXPECT_DOUBLE_EQ(sample->yaw_rad, 0.5);
}

TEST(TransportRouter, ReportsLocalizationUnavailableBeforeTheEkfExists) {
    StubStateProvider provider;
    RequestRouter router(provider);
    std::string error;
    const auto response =
        pjson::parse(router.handle(R"({"action":"perception.get_localization"})"), &error);
    ASSERT_TRUE(response.has_value());
    const pjson::Value* localization = response->find("localization");
    ASSERT_NE(localization, nullptr);
    EXPECT_FALSE(*localization->find("available")->as_bool());
    EXPECT_EQ(*localization->find("quality")->as_string(), "lost");
}

TEST(TransportRouter, RefusesUnknownActionsAndForeignProtocolVersions) {
    StubStateProvider provider;
    RequestRouter router(provider);
    std::string error;

    const auto unknown = pjson::parse(router.handle(R"({"action":"camera.take_photo"})"), &error);
    ASSERT_TRUE(unknown.has_value());
    EXPECT_EQ(*unknown->find("error")->as_string(), "unsupported action");

    const auto foreign = pjson::parse(
        router.handle(R"({"action":"service.health","protocol_version":99})"), &error);
    ASSERT_TRUE(foreign.has_value());
    EXPECT_EQ(*foreign->find("error")->as_string(), "unsupported protocol_version");
}

TEST(TransportRouter, ValidatesMapOriginRange) {
    StubStateProvider provider;
    RequestRouter router(provider);
    std::string error;
    const auto rejected = pjson::parse(
        router.handle(
            R"({"action":"perception.set_map_origin","payload":{"latitude_deg":120.0,"longitude_deg":0.0}})"),
        &error);
    ASSERT_TRUE(rejected.has_value());
    EXPECT_EQ(*rejected->find("status")->as_string(), "error");
}

TEST(Config, RejectsTransportPortCollidingWithRtsp) {
    perception::core::ServiceConfig config;
    config.transport.enabled = true;
    config.transport.port = config.streaming.port;
    EXPECT_THROW(perception::core::validate_config(config), std::invalid_argument);
}

TEST(Config, RejectsPeerPollWithStalenessBelowPollInterval) {
    perception::core::ServiceConfig config;
    config.transport.payload_service.enabled = true;
    config.transport.payload_service.poll_interval_ms = 500;
    config.transport.payload_service.staleness_timeout_ms = 100;
    EXPECT_THROW(perception::core::validate_config(config), std::invalid_argument);
}

TEST(TransportHttp, BuildsOneShotGetRequest) {
    const std::string request =
        perception::transport::http::build_get_request("192.168.1.206", 5080, "/api/v1/status");
    EXPECT_EQ(request.rfind("GET /api/v1/status HTTP/1.1\r\n", 0), 0U);
    EXPECT_NE(request.find("Connection: close\r\n"), std::string::npos);
}

TEST(TransportHttp, RespectsContentLengthAndRejectsTruncatedBodies) {
    std::string error;
    const auto parsed = perception::transport::http::parse_response(
        "HTTP/1.1 200 OK\r\nContent-Length: 7\r\n\r\n{\"a\":1}extra", &error);
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ(parsed->status_code, 200);
    EXPECT_EQ(parsed->body, "{\"a\":1}");

    EXPECT_FALSE(perception::transport::http::parse_response(
                     "HTTP/1.1 200 OK\r\nContent-Length: 40\r\n\r\n{\"a\":1}", &error)
                     .has_value());
    EXPECT_FALSE(error.empty());
}

TEST(TransportHttp, RejectsChunkedResponses) {
    std::string error;
    EXPECT_FALSE(perception::transport::http::parse_response(
                     "HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\n0\r\n\r\n", &error)
                     .has_value());
}

TEST(TransportProtocol, DecodesTheLiveVendorMotionShape) {
    // Field names and magnitudes from a live capture of robot-agent 192.168.1.206 (2026-09-19).
    std::string error;
    const auto snapshot = pjson::parse(
        R"({"streams":{"motion_status":{
            "received_at_utc":"2026-09-19T11:15:40.575+00:00",
            "source_time":"2026-09-19 19:15:40.573",
            "payload":{"PatrolDevice":{"Type":1002,"Command":4,"Items":{
                "MotionStatus":{"Roll":0.005853095557540655,"Pitch":0.003533279523253441,
                                "Yaw":-1.4777541160583496,"OmegaZ":-0.0031251865439116955,
                                "LinearX":0.0,"LinearY":0.0,"Height":0.10076384246349335},
                "MotorStatus":{"LeftFrontHipX":0.6266317367553711}}}}}}})",
        &error);
    ASSERT_TRUE(snapshot.has_value());

    const auto sample = perception::transport::decode_vendor_motion(*snapshot, &error);
    ASSERT_TRUE(sample.has_value());
    EXPECT_DOUBLE_EQ(sample->yaw_rad, -1.4777541160583496);
    EXPECT_TRUE(sample->has_attitude);
    EXPECT_TRUE(sample->has_yaw_rate);
    EXPECT_DOUBLE_EQ(sample->yaw_rate_rps, -0.0031251865439116955);
    // A stationary robot reports exact zero velocity; that is a measurement, not a missing field.
    EXPECT_TRUE(sample->has_velocity);
    EXPECT_DOUBLE_EQ(sample->linear_x_mps, 0.0);
    EXPECT_TRUE(sample->has_height);
}

TEST(TransportProtocol, TreatsHalfPresentVendorVelocityAsMissing) {
    std::string error;
    const auto snapshot = pjson::parse(
        R"({"streams":{"motion_status":{"received_at_utc":"2026-09-19T11:15:40.575+00:00",
            "payload":{"PatrolDevice":{"Items":{"MotionStatus":{"Yaw":0.5,"LinearX":0.3}}}}}}})",
        &error);
    ASSERT_TRUE(snapshot.has_value());
    const auto sample = perception::transport::decode_vendor_motion(*snapshot, &error);
    ASSERT_TRUE(sample.has_value());
    EXPECT_TRUE(sample->has_yaw);
    EXPECT_FALSE(sample->has_velocity);
}

