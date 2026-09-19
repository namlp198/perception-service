#include "perception/transport/remote_inputs.hpp"

#include "perception/transport/http.hpp"
#include "perception/transport/json.hpp"

#include "socket_support.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <utility>

#if defined(PERCEPTION_HAS_SPDLOG)
#include <spdlog/spdlog.h>
#endif

namespace perception::transport {
namespace {

using Clock = std::chrono::steady_clock;

// Upper bound on a peer's reply. It is deliberately separate from `transport.max_request_bytes`,
// which limits what clients may send us: robot-agent's status snapshot carries every vendor stream
// and is far larger than any request this service accepts.
constexpr std::size_t kMaxPeerResponseBytes = 1'048'576;

auto elapsed_ms(Clock::time_point since) -> std::uint64_t {
    if (since.time_since_epoch().count() == 0) {
        return 0;
    }
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - since);
    return elapsed.count() <= 0 ? 0U : static_cast<std::uint64_t>(elapsed.count());
}

// Newest-sample-wins storage for one peer, with the arrival instant kept separately from the
// peer's own timestamp so staleness can be judged on both.
template <typename Sample> struct LatestSample {
    mutable std::mutex mutex;
    std::optional<Sample> value;
    Clock::time_point arrived_at{};
    PeerMetrics metrics;

    void store(Sample sample) {
        const std::lock_guard<std::mutex> guard(mutex);
        value = std::move(sample);
        arrived_at = Clock::now();
        ++metrics.requests;
        ++metrics.samples;
        metrics.last_error.clear();
    }

    void record_empty(std::string reason) {
        const std::lock_guard<std::mutex> guard(mutex);
        ++metrics.requests;
        metrics.last_error = std::move(reason);
    }

    void record_failure(std::string reason) {
        const std::lock_guard<std::mutex> guard(mutex);
        ++metrics.requests;
        ++metrics.failures;
        metrics.last_error = std::move(reason);
    }

    [[nodiscard]] auto fresh_copy(std::uint32_t staleness_timeout_ms) const
        -> std::optional<Sample> {
        const std::lock_guard<std::mutex> guard(mutex);
        if (!value.has_value()) {
            return std::nullopt;
        }
        if (elapsed_ms(arrived_at) > staleness_timeout_ms) {
            return std::nullopt;
        }
        return value;
    }

    [[nodiscard]] auto snapshot(bool enabled, std::uint32_t staleness_timeout_ms) const
        -> PeerMetrics {
        const std::lock_guard<std::mutex> guard(mutex);
        PeerMetrics copy = metrics;
        copy.enabled = enabled;
        copy.last_sample_age_ms = value.has_value() ? elapsed_ms(arrived_at) : 0U;
        copy.fresh = value.has_value() && copy.last_sample_age_ms <= staleness_timeout_ms;
        return copy;
    }
};

} // namespace

class RemoteInputs::Impl {
  public:
    explicit Impl(core::TransportConfig value) : config(std::move(value)) {}

    core::TransportConfig config;
    std::atomic_bool running{false};
    std::mutex sleep_mutex;
    std::condition_variable sleep_signal;

    LatestSample<GnssSample> gnss;
    LatestSample<VendorMotionSample> vendor_motion;

    std::thread gnss_worker;
    std::thread vendor_worker;

    // Interruptible wait so stop() does not have to outlast a poll interval.
    void wait_for_next_poll(std::uint32_t interval_ms) {
        std::unique_lock<std::mutex> lock(sleep_mutex);
        sleep_signal.wait_for(lock, std::chrono::milliseconds(interval_ms),
                              [this]() { return !running.load(); });
    }

#if defined(PERCEPTION_HAS_POSIX_SOCKETS)
    // One request per connection towards payload-service: connect, write, half-close, read to EOF.
    auto request_payload_service(std::string_view request_body, std::string& response,
                                 std::string& error) -> bool {
        const core::PeerPollConfig& peer = config.payload_service;
        sockets::Descriptor socket_descriptor =
            sockets::connect_with_timeout(peer.host, peer.port, peer.request_timeout_ms, error);
        if (!socket_descriptor.valid()) {
            return false;
        }
        if (!sockets::send_all(socket_descriptor.get(), request_body, error)) {
            return false;
        }
        ::shutdown(socket_descriptor.get(), SHUT_WR);
        return sockets::receive_until_eof(socket_descriptor.get(), kMaxPeerResponseBytes, false,
                                          response, error);
    }

    auto request_robot_agent(std::string_view path, std::string& response, std::string& error)
        -> bool {
        const core::PeerPollConfig& peer = config.robot_agent_status;
        sockets::Descriptor socket_descriptor =
            sockets::connect_with_timeout(peer.host, peer.port, peer.request_timeout_ms, error);
        if (!socket_descriptor.valid()) {
            return false;
        }
        const std::string request = http::build_get_request(peer.host, peer.port, path);
        if (!sockets::send_all(socket_descriptor.get(), request, error)) {
            return false;
        }
        return sockets::receive_until_eof(socket_descriptor.get(), kMaxPeerResponseBytes, false,
                                          response, error);
    }

    void poll_gnss() {
        const core::PeerPollConfig& peer = config.payload_service;
        while (running.load()) {
            std::string response;
            std::string error;
            if (!request_payload_service(R"({"action":"gnss.get_status"})", response, error)) {
                gnss.record_failure(error.empty() ? "payload-service request failed" : error);
                wait_for_next_poll(peer.poll_interval_ms);
                continue;
            }
            const auto document = json::parse(response, &error);
            if (!document) {
                gnss.record_failure(error.empty() ? "invalid JSON from payload-service" : error);
                wait_for_next_poll(peer.poll_interval_ms);
                continue;
            }
            auto sample = decode_gnss_status(*document, &error);
            if (!sample) {
                // A peer with no fix yet is a normal state, not a transport failure.
                gnss.record_empty(error);
                wait_for_next_poll(peer.poll_interval_ms);
                continue;
            }
            gnss.store(std::move(*sample));
            wait_for_next_poll(peer.poll_interval_ms);
        }
    }

    void poll_vendor_motion() {
        const core::PeerPollConfig& peer = config.robot_agent_status;
        while (running.load()) {
            std::string response;
            std::string error;
            if (!request_robot_agent("/api/v1/status", response, error)) {
                vendor_motion.record_failure(error.empty() ? "robot-agent request failed" : error);
                wait_for_next_poll(peer.poll_interval_ms);
                continue;
            }
            const auto parsed = http::parse_response(response, &error);
            if (!parsed) {
                vendor_motion.record_failure(error.empty() ? "malformed HTTP response" : error);
                wait_for_next_poll(peer.poll_interval_ms);
                continue;
            }
            if (parsed->status_code != 200) {
                vendor_motion.record_failure("robot-agent returned HTTP " +
                                             std::to_string(parsed->status_code));
                wait_for_next_poll(peer.poll_interval_ms);
                continue;
            }
            const auto document = json::parse(parsed->body, &error);
            if (!document) {
                vendor_motion.record_failure(error.empty() ? "invalid JSON from robot-agent"
                                                           : error);
                wait_for_next_poll(peer.poll_interval_ms);
                continue;
            }
            auto sample = decode_vendor_motion(*document, &error);
            if (!sample) {
                vendor_motion.record_empty(error);
                wait_for_next_poll(peer.poll_interval_ms);
                continue;
            }
            vendor_motion.store(std::move(*sample));
            wait_for_next_poll(peer.poll_interval_ms);
        }
    }
#endif
};

RemoteInputs::RemoteInputs(core::TransportConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

RemoteInputs::~RemoteInputs() { stop(); }

auto RemoteInputs::start() -> bool {
    const bool any_peer_enabled =
        impl_->config.payload_service.enabled || impl_->config.robot_agent_status.enabled;
    if (!any_peer_enabled) {
        return true;
    }
#if defined(PERCEPTION_HAS_POSIX_SOCKETS)
    if (impl_->running.exchange(true)) {
        return true;
    }
    if (impl_->config.payload_service.enabled) {
        impl_->gnss_worker = std::thread([impl = impl_.get()]() { impl->poll_gnss(); });
#if defined(PERCEPTION_HAS_SPDLOG)
        spdlog::info("polling payload-service GNSS at {}:{} every {} ms",
                     impl_->config.payload_service.host, impl_->config.payload_service.port,
                     impl_->config.payload_service.poll_interval_ms);
#endif
    }
    if (impl_->config.robot_agent_status.enabled) {
        impl_->vendor_worker = std::thread([impl = impl_.get()]() { impl->poll_vendor_motion(); });
#if defined(PERCEPTION_HAS_SPDLOG)
        spdlog::info("polling robot-agent status at {}:{} every {} ms",
                     impl_->config.robot_agent_status.host, impl_->config.robot_agent_status.port,
                     impl_->config.robot_agent_status.poll_interval_ms);
#endif
    }
    return true;
#else
#if defined(PERCEPTION_HAS_SPDLOG)
    spdlog::warn("remote inputs are unavailable: this platform has no POSIX sockets");
#endif
    return false;
#endif
}

void RemoteInputs::stop() noexcept {
    if (!impl_->running.exchange(false)) {
        return;
    }
    impl_->sleep_signal.notify_all();
    if (impl_->gnss_worker.joinable()) {
        impl_->gnss_worker.join();
    }
    if (impl_->vendor_worker.joinable()) {
        impl_->vendor_worker.join();
    }
#if defined(PERCEPTION_HAS_SPDLOG)
    spdlog::info("remote input polling stopped");
#endif
}

auto RemoteInputs::gnss() const -> std::optional<GnssSample> {
    if (!impl_->config.payload_service.enabled) {
        return std::nullopt;
    }
    auto sample = impl_->gnss.fresh_copy(impl_->config.payload_service.staleness_timeout_ms);
    // The peer reports its own age too; a sample that arrived promptly but was already old at the
    // source is just as unusable to a state estimator.
    if (sample.has_value() &&
        sample->age_ms > impl_->config.payload_service.staleness_timeout_ms) {
        return std::nullopt;
    }
    return sample;
}

auto RemoteInputs::vendor_motion() const -> std::optional<VendorMotionSample> {
    if (!impl_->config.robot_agent_status.enabled) {
        return std::nullopt;
    }
    return impl_->vendor_motion.fresh_copy(impl_->config.robot_agent_status.staleness_timeout_ms);
}

auto RemoteInputs::metrics() const -> RemoteInputMetrics {
    RemoteInputMetrics snapshot;
    snapshot.gnss = impl_->gnss.snapshot(impl_->config.payload_service.enabled,
                                         impl_->config.payload_service.staleness_timeout_ms);
    snapshot.vendor_motion =
        impl_->vendor_motion.snapshot(impl_->config.robot_agent_status.enabled,
                                      impl_->config.robot_agent_status.staleness_timeout_ms);
    return snapshot;
}

} // namespace perception::transport
