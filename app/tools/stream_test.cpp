#include "perception/camera/camera_types.hpp"
#include "perception/core/config.hpp"
#include "perception/streaming/rtsp_server.hpp"

#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

namespace {
volatile std::sig_atomic_t stop_requested = 0;

extern "C" void handle_signal(int) { stop_requested = 1; }
}  // namespace

int main(int argc, char* argv[]) {
    const int duration_seconds = argc > 1 ? std::stoi(argv[1]) : 0;
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    perception::core::StreamingConfig config;
    config.infrared_left.enabled = false;
    config.infrared_right.enabled = false;
    config.depth_visual.enabled = false;
    perception::streaming::RtspServer server(config);
    if (!server.start()) {
        std::cerr << "Unable to start RTSP server. Check GStreamer plugins and port 8554.\n";
        return EXIT_FAILURE;
    }

    constexpr int width = 640;
    constexpr int height = 480;
    constexpr int channels = 3;
    constexpr auto frame_period = std::chrono::milliseconds(33);
    perception::camera::ImageFrame frame;
    frame.width = width;
    frame.height = height;
    frame.stride_bytes = width * channels;
    frame.format = perception::camera::PixelFormat::Bgr8;
    frame.data.resize(static_cast<std::size_t>(frame.stride_bytes * frame.height));

    const auto started_at = std::chrono::steady_clock::now();
    auto next_frame_at = started_at;
    std::uint64_t frame_number = 0;
    std::cout << "Synthetic low-latency stream: rtsp://127.0.0.1:8554/camera/rgb\n";

    while (stop_requested == 0) {
        const auto now = std::chrono::steady_clock::now();
        if (duration_seconds > 0 && now - started_at >= std::chrono::seconds(duration_seconds)) {
            break;
        }

        frame.capture_timestamp_ns = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count());
        frame.sensor_timestamp_ns = frame.capture_timestamp_ns;
        frame.frame_number = frame_number++;
        const auto phase = static_cast<std::uint8_t>(frame.frame_number % 255U);
        for (int row = 0; row < height; ++row) {
            for (int column = 0; column < width; ++column) {
                const auto offset = static_cast<std::size_t>((row * width + column) * channels);
                frame.data[offset] = static_cast<std::uint8_t>((column + phase) % 255);
                frame.data[offset + 1] = static_cast<std::uint8_t>((row + phase) % 255);
                frame.data[offset + 2] = phase;
            }
        }
        if (!server.publish(perception::streaming::StreamId::Rgb, frame)) {
            std::cerr << "Failed to publish synthetic frame.\n";
        }

        next_frame_at += frame_period;
        std::this_thread::sleep_until(next_frame_at);
    }

    server.stop();
    return EXIT_SUCCESS;
}
