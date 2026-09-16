#include "perception/streaming/depth_visualizer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace perception::streaming {
namespace {

auto to_byte(float value) -> std::uint8_t {
    return static_cast<std::uint8_t>(std::clamp(value, 0.0F, 1.0F) * 255.0F);
}

} // namespace

auto colorize_depth(const camera::DepthFrame& depth, float min_distance_m, float max_distance_m)
    -> camera::ImageFrame {
    camera::ImageFrame visual;
    visual.sensor_timestamp_ns = depth.sensor_timestamp_ns;
    visual.capture_timestamp_ns = depth.capture_timestamp_ns;
    visual.frame_number = depth.frame_number;
    visual.width = depth.width;
    visual.height = depth.height;
    visual.stride_bytes = depth.width * 3;
    visual.format = camera::PixelFormat::Bgr8;

    if (depth.width <= 0 || depth.height <= 0 || depth.depth_scale_m <= 0.0F ||
        max_distance_m <= min_distance_m) {
        return visual;
    }

    const auto expected_pixels =
        static_cast<std::size_t>(depth.width) * static_cast<std::size_t>(depth.height);
    if (depth.data.size() < expected_pixels) {
        return visual;
    }

    visual.data.resize(expected_pixels * 3U, 0U);
    const float distance_range_m = max_distance_m - min_distance_m;
    constexpr std::size_t histogram_bins = 2'048U;
    std::array<std::uint32_t, histogram_bins> histogram{};
    std::uint32_t valid_pixels = 0U;
    for (std::size_t pixel = 0; pixel < expected_pixels; ++pixel) {
        const std::uint16_t raw_depth = depth.data[pixel];
        if (raw_depth == 0U) {
            continue;
        }
        const float distance_m = static_cast<float>(raw_depth) * depth.depth_scale_m;
        if (distance_m < min_distance_m || distance_m > max_distance_m) {
            continue;
        }
        const float linear = (distance_m - min_distance_m) / distance_range_m;
        const auto bin = static_cast<std::size_t>(linear * static_cast<float>(histogram_bins - 1U));
        ++histogram[bin];
        ++valid_pixels;
    }
    if (valid_pixels == 0U) {
        return visual;
    }
    for (std::size_t bin = 1; bin < histogram.size(); ++bin) {
        histogram[bin] += histogram[bin - 1U];
    }

    for (std::size_t pixel = 0; pixel < expected_pixels; ++pixel) {
        const std::uint16_t raw_depth = depth.data[pixel];
        if (raw_depth == 0U) {
            continue;
        }
        const float distance_m = static_cast<float>(raw_depth) * depth.depth_scale_m;
        if (distance_m < min_distance_m || distance_m > max_distance_m) {
            continue;
        }
        const float linear = (distance_m - min_distance_m) / distance_range_m;
        const auto bin = static_cast<std::size_t>(linear * static_cast<float>(histogram_bins - 1U));
        const float normalized =
            static_cast<float>(histogram[bin]) / static_cast<float>(valid_pixels);
        // RealSense Viewer-style Jet palette in BGR order: dark blue, blue, cyan,
        // green, yellow, red, then dark red across the configured distance range.
        const float blue = 1.5F - std::abs((4.0F * normalized) - 1.0F);
        const float green = 1.5F - std::abs((4.0F * normalized) - 2.0F);
        const float red = 1.5F - std::abs((4.0F * normalized) - 3.0F);
        const std::size_t offset = pixel * 3U;
        visual.data[offset] = to_byte(blue);
        visual.data[offset + 1U] = to_byte(green);
        visual.data[offset + 2U] = to_byte(red);
    }
    return visual;
}

} // namespace perception::streaming
