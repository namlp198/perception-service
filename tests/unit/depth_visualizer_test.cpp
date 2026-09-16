#include "test_support.hpp"

#include "perception/streaming/depth_visualizer.hpp"

auto depth_visualizer_test() -> bool {
    perception::camera::DepthFrame depth;
    depth.sensor_timestamp_ns = 10;
    depth.capture_timestamp_ns = 20;
    depth.frame_number = 30;
    depth.width = 4;
    depth.height = 1;
    depth.depth_scale_m = 0.001F;
    depth.data = {0U, 500U, 1'250U, 2'000U};

    const auto visual = perception::streaming::colorize_depth(depth, 0.5F, 2.0F);
    CHECK_TRUE(visual.sensor_timestamp_ns == depth.sensor_timestamp_ns);
    CHECK_TRUE(visual.capture_timestamp_ns == depth.capture_timestamp_ns);
    CHECK_TRUE(visual.frame_number == depth.frame_number);
    CHECK_TRUE(visual.width == 4);
    CHECK_TRUE(visual.height == 1);
    CHECK_TRUE(visual.stride_bytes == 12);
    CHECK_TRUE(visual.format == perception::camera::PixelFormat::Bgr8);
    CHECK_TRUE(visual.data.size() == 12);
    CHECK_TRUE(visual.data[0] == 0U && visual.data[1] == 0U && visual.data[2] == 0U);
    CHECK_TRUE(visual.data[3] > visual.data[5]);
    CHECK_TRUE(visual.data[6] > 0U || visual.data[7] > 0U || visual.data[8] > 0U);
    CHECK_TRUE(visual.data[9] == 0U && visual.data[10] == 0U && visual.data[11] > 0U);
    return true;
}
