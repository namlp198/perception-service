#include "perception/camera/realsense_camera.hpp"
#include "perception/core/config.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <cstddef>
#include <iostream>

namespace {

void show_image(const char* name, const perception::camera::ImageFrame& frame, int type) {
    if (frame.data.empty()) {
        return;
    }
    const cv::Mat image(frame.height, frame.width, type,
                        const_cast<std::uint8_t*>(frame.data.data()),
                        static_cast<std::size_t>(frame.stride_bytes));
    cv::imshow(name, image);
}

void show_depth(const perception::camera::DepthFrame& frame) {
    if (frame.data.empty()) {
        return;
    }
    const cv::Mat depth(frame.height, frame.width, CV_16UC1,
                        const_cast<std::uint16_t*>(frame.data.data()));
    cv::Mat normalized;
    cv::Mat colorized;
    cv::normalize(depth, normalized, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::applyColorMap(normalized, colorized, cv::COLORMAP_TURBO);
    cv::imshow("Depth visual", colorized);
}

} // namespace

int main() {
    perception::camera::RealSenseCamera camera(perception::core::CameraConfig{});
    if (!camera.initialize() || !camera.start()) {
        std::cerr << "Unable to start RealSense camera.\n";
        return 1;
    }
    perception::camera::CameraFrameSet frames;
    while (camera.capture(frames)) {
        show_image("RGB", frames.rgb, CV_8UC3);
        show_image("IR Left", frames.ir_left, CV_8UC1);
        show_image("IR Right", frames.ir_right, CV_8UC1);
        show_depth(frames.depth);
        if (cv::waitKey(1) == 27) {
            break;
        }
    }
    camera.stop();
    return 0;
}
