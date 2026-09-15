#include "perception/camera/realsense_camera.hpp"
#include "perception/core/config.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <cstddef>
#include <iostream>

int main() {
    perception::camera::RealSenseCamera camera(perception::core::CameraConfig{});
    if (!camera.initialize() || !camera.start()) {
        std::cerr << "Unable to start RealSense camera.\n";
        return 1;
    }
    perception::camera::CameraFrameSet frames;
    while (camera.capture(frames)) {
        if (!frames.rgb.data.empty()) {
            cv::Mat rgb(frames.rgb.height, frames.rgb.width, CV_8UC3, frames.rgb.data.data(),
                        static_cast<std::size_t>(frames.rgb.stride_bytes));
            cv::imshow("RGB", rgb);
        }
        if (cv::waitKey(1) == 27) {
            break;
        }
    }
    camera.stop();
    return 0;
}
