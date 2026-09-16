#pragma once

#include "perception/camera/camera_types.hpp"

namespace perception::streaming {

[[nodiscard]] auto colorize_depth(const camera::DepthFrame& depth, float min_distance_m,
                                  float max_distance_m) -> camera::ImageFrame;

} // namespace perception::streaming
