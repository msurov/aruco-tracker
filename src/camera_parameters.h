#pragma once

#include <opencv2/core.hpp>

struct CameraIntrinsics
{
    cv::Matx33f K;
    std::vector<float> distortion;
    cv::Size2i resolution;
};
