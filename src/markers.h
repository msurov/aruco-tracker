#pragma once
#include "cvmath.h"
#include "transforms.h"


enum MarkerStatus : uint32_t
{
    Fine = 0,

    TooAcuteViewAngle = 1 << 1,
    IncorrectVertices = 1 << 2,
    FittingFailed = 1 << 3,
    TooSmall = 1 << 4,

    PoseEstimationFailed = 1 << 16,
    BigReprojectionError = 1 << 17,
};

inline MarkerStatus operator | (MarkerStatus const& a, MarkerStatus const& b)
{
    return static_cast<MarkerStatus>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}

inline MarkerStatus operator |= (MarkerStatus& a, MarkerStatus b)
{
    a = a | b;
    return a;
}

struct Marker
{
    int id = -1;
    MarkerStatus status = Fine;
    cv::Matx<double, 4, 2> corners;
    cv::Matx<double, 8, 8> corners_cov;
    double reprojection_error = -1.;
    PoseCov camera_marker_pose;
    Pose world_marker_pose;
};
