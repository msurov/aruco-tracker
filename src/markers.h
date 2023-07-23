#pragma once
#include "cvmath.h"
#include "transforms.h"


struct Marker
{
    static const uint32_t CornersFound = 1 << 0;
    static const uint32_t PoseFound = 1 << 1;
    static const uint32_t TooAcuteViewAngle = 1 << 8;
    static const uint32_t IncorrectVertices = 1 << 9;
    static const uint32_t TooSmall = 1 << 10;
    static const uint32_t BigReprojectionError = 1 << 11;
    static const uint32_t FittingFailed = 1 << 12;

    int id = -1;
    uint32_t flags = 0;
    cv::Matx<double, 4, 2> corners;
    cv::Matx<double, 8, 8> corners_cov;
    double reprojection_error = -1.;
    PoseCov camera_marker_pose;
    Pose world_marker_pose;
};
