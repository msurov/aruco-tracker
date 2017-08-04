#pragma once

#include <map>
#include <memory>
#include <stdexcept>
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include "jsonxx.h"


typedef std::vector<cv::Point2f> polygon_t;

struct cam_intrinsics_t
{
    cv::Matx33f K;
    std::vector<float> distortion;
};

class ArucoDetector
{
private:
    cv::Ptr<cv::aruco::Dictionary> m_dict;
    float m_pattern_side;
    polygon_t m_aligned_pattern;
    cam_intrinsics_t m_intrinsics;

public:
    ArucoDetector(int nrows, float side, cam_intrinsics_t const& intrinsics);
    void get_marker_3d_coords(polygon_t const& polygon, cv::Vec3f& displacement, cv::Vec4f& quaternion) const;
    void find_markers(cv::Mat const& im, std::map<int, polygon_t>& markers) const;
    void draw_frame(cv::Mat& plot, cv::Vec3f const& p, cv::Vec4f q) const;
    void draw_found_markers(cv::Mat& plot, std::map<int, polygon_t> const& markers) const;
};

std::unique_ptr<ArucoDetector> get_aruco_detector(jsonxx::Object const& cfg);
