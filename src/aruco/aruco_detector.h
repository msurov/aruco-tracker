#pragma once

#include <map>
#include <memory>
#include <stdexcept>
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <misc/json.h>


typedef std::vector<cv::Point2f> polygon_t;


struct CameraIntrinsics
{
    cv::Matx33f K;
    std::vector<float> distortion;
    cv::Size2i resolution;
};


class ArucoDetector
{
private:
    cv::Ptr<cv::aruco::Dictionary> m_dict;
    float m_pattern_side;
    polygon_t m_aligned_pattern;
    CameraIntrinsics m_intrinsics;
    std::vector<cv::Vec3f> m_obj_points;

public:
    ArucoDetector(std::string const& dict_name, float side, CameraIntrinsics intrinsics);
    void set_intrinsics(CameraIntrinsics intrinsics);
    bool get_marker_pose(polygon_t const& polygon, cv::Vec3f& displacement, cv::Vec4f& quaternion) const;
    void locate_markers(cv::Mat const& im, std::map<int, polygon_t>& markers) const;
    void draw_frame(cv::Mat& plot, cv::Vec3f const& p, cv::Vec4f q) const;
    void draw_markers(cv::Mat& plot, std::map<int, polygon_t> const& markers) const;
    void draw_marker(cv::Mat& plot, polygon_t const& marker) const;
};

std::unique_ptr<ArucoDetector> get_aruco_detector(Json::Value const& cfg);
void refine_quad(cv::Mat const& im, polygon_t& quad, bool dbg);
