#pragma once

#include <vector>
#include <array>
#include <algorithm>
#include <assert.h>

#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "json_loaders.h"
#include "cvmath.h"
#include "camera_parameters.h"
#include "transforms.h"
#include "markers.h"

class ArucoDetector;
using ArucoDetectorPtr = std::shared_ptr<ArucoDetector>;
using Polygon = std::vector<cv::Point2f>;


class ArucoDetector
{
private:
    using DictionaryPtr = cv::Ptr<cv::aruco::Dictionary>;
    using DetectorParametersPtr = cv::Ptr<cv::aruco::DetectorParameters>;
    using Mat43d = cv::Matx<double, 4, 3>;

    DictionaryPtr _dict;
    int _img_side_to_scan;
    int _min_quad_diag;
    double _marker_side;
    double _reprojection_err_threshold;
    double _max_marker_view_angle;
    CameraIntrinsics _intr;
    Mat43d _obj_pts;
    Pose _world_camera_pose;
    DetectorParametersPtr _detector_parameters;
    DictionaryPtr get_dict(std::string const& name);

public:
    ArucoDetector(
        std::string const& dict_name,
        double marker_side,
        CameraIntrinsics const& intr,
        Pose const& cam_pose
    );
    void find_markers(cv::Mat const& gray, std::vector<Marker>& markers);
    bool get_marker_pose(Marker& marker);

    void draw_marker(cv::Mat& plot, Marker const& marker) const;
    void draw_markers(cv::Mat& plot, std::vector<Marker> const& markers) const;
    void draw_frame(cv::Mat& plot, Marker const& marker) const;
    void draw_frames(cv::Mat& plot, std::vector<Marker> const& markers) const;
};

ArucoDetectorPtr create_aruco_detector(Json::Value const& cfg);
