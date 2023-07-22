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


using Polygon = std::vector<cv::Point2f>;

struct MarkerLoc
{
    int id;
    cv::Matx<double,4,2> corners;
    cv::Matx<double,8,8> corners_cov;
};

struct MarkerPose
{
    int id;
    Pose pose;
    MarkerPose() : id(-1) {}
    MarkerPose(int id, Pose const& pose) : id(id), pose(pose) {}
};

class ArucoDetector;
using ArucoDetectorPtr = std::shared_ptr<ArucoDetector>;

class ArucoDetector
{
private:
    using DictionaryPtr = cv::Ptr<cv::aruco::Dictionary>;
    using Mat43d = cv::Matx<double, 4, 3>;

    DictionaryPtr _dict;
    int _img_side_to_scan;
    int _min_quad_diag;
    double _marker_side;
    double _reprojection_err_threshold;
    double _max_marker_view_angle;
    CameraIntrinsics _intr;
    Mat43d _obj_pts;
    Pose _pose_world_cam;

    DictionaryPtr get_dict(std::string const& name);

public:
    ArucoDetector(
        std::string const& dict_name,
        double marker_side,
        CameraIntrinsics const& intr,
        Pose const& cam_pose
    );
    void find_markers(cv::Mat const& gray, std::vector<MarkerLoc>& markers);
    bool get_marker_pose(MarkerLoc const& marker, PoseCov& pose);
    bool get_markers_poses(std::vector<MarkerLoc> const& markers, std::vector<MarkerPose>& poses);

    void draw_marker(cv::Mat& plot, MarkerLoc const& loc) const;
    void draw_markers(cv::Mat& plot, std::vector<MarkerLoc> const& locations) const;
    void draw_frame(cv::Mat& plot, MarkerPose const& pose) const;
    void draw_frames(cv::Mat& plot, std::vector<MarkerPose> const& poses) const;
};

ArucoDetectorPtr create_aruco_detector(Json::Value const& cfg);
