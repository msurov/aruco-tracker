#pragma once

#include <vector>
#include <array>
#include <algorithm>
#include <assert.h>
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <cppmath/math.h>
#include "cvmath.h"


using Polygon = std::vector<cv::Point2f>;

#include <cppmisc/json.h>


struct MarkerLoc
{
    int id;
    cv::Matx<double,4,2> corners;
    cv::Matx<double,8,8> corners_cov;
};

struct MarkerPose
{
    int id;
    PoseCov pose;
    MarkerPose() : id(-1) {}
    MarkerPose(int id, PoseCov const& pose) : id(id), pose(pose) {}
};

class ArucoDetector;
using ArucoDetectorPtr = std::shared_ptr<ArucoDetector>;

class ArucoDetector
{
private:
    cv::Ptr<cv::aruco::Dictionary> _dict;
    int _img_side_to_scan;
    int _min_quad_diag;
    float _marker_side;
    double _reprojection_err_threshold;
    double _max_marker_view_angle;
    CameraIntrinsics _intr;
    cv::Matx<double,4,3> _obj_pts;

    cv::Ptr<cv::aruco::Dictionary> get_dict(std::string const& name);

public:
    ArucoDetector(std::string const& dict_name, float marker_side, CameraIntrinsics const& intr);
    void find_markers(cv::Mat const& gray, std::vector<MarkerLoc>& markers);
    bool get_marker_pose(MarkerLoc const& marker, PoseCov& pose);
    bool get_markers_poses(std::vector<MarkerLoc> const& markers, std::vector<MarkerPose>& poses);

    void draw_marker(cv::Mat& plot, MarkerLoc const& loc) const;
    void draw_markers(cv::Mat& plot, std::vector<MarkerLoc> const& locations) const;
    void draw_frame(cv::Mat& plot, MarkerPose const& pose) const;
    void draw_frames(cv::Mat& plot, std::vector<MarkerPose> const& poses) const;
};


static void json_parse(Json::Value const& json, CameraIntrinsics& intr)
{
    std::array<float,9> k;
    json_parse(json_get(json, "K"), k);
    cv::Matx33f& K = intr.K;
    for (int y = 0; y < 3; ++ y)
    {
        for (int x = 0; x < 3; ++ x)
            K(y,x) = k[x + y * 3];
    }

    json_parse(json_get(json, "distortion"), intr.distortion);

    std::array<int,2> resolution;
    json_parse(json_get(json, "resolution"), resolution);
    intr.resolution = cv::Size2i(resolution[0], resolution[1]);
}

ArucoDetectorPtr create_aruco_detector(Json::Value const& cfg);
