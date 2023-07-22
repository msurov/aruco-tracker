#include "aruco_detector.h"
#include "imgdump.h"
#include <cppmisc/timing.h>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <cppmisc/traces.h>
#include "edges_fitting.h"
#include "pnp_4pts.h"
#include "cvmath.h"
#include "json_loaders.h"


ArucoDetector::DictionaryPtr ArucoDetector::get_dict(std::string const& name)
{
    std::map<std::string, cv::aruco::PREDEFINED_DICTIONARY_NAME> dicts = {
        {"4x4x50", cv::aruco::DICT_4X4_50},
        {"4x4x100", cv::aruco::DICT_4X4_100},
        {"4x4x250", cv::aruco::DICT_4X4_250},
        {"4x4x1000", cv::aruco::DICT_4X4_1000},
        {"5x5x50", cv::aruco::DICT_5X5_50},
        {"5x5x100", cv::aruco::DICT_5X5_100},
        {"5x5x250", cv::aruco::DICT_5X5_250},
        {"5x5x1000", cv::aruco::DICT_5X5_1000},
        {"6x6x50", cv::aruco::DICT_6X6_50},
        {"6x6x100", cv::aruco::DICT_6X6_100},
        {"6x6x250", cv::aruco::DICT_6X6_250},
        {"6x6x1000", cv::aruco::DICT_6X6_1000},
        {"7x7x50", cv::aruco::DICT_7X7_50},
        {"7x7x100", cv::aruco::DICT_7X7_100},
        {"7x7x250", cv::aruco::DICT_7X7_250},
        {"7x7x1000", cv::aruco::DICT_7X7_1000},
        {"orig", cv::aruco::DICT_ARUCO_ORIGINAL},
    };
    auto p = dicts.find(name);
    if (p == dicts.end())
        throw_runtime_error("invalid aruco dict name: ", name);

    return cv::aruco::getPredefinedDictionary(p->second);
}

ArucoDetector::ArucoDetector(
    std::string const& dict_name,
    double marker_side_meters,
    CameraIntrinsics const& intr,
    Pose const& cam_pose
    )
{
    _dict = get_dict(dict_name);
    _img_side_to_scan = 1024;
    _min_quad_diag = 15;
    _intr = intr;
    _marker_side = marker_side_meters;
    _obj_pts = Mat43d(
        0.f, _marker_side, 0.f,
        _marker_side, _marker_side, 0.f,
        _marker_side, 0.f, 0.f,
        0.f, 0.f, 0.f
    );
    _reprojection_err_threshold = 1.0;
    _max_marker_view_angle = 60 * M_PI / 180;
    _pose_world_cam = cam_pose;
}

inline double dist(cv::Vec2d const& a, cv::Vec2d const& b)
{
    cv::Vec2d d = a - b;
    return sqrtf(d.dot(d));
}

inline double quad_min_diag(cv::Matx<double,4,2> const& quad)
{
    cv::Vec2d p1(quad(0,0), quad(0,1));
    cv::Vec2d p2(quad(1,0), quad(1,1));
    cv::Vec2d p3(quad(2,0), quad(2,1));
    cv::Vec2d p4(quad(3,0), quad(3,1));
    double d1 = dist(p1, p3);
    double d2 = dist(p2, p4);
    return std::min(d1, d2);
}

void ArucoDetector::find_markers(cv::Mat const& gray, std::vector<MarkerLoc>& markers)
{
    // TODO: pyrdown within a loop
    cv::Mat blurred = gray;

    const float scale = 1.f * gray.rows / blurred.rows;
    const cv::Point2f _05(0.5f, 0.5f);

    // find corners
    auto parameters = cv::aruco::DetectorParameters::create();
    parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;
    std::vector<int> ids;
    std::vector<Polygon> polygons;
    cv::aruco::detectMarkers(blurred, _dict, polygons, ids, parameters);

    markers.reserve(ids.size());

    // refine corners
    for (int i = 0; i < int(ids.size()); ++i)
    {
        Polygon const& polygon = polygons[i];
        int id = ids[i];

        if (polygon.size() != 4)
        {
            warn_msg("opencv gave marker with incorrect number of vertices");
            continue;
        }

        cv::Matx<double,4,2> quad;
        for (int v = 0; v < 4; ++v)
        {
            cv::Point2d corner = scale * (polygon[v] + _05) - _05;
            quad(v, 0) = corner.x;
            quad(v, 1) = corner.y;
        }

        double diag = quad_min_diag(quad);
        if (diag < 15.0)
        {
            info_msg("skipped marker ", id, "; diag is less than 15px");
            continue;
        }

        MarkerLoc marker;
        bool ok = refine_quad(gray, quad, 25, marker.corners, marker.corners_cov);
        if (!ok)
        {
            info_msg("can't fit marker ", id);
            continue;
        }
        marker.id = id;
        markers.push_back(marker);
    }
}

inline double angle_between(cv::Vec3d const& a, cv::Vec3d const& b)
{
    return std::acos(a.dot(b) / std::sqrt(a.dot(a) * b.dot(b)));
}

bool ArucoDetector::get_marker_pose(MarkerLoc const& marker, PoseCov& pose)
{
    bool ok = solve_pnp_4pts(_obj_pts, marker.corners, 
        marker.corners_cov, _intr, pose.r, pose.p, pose.cov);
    if (!ok)
    {
        err_msg("solving pnp failed");
        return false;
    }
    double err = reprojection_error(_obj_pts, marker.corners, _intr, pose.r, pose.p);
    if (err > _reprojection_err_threshold)
    {
        dbg_msg("solution pnp for marker ", marker.id, " has too big error: ", err);
        return false;
    }
    const auto R = rotmat(pose.r);
    cv::Vec3d ez {R(0,2), R(1,2), R(2,2)};
    double angle = angle_between(ez, -pose.p);
    if (angle > _max_marker_view_angle)
    {
        dbg_msg("angle of view of marker ", marker.id, " is too big: ", angle);
        return false;
    }

    return true;
}

bool ArucoDetector::get_markers_poses(
    std::vector<MarkerLoc> const& markers, 
    std::vector<MarkerPose>& poses)
{
    bool result = true;
    const int n = markers.size();
    poses.reserve(n);
    poses.resize(0);

    for (int i = 0; i < n; ++ i)
    {
        MarkerLoc const& marker = markers[i];
        PoseCov pose;
        bool ans = get_marker_pose(marker, pose);
        if (ans)
            poses.emplace_back(marker.id, compose(_pose_world_cam, pose));
        else
            result = false;
    }

    return result;
}

ArucoDetectorPtr create_aruco_detector(Json::Value const& cfg)
{
    const double marker_side = json_get<double>(cfg, "marker_side");
    const auto markers_dict = json_get<std::string>(cfg, "markers_dict");
    const auto cam_intrinsics = json_get<CameraIntrinsics>(cfg, "camera_intrinsics");
    const auto cam_pose = json_get<Pose>(cfg, "camera_pose");
    return std::make_shared<ArucoDetector>(
        markers_dict, marker_side, cam_intrinsics, cam_pose
    );
}

void ArucoDetector::draw_marker(cv::Mat& plot, MarkerLoc const& loc) const
{
    for (int i = 0; i < 4; ++ i)
    {
        int j = (i + 1) % 4;
        cv::Vec2d v1 = row_as_vec(loc.corners, i);
        cv::Vec2d v2 = row_as_vec(loc.corners, j);
        cv::line(plot, round(v1), round(v2), cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    }

    cv::Vec2i center = round(row_as_vec(loc.corners, 3));
    cv::circle(plot, center, 5, cv::Scalar(0, 255, 0), -1);
    auto s = std::to_string(loc.id);
    cv::Vec2i textpos = center + cv::Vec2i(50, -50);
    cv::putText(plot, s, textpos, 
        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1, cv::LINE_AA);
}

void ArucoDetector::draw_markers(cv::Mat& plot, std::vector<MarkerLoc> const& locations) const
{
    for (MarkerLoc const& loc : locations)
    {
        draw_marker(plot, loc);
    }
}

void ArucoDetector::draw_frame(cv::Mat& plot, MarkerPose const& pose) const
{
    cv::Vec3d tvec = pose.pose.p;
    cv::Vec3d rvec = pose.pose.r;

    std::vector<cv::Point3f> objorths = {
        {0.f, 0.f, 0.f},
        {float(_marker_side / 2), 0.f, 0.f},
        {0.f, float(_marker_side / 2), 0.f},
        {0.f, 0.f, float(_marker_side / 2)}
    };
    std::vector<cv::Point2f> impts(4);

    cv::projectPoints(objorths, rvec, tvec, _intr.K, _intr.distortion, impts);
    cv::arrowedLine(plot, impts[0], impts[1], cv::Scalar(255,0,0), 2, cv::LINE_AA);
    cv::arrowedLine(plot, impts[0], impts[2], cv::Scalar(0,255,0), 2, cv::LINE_AA);
    cv::arrowedLine(plot, impts[0], impts[3], cv::Scalar(0,0,255), 2, cv::LINE_AA);

    char buf[64];
    sprintf(buf,
        "p = %.3f %.3f %.3f", 
        tvec[0], tvec[1], tvec[2]
    );
    cv::Point2f textpos = impts[0] + cv::Point2f(50, 30);
    cv::putText(plot, buf, round(textpos), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1, cv::LINE_AA);

    sprintf(buf,
        "r = %.3f %.3f %.3f", 
        rvec[0], rvec[1], rvec[2]
    );
    textpos = impts[0] + cv::Point2f(50, 50);
    cv::putText(plot, buf, round(textpos), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1, cv::LINE_AA);
}

void ArucoDetector::draw_frames(cv::Mat& plot, std::vector<MarkerPose> const& poses) const
{
    for (MarkerPose const& pose : poses)
    {
        draw_frame(plot, pose);
    }
}

