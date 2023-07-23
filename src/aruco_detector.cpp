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
    _world_camera_pose = cam_pose;

    _detector_parameters = cv::aruco::DetectorParameters::create();
    _detector_parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;
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

void ArucoDetector::find_markers(cv::Mat const& gray, std::vector<Marker>& markers)
{
    // TODO: pyrdown within a loop
    cv::Mat blurred = gray;

    const float scale = 1.f * gray.rows / blurred.rows;
    const cv::Point2f _05(0.5f, 0.5f);

    // find corners
    std::vector<int> ids;
    std::vector<Polygon> polygons;
    cv::aruco::detectMarkers(blurred, _dict, polygons, ids, _detector_parameters);

    markers.resize(ids.size());

    // refine corners
    for (int i = 0; i < int(ids.size()); ++i)
    {
        Polygon const& polygon = polygons[i];
        auto& marker = markers[i];
        marker.id = ids[i];

        marker.flags &= ~Marker::CornersFound;

        if (polygon.size() != 4)
        {
            marker.flags |= Marker::IncorrectVertices;
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
            marker.flags |= Marker::TooSmall;
            continue;
        }

        bool ok = refine_quad(gray, quad, 25, marker.corners, marker.corners_cov);
        if (!ok)
        {
            marker.flags |= Marker::FittingFailed;
            continue;
        }

        marker.flags |= Marker::CornersFound;
    
        if (get_marker_pose(marker))
            marker.world_marker_pose = compose(_world_camera_pose, marker.camera_marker_pose);
    }
}

inline double angle_between(cv::Vec3d const& a, cv::Vec3d const& b)
{
    return std::acos(a.dot(b) / std::sqrt(a.dot(a) * b.dot(b)));
}

bool ArucoDetector::get_marker_pose(Marker& marker)
{
    auto& p = marker.camera_marker_pose.p;
    auto& r = marker.camera_marker_pose.r;
    auto& cov = marker.camera_marker_pose.cov;

    marker.flags &= ~Marker::PoseFound;

    bool ok = solve_pnp_4pts(_obj_pts, marker.corners, marker.corners_cov, _intr, r, p, cov);
    if (!ok)
        return false;

    marker.reprojection_error = reprojection_error(_obj_pts, marker.corners, _intr, r, p);
    if (marker.reprojection_error > _reprojection_err_threshold)
        marker.flags |= Marker::BigReprojectionError;

    const auto R = rotmat(r);
    const cv::Vec3d ez {R(0,2), R(1,2), R(2,2)};
    const double angle = angle_between(ez, -p);
    if (angle > _max_marker_view_angle)
        marker.flags |= Marker::TooAcuteViewAngle;
    
    marker.flags |= Marker::PoseFound;
    return true;
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

void ArucoDetector::draw_marker(cv::Mat& plot, Marker const& marker) const
{
    for (int i = 0; i < 4; ++ i)
    {
        int j = (i + 1) % 4;
        cv::Vec2d v1 = row_as_vec(marker.corners, i);
        cv::Vec2d v2 = row_as_vec(marker.corners, j);
        cv::line(plot, round(v1), round(v2), cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    }

    cv::Vec2i center = round(row_as_vec(marker.corners, 3));
    cv::circle(plot, center, 5, cv::Scalar(0, 255, 0), -1);
    auto s = std::to_string(marker.id);
    cv::Vec2i textpos = center + cv::Vec2i(50, -50);
    cv::putText(plot, s, textpos, 
        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1, cv::LINE_AA);
}

void ArucoDetector::draw_markers(cv::Mat& plot, std::vector<Marker> const& markers) const
{
    for (Marker const& marker : markers)
        draw_marker(plot, marker);
}

void ArucoDetector::draw_frame(cv::Mat& plot, Marker const& marker) const
{
    cv::Vec3d tvec = marker.camera_marker_pose.p;
    cv::Vec3d rvec = marker.camera_marker_pose.r;

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

void ArucoDetector::draw_frames(cv::Mat& plot, std::vector<Marker> const& markers) const
{
    for (Marker const& marker : markers)
        draw_frame(plot, marker);
}

