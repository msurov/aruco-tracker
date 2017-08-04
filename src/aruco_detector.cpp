#include "aruco_detector.h"

#include <tuple>
#include <opencv2/opencv.hpp>
#include "traces.h"
#include "transforms.h"


template <int Ny, int Nx>
cv::Matx<double, Ny, Nx> json_get_matx(jsonxx::Object const& cfg, std::string const& name)
{
    auto json_A = json_get_vector<double>(cfg, name);
    cv::Matx33d A;

    if (json_A.size() != Nx * Ny)
        throw std::runtime_error("expected array " + name + " len is " + std::to_string(Nx * Ny));

    for (int y = 0; y < Ny; ++y)
    {
        for (int x = 0; x < Nx; ++x)
        {
            A(y, x) = json_A[x + Nx * y];
        }
    }

    return A;
}

std::unique_ptr<ArucoDetector> get_aruco_detector(jsonxx::Object const& cfg)
{
    auto cam = json_get<jsonxx::Object>(cfg, "intrinsics");
    auto K = json_get_matx<3, 3>(cam, "K");
    auto distortion = json_get_vector<float>(cam, "distortion");

    auto marker = json_get<jsonxx::Object>(cfg, "marker");
    int nrows = json_get<jsonxx::Number>(marker, "nrows");
    float side = json_get<jsonxx::Number>(marker, "side");

    cam_intrinsics_t intrinsics = {K, distortion};
    return std::unique_ptr<ArucoDetector>(new ArucoDetector(nrows, side, intrinsics));
}


ArucoDetector::ArucoDetector(int nrows, float side, cam_intrinsics_t const& intrinsics)
{
    int v =
        nrows == 4 ? cv::aruco::DICT_4X4_250 :
        nrows == 5 ? cv::aruco::DICT_5X5_250 :
        nrows == 6 ? cv::aruco::DICT_6X6_250 : -1;

    if (v == -1)
        throw std::runtime_error("unknown argument");

    m_dict = cv::aruco::getPredefinedDictionary(v);
    m_pattern_side = side;
    m_intrinsics = intrinsics;
    m_aligned_pattern.push_back(cv::Point2f(0, side));
    m_aligned_pattern.push_back(cv::Point2f(side, side));
    m_aligned_pattern.push_back(cv::Point2f(side, 0));
    m_aligned_pattern.push_back(cv::Point2f(0, 0));
}

void ArucoDetector::get_marker_3d_coords(polygon_t const& polygon, cv::Vec3f& displacement, cv::Vec4f& quaternion) const
{
    assert(polygon.size() == 4);

    if (true)
    {
        std::vector<cv::Vec3d> rvec_arr(1);
        std::vector<cv::Vec3d> tvec_arr(1);
        std::vector<polygon_t> polygon_arr = {polygon};

        cv::aruco::estimatePoseSingleMarkers(polygon_arr, m_pattern_side, m_intrinsics.K, m_intrinsics.distortion, rvec_arr, tvec_arr);

        displacement = tvec_arr[0];
        quaternion = rodrigues_to_quat(rvec_arr[0]);
    }
    else
    {
        std::vector<cv::Point2f> polygon_normed;

        cv::undistortPoints(polygon, polygon_normed, m_intrinsics.K, m_intrinsics.distortion);
        cv::Matx33f H = cv::findHomography(m_aligned_pattern, polygon_normed);

        auto c1 = H.col(0);
        auto c2 = H.col(1);

        float c1_norm = sqrtf(c1.dot(c1));
        float c2_norm = sqrtf(c2.dot(c2));
        float k = -2.f / (c1_norm + c2_norm);

        cv::Matx33f P = H * k * sign(cv::determinant(H));
        cv::Point3f eu = cv::Point3f(P(0,0), P(1,0), P(2,0));
        cv::Point3f ev = cv::Point3f(P(0,1), P(1,1), P(2,1));
        cv::Point3f ew = eu.cross(ev);
        cv::Matx33f R = cv::Matx33f(
            eu.x, ev.x, ew.x,
            eu.y, ev.y, ew.y,
            eu.z, ev.z, ew.z
        );
        R = get_closest_rotmat(R);

        quaternion = rotmat_to_quat(R);

        displacement[0] = P(0,2);
        displacement[1] = P(1,2);
        displacement[2] = P(2,2);
    }
}

void ArucoDetector::find_markers(cv::Mat const& im, std::map<int, polygon_t>& markers) const
{
    std::vector<int> ids;
    std::vector<polygon_t> polygons;
    cv::Mat blurred;
    cv::GaussianBlur(im, blurred, cv::Size(15,15), 1.);
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    parameters->doCornerRefinement = true;
    parameters->cornerRefinementWinSize = 7;
    parameters->cornerRefinementMaxIterations = 30;
    parameters->cornerRefinementMinAccuracy = 0.1;
    cv::aruco::detectMarkers(blurred, m_dict, polygons, ids, parameters);

    // TODO cornersubpix

    for (int i = 0; i < (int)ids.size(); ++ i)
    {
        int id = ids[i];
        polygon_t& polygon = polygons[i];
        markers[id] = polygon;
    }
}

void ArucoDetector::draw_found_markers(cv::Mat& plot, std::map<int, polygon_t> const& markers) const
{
    for (auto const& it : markers)
    {
        polygon_t const& p = it.second;
        for (int i = 0; i < 4; ++ i)
        {
            int j = (i + 1) % 4;
            cv::line(plot, cv::Point2i(p[i]), cv::Point2i(p[j]), cv::Scalar(255, 0, 0), 1);
        }
    }
}

void ArucoDetector::draw_frame(cv::Mat& plot, cv::Vec3f const& p, cv::Vec4f q) const
{
    if (true)
    {
        cv::Vec3d tvec = p;
        cv::Vec3d rvec = quat_to_rodrigues(q);
        cv::aruco::drawAxis(plot, m_intrinsics.K, m_intrinsics.distortion, rvec, tvec, m_pattern_side);
    }
    else
    {
        cv::Matx33f R = quat_to_rotmat(q);
        cv::Vec3f eu = get_col(R, 0);
        cv::Vec3f ev = get_col(R, 1);
        cv::Vec3f ew = get_col(R, 2);

        std::vector<cv::Vec3f> scene_pts = {
            p, p + eu * 0.08f, p + ev * 0.08f, p + ew * 0.08f
        };
        std::vector<cv::Vec2f> im_pts(4);

        cv::Vec3f rvec(0,0,0);
        cv::Vec3f tvec(0,0,0);
        cv::projectPoints(scene_pts, rvec, tvec, m_intrinsics.K, m_intrinsics.distortion, im_pts);

        cv::line(plot, cv::Point2i(im_pts[0]), cv::Point2i(im_pts[1]), cv::Scalar(255, 0, 0), 1);
        cv::line(plot, cv::Point2i(im_pts[0]), cv::Point2i(im_pts[2]), cv::Scalar(255, 0, 0), 1);
        cv::line(plot, cv::Point2i(im_pts[0]), cv::Point2i(im_pts[3]), cv::Scalar(255, 0, 0), 1);
    }
}
