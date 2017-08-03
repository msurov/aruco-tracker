#include "aruco_detector.h"

#include <tuple>
#include <opencv2/opencv.hpp>
#include "traces.h"


template <typename T>
inline T sign(T const& a)
{
    return a > static_cast<T>(0) ? static_cast<T>(1) : static_cast<T>(-1);
}

inline cv::Matx33f get_closest_rotmat(cv::Matx33f const& M)
{
    cv::Matx33f u, vt;
    cv::Vec3f w;
    cv::SVDecomp(M, w, u, vt);
    cv::Matx33f R = u * vt;
    return R;
}

inline cv::Vec4f rotmat_to_quat(cv::Matx33f const& R)
{
    float t = cv::trace(R);
    float r = sqrtf(1.f + t);
    float s = 0.5f / r;
    float w = 0.5f * r;
    float x = (R(2,1) - R(1,2)) * s;
    float y = (R(0,2) - R(2,0)) * s;
    float z = (R(1,0) - R(0,1)) * s;
    cv::Vec4f quat;
    return cv::Vec4f(w,x,y,z);
}

template <int n>
inline cv::Vec<float,n> get_col(cv::Matx<float,n,n> const& m, int c)
{
    cv::Vec<float,n> col;

    for (int i = 0; i < n; ++ i)
        col[i] = m(i, c);

    return col;
}

inline cv::Matx33f quat_to_rotmat(cv::Vec4f const& q)
{
    float w = q[0];
    float x = q[1];
    float y = q[2];
    float z = q[3];

    cv::Matx33f R(
        1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,       2*x*z + 2*y*w,
            2*x*y + 2*z*w, 1 - 2*x*x - 2*z*x,       2*y*z - 2*x*w,
            2*x*z - 2*y*w,     2*y*z + 2*x*w,   1 - 2*x*x - 2*y*y
    );

    return R;
}

inline std::tuple<cv::Vec3f, cv::Vec4f> decompose_homogeneous_mat(cv::Matx44f const& T)
{
    cv::Vec3f d(T(0, 3), T(1, 3), T(2, 3));
    cv::Matx33f R(
        T(0,0), T(0,1), T(0,2),
        T(1,0), T(1,1), T(1,2),
        T(2,0), T(2,1), T(2,2)
    );
    cv::Vec4f q = rotmat_to_quat(R);
    return std::make_tuple(d, q);
}

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

void ArucoDetector::find_markers(cv::Mat const& im, std::map<int, polygon_t>& markers)
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

void ArucoDetector::draw_found_markers(cv::Mat& plot, std::map<int, polygon_t> const& markers)
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

void ArucoDetector::draw_frame(cv::Mat& plot, cv::Vec3f const& p, cv::Vec4f q)
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
