#include "aruco_detector.h"

#include <tuple>
#include <opencv2/opencv.hpp>
#include <misc/traces.h>
#include <misc/json.h>
#include <mmath/transforms.h>


using namespace std;
using namespace cv;


template <int Ny, int Nx>
cv::Matx<double, Ny, Nx> json_get_matx(Json::Value const& cfg, std::string const& name)
{
    auto json_A = json_get_vec<double>(cfg, name);
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

unique_ptr<ArucoDetector> get_aruco_detector(Json::Value const& cfg)
{
    auto const& marker = json_get<Json::Value>(cfg, "marker");
    string dict_type = json_get<std::string>(marker, "dict_type");
    double side = json_get<double>(marker, "side");

    auto const& json_intr = json_get<Json::Value>(cfg, "intrinsics");
    CameraIntrinsics intr;
    intr.K = json_get_matx<3,3>(json_intr, "K");
    intr.distortion = json_get_vec<float>(json_intr, "distortion");

    return unique_ptr<ArucoDetector>(new ArucoDetector(dict_type, side, intr));
}

inline Ptr<aruco::Dictionary> get_dict(string const& name)
{
    map<string, aruco::PREDEFINED_DICTIONARY_NAME> dicts = {
        {"4x4x50", aruco::DICT_4X4_50},
        {"4x4x100", aruco::DICT_4X4_100},
        {"4x4x250", aruco::DICT_4X4_250},
        {"4x4x1000", aruco::DICT_4X4_1000},
        {"5x5x50", aruco::DICT_5X5_50},
        {"5x5x100", aruco::DICT_5X5_100},
        {"5x5x250", aruco::DICT_5X5_250},
        {"5x5x1000", aruco::DICT_5X5_1000},
        {"6x6x50", aruco::DICT_6X6_50},
        {"6x6x100", aruco::DICT_6X6_100},
        {"6x6x250", aruco::DICT_6X6_250},
        {"6x6x1000", aruco::DICT_6X6_1000},
        {"7x7x50", aruco::DICT_7X7_50},
        {"7x7x100", aruco::DICT_7X7_100},
        {"7x7x250", aruco::DICT_7X7_250},
        {"7x7x1000", aruco::DICT_7X7_1000},
        {"orig", aruco::DICT_ARUCO_ORIGINAL},
    };
    auto p = dicts.find(name);
    if (p == dicts.end())
        throw runtime_error("invalid aruco dict name: " + string(name));

    return aruco::getPredefinedDictionary(p->second);
}

ArucoDetector::ArucoDetector(string const& dict_name, float side /* meters */, CameraIntrinsics intrinsics)
{
    m_dict = get_dict(dict_name);
    m_pattern_side = side;
    m_intrinsics = intrinsics;

    vector<Vec3f> obj_points = {
        Vec3f(0, side, 0),
        Vec3f(side, side, 0),
        Vec3f(side, 0, 0),
        Vec3f(0, 0, 0),
    };
    m_obj_points = obj_points;
}

void polygon_scale(polygon_t& polygon, float scale)
{
    for (auto& pt : polygon)
        pt *= scale;
}

void ArucoDetector::set_intrinsics(CameraIntrinsics intrinsics)
{
    m_intrinsics = intrinsics;
}

inline float pt2f_len(Point2f const& p)
{
    return sqrtf(p.dot(p));
}

inline float quad_min_diag(polygon_t const& p)
{
    float d1 = pt2f_len(p[0] - p[2]);
    float d2 = pt2f_len(p[1] - p[3]);
    return std::min(d1, d2);
}

struct LineSeg {
    Point2f a, b;
    LineSeg(Point2f const& a, Point2f const& b) : a(a), b(b) {}
    inline Point2f at(float t) const { return a + t * (b - a); }
};

inline float dist(Point2f const& a, Point2f const& b)
{
    Point2f d = b - a;
    return sqrtf(d.dot(d));
}

inline float im_at(Mat const& im, float x, float y)
{
    assert(im.type() == CV_8U);
    assert(x >= 0.f && x <= float(im.cols - 1));
    assert(y >= 0.f && y <= float(im.rows - 1));

    int x0 = int(x);
    int y0 = int(y);
    float dx = x - x0;
    float dy = y - y0;

    uchar u00 = im.at<uchar>(y0, x0);
    uchar u10 = im.at<uchar>(y0 + 1, x0);
    uchar u01 = im.at<uchar>(y0, x0 + 1);
    uchar u11 = im.at<uchar>(y0 + 1, x0 + 1);

    float a = u00;
    float b = u01 - a;
    float c = u10 - a;
    float d = u00 + u11 - u01 - u10;

    return a + b * dx + c * dy + d * dx * dy;
}

inline float im_at(Mat const& im, Point2f const& p)
{
    return im_at(im, p.x, p.y);
}

bool ArucoDetector::get_marker_pose(polygon_t const& polygon, Vec3f& displacement, Vec4f& quaternion) const
{
    assert(polygon.size() == 4);
    Vec3d rvec;
    Vec3d tvec;

    bool b = solvePnP(m_obj_points, polygon, m_intrinsics.K, m_intrinsics.distortion, rvec, tvec, false, SOLVEPNP_ITERATIVE);
    if (!b)
        return false;

    quaternion = rodrigues_to_quat(rvec);
    displacement = tvec;
    return true;
}

void ArucoDetector::draw_marker(Mat& plot, polygon_t const& p) const
{
    for (int i = 0; i < 4; ++ i)
    {
        int j = (i + 1) % 4;
        line(plot, Point2i(p[i]), Point2i(p[j]), Scalar(255, 0, 0), 1);
    }

    circle(plot, p[3], 5, Scalar(255, 0, 0), -1);
}

void ArucoDetector::draw_markers(Mat& plot, map<int, polygon_t> const& markers) const
{
    for (auto const& m : markers)
    {
        draw_marker(plot, m.second);
    }
}

void ArucoDetector::draw_frame(Mat& plot, Vec3f const& p, Vec4f q) const
{
    Vec3d tvec = p;
    Vec3d rvec = quat_to_rodrigues(q);

    std::vector<cv::Point3f> objpts = {
        {0, 0, 0},
        {m_pattern_side, 0, 0},
        {0, m_pattern_side, 0},
        {0, 0, m_pattern_side}
    };
    std::vector<cv::Point2f> impts(4);
    cv::projectPoints(objpts, rvec, tvec, m_intrinsics.K, m_intrinsics.distortion, impts);
    cv::line(plot, impts[0], impts[1], cv::Scalar(255,0,0));
    cv::line(plot, impts[0], impts[2], cv::Scalar(0,255,0));
    cv::line(plot, impts[0], impts[3], cv::Scalar(0,0,255));
}

Matx13f fit_line(Mat const& W)
{
    int const Ny = W.rows;
    int const Nx = W.cols;

    Mat A(Nx * Ny, 2, CV_32F);
    Mat B(Nx * Ny, 1, CV_32F);
    int neqs = 0;

    for (int y = 0; y < Ny; ++ y)
    {
        uchar const* pw = W.ptr<uchar>(y);

        for (int x = 0; x < Nx; ++ x)
        {
            float w = pw[x] / 256.f;
            if (w > 0.05)
            {
                A.at<float>(neqs, 0) = x * w;
                A.at<float>(neqs, 1) = w;
                B.at<float>(neqs) = y * w;
                ++ neqs;
            }
        }
    }

    A.resize(neqs);
    B.resize(neqs);

    Matx21f ab;
    cv::solve(A, B, ab, DECOMP_SVD);

    return Matx13f(ab(0,0), -1.f, ab(1,0));
}

inline Point2f normalized(Point2f const& v)
{
    float n2 = v.dot(v);

    if (n2 < 1e-12)
        return Point2f(0,0);
    return v / sqrtf(v.dot(v));
}

RotatedRect get_neib_rect(Point2f const& p1, Point2f const& p2, float sz)
{
    Point2f center = (p1 + p2) * 0.5f;
    Point2f v = p2 - p1;
    Size2f rect_sz(sqrtf(v.dot(v)), sz);
    float alpha = atan2(v.y, v.x);
    return RotatedRect(center, rect_sz, alpha);
}

Matx13f refine_edge_line(Mat const& im, Point2f const& p1, Point2f const& p2, float neighborhood_sz, bool dbg)
{
    auto const& r = get_neib_rect(p1, p2, neighborhood_sz);
    float c = cosf(-r.angle + M_PI);
    float s = sinf(-r.angle + M_PI);

    Matx33f S1(
        1, 0, -r.center.x,
        0, 1, -r.center.y,
        0, 0, 1
    );
    Matx33f R(
        c, -s, 0,
        s, c, 0,
        0, 0, 1
    );
    Matx33f S2(
        1, 0, r.size.width / 2.f,
        0, 1, r.size.height / 2.f,
        0, 0, 1
    );
    Matx33f T = S2 * R * S1;
    Matx23f _T(
        T(0,0), T(0,1), T(0,2),
        T(1,0), T(1,1), T(1,2)
    );

    Mat cropped, grad;
    cv::GaussianBlur(im, cropped, cv::Size(5,5), 2.);
    // cropped = im.clone();
    cv::warpAffine(cropped, cropped, _T, r.size, INTER_LINEAR);
    // cv::GaussianBlur(cropped, cropped, cv::Size(5,5), 2.);
    cv::Sobel(cropped, grad, CV_8U, 0, 1, 3, 0.25, 0, BORDER_DEFAULT);
    Matx13f f = fit_line(grad);
    Matx13f l = f * T;


    if (dbg)
    {
        static int i = 0;
        ++ i;
        std::string name = "sobel-" + std::to_string(i);

        cv::Mat plot;
        float scale = 1.f;
        cv::cvtColor(cropped, plot, cv::COLOR_GRAY2BGR);
        cv::resize(plot, plot, cv::Size(), scale, scale, INTER_NEAREST);
        Point2f p1(0, -f(0,2)/f(0,1));
        Point2f p2(cropped.cols, (-f(0,2) - f(0,0)*cropped.cols)/f(0,1));
        cv::line(plot, scale*p1, scale*p2, Scalar(0,255,0));

        cv::namedWindow(name, cv::WINDOW_NORMAL);
        cv::imshow(name, plot);
    }

    return l;
}

inline Point2f lines_crossong_pt(Matx13f const& l1, Matx13f const& l2)
{
    Matx22f A(
        l1(0), l1(1),
        l2(0), l2(1)
    );
    Matx21f B(
        -l1(2),
        -l2(2)
    );
    Matx21f c = A.inv() * B;
    return Point2f(c(0), c(1));
}

void refine_quad(Mat const& im, polygon_t& quad, bool dbg)
{
    assert(quad.size() == 4);

    Matx13f edges[4];

    float diag = quad_min_diag(quad);
    float neighborhood_sz = clip(diag / 5.f, 6.f, 30.f);

    for (int i = 0; i < 4; ++ i)
    {
        Point2f const& p1 = quad[i];
        Point2f const& p2 = quad[(i + 1) % 4];
        edges[i] = refine_edge_line(im, p1, p2, neighborhood_sz, dbg);
    }

    for (int i = 0; i < 4; ++ i)
    {
        quad[i] = lines_crossong_pt(edges[(i -1 + 4) % 4], edges[i]);
    }
}

void ArucoDetector::locate_markers(Mat const& im, map<int, polygon_t>& markers) const
{
    vector<int> ids;
    vector<polygon_t> polygons;
    Mat blurred;
    pyrDown(im, blurred);
    // pyrDown(blurred, blurred);
    // blurred = im;

    float scale = 1.f * im.rows / blurred.rows;

    // find corners
    Ptr<aruco::DetectorParameters> parameters = aruco::DetectorParameters::create();
    parameters->doCornerRefinement = false;
    aruco::detectMarkers(blurred, m_dict, polygons, ids, parameters);

    // refine corners
    for (int i = 0; i < ids.size(); ++i)
    {
        auto& polygon = polygons[i];
        int id = ids[i];

        polygon_scale(polygon, scale);
        float diag = quad_min_diag(polygon);
        if (diag < 15.f)
            continue;

        refine_quad(im, polygon, id==42);
        markers[id] = polygon;
    }
}
