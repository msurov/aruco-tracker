#include "aruco_detector.h"

#include <tuple>
#include <opencv2/opencv.hpp>
#include "traces.h"
#include "transforms.h"


using namespace std;
using namespace cv;


unique_ptr<ArucoDetector> get_aruco_detector(jsonxx::Object const& cfg)
{
    auto marker = json_get<jsonxx::Object>(cfg, "marker");
    string dict_type = json_get<jsonxx::String>(marker, "dict_type");
    float side = json_get<jsonxx::Number>(marker, "side");

    return unique_ptr<ArucoDetector>(new ArucoDetector(dict_type, side));
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

void ArucoDetector::locate_markers(Mat const& im, map<int, polygon_t>& markers) const
{
    vector<int> ids;
    vector<polygon_t> polygons;
    Mat blurred;
    pyrDown(im, blurred);

    // find corners
    Ptr<aruco::DetectorParameters> parameters = aruco::DetectorParameters::create();
    parameters->doCornerRefinement = false;
    aruco::detectMarkers(blurred, m_dict, polygons, ids, parameters);

    // refine corners
    TermCriteria term_criteria(TermCriteria::MAX_ITER + TermCriteria::EPS, 50, 0.01);

    for (int i = 0; i < ids.size(); ++i)
    {
        auto& polygon = polygons[i];
        int id = ids[i];

        polygon_scale(polygon, 2.f);
        float diag = quad_min_diag(polygon);

        if (diag < 30.f)
            continue;

        int sz = lroundf(diag / 12.f);
        sz = clamp(sz, 2, 20);
        cornerSubPix(im, polygon, Size(sz, sz), Size(-1, -1), term_criteria);

        markers[id] = polygon;
    }
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

void ArucoDetector::draw_found_markers(Mat& plot, map<int, polygon_t> const& markers) const
{
    for (auto const& it : markers)
    {
        polygon_t const& p = it.second;
        for (int i = 0; i < 4; ++ i)
        {
            int j = (i + 1) % 4;
            line(plot, Point2i(p[i]), Point2i(p[j]), Scalar(255, 0, 0), 1);
        }
    }
}

void ArucoDetector::draw_frame(Mat& plot, Vec3f const& p, Vec4f q) const
{
    Vec3d tvec = p;
    Vec3d rvec = quat_to_rodrigues(q);
    aruco::drawAxis(plot, m_intrinsics.K, m_intrinsics.distortion, rvec, tvec, m_pattern_side);
}
