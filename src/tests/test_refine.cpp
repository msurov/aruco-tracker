#include <opencv2/opencv.hpp>
#include <aruco/aruco_detector.h>
#include <misc/json.h>
#include <misc/files.h>


void local_autocontrast(cv::Mat const& in, cv::Mat& out, int sz, double alpha=1.)
{
    assert(in.type() == CV_8U);

    cv::Mat blurred;
    cv::blur(in, blurred, cv::Size(sz, sz));
    cv::addWeighted(in, alpha, blurred, -alpha, 128, out, CV_8U);
}

void test_detector(std::string const& filepath)
{
    cv::Mat im0 = cv::imread(filepath, 0);
    cv::Mat im;
    // local_autocontrast(im0, im, 128, 1.5);
    cv::equalizeHist(im0, im);
    im = (im0 + im) / 2;
    // cv::imwrite("dataset/00061_ac.png", im);

    CameraIntrinsics intrinsics;
    intrinsics.K = cv::Matx33f(
        2287.735107421875, 0, 2029.416015625,
        0, 2285.650146484375, 1081.11083984375,
        0, 0, 1
        // 4096, 0, 4112/2,
        // 0, 4096, 2176/2,
        // 0, 0, 1
    );
    intrinsics.distortion = {{-0.014937651343643665, -0.036428362131118774, 0.00019892562704626471, -4.5335535105550662e-05, 0.0054666893556714058}};
    ArucoDetector detector("5x5x1000", 8e-2, intrinsics);

    std::map<int, polygon_t> markers;
    detector.locate_markers(im, markers);
    std::cout << markers.size() << std::endl;

    cv::Mat plot;
    cv::cvtColor(im, plot, cv::COLOR_GRAY2BGR);

    for (auto const& m : markers)
    {
        cv::Vec3f d;
        cv::Vec4f q;

        if (m.first != 42 && m.first != 48)
            continue;

        detector.get_marker_pose(m.second, d, q);
        detector.draw_marker(plot, m.second);
        detector.draw_frame(plot, d, q);
    }

    if (true)
    {
        cv::Vec3f d42, d48;
        cv::Vec4f q42, q48;
        detector.get_marker_pose(markers[42], d42, q42);
        std::cout << 42 << ": " << markers[42] << std::endl;

        detector.get_marker_pose(markers[48], d48, q48);
        std::cout << 48 << ": " << markers[48] << std::endl;

        cv::Vec3f d = d48-d42;
        std::cout << "dist 42-48" << ": " << sqrtf(d.dot(d)) << std::endl;
    }

    // detector.draw_found_markers(im, markers);
    cv::namedWindow("1", cv::WINDOW_NORMAL);
    cv::imshow("1", plot);
    cv::waitKey();
}

void test_autocontrast(std::string const& filepath)
{
    cv::Mat im = cv::imread(filepath);
    cv::Mat im1, im2;

    cv::cvtColor(im, im1, cv::COLOR_BGR2GRAY);
    local_autocontrast(im1, im2, 300, 2.0);

    cv::namedWindow("src", cv::WINDOW_NORMAL);
    cv::imshow("src", im1);
    cv::namedWindow("autocontrasted", cv::WINDOW_NORMAL);
    cv::imshow("autocontrasted", im2);
    cv::waitKey();
}

inline cv::Vec2f normalized(cv::Vec2f const& p)
{
    return p * (1.f / (sqrtf(p.dot(p)) + 1e-8));
}

inline cv::Vec3f line_crossing_pts(cv::Point2f const& p1, cv::Point2f const& p2)
{
    float a = p2.y - p1.y;
    float b = -p2.x + p1.x;
    float c = p2.x * p1.y - p2.y * p1.x;
    return cv::Vec3f(a,b,c);
}

inline cv::Point2f lines_crossing_pt(cv::Vec3f const& l1, cv::Vec3f const& l2)
{
    cv::Matx22f A(
        l1[0], l1[1], 
        l2[0], l2[1]
    );
    cv::Vec2f b(
        l1[2],
        l2[2]
    );
    cv::Vec2f p = -A.inv() * b;
    return cv::Point2f(p[0], p[1]);
}


polygon_t enlarge_poly_v3(polygon_t const& p, float d)
{
    const int N = p.size();
    polygon_t enlarged;
    enlarged.reserve(2*N);

    for (int i = 0; i < N; ++ i)
    {
        cv::Point2f p1 = p[i];
        cv::Point2f p2 = p[(i + 1) % N];
        cv::Matx22f J(0, 1, -1, 0);
        cv::Point2f s = d * J * normalized(p2 - p1);
        enlarged.emplace_back(p1 + s);
        enlarged.emplace_back(p2 + s);
    }

    return enlarged;
}

polygon_t enlarge_poly_v2(polygon_t const& p, float d)
{
    const int N = p.size();
    std::vector<cv::Vec3f> edges;
    edges.reserve(N);

    for (int i = 0; i < N; ++ i)
    {
        cv::Point2f p1 = p[i];
        cv::Point2f p2 = p[(i + 1) % N];
        cv::Matx22f J(0, 1, -1, 0);
        cv::Point2f s = d * J * normalized(p2 - p1);
        edges.emplace_back(line_crossing_pts(p1 + s, p2 + s));
    }

    polygon_t enlarged;
    enlarged.reserve(N);

    for(int i = 0; i < N; ++ i)
    {
        cv::Vec3f const& e1 = edges[(i - 1 + N) % N];
        cv::Vec3f const& e2 = edges[i];
        enlarged.emplace_back(lines_crossing_pt(e1, e2));
    }

    return enlarged;
}

inline cv::Point2f normalized(cv::Point2f const& p)
{
    return p * (1.f / sqrtf(p.dot(p)) + 1e-8);
}

polygon_t enlarge_poly(polygon_t const& p, float d)
{
    const int N = p.size();
    polygon_t enlarged;
    enlarged.reserve(N);

    for (int i = 0; i < N; ++ i)
    {
        cv::Point2f p1 = p[(i - 1 + N) % N];
        cv::Point2f p2 = p[i];
        cv::Point2f p3 = p[(i + 1) % N];

        cv::Point2f v1 = normalized(p2 - p1);
        cv::Point2f v2 = normalized(p3 - p2);

        cv::Matx22f J(0, -1, 1, 0);
        cv::Point2f vm = -J * normalized(v1 + v2) * 0.5f;
        float l = sqrtf(2) * d / sqrtf(1 + v1.dot(v2));
        enlarged.emplace_back(p2 + l * vm);
    }

    return enlarged;
}

inline std::vector<cv::Point> cvt_poly(std::vector<cv::Point2f> const& poly)
{
    std::vector<cv::Point> result(poly.size());

    for (int i = 0; i < poly.size(); ++ i)
    {
        float x = poly.at(i).x;
        float y = poly.at(i).y;
        result.at(i) = cv::Point(lroundf(x), lroundf(y));
    }

    return result;
}

void draw_marker(cv::Mat& plot, polygon_t const& poly)
{
    int sz = 50;
    polygon_t enlarged = enlarge_poly(poly, sz);
    std::vector<std::vector<cv::Point>> arr1 = {cvt_poly(poly)};
    std::vector<std::vector<cv::Point>> arr2 = {cvt_poly(enlarged)};
    
    cv::fillPoly(plot, arr2, cv::Scalar(255));
    cv::fillPoly(plot, arr1, cv::Scalar(0));
    // cv::polylines(plot, arr1, true, cv::Scalar(128));
}

std::string format_arr(std::vector<cv::Point2f> const& a)
{
    std::string s;
    s.reserve(a.size() * 32);

    s = "[";

    for (int i = 0; i < a.size(); ++ i)
    {
        s += "[";
        s += std::to_string(a[i].x);
        s += ", ";
        s += std::to_string(a[i].y);
        s += "]";
        if (i != a.size() - 1)
            s += ", ";
    }

    s += "]";
    return s;
}

void test_rhombus()
{
    cv::Mat im(1024, 1024, CV_8U);
    im = 0;
    polygon_t rhombus = {
        {300, 200},
        {400, 300},
        {300, 400},
        {200, 300}
    };
    draw_marker(im, rhombus);
    cv::imwrite("dataset/test_rhombus.png", im);
    polygon_t refined = rhombus;
    refine_quad(im, refined, false);

    std::string const& s =
        R"({ "src": )" + format_arr(rhombus) + ","
        R"("refined": )" + format_arr(refined) + "}";
    write_file("dataset/test_rhombus.json", s);

}

void test_rect()
{
    cv::Mat im(1024, 1024, CV_8U);
    im = 0;
    polygon_t quad = {
        {200, 200},
        {300, 200},
        {300, 300},
        {200, 300}
    };
    draw_marker(im, quad);
    cv::imwrite("dataset/test_quad.png", im);
    polygon_t refined = quad;
    refine_quad(im, refined, false);

    std::string const& s =
        R"({ "src": )" + format_arr(quad) + ","
        R"("refined": )" + format_arr(refined) + "}";
    write_file("dataset/test_quad.json", s);
}

std::vector<cv::Point3f> interpolate(std::vector<cv::Point3f> const& a, int n)
{
    const int N = a.size();

    std::vector<cv::Point3f> a2;
    a2.reserve(N * n);

    for (int i = 0; i < N; ++ i)
    {
        cv::Point3f p1 = a[i];
        cv::Point3f p2 = a[(i + 1) % N];

        for (int j = 0; j < n; ++ j)
        {
            cv::Point3f p = p1 + (p2 - p1) * j * 1.f / n;
            a2.push_back(p);
        }
    }

    return a2;
}

void test_projection()
{
    cv::Mat im(2400, 4096, CV_8U);
    im = 0;

    cv::Matx33f K(
        2287.735107421875, 0, 2029.416015625,
        0, 2285.650146484375, 1081.11083984375,
        0, 0, 1
    );
    std::vector<float> distortion = {-0.014937651343643665, -0.036428362131118774, 0.00019892562704626471, -4.5335535105550662e-05, 0.0054666893556714058};
    cv::Vec3d tvec(-0.5, 0.3, 1.5);
    cv::Vec3d rvec(0.9, -0.1, 0.8);
    std::vector<cv::Point3f> objpts = {
        {   0,    0, 0},
        {8e-2,    0, 0},
        {8e-2, 8e-2, 0},
        {   0, 8e-2, 0},
    };
    std::vector<cv::Point3f> objpts2 = interpolate(objpts, 2);
    std::vector<cv::Point2f> imgpts(4);
    std::vector<cv::Point2f> imgpts2(objpts2.size());

    cv::projectPoints(objpts2, rvec, tvec, K, distortion, imgpts2);
    draw_marker(im, imgpts2);

    cv::projectPoints(objpts, rvec, tvec, K, distortion, imgpts);
    cv::imwrite("dataset/test_projection.png", im);
    polygon_t refined = imgpts;
    refine_quad(im, refined, false);
    // cv::waitKey();

    std::string const& s =
        R"({"src": )" + format_arr(imgpts) + ","
        R"("refined": )" + format_arr(refined) + "}";
    write_file("dataset/test_projection.json", s);

    cv::Vec3d tvec_solved;
    cv::Vec3d rvec_solved;
    cv::solvePnP(objpts, refined, K, distortion, rvec_solved, tvec_solved, false, cv::SOLVEPNP_ITERATIVE);

    std::cout << "tvec: " << tvec << " ≈ " << tvec_solved << std::endl;
    std::cout << "rvec: " << rvec << " ≈ " << rvec_solved << std::endl;
}


struct ObjPose
{
    cv::Vec3d tvec;
    cv::Vec3d rvec;
};

ObjPose test_pnp(CameraIntrinsics const& intr, ObjPose const& pose, std::vector<cv::Point3f> const& marker, cv::Mat& projection)
{
    std::vector<cv::Point3f> marker_interpolated = interpolate(marker, 50);
    std::vector<cv::Point2f> imgpts_interpolated(marker_interpolated.size());

    cv::projectPoints(marker_interpolated, pose.rvec, pose.tvec, intr.K, intr.distortion, imgpts_interpolated);

    std::vector<cv::Point2f> imgpts(marker.size());
    cv::projectPoints(marker, pose.rvec, pose.tvec, intr.K, intr.distortion, imgpts);

    projection.create(intr.resolution, CV_8U);
    projection = 0;
    draw_marker(projection, imgpts_interpolated);

    std::vector<cv::Point2f> refined = imgpts;
    refine_quad(projection, refined, false);
    // TODO: think about this:
    // refined = enlarge_poly(refined, -0.5f);
    // dbg_msg("source:  ", imgpts);
    // dbg_msg("refined: ", refined);

    ObjPose pose_found;
    cv::solvePnP(marker, refined, intr.K, intr.distortion, pose_found.rvec, pose_found.tvec, false, cv::SOLVEPNP_ITERATIVE);
    return pose_found;
}

void test_pnp_set_()
{
    CameraIntrinsics intr;
    intr.K = cv::Matx33f(
        2287.735107421875, 0, 2029.416015625,
        0, 2285.650146484375, 1081.11083984375,
        0, 0, 1
    );
    intr.distortion = {-0.014937651343643665, -0.036428362131118774, 0.00019892562704626471, -4.5335535105550662e-05, 0.0054666893556714058};
    intr.resolution = {4200, 2400};

    std::vector<cv::Point3f> marker = {
        {   0,    0, 0},
        {8e-2,    0, 0},
        {8e-2, 8e-2, 0},
        {   0, 8e-2, 0},
    };

    const int N = 2;
    for (int i = 0; i < N; ++ i)
    {
        ObjPose pose;
        pose.tvec = cv::Vec3d(0.0, 0.0, 1. + 2. * i / N);
        pose.rvec = cv::Vec3d(0.9, -0.1, 0.8);
        ObjPose pose2;
        cv::Mat projection;
        pose2 = test_pnp(intr, pose, marker, projection);
        cv::namedWindow("proj-" + std::to_string(i), cv::WINDOW_NORMAL);
        cv::imshow("proj-" + std::to_string(i), projection);
        dbg_msg("dif: ", (pose.tvec - pose2.tvec) * 1e+3);
    }

    cv::waitKey();
}

void test_pnp_set()
{
    CameraIntrinsics intr;
    intr.K = cv::Matx33f(
        1000.0, 0, 500.5,
        0, 1000.0, 500.5,
        0, 0, 1.
    );
    intr.distortion = {0, 0, 0, 0, 0};
    intr.resolution = {1000, 1000};

    std::vector<cv::Point3f> marker = {
        {    0,     0, 0},
        {9.99e-2,     0, 0},
        {9.99e-2, 9.99e-2, 0},
        {    0, 9.99e-2, 0},
    };

    const int N = 17;
    for (int i = 0; i < N; ++ i)
    {
        ObjPose pose;
        pose.tvec = cv::Vec3d(0.0, 0.0, 1. + 3. * i / (N - 1));
        pose.rvec = cv::Vec3d(0.0, 0.0, 0);
        ObjPose pose2;
        cv::Mat projection;
        pose2 = test_pnp(intr, pose, marker, projection);
        cv::namedWindow("proj-" + std::to_string(i), cv::WINDOW_NORMAL);
        cv::imshow("proj-" + std::to_string(i), projection);
        dbg_msg("dif: ", (pose.tvec - pose2.tvec) * 1e+3);
    }

    cv::waitKey();
}

void test_enlarge_poly()
{
    polygon_t p = {
        {100, 100},
        {250, 100},
        {400, 400},
        {200, 200},
        {100, 300},
    };
    cv::Mat m(500,500,CV_8UC3);
    m = 0;

    auto const& q = enlarge_poly(p, 10);
    std::vector<std::vector<cv::Point>> arr2 = {cvt_poly(q)};
    cv::fillPoly(m, arr2, cv::Scalar(0, 255, 0));

    std::vector<std::vector<cv::Point>> arr1 = {cvt_poly(p)};
    cv::fillPoly(m, arr1, cv::Scalar(255, 0, 0));

    cv::imshow("1", m);
    cv::waitKey();
}

void test_line_fitting()
{
    // test_enlarge_poly();
    test_pnp_set();
    // test_projection();
    // test_rect();
    // test_rhombus();
}

int main()
{
    test_line_fitting();
    return 0;
}
