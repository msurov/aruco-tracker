#include <opencv2/opencv.hpp>
#include <aruco_detector.h>


void local_autocontrast(cv::Mat const& in, cv::Mat& out, int sz, double alpha=1.)
{
    assert(in.type() == CV_8U);

    cv::Mat blurred;
    cv::blur(in, blurred, cv::Size(sz, sz));
    cv::addWeighted(in, alpha, blurred, -alpha, 128, out, CV_8U);
}

void test_detector()
{
    cv::Mat im0 = cv::imread("dataset/00061.png", 0);
    cv::Mat im;
    local_autocontrast(im0, im, 128, 2.0);
    cv::imwrite("dataset/00061_ac.png", im);

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

    for (auto const& m : markers)
    {
        std::cout << m.first << " ";
    }

    std::cout << std::endl;

    for (auto const& m : markers)
    {
        cv::Point2f p1 = m.second.at(0);
        cv::Point2f p2 = m.second.at(1);
        cv::Point2f p3 = m.second.at(2);
        cv::Point2f p4 = m.second.at(3);

        std::cout << "[" << p1 << ", " << p2 << ", " << p3 << ", " << p4 << "]," << std::endl;
    }

    for (auto const& m : markers)
    {
        cv::Vec3f d;
        cv::Vec4f q;
        detector.get_marker_pose(m.second, d, q);
        std::cout << m.first << ": " << d << " " << q << std::endl;
        detector.draw_frame(im, d, q);
    }

    // detector.draw_found_markers(im, markers);
    cv::imshow("1", im);
    cv::waitKey();

    // 15 16 43 46 48 49
}

void test_autocontrast()
{
    cv::Mat im = cv::imread("dataset/00061.png");
    cv::Mat im1, im2;

    cv::cvtColor(im, im1, cv::COLOR_BGR2GRAY);
    local_autocontrast(im1, im2, 300, 1.5);

    cv::namedWindow("src", cv::WINDOW_NORMAL);
    cv::imshow("src", im1);
    cv::namedWindow("autocontrasted", cv::WINDOW_NORMAL);
    cv::imshow("autocontrasted", im2);
    cv::waitKey();
}

int main()
{
    // test_autocontrast();
    test_detector();
    return 0;
}
