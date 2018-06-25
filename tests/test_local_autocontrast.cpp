#include <opencv2/opencv.hpp>
#include <aruco_detector.h>


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

        detector.get_marker_pose(m.second, d, q);
        detector.draw_marker(plot, m.second);
        detector.draw_frame(plot, d, q);
    }

    if (true)
    {
        cv::Vec3f d13, d48;
        cv::Vec4f q13, q48;
        detector.get_marker_pose(markers[13], d13, q13);
        std::cout << 13 << ": " << d13 << " " << q13 << std::endl;

        detector.get_marker_pose(markers[48], d48, q48);
        std::cout << 48 << ": " << d48 << " " << q48 << std::endl;

        cv::Vec3f d = d48-d13;
        std::cout << "dist 13-48" << ": " << sqrtf(d.dot(d)) << std::endl;
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

int main()
{
    // test_autocontrast();
    test_detector("dataset/00000.png");
    return 0;
}
