#include "aruco_detector.h"
#include "camera.h"
#include "imgdump.h"
#include "publisher.h"
#include <mutex>
#include <cppmisc/argparse.h>
#include <cppmisc/traces.h>
#include <cppmisc/timing.h>
#include <cppmisc/signals.h>
#include <cppmisc/misc.h>
#include <iostream>

struct RenderData
{
    cv::Mat img;
    std::vector<MarkerLoc> locations;
    std::vector<MarkerPose> poses;
};

class Render
{
public:
    Render()
    {
        cv::namedWindow("aruco tracker", 0);
    }

    ~Render()
    {
        cv::destroyWindow("aruco tracker");
    }

    bool update(ArucoDetector const& detector, RenderData const& rendata)
    {
        auto const& img = rendata.img;
        auto const& locations = rendata.locations;
        auto const& poses = rendata.poses;
        if (img.total() == 0)
            return true;
        cv::Mat plot;
        cv::cvtColor(img, plot, cv::COLOR_GRAY2BGR);
        detector.draw_markers(plot, locations);
        detector.draw_frames(plot, poses);
        cv::imshow("aruco tracker", plot);
        int val = cv::waitKey(25);
        if (val == 27)
            return false;
        return true;
    }
};

class MainProcessor
{
private:
    std::shared_ptr<ArucoDetector> _aruco_detector;
    std::string _config_path;
    std::vector<std::string> _inputs;

    void parse_args(int argc, char const* argv[])
    {
        Arguments args {
            Argument("-c", "config", "path to json config file", "", ArgumentsCount::One)
        };

        auto map = args.parse(argc, argv);
        _config_path = map["config"];

        int ninputs = map.size("rest");
        _inputs.reserve(ninputs);
        for (int i = 0; i < ninputs; ++ i)
            _inputs.push_back(map.get("rest", i));
    }

    bool process_sample(std::string const& impath)
    {
        cv::Mat im = cv::imread(impath, 0);
        std::vector<MarkerLoc> locations;
        _aruco_detector->find_markers(im, locations);
        if (locations.size() == 0)
        {
            info_msg("no markers found in ", impath);
            return false;
        }

        std::vector<MarkerPose> poses;
        _aruco_detector->get_markers_poses(locations, poses);
        if (poses.size() == 0)
        {
            info_msg("can't estimated marker pose in ", impath);
            return false;
        }

        info_msg("found markers:");

        for (auto const& pose : poses)
        {
            info_msg(pose.id, ": ", pose.pose.p, ", ", pose.pose.r);
        }

        return true;
    }

public:
    MainProcessor(int argc, char const* argv[])
    {
        info_msg("parsing arguments");
        parse_args(argc, argv);
        auto cfg = json_load(_config_path);

        if (json_has(cfg, "imgdump"))
        {
            auto cfg_imgdump = json_get(cfg, "imgdump");
            imgdump::init(cfg_imgdump);
        }

        if (json_has(cfg, "traces"))
        {
            auto cfg_traces = json_get(cfg, "traces");
            traces::init(cfg_traces);
        }

        auto cfg_aruco = json_get(cfg, "aruco");
        _aruco_detector = create_aruco_detector(cfg_aruco);

        for (auto const& inp : _inputs)
        {
            process_sample(inp);
        }
    }

};

int main(int argc, char const* argv[])
{
    try
    {
        MainProcessor aruco_tracker(argc, argv);
    }
    catch (std::exception const& e)
    {
        std::cerr << "failed: " << e.what() << std::endl;
        return -1;
    }
    catch (...)
    {
        std::cerr << "failed: unknown" << std::endl;
        return -1;
    }

    return 0;
}