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
#include <cppmisc/files.h>
#include <iostream>
#include <fstream>
#include "json_formatter.h"


template <typename T, int N>
inline std::string format_points(cv::Matx<T, N, 2> const& pts)
{
    std::stringstream ss;
    ss << "[";
    for (int i = 0; i < N; ++ i)
        ss << pts(i, 0) << ", " << pts(i, 1) << (i == N - 1 ? "" : "; ");
    ss << "]";
    return ss.str();
}


class Report
{
private:
    Json::Value _result;
    std::string _filepath;

public:
    Report(std::string const& filepath) : _filepath(filepath) {}

    ~Report()
    {
        Json::Value rootJsonValue;
        rootJsonValue["foo"] = "bar";

        Json::StreamWriterBuilder builder;
        builder["commentStyle"] = "None";
        builder["indentation"] = "  ";

        std::unique_ptr<Json::StreamWriter> _writer(builder.newStreamWriter());

        std::ofstream file(_filepath);
        file.exceptions(std::ios_base::badbit);

        _writer->write(_result, &file);
    }

    void add(std::string const& sample_name, std::vector<Marker> const& markers)
    {
        for (auto const& marker : markers)
        {
            Json::Value marker_data;

            marker_data["id"] = marker.id;

            if (marker.flags & Marker::CornersFound)
                marker_data["corners"] = to_json(marker.corners);

            if (marker.flags & Marker::PoseFound)
            {
                marker_data["camera_frame"] = to_json(marker.camera_marker_pose);
                marker_data["world_frame"] = to_json(marker.world_marker_pose);
            }

            _result[sample_name].append(marker_data);
        }
    }
};

class MainProcessor
{
private:
    std::shared_ptr<ArucoDetector> _aruco_detector;
    std::string _config_path;
    std::vector<std::string> _inputs;
    std::unique_ptr<Report> _report;

    void parse_args(int argc, char const* argv[])
    {
        Arguments args {
            Argument("-c", "config", "path to json config file", "", ArgumentsCount::One),
            Argument("-o", "output", "path to json report file", "", ArgumentsCount::Optional),
        };

        auto map = args.parse(argc, argv);
        _config_path = map["config"];

        int ninputs = map.size("rest");
        _inputs.reserve(ninputs);
        for (int i = 0; i < ninputs; ++ i)
            _inputs.push_back(map.get("rest", i));
        
        if (map.has("output"))
            _report = std::make_unique<Report>(map["output"]);
    }

    bool process_sample(std::string const& impath)
    {
        cv::Mat im = cv::imread(impath, 0);
        std::vector<Marker> markers;
        _aruco_detector->find_markers(im, markers);
        if (markers.size() == 0)
        {
            info_msg("no markers found in ", impath);
            return false;
        }

        info_msg("found markers:");

        for (auto const& marker : markers)
        {
            info_msg("found #", marker.id, ":");
            info_msg("  corners: ", format_points(marker.corners));
            info_msg("  world frame pose:  ", marker.world_marker_pose.p, ", ", marker.world_marker_pose.r);
            info_msg("  camera frame pose: ", marker.camera_marker_pose.p, ", ", marker.camera_marker_pose.r);
        }

        if (_report)
            _report->add(getname(impath), markers);

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
