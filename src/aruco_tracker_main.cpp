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

class ArucoTracker
{
private:
    std::string _config_path;
    CameraPtr _cam;
    std::shared_ptr<ArucoDetector> _aruco_detector;
    bool _stop;
    bool _term;
    bool _gui;
    RenderData _rendata;
    std::mutex _rendata_mtx;
    PublisherPtr _publisher;


    RenderData get_render_data()
    {
        std::unique_lock<std::mutex> lock(_rendata_mtx);
        return _rendata;
    }

    void set_render_data(
        cv::Mat const& img,
        std::vector<MarkerLoc> const& locations,
        std::vector<MarkerPose> const& poses
        )
    {
        std::unique_lock<std::mutex> lock(_rendata_mtx);
        _rendata = {img, locations, poses};
    }

    void parse_args(int argc, char const* argv[])
    {
        Arguments args {
            Argument("-c", "config", "path to json config file", "", ArgumentsCount::One),
            Argument("-g", "gui", "use gui", "0")
        };

        auto map = args.parse(argc, argv);
        _config_path = map["config"];
        _gui = tobool(map["gui"]);
    }

    void handler(RawImageWrap const& raw)
    {
        if (_stop)
            return;

        assert(!strcmp(raw.format, "gray"));
        cv::Mat img(raw.Ny, raw.Nx, CV_8U, (void*)raw.data);

        std::vector<MarkerLoc> locations;
        _aruco_detector->find_markers(img, locations);
        if (locations.size() == 0)
        {
            if (_gui)
                set_render_data(img.clone(), {}, {});
            return;
        }

        std::vector<MarkerPose> poses;
        _aruco_detector->get_markers_poses(locations, poses);
        if (poses.size() == 0)
        {
            if (_gui)
                set_render_data(img.clone(), locations, {});
            return;
        }

        bool ok = _publisher->publish(raw.frame_timestamp, poses);
        if (!ok)
            _stop = true;

        if (_gui)
        {
            set_render_data(img.clone(), locations, poses);
        }
    }

    bool run()
    {
        bool result = true;

        try
        {
            bool ok = _publisher->start();
            if (!ok)
            {
                return false;
            }

            info_msg("running");

            std::unique_ptr<Render> render;
            if (_gui)
            {
                render.reset(new Render);
            }

            _cam->run();
            _stop = false;

            while (!_stop)
            {
                if (render)
                {
                    ok = render->update(*_aruco_detector, get_render_data());
                    if (!ok)
                    {
                        _stop = true;
                    }
                }
                else
                {
                    sleep_sec(1.);
                }
            }
            _cam->stop();
            info_msg("stopped");
        }
        catch (std::exception const& e)
        {
            err_msg("failed: ", e.what());
            result = false;
        }
        catch (...)
        {
            err_msg("failed: unknown");
            result = false;
        }

        dbg_msg("5");

        return result;
    }

public:
    ArucoTracker(int argc, char const* argv[])
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

        _cam = init_camera(cfg);
        auto f = [this](RawImageWrap const& img) { this->handler(img); };
        _cam->set_handler(f);

        _publisher = create_publisher(cfg);

        auto& signals = SysSignals::instance();
        auto ctrlc = [this]() { 
            info_msg("interrupted");
            _term = true;
            _stop = true;
            _publisher->term();
        };
        signals.set_sigint_handler(ctrlc);

        _term = false;

        while (!_term)
        {
            run();
        }
    }

    ~ArucoTracker()
    {
    }
};

int main(int argc, char const* argv[])
{
    try
    {
        ArucoTracker aruco_tracker(argc, argv);
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
