#include "camera.h"

#include <cppmisc/json.h>
#include <cppmisc/misc.h>
#include <cppmisc/traces.h>
#include <cppmisc/argparse.h>
#include <cppmisc/timing.h>
#include <cppmisc/task_queue.h>
#include <cppmisc/formatting.h>
#include <cppmisc/signals.h>
#include <sstream>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>


cv::Mat copy_to_cvmat(RawImageWrap const& raw)
{
    const int Nx = raw.Nx, Ny = raw.Ny;

    cv::Mat dst(Ny, Nx, CV_8U);
    for (int y = 0; y < Ny; ++ y)
    {
        uchar const* praw = &raw.data[Nx * y];
        uchar* pdst = dst.ptr<uchar>(y);
        memcpy(pdst, praw, Nx);
    }

    assert(dst.total() > strlen(raw.format));
    std::strcpy(dst.ptr<char>(0), raw.format);

    return dst;
}

void save_rawfile(std::string const& name, cv::Mat const& im)
{
    dbg_msg("saving ", name);
    cv::Mat dst;
    cv::cvtColor(im, dst, cv::COLOR_BayerBG2BGR);
    cv::imwrite(name, dst);
}

class Recorder
{
private:
    int _stop;
    std::shared_ptr<Camera> _cam;
    int _nsaved;
    int _needsave;
    int _max_queue_size;
    float _savefps;
    int64_t _tstart;
    int64_t _saveduring;
    std::string _dstdir;
    std::string _config_path;
    std::string _fileext;
    TaskQueue _tasks;


    std::string genfilename(int64_t ts)
    {
        if (_dstdir.empty())
            return format(ts, _fileext);
        return format(_dstdir, "/", ts, _fileext);
    }

    void handler(RawImageWrap const& img)
    {
        if (_stop)
            return;
        
        if (_nsaved == 0)
            _tstart = epoch_usec();
        
        if (_savefps > 0.)
        {
            double actual_fps = _nsaved * 1e+6 / (epoch_usec() - _tstart + 1);
            if (actual_fps > _savefps)
                return;
        }

        auto cvimg = copy_to_cvmat(img);
        auto name = genfilename(img.frame_timestamp);
        auto f = [cvimg,name]() {
            save_rawfile(name, cvimg);
            return true;
        };
        _tasks.emplace(f);
        ++ _nsaved;
        if (_saveduring > 0)
            _stop |= epoch_usec() - _tstart >= _saveduring;
        if (_needsave > 0)
            _stop |= _nsaved >= _needsave;
    }

    void parse_args(int argc, char const* argv[])
    {
        Arguments args {
            Argument("-c", "config", "path to json config file", "", ArgumentsCount::One),
            Argument("-n", "nphotos", "number of images to save", ""),
            Argument("-d", "dst", "path to folder to save images", ""),
            Argument("-i", "interval", "save photos during time interval in sec", ""),
            Argument("-f", "fps", "save photos with given fps", ""),
            Argument("-t", "type", "image type, one of: raw,bmp,png,jpg", "png")
        };

        try
        {
            auto map = args.parse(argc, argv);

            // parse 'interval'
            _saveduring = 0;
            if (map.has("interval"))
                _saveduring = int64_t(atof(map["interval"].c_str()) * 1e+6 + 0.5);

            // parse 'nphotos'
            _needsave = 0;
            if (map.has("nphotos"))
                _needsave = atoi(map["nphotos"].c_str());

            // parse 'dst'
            _dstdir = "";
            if (map.has("dst"))
                _dstdir = map["dst"].c_str();

            // parse 'fps'
            _savefps = 0.;
            if (map.has("fps"))
                _savefps = atof(map["fps"].c_str());

            // parse 'type'
            std::string type = map["type"];
            if (!one_of(type, {"raw", "bmp", "png", "jpg"}))
                throw_invalid_argument("expect 'type' parameter is one of: raw, bmp, png, jpg");
            _fileext = "." + type;

            // parse 'config'
            _config_path = map["config"];
        }
        catch (std::invalid_argument const& e)
        {
            print_msg(args.help_message());
            throw e;
        }
        catch (std::exception const& e)
        {
            throw e;
        }
    }

public:
    Recorder(int argc, char const* argv[]) : 
        _nsaved(0),
        _max_queue_size(1000),
        _tasks(std::max<int>(std::thread::hardware_concurrency() - 1, 1))
    {
        info_msg("parsing arguments");
        parse_args(argc, argv);
        auto cfg = json_load(_config_path);

        auto cfg_traces = json_get(cfg, "traces");
        traces::init(cfg_traces);

        _cam = init_camera(cfg);
        auto f = [this](RawImageWrap const& img) { this->handler(img); };
        _cam->set_handler(f);
        _stop = false;

        auto& signals = SysSignals::instance();
        auto ctrlc = [this]() { 
            info_msg("interrupted");
            _stop = true;
        };
        signals.set_sigint_handler(ctrlc);

        info_msg("running");
        _cam->run();
        while (!_stop)
        {
            sleep_sec(0.1);
        }
        _cam->stop();
        info_msg("stopped");
    }
};

int main(int argc, char const* argv[])
{
    try
    {
        Recorder recorder(argc, argv);
        return 0;
    }
    catch (std::exception const& e)
    {
        err_msg("failed: ", e.what());
    }
    catch (...)
    {
        err_msg("failed: undefined");
    }
    return -1;
}
