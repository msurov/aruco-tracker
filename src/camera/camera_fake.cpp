#include <sys/types.h>
#include <vector>
#include <algorithm>
#include <thread>
#include <memory>
#include "traces.h"
#include "common.h"
#include "camera.h"
#include "cv_helpers.h"

using namespace std;
using namespace cv;


class FakeCamera : public Camera
{
private:
    image_handler_t         handler;
    vector<string>          files;
    unique_ptr<thread>      caller;
    string                  sources_format;
    bool                    bexit;
    int                     period_ms;

    int64_t get_timestamp(string const& filepath)
    {
        string&& name = getname(filepath);
        auto nameext = splitext(name);
        string&& digits = get_digits_substr(get<0>(nameext));
        if (digits.empty())
            return get_time_usec();
        return stoll(digits);
    }

    Mat read(std::string const& path)
    {
        std::string ext = std::get<1>(splitext(path));
        if (sources_format == "bggr")
        {
            if (ext == ".raw" || ext == ".bggr")
                return read_raw(path);
            throw std::runtime_error("unsupported image format");
        }
        else if (sources_format == "gray")
        {
            if (ext == ".png" || ext == ".bmp")
                return cv::imread(path, IMREAD_GRAYSCALE);
            throw std::runtime_error("unsupported image format");
        }
        throw std::runtime_error("unsupported image format");
    }

    void handler_loop()
    {
        auto t0 = chrono::high_resolution_clock::now();
        auto t1 = t0;
        int i = 0;

        while (!bexit)
        {
            if (handler)
            {
                Mat img = read(files[i]);
                auto ts = get_timestamp(files[i]);
                handler((uint8_t const*)img.data, img.cols, img.rows, ts);
            }

            auto t = chrono::high_resolution_clock::now();
            auto t_left = t - t1;
            t1 = t;

            if (t_left < chrono::milliseconds(period_ms))
                this_thread::sleep_for(chrono::milliseconds(period_ms) - t_left);

            i = (i + 1) % files.size();
        }
    }

public:
    FakeCamera(string const& files_mask, string const& sources_format, float fps)
    {
        if (sources_format != "bggr" && sources_format != "rggb" && sources_format != "gray")
            throw runtime_error("incorrect sources_format value");

        period_ms = int(1e+3f / fps);

        this->bexit = true;
        this->sources_format = sources_format;
        files = move(get_files(files_mask));

        if (files.empty())
            throw runtime_error("there are no files in " + files_mask);

        sort(files.begin(), files.end());
    }

    ~FakeCamera()
    {
        stop();
    }

    void set_handler(image_handler_t const& handler)
    {
        this->handler = handler;
    }

    void run()
    {
        bexit = false;
        caller.reset(new thread([this]() { this->handler_loop(); }));
    }

    void stop()
    {
        bexit = true;
        if (caller && caller->joinable())
            caller->join();
    }

    string description()
    {
        return "Fake Camera";
    }

    string image_type()
    {
        return sources_format;
    }
};

static unique_ptr<FakeCamera> cam;

void init_camera(jsonxx::Object const& jsoncfg)
{
    if (!jsoncfg.has<jsonxx::Object>("camera_fake"))
        throw runtime_error("config file doesn't have 'camera_fake' entry");

    auto const& json_fake_cam = jsoncfg.get<jsonxx::Object>("camera_fake");
    auto const& json_sources = json_fake_cam.get<jsonxx::String>("sources");
    auto const& json_format = json_fake_cam.get<jsonxx::String>("format");
    float const fps = json_get(json_fake_cam, "fps", 1., 200.);
    cam.reset(new FakeCamera(json_sources, json_format, fps));
}

Camera* get_camera()
{
    if (!cam)
        throw runtime_error("camera is not initialized");

    return cam.get();
}
