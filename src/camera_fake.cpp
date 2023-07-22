#include <sys/types.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <thread>
#include <memory>
#include <cppmisc/traces.h>
#include <cppmisc/files.h>
#include <cppmisc/strings.h>
#include <cppmisc/timing.h>
#include <opencv2/imgcodecs.hpp>
#include "camera.h"


using namespace std;
using namespace cv;


// Size2i get_img_dims(int imgsz)
// {
//     if (imgsz % 9 == 0 && imgsz % 16 == 0)
//     {
//         int w = int(sqrtf(imgsz * 16 / 9));
//         int h = w * 9 / 16;
//         if (imgsz == h * w)
//             return Size2i(w, h);
//     }

//     if (imgsz % 3 == 0 && imgsz % 4 == 0)
//     {
//         int w = int(sqrtf(imgsz * 4 / 3));
//         int h = w * 3 / 4;
//         if (imgsz == h * w)
//             return Size2i(w, h);
//     }

//     return Size2i(0,0);
// }

// Mat read_raw_img(char const* path)
// {
//     auto sz = file_size(path);
//     if (sz <= 0)
//         throw_runtime_error("Can't read file ", path);
//     auto dims = get_img_dims(sz);
//     FILE* f = fopen(path, "rb");
//     Mat img(dims.height, dims.width, CV_8U);
//     for (int y = 0; y < dims.height; ++ y)
//     {
//         uchar* p = img.ptr<uchar>(y);
//         auto n = fread(p, 1, dims.height, f);
//         if (n != dims.height)
//         {
//             fclose(f);
//             throw_runtime_error("Error during reading file ", path);
//         }
//     }
//     fclose(f);
//     return img;
// }

class FakeCamera : public Camera
{
private:
    ImageHandler            _handler;
    vector<string>          _files;
    unique_ptr<thread>      _caller;
    string                  _sources_format;
    bool                    _bexit;
    int                     _period_ms;


    int64_t get_timestamp(string const& filepath)
    {
        string&& name = getname(filepath);
        auto nameext = splitext(name);
        string&& digits = get_digits_substr(get<0>(nameext));
        if (digits.empty())
            return epoch_usec();
        return stoll(digits);
    }

    Mat read(std::string const& path)
    {
        std::string ext = std::get<1>(splitext(path));
        if (_sources_format == "bggr")
        {
            // if (ext == ".raw" || ext == ".bggr")
            //     return read_raw_img(path.c_str());
            throw_runtime_error("unsupported image format");
        }
        else if (_sources_format == "gray")
        {
            if (ext == ".png" || ext == ".bmp" || ext == ".jpg")
                return cv::imread(path, IMREAD_GRAYSCALE);
            throw_runtime_error("unsupported image format");
        }
        throw_runtime_error("unsupported image format");
    }

    void handler_loop()
    {
        LoopRate rate(_period_ms);
        int i = 0;

        while (!_bexit)
        {
            if (_handler)
            {
                Mat img = read(_files[i]);

                RawImageWrap imgwrap;
                imgwrap.data = (uint8_t const*)img.data;
                imgwrap.Nx = img.cols;
                imgwrap.Ny = img.rows;
                imgwrap.format = "gray";
                // imgwrap.frame_timestamp = get_timestamp(_files[i]);
                imgwrap.frame_timestamp = epoch_usec();

                _handler(imgwrap);
            }

            rate.wait();
            i = (i + 1) % _files.size();
        }
    }

public:
    FakeCamera(string const& files_mask, string const& image_type, float fps)
    {
        if (image_type != "bggr" && image_type != "rggb" && image_type != "gray")
            throw_runtime_error("incorrect sources_format value");

        _period_ms = int(1e+3f / fps);
        this->_bexit = true;
        this->_sources_format = image_type;
        _files = get_files(files_mask);

        if (_files.empty())
            throw_runtime_error("there are no files in ", files_mask);

        sort(_files.begin(), _files.end());
    }

    ~FakeCamera()
    {
        stop();
    }

    void set_handler(ImageHandler const& handler) override
    {
        this->_handler = handler;
    }

    void run() override
    {
        _bexit = false;
        _caller.reset(new thread([this]() { this->handler_loop(); }));
    }

    void stop() override
    {
        _bexit = true;
        if (_caller && _caller->joinable())
            _caller->join();
    }

    char const* description() override
    {
        return "Fake Camera";
    }

    char const* image_type() override
    {
        return _sources_format.c_str();
    }
};

CameraPtr create_camera_fake(Json::Value const& cfg)
{
    auto const& json_sources = json_get<std::string>(cfg, "files_mask");
    auto const& json_format = json_get<std::string>(cfg, "image_type");
    float const fps = json_get<float>(cfg, "fps");
    auto result = std::make_shared<FakeCamera>(json_sources, json_format, fps);
    return result;
}
