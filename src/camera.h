#pragma once

#include <string>
#include <functional>
#include <stdint.h>
#include <cppmisc/json.h>
#include <memory>
#include <map>


struct RawImageWrap {
    uint8_t const* data;
    int Nx;
    int Ny;
    char const* format;
    int64_t frame_timestamp;
};

typedef std::function<void(RawImageWrap const& raw)> ImageHandler;

class Camera
{
public:
    Camera() {}
    virtual ~Camera() {}
    virtual void set_handler(ImageHandler const& handler) = 0;
    virtual void run() = 0;
    virtual void stop() = 0;
    virtual char const* description() = 0;
    virtual char const* image_type() = 0;
};

using CameraPtr = std::shared_ptr<Camera>;
CameraPtr init_camera(Json::Value const& cfg);
