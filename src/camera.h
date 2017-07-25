#pragma once

#include <string>
#include <functional>
#include <stdint.h>
#include "jsonxx.h"


typedef std::function<void(uint8_t const* data, int Nx, int Ny, int64_t frame_timestamp)> image_handler_t;

enum ImageFormat {
    FormatUnknown,
    FormatBayerBGGR,
    FormatBayerRGGB,
    FormatGrayscaled
};

class camera
{
public:
    camera() {}
    virtual ~camera() {}
    virtual void set_handler(image_handler_t const& handler) = 0;
    virtual void run() = 0;
    virtual void stop() = 0;
    virtual std::string description() = 0;
    virtual std::string image_type() = 0;
};

void init_camera(jsonxx::Object const& jsoncfg);
camera* get_camera();
