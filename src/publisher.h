#pragma once

#include "markers.h"
#include <cppmisc/json.h>

class Publisher;
using PublisherPtr = std::shared_ptr<Publisher>;

class Publisher
{
public:
    virtual ~Publisher() {}
    virtual bool start() = 0;
    virtual void term() = 0;
    virtual bool publish(int64_t t, std::vector<Marker> const& markers) = 0;
};

PublisherPtr create_publisher(Json::Value const& cfg);
