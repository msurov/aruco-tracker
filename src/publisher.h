#pragma once

#include "aruco_detector.h"

class Publisher;
using PublisherPtr = std::shared_ptr<Publisher>;

class Publisher
{
public:
    virtual bool start() = 0;
    virtual void term() = 0;
    virtual bool publish(int64_t t, std::vector<MarkerPose> const& marker_pose_arr) = 0;
};

PublisherPtr create_publisher(Json::Value const& cfg);
