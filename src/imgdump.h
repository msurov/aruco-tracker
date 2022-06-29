#pragma once

#include <opencv2/core.hpp>
#include <cppmisc/task_queue.h>
#include <cppmisc/json.h>


namespace imgdump
{
    void init(Json::Value const& cfg);
    void push(char const* name, cv::Mat const& im);
    void push(std::string const& name, cv::Mat const& im);
    bool enabled();
};
