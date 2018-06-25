#pragma once

#include <opencv2/opencv.hpp>
#include "jsonxx.h"


void imgdump_init(jsonxx::Object const& cfg);
void imgdump(std::string const& name, cv::Mat const& img, std::string const& type="auto");
