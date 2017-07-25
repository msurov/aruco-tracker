#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include "jsonxx.h"

void BinBox3x3(cv::Mat const& src, cv::Mat& dst);
void BinMedian3x3(cv::Mat const& src, cv::Mat& dst);
void BinEdgeRect3x3(cv::Mat const& src, cv::Mat& dst);
void BinEdgeCross3x3(cv::Mat const& src, cv::Mat& dst);
void DrawPoints(cv::Mat& plot, std::vector<cv::Point2i> const& points, cv::Scalar const& color);
void DrawCircle(cv::Mat& plot, float cx, float cy, float r, cv::Scalar const& color);
void DrawRect(cv::Mat& plot, cv::Rect const& r, cv::Scalar const& color);
void CopyChannel(cv::Mat const& src, cv::Mat& dst, int ch);
void DrawCross(cv::Mat& plot, int x, int y, int size, cv::Scalar const& color);
void DrawLine(cv::Mat& plot, cv::Point2f const& p1, cv::Point2f const& p2, cv::Scalar const& color);
cv::Mat read_raw(std::string const& path);
cv::Mat bggr2bgr(cv::Mat const& bggr);
cv::Mat bgr2bggr(cv::Mat const& bgr);
