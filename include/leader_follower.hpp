#pragma once

#include "Boid.hpp"
#include "follower.hpp"
#include <opencv2/opencv.hpp>
#include <random>

cv::Point convert_point(double x, double y);
Eigen::Vector2d inverse_convert_point(const cv::Point& point);

class leader
{
public:
    cv::Point pos;

    void update_viz(cv::Mat img);
};
