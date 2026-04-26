#pragma once

#include "Boid.hpp"
#include <opencv2/opencv.hpp>
#include <random>

class leader
{
public:
    cv::Point pos;

    void update_viz(cv::Mat img);
};
