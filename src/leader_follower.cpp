#include "leader_follower.hpp"

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>

const cv::Size image_size(1280, 960);
const int max = 20;

cv::Point convert_point(double x, double y)
{
    const double min_len = std::min((double)image_size.height, (double)image_size.width) / (2.0 * max);
    const double cx = image_size.width / 2.0;
    const double cy = image_size.height / 2.0;
    return cv::Point(
        cvRound(cx + x * min_len),
        cvRound(cy - y * min_len)
    );
}

cv::Mat render_frame(const Boids& boids)
{
    cv::Mat img(image_size.height, image_size.width, CV_8UC3, cv::Scalar(255, 255, 255));

    for (const auto& boid : boids.boids) {
        cv::Point p = convert_point(boid.pos(0), boid.pos(1));
        cv::circle(img, p, 20, cv::Scalar(255, 0, 0), -1);
    }

    return img;
}

int main()
{
    const int boid_count = 50;
    const int frame_count = 120;
    const int frame_duration_ms = 50;

    Boids boids(boid_count, max, 0.9, 1.1, 0.5, 0.5);

    std::filesystem::create_directories("img");

    std::filesystem::create_directories("img/frames");

    for (int frame_index = 0; frame_index < frame_count; ++frame_index) {
        cv::Mat frame = render_frame(boids);

        std::ostringstream filename;
        filename << "img/frames/frame_" << std::setw(4) << std::setfill('0') << frame_index << ".png";
        cv::imwrite(filename.str(), frame);
        boids.control_loop();
    }

    return 0;
}
