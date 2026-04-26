#include "leader_follower.hpp"

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <chrono>

const cv::Size image_size(640, 480);
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

Eigen::Vector2d inverse_convert_point(const cv::Point& point)
{
    const double min_len = std::min((double)image_size.height, (double)image_size.width) / (2.0 * max);
    const double cx = image_size.width / 2.0;
    const double cy = image_size.height / 2.0;

    return Eigen::Vector2d(
        (point.x - cx) / min_len,
        (cy - point.y) / min_len
    );
}

void leader::update_viz(cv::Mat img)
{
    cv::circle(img, pos, 10, cv::Scalar(255, 0, 0), -1);
}

void onMouse(int event, int x, int y, int flags, void* userdata)
{
    auto* leader_mouse = static_cast<leader*>(userdata);
    if (event == cv::EVENT_MOUSEMOVE) {
        leader_mouse->pos.x = x;
        leader_mouse->pos.y = y;
    }
}

int main()
{
    leader leader_mouse;
    followers followers_(50, 20, 20.0, 1.1, 0.5, 1.5, 0.05);

    cv::namedWindow("swerm");
    cv::setMouseCallback("swerm", onMouse, &leader_mouse);
    
    while (true)
    {
        cv::Mat img(image_size.height, image_size.width, CV_8UC3, cv::Scalar(255, 255, 255));
        leader_mouse.update_viz(img);
        Eigen::Vector2d leader_pos = inverse_convert_point(leader_mouse.pos);
        auto start = std::chrono::steady_clock::now();
        followers_.control_loop(leader_pos);
        auto end = std::chrono::steady_clock::now();
        double ms = std::chrono::duration<double, std::milli>(end - start).count();
        std::cout << ms << std::endl;
        for (int i = 0; i < followers_.Followers.size(); ++i)
        {
            cv::Point point = convert_point(followers_.Followers[i].pos(0), followers_.Followers[i].pos(1));
            cv::circle(img, point, 10, cv::Scalar(0, 255, 0), -1);
        }

        cv::imshow("swerm", img);
        if (cv::waitKey(1) == 27) {
            break;
        }
    }
    
    cv::destroyAllWindows();
    return 0;
}
