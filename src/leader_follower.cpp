#include "leader_follower.hpp"

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>

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

    cv::namedWindow("swerm");
    cv::setMouseCallback("swerm", onMouse, &leader_mouse);
    
    while (true)
    {
        cv::Mat img(image_size.height, image_size.width, CV_8UC3, cv::Scalar(255, 255, 255));
        leader_mouse.update_viz(img);
        cv::imshow("swerm", img);
        if (cv::waitKey(1) == 27) {
            break;
        }
    }
    
    cv::destroyAllWindows();
    return 0;
}
