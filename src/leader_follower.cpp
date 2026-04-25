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

void onMouse(int event, int x, int y, int flags, void* userdata)
{
    if (event == cv::EVENT_MOUSEMOVE) {
        std::cout << "mouse: x=" << x << ", y=" << y << std::endl;
    }
}

int main()
{
    cv::Mat img(image_size.height, image_size.width, CV_8UC3, cv::Scalar(255, 255, 255));

    cv::namedWindow("swerm");
    cv::setMouseCallback("swerm", onMouse);
    
    while (true)
    {
        cv::imshow("swerm", img);
        if (cv::waitKey(1) == 27) {
            break;
        }
    }
    
    cv::destroyAllWindows();
    return 0;
}
