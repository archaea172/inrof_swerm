#include "leader_follower.hpp"

int main()
{
    const cv::Size image_size(1280, 960);
    const cv::Mat img = cv::Mat::zeros(image_size.height, image_size.width, CV_8UC3);

    cv::imwrite("img/test.png", img);
    return 0;
}