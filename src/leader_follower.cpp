#include "leader_follower.hpp"

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

int main()
{
    const cv::Mat img(image_size.height, image_size.width, CV_8UC3, cv::Scalar(255, 255, 255));

    double x = 0;
    double y = 0;
    cv::Point point = convert_point(x, y);
    const cv::Scalar color_red(0, 0, 255);
    cv::circle(img, point, 10, color_red, -1);
    
    cv::imwrite("img/test.png", img);
    return 0;
}