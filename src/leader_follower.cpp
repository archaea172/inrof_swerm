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

    Boid test_boid(max);
    cv::Point test_point = convert_point(test_boid.x(0), test_boid.x(1));
    cv::circle(img, test_point, 10, (255, 0, 0), -1);
    cv::imwrite("img/test.png", img);
    return 0;
}