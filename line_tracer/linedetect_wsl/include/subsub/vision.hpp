#ifndef SUBSUB_VISION_HPP_
#define SUBSUB_VISION_HPP_

#include "rclcpp/rclcpp.hpp"
#include "opencv2/opencv.hpp"
#include <vector>

using namespace cv;
using namespace std;

class CamSubNode : public rclcpp::Node
{
public:
    CamSubNode();
    ~CamSubNode();

    void preprocess(const Mat& source, Mat& binary_out, Mat& view_out);
    double process_tracking(const Mat& binary_roi, Mat& view_roi);
    void display_result(const Mat& original, const Mat& result_view);

private:
    Point previousCenter_;
    bool firstFrame_;
};

#endif