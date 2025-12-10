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

    void preprocess(const Mat& source); 
    double process_tracking();          
    void display_result(const Mat& source);

private:
    Point previousCenter_;
    bool firstFrame_;

    // 이전 프레임의 라인 면적을 기억하는 변수
    int previousArea_; 

    // 멤버 변수
    Mat gray_;
    Mat binary_roi_;
    Mat view_roi_;
    Mat labelImage_, stats_, centroids_;
};

#endif
