#ifndef PUBPUB_CAM_PUB_NODE_HPP_
#define PUBPUB_CAM_PUB_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_msgs/msg/header.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <string>

class CamPubNode : public rclcpp::Node
{
public:
    // 생성자: 인자를 받아 멤버 변수를 초기화합니다.
    CamPubNode(
        const std::string & node_name,
        const std::string & topic_name,
        const std::string & video_source);

    void publish_frame();

private:
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
    rclcpp::QoS qos_profile_;

    cv::VideoCapture cap_;
    cv::Mat frame_;
    std::string video_source_;
};

#endif