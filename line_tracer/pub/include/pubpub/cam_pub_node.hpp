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
    CamPubNode(
        const std::string & node_name;
        const std::string & topic_name;
        const std::string & video_source;);

    ~CamPubNode() override = default;

private:
    void publish_frame();  // 타이머 콜백에서 한 프레임씩 퍼블리시

    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::QoS qos_profile_;

    cv::VideoCapture cap_;
    cv::Mat frame_;
    std::string video_source_;
};

#endif 
