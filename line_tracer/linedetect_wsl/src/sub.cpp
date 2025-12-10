#include "rclcpp/rclcpp.hpp"
#include "subsub/vision.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include <memory>
#include <functional>
#include <iostream>

void mysub_callback(std::shared_ptr<CamSubNode> node, const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    // 1. 주기(Period) 측정용 시간 기록
    rclcpp::Time now = node->now();
    static rclcpp::Time last_time = now;
    double period_ms = (now - last_time).nanoseconds() / 1000000.0;
    last_time = now;

    // 2. 컬러 모드로 디코딩
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    
    if (frame.empty()) {
        RCLCPP_WARN(node->get_logger(), "frame empty!!!");
        return;
    }

    // 전처리 (내부에서 ROI 자르고 흑백 변환함)
    node->preprocess(frame);
    
    // 트래킹
    double error = node->process_tracking();

    // 화면 출력 (컬러 영상 출력됨)
    node->display_result(frame);

    // 로그 출력 (Period 확인)
    if (period_ms > 0.1) {
        RCLCPP_INFO(node->get_logger(), "Error: %.2f | Period: %.2f ms", error, period_ms);
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CamSubNode>();
    

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    std::function<void(const sensor_msgs::msg::CompressedImage::SharedPtr)> fn;
    fn = std::bind(mysub_callback, node, std::placeholders::_1);

    auto mysub = node->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/compressed_7", 
        qos_profile, 
        fn);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
