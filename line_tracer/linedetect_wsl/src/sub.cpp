#include "rclcpp/rclcpp.hpp"
#include "subsub/vision.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include <memory>
#include <functional>
#include <iostream>
#include <chrono> 

void mysub_callback(std::shared_ptr<CamSubNode> node, const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    if (frame.empty()) {
        RCLCPP_WARN(node->get_logger(), "frame empty!!!");
        return;
    }

    // 1. 시간 측정 시작
    auto start = std::chrono::steady_clock::now();

    cv::Mat binary_roi, view_roi;

    // 2. 기능 실행
    node->preprocess(frame, binary_roi, view_roi);
    double error = node->process_tracking(binary_roi, view_roi);

    // 3. 시간 측정 끝 및 계산
    auto end = std::chrono::steady_clock::now();
    auto diff = end - start;
    double dt = std::chrono::duration<double, std::milli>(diff).count();

    // 4. 화면 출력
    node->display_result(frame, view_roi);

    // 5. 로그 출력 
    RCLCPP_INFO(node->get_logger(), "Error: %.2f | Time: %.2f ms", error, dt);
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
