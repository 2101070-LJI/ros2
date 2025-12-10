#include "rclcpp/rclcpp.hpp"
#include "subsub/vision.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include <signal.h>

void ctrlc(int)
{
    rclcpp::shutdown();
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    signal(SIGINT, ctrlc);

    // 1. 노드(도구 상자) 생성
    auto node = std::make_shared<CamSubNode>();

    // 2. QoS 설정
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    // 3. 콜백 함수 정의 (여기에 로직의 순서를 적습니다)
    auto callback = [node](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        
        // (1) 디코딩
        cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        if (frame.empty()) {
            RCLCPP_WARN(node->get_logger(), "frame empty!!!");
            return;
        }

        cv::Mat binary_roi, view_roi;

        // (2) 전처리 호출
        node->preprocess(frame, binary_roi, view_roi);

        // (3) 트래킹 및 에러 계산 호출
        double error = node->process_tracking(binary_roi, view_roi);

        // (4) 화면 출력 호출
        node->display_result(frame, view_roi);

        // (5) 결과 확인 (로그)
        RCLCPP_INFO(node->get_logger(), "Error: %.2f", error);
    };

    // 4. 구독 생성
    auto sub = node->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/compressed_7", 
        qos_profile, 
        callback);

    // 5. 실행 대기
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}