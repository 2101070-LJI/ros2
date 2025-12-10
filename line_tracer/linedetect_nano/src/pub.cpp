#include "rclcpp/rclcpp.hpp"
#include "pubpub/cam_pub_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    // 생성자에 인자 3개를 직접 전달하여 노드 생성
    auto node = std::make_shared<CamPubNode>(
        "campub7",                               // 1. 노드 이름
        "image/compressed_7",                                // 2. 토픽 이름
        "/home/linux/simulation/7_lt_ccw_100rpm_in.mp4"  // 3. 비디오 경로
    );

    // 루프 주기 설정 (30Hz)
    rclcpp::WallRate loop_rate(30.0);

    while (rclcpp::ok()) {
        // 프레임 퍼블리시 
        node->publish_frame();

        // 주기 맞추기
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
