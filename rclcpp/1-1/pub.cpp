#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("node_pub1_1"); // 노드 생성
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)); // Qos 설정
    auto mypub = node->create_publisher<std_msgs::msg::Int32>("topic_pub1_1", qos_profile); // topic_pub1_1 토픽에 전송

    std_msgs::msg::Int32 message; // 인터페이스 생성
    message.data = 0; // 값 초기화
    rclcpp::WallRate loop_rate(1.0); // 1hz

    while(rclcpp::ok()){
        RCLCPP_INFO(node->get_logger(), "Publish: %d", message.data++); // 값 확인
        mypub->publish(message); // 값 전송
        loop_rate.sleep(); //반복주파수에서 남은 시간 만큼 sleep
    }

    rclcpp::shutdown();
    return 0;
}