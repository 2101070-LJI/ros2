#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("node_pub1_2"); // 노드 생성
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)); // Qos설정
    auto mypub = node->create_publisher<geometry_msgs::msg::Vector3>("topic_pub1_2", qos_profile); // topic_pub1_2에 전송

    geometry_msgs::msg::Vector3 message; // float64 x, y, z

    while(rclcpp::ok()){
        std::cin>>message.x; // 값 입력
        std::cin>>message.y; // 값 입력
        std::cin>>message.z; // 값 입력
        RCLCPP_INFO(node->get_logger(), "Publish: %lf %lf %lf", message.x, message.y, message.z);
        mypub->publish(message); // 전송
    }

    rclcpp::shutdown();
    return 0;
}
