#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("node_pub1_2");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    auto mypub = node->create_publisher<geometry_msgs::msg::Vector3>("topic_pub1_2", qos_profile);

    geometry_msgs::msg::Vector3 message;
    rclcpp::WallRate loop_rate(1.0);

    while(rclcpp::ok()){
        std::cin>>message.x;
        std::cin>>message.y;
        std::cin>>message.z;
        RCLCPP_INFO(node->get_logger(), "Publish: %lf %lf %lf", message.x, message.y, message.z);
        mypub->publish(message);
        //rclcpp::spin_some(node);
        loop_rate.sleep(); //반복주파수에서 남은 시간 만큼 sleep
    }

    rclcpp::shutdown();
    return 0;
}