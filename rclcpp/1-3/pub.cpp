#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("node_pub1_3"); // 노드 생성
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)); // Qos 설정
    auto mypub = node->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", qos_profile); //turtle1/cmd_vel 토픽에 전송

    geometry_msgs::msg::Twist message; // Vector3 linear, angular
    std_msgs::msg::String str; // String data
    
    while(rclcpp::ok()){
        message.linear.x = 0; // 값 초기화
        message.angular.z = 0; // 값 초기화
        
        std::cin>>str.data;
        if(str.data == "f"){ // 전진
            message.linear.x = 3;
        }
        else if(str.data == "b"){ // 후진
            message.linear.x = -3;
        }
        else if(str.data == "l"){ // 좌회전
            message.angular.z = -3;
        }
        else if(str.data == "r"){ // 우회전
            message.angular.z = 3;
        }
        RCLCPP_INFO(node->get_logger(), "Publish: %s",str.data.c_str());
        mypub->publish(message);
    }

    rclcpp::shutdown();
    return 0;
}
