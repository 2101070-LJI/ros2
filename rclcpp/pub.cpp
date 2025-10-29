#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("node_pub1_3");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    auto mypub = node->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", qos_profile);

    geometry_msgs::msg::Twist message;
    std_msgs::msg::String str;
    rclcpp::WallRate loop_rate(1.0);
    
    while(rclcpp::ok()){
        message.linear.x = 0;
        message.angular.z = 0;
        
        std::cin>>str.data;
        if(str.data == "f"){
            message.linear.x = 3;
        }
        else if(str.data == "b"){
            message.linear.x = -3;
        }
        else if(str.data == "l"){
            message.angular.z = -3;
        }
        else if(str.data == "r"){
            message.angular.z = 3;
        }
        RCLCPP_INFO(node->get_logger(), "Publish: %s",str.data.c_str());
        mypub->publish(message);
        //rclcpp::spin_some(node);
        loop_rate.sleep(); //반복주파수에서 남은 시간 만큼 sleep
    }

    rclcpp::shutdown();
    return 0;
}