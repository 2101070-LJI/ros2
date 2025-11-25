#include "rclcpp/rclcpp.hpp"
#include "dxl2/dxl.hpp"
#include <memory>
#include <chrono>
#include <iostream>
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("dxl2");
    rclcpp::WallRate loop_rate(10.0);
    Dxl mx;
    int vel1 = 0,vel2 = 0;
    bool check = false;
    if(!mx.open())
    {
        RCLCPP_INFO(node->get_logger(), "dynamixel open error\n");
        rclcpp::shutdown();
        return -1; //장치열기
    }
    while(rclcpp::ok())
    {
        if(vel1 == 400 && vel2 == 400) {check = true;}
        else if(vel1 == -400 && vel2 == -400) {check = false;}

        if(check) {vel1-=10; vel2-=10;}
        else {vel1+=10; vel2+=10;}


        mx.setVelocity(vel1,vel2);
        
        RCLCPP_INFO(node->get_logger(),"left speed:%d,right speed:%d",
        vel1,vel2);
        loop_rate.sleep();
    }
    mx.close();
    rclcpp::shutdown();
    return 0;
}
