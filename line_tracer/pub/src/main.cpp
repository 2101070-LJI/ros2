#include "rclcpp/rclcpp.hpp"
#include "pubpub/cam_pub_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<CamPubNode>(
        "linedetect_nano",
        "linedetect_wsl",  
        "/home/linux/simulation/5_lt_cw_100rpm_out.mp4");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
