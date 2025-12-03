#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "opencv2/opencv.hpp"
#include <memory>
#include <functional>
#include <iostream>
using std::placeholders::_1;

cv::VideoWriter recorder("save.mp4",cv::VideoWriter::fourcc('X','2','6','4'),30,cv::Size(640, 90));

void mysub_callback(rclcpp::Node::SharedPtr node, const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    
    cv::Scalar m1 = cv::mean(gray);
    gray = gray + (100 - m1[0]);
    
    cv::Mat bin;
    cv::threshold(gray, bin, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    
    cv::Mat labels, stats, centroids;
    int n = cv::connectedComponentsWithStats(bin, labels, stats, centroids);
    
    //cv::Mat bin_color;
    //cv::cvtColor(bin, bin_color, cv::COLOR_GRAY2BGR);
    
    cv::RNG rng(12345);
    
    for (int i = 1; i < n; ++i) {
        int x = stats.at<int>(i, cv::CC_STAT_LEFT);
        int y = stats.at<int>(i, cv::CC_STAT_TOP);
        int width = stats.at<int>(i, cv::CC_STAT_WIDTH);
        int height = stats.at<int>(i, cv::CC_STAT_HEIGHT);
    
        cv::Scalar color(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
    
        cv::rectangle(bin, cv::Rect(x, y, width, height), color, 2);
    
        double cx = centroids.at<double>(i, 0);
        double cy = centroids.at<double>(i, 1);
    
        cv::circle(bin, cv::Point((int)cx, (int)cy), 4, color, -1);
    }
    
    cv::imshow("bin", bin);
    

    //recorder.write(frame);

    cv::waitKey(1);
    RCLCPP_INFO(node->get_logger(), "Received Image : %s,%d,%d", msg->format.c_str(),frame.rows,frame.cols);
}
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("camsub_wsl_7");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)); //TCP
    //auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(); //UDP
    std::function<void(const sensor_msgs::msg::CompressedImage::SharedPtr msg)> fn;
    fn = std::bind(mysub_callback, node, _1);
    auto mysub = node->create_subscription<sensor_msgs::msg::CompressedImage>("image/compressed_7",qos_profile,fn);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}