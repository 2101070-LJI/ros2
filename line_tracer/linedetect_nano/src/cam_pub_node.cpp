#include "pubpub/cam_pub_node.hpp"

CamPubNode::CamPubNode(
    const std::string & node_name,
    const std::string & topic_name,
    const std::string & video_source)
: Node(node_name),               
  qos_profile_(10), 
  video_source_(video_source)   
{
    // QoS 설정
    qos_profile_ = rclcpp::QoS(rclcpp::KeepLast(10));

    // 2. 전달받은 topic_name으로 퍼블리셔 생성
    publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        topic_name, qos_profile_);

    // 3. 저장된 video_source_로 영상 열기
    cap_.open(video_source_);

    if (!cap_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Could not open video source: %s", video_source_.c_str());
    }
    
}

void CamPubNode::publish_frame()
{
    cap_ >> frame_;
    if (frame_.empty()) {
        // 영상이 끝나거나 에러 시 로그 출력 후 종료
        RCLCPP_ERROR(this->get_logger(), "Frame empty (video end or read error)");
        rclcpp::shutdown();
        return;
    }

    std_msgs::msg::Header header;
    header.stamp = this->now();

    auto msg = cv_bridge::CvImage(header, "bgr8", frame_).toCompressedImageMsg();
    publisher_->publish(*msg);
}
